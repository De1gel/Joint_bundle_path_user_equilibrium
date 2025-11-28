#include "Solver.h"
#include "Network.h"
#include "Layer.h"
#include "Node.h"
#include "Link.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>


void DemandStore::loadTripFile(const std::string& filename,
                               const Network*     net,
                               int                user_count)
{
    std::ifstream fin(filename);
    if (!fin) throw std::runtime_error("Cannot open " + filename);

    std::string line;
    // 跳过注释行/说明行，直到遇到 "Destination"
    while (std::getline(fin, line)) {
        if (line.empty()) continue;                     // ★ 跳过空行
        if (line.rfind("Destination", 0) == 0) break;
    }

    // 获取 Walk 层
    Layer* base = net->getLayer("Walk");
    auto* walk = dynamic_cast<WalkLayer*>(base);
    if (!walk) throw std::runtime_error("Walk layer not found");

    // 逐条读取
    while (!line.empty()) {
        // 解析 Destination 行：Destination <id> <#origins>
        std::istringstream head(line);
        std::string tag, destIdStr, originNumStr;
        head >> tag >> destIdStr >> originNumStr;
        Node* dest = walk->getNode(walk->getMode() + destIdStr);
        if (!dest) throw std::runtime_error("Dest node " + destIdStr + " not found");

        // 读取下一行，格式 "org:dem1,dem2,…;org2:…"
        if (!std::getline(fin, line)) break;
        std::istringstream data(line);
        std::string odToken;
        while (std::getline(data, odToken, ';')) {
            std::istringstream pairTok(odToken);
            std::string orgStr, demStr;
            if (!std::getline(pairTok, orgStr, ':')) continue;
            std::getline(pairTok, demStr);

            Node* org = walk->getNode(walk->getMode() + orgStr);
            if (!org) throw std::runtime_error("Origin node " + orgStr + " not found");

            // 解析逗号分隔的各用户需求
            std::vector<double> demands;
            std::istringstream demStream(demStr);
            std::string num;
            while (std::getline(demStream, num, ',')) {
                if (!num.empty()) {
                    demands.push_back(std::stod(num));
                }
            }

            // 根据 user_count 调整长度
            if ((int)demands.size() < user_count) {
                // 不足则补 0
                demands.resize(user_count, 0.0);
                // 抛出警告
                // std::cerr << "Note: Demand for " << orgStr << " to " << destIdStr
                //           << " has less than " << user_count << " users, padding with zeros." << std::endl;
            } else if ((int)demands.size() > user_count) {
                // 超出则截断
                demands.resize(user_count);
            }
            // 存入 OD 表
            od_[{org, dest}] = std::move(demands);
        }

        // 读取下一个 Destination 行
        if (!std::getline(fin, line)) break;
    }
}

void DemandStore::updateDemand(const ODKey& key, const std::vector<double>& demand, int iter)
{
    // 检查新旧需求总和与长度是否相同
    if (od_.count(key) && od_[key].size() == demand.size()) {
        double oldSum = std::accumulate(od_[key].begin(), od_[key].end(), 0.0);
        double newSum = std::accumulate(demand.begin(), demand.end(), 0.0);
        if (std::abs(oldSum - newSum) > 1e-6) {
            std::cerr << "Warning: Demand sum mismatch" << std::endl;
        }
    }
    for (size_t i = 0; i < od_[key].size(); ++i) {
        od_[key][i] = (1.0 - 1.0 / iter) * od_[key][i] + 1.0 / iter * demand[i];
    }

}


/* ---- 取单条 OD, 若无返回空向量 ---- */
const std::vector<double>& DemandStore::getDemand(Node* o, Node* d) const
{
    static const std::vector<double> empty;
    auto it = od_.find({o,d});
    return (it==od_.end()) ? empty : it->second;
}

const double DemandStore::getTotalDemand(const ODKey& key) const
{
    static const double empty = 0.0;
    auto it = od_.find(key);
    return (it == od_.end()) ? empty : std::accumulate(it->second.begin(), it->second.end(), 0.0);
}

size_t AssignmentSolver::addOrAccumulatePath(const ODBundle& key,
                                             const std::vector<Link*>& links)
{
    auto& vec = pathPool_[key];
    for (size_t i=0;i<vec.size();++i)
        if (vec[i].links == links){
            return i;      // 已有返回下标
        }
    vec.push_back(Path{links, 0.0, /*cost*/0.0});
    vec.back().assignId();  // 分配新 ID
    return vec.size()-1;                          // 新插入
}


double AssignmentSolver::computePathCost(const std::vector<Link*>& links, int mode) const
{
    /* ---------- layer 计费 ---------- */
    double cost = 0.0;
    std::vector<Node*> seg;  // 当前仍在同一层的节点序列

    Layer* curL = nullptr;
    for (size_t i = 0; i < links.size(); ++i) {
        Link* lk = links[i];
        Node* h  = lk->getHead();
        Node* t  = lk->getTail();
        Layer* Lh = h->getParentLayer();
        Layer* Lt = t->getParentLayer();

        if (!curL) curL = Lh;
        seg.push_back(h);

        /* 累加 link 行驶时间（或 0） */
        if (mode == 0) {
            cost += lk->getLength() * (user_config::heter_user ?
                user_config::heter_VOT.at(user_config::current_user) :
                user_config::VOT);
            if (lk->getCapacity() > 0.0) {
                cost += lk->getShadowPrice();  // 叠加影子价格
            }
        }

        if (Lh != Lt) {                      // 离开 curL
            // mode <= 1时正常计费，mode=2只计算walk层，mode=3只计算bike层。。。
            if (mode <= 1 || (mode == 5 && dynamic_cast<WalkLayer*>(curL)) || 
                (mode == 2 && dynamic_cast<WalkLayer*>(curL)) || 
                (mode == 3 && typeid(*curL) == typeid(Layer) && seg.front() == nullptr) || 
                (mode == 4 && dynamic_cast<TransitLine*>(curL))) {
                cost += curL->farematrix(seg);  // 累计当前层的 fare
            }
            seg.clear();
            if (Lt->getMode() == "Road" && dynamic_cast<ARHlink*>(lk)) {
                seg.push_back(nullptr);  // 加入空节点表示换层
            }
            curL = Lt;
        }

        /* 最后一条 link，把尾节点塞进去 */
        if (i == links.size()-1) seg.push_back(t);
    }
    /* 末尾若仍在层内需结算 */
    if (!seg.empty() && curL) {
        cost += curL->farematrix(seg);
    }
    return cost;
}



void AssignmentSolver::buildAuxFlow(FlowMap& aux, HeterFlowMap& auxH, 
                                    PathMap& auxPathFlow)
{
    aux.clear(); auxH.clear();

    const bool heter = user_config::heter_user;
    const auto& types = user_config::heter_types;
    const int B = heter ? (int)types.size() : 1;

    for (int b = 0; b < B; ++b) {
        if (heter) net_->setCurrentUser(types[b]);
        for (const auto& [odPair, vec] : dem_->table()) {
            Node* o = odPair.first;
            Node* d = odPair.second;

            std::vector<double> demand = vec;
            if (demand.size() < B) demand.resize(B, 0.0);
            double q = demand[b];
            if (q <= 0) continue;  // 无需求则跳过
            Path path = net_->shortestPath(o, d, "Dijkstra", "Layer", 2);
            spCost_[{o, d, b}] = path.cost;  // 缓存最短路径成本
            size_t idx = addOrAccumulatePath({o, d, b}, path.links);
            auxPathFlow[{o, d, b}][idx] = q;
            for (Link* lk : path.links) {
                aux[lk] += q;
                if (heter) {
                    auto& v = auxH[lk];
                    if (v.size() < B) v.resize(B, 0.0);
                    v[b] += q;
                }
            }
        }
    }
}

void AssignmentSolver::buildBundleAuxFlow(FlowMap& aux, HeterFlowMap& auxH, 
                                    PathMap& auxPathFlow)
{
    aux.clear(); auxH.clear();

    const bool heter = user_config::heter_user;
    const auto& types = user_config::heter_types;
    const int B = heter ? (int)types.size() : 1;

    for (const auto& [odPair, vec] : dem__.table()) {
        Node* o = odPair.first;
        Node* d = odPair.second;

        std::vector<double> demand = vec;
        if (demand.size() < B) demand.resize(B, 0.0);
        double q_total = std::accumulate(demand.begin(), demand.end(), 0.0);
        double q = 0.0;  // 总需求
        if (q_total <= 0) continue;  // 无需求则跳过
        std::vector<Path> spath;
        for (int b = 0; b < B; ++b) {
            if (heter) net_->setCurrentUser(types[b]);
            spath.push_back(net_->shortestPath(o, d, "Dijkstra", "Layer", 2));
        }
        for (int b = 0; b < B; ++b) {
            double q_1 = q + demand[b];
            double x_1 = user_config::inverseCDF(q / q_total);  // 原比例
            double x_2 = user_config::inverseCDF(q_1 / q_total);  // 新比例
            q = q_1;
            double a_L = bundle_price[b] / x_1;  // bundle 成本L
            sbpCost_[{o, d, b}] = a_L;
            spCost_[{o, d, b}] = spath[b].cost + a_L;
            int s_bundle = b;  // 保存 bundle id
            for (int b_ = 0; b_ < B; ++b_) {
                if (b_ < b && spath[b_].cost + bundle_price[b_] / x_1 < spCost_[{o, d, b}]) {
                    spCost_[{o, d, b}] = spath[b_].cost + bundle_price[b_] / x_1;
                    s_bundle = b_;
                }
                if (b_ > b && spath[b_].cost + bundle_price[b_] / x_2 < spath[b].cost + bundle_price[b] / x_2) {
                    spCost_[{o, d, b}] = spath[b_].cost + bundle_price[b_] / x_2;
                    s_bundle = b_;
                }
            }
            size_t idx = addOrAccumulatePath({o, d, s_bundle}, spath[s_bundle].links);
            auxPathFlow[{o, d, s_bundle}][idx] += demand[b];
            for (Link* lk : spath[s_bundle].links) {
                aux[lk] += demand[b];
                if (heter) {
                    auto& v = auxH[lk];
                    if (v.size() < B) v.resize(B, 0.0);
                    v[s_bundle] += demand[b];
                }
            }
        }
        // size_t idx = addOrAccumulatePath({o, d, sb_bundle}, spath.links);
        // auxPathFlow[{o, d, sb_bundle}][idx] = q;
        // for (Link* lk : spath.links) {
        //     aux[lk] += q;
        //     if (heter) {
        //         auto& v = auxH[lk];
        //         if (v.size() < B) v.resize(B, 0.0);
        //         v[sb_bundle] += q;
        //     }
        // }
    }
}

/* ---------- (B) 计算相对 Z-gap ---------- */
double AssignmentSolver::calcRelGap(const FlowMap& aux) const
{
    double zCur = 0.0, zAux = 0.0;
    for (Link* lk : net_->getLinks()) {
        double c = lk->getCost();          // c(x^k)
        zCur += c * lk->getVolume();
        auto it = aux.find(lk);
        if (it != aux.end()) zAux += c * it->second;
    }
    return (zCur < 1e-12) ? 0.0 : (zCur - zAux) / zCur;
}

double AssignmentSolver::calcBundlePathGap() const
{
    double numer=0.0, denom=0.0;

    for (const auto& [key, vec] : pathPool_) {

        for (const auto& p : vec) if (p.flow>0) {
            double a = sbpCost_.at({key.o,key.d});
            numer += p.flow * (p.cost - sbpCost_.at({key.o,key.d}));
            denom += p.flow *  p.cost;
        }
    }
    return (denom<1e-12) ? 0.0 : numer/denom;
}

double AssignmentSolver::calcPathGap() const
{
    double numer=0.0, denom=0.0;

    for (const auto& [key, vec] : pathPool_) {
        /* 找该 OD-bundle 最短 */
        double Cmin = spCost_.at(key);

        for (const auto& p : vec) if (p.flow>0) {
            numer += p.flow * (p.cost - Cmin);
            denom += p.flow *  p.cost;
        }
    }
    return (denom<1e-12) ? 0.0 : numer/denom;
}

/* ---------- (C) MSA 更新 volume ---------- */
void AssignmentSolver::updateLinkVolumeMSA(const FlowMap& aux,
                                       const HeterFlowMap& auxH,
                                       int iter,
                                       PathMap& auxPathFlow)
{
    const bool heter = user_config::heter_user;
    const auto& types = user_config::heter_types;
    const int B = heter ? (int)types.size() : 1;

    double alpha = 1.0 / iter;  // MSA 更新系数
    for (auto& [key, vec] : pathPool_) {
        auto& auxVec = auxPathFlow[key];      // 可能为空
        for (size_t i=0;i<vec.size();++i) {
            double old = vec[i].flow;
            double aux = auxVec.count(i) ? auxVec[i] : 0.0;
            vec[i].flow = (1 - alpha) * old + alpha * aux;
        }
    }
    for (Link* lk : net_->getLinks()) {
        double x_aux = aux.count(lk) ? aux.at(lk) : 0.0;
        double x_old = lk->getVolume();
        lk->setVolume((1 - alpha) * x_old + alpha * x_aux);

        if (heter) {
            auto hv = auxH.count(lk) ? auxH.at(lk) : std::vector<double>(B, 0.0);
            hv.resize(B, 0.0);
            for (int b = 0; b < B; ++b) {
                double oldH = lk->getHeterVolume(types[b]);
                double newH = (1 - alpha) * oldH + alpha * hv[b];
                lk->setHeterVolume(types[b], newH);
            }
        }
    }
}

void AssignmentSolver::updateBundleLinkVolumeMSA(const FlowMap& aux,
                                       const HeterFlowMap& auxH,
                                       int iter,
                                       PathMap& auxPathFlow/*,
                                       PathMap& prevPathFlow,
                                       PathMap& prevAuxPathFlow,
                                       double& h_k*/)
{
    const bool heter = user_config::heter_user;
    const auto& types = user_config::heter_types;
    const int B = heter ? (int)types.size() : 1;

    double delta = 0.0, gap = 0.0;
    for (Link* lk : net_->getLinks()) {
        double newVol = aux.count(lk) ? aux.at(lk) : 0.0;
        // double diff = lk->getVolume() - newVol;  // 边流量差
        if (lk->getCapacity() <= 0) continue;  // 无容量限制的边不考虑
        gap   += std::max(0.0, -lk->getVacancy(newVol));      // t(x_aux)
        delta += std::max(0.0, -lk->getVacancy());      // t(x_cur)
    }
    h_k += std::pow(2* gap / std::max((delta + gap), 1e-12), 2.0);  // SRA 步长更新
    h_k_ += std::pow(2* gap / std::max((delta + gap), 1e-12), 0.5);  // SRA 步长更新
    // h_k += gap / std::max((delta + gap), 1e-12);  // SRA 步长更新
    // h_k_ += gap / std::max((delta + gap), 1e-12);  // SRA 步长更新
    double alpha = 1.0 / h_k;  // SRA 步长
    double alpha_ = 1.0 / h_k_;  // SRA 步长
    // double alpha = 1.0 / iter;  // MSA 更新系数
    // double alpha_ = 1.0 / iter;  // MSA 更新系数
    if (iter == 1) {
        alpha = 1.0;
        alpha_ = 1.0;
        h_k = 1.0;
        h_k_ = 1.0;
    }
    // std::cerr << "Gap: " << gap << ", Delta: " << delta << ", step: " << (gap / (delta + gap)) << std::endl;
    for (auto& od : dem__.table()) {
        std::vector<double> new_demand;
        for (int b = 0; b < B; b++) {
            double new_q = 0.0;
            ODBundle odb{od.first.first, od.first.second, b};
            auto& vec = pathPool_[odb];
            // auto& prevVec = prevPathFlow[odb];   // 上一轮的路径流量
            auto& auxVec = auxPathFlow[odb];      // 可能为空
            for (size_t i = 0; i < vec.size(); ++i) {
                double old = vec[i].flow;
                double aux = auxVec.count(i) ? auxVec[i] : 0.0;
                vec[i].flow = (1 - alpha) * old + alpha * aux;
                new_q += vec[i].flow;
                // prevVec[i] = vec[i].flow;  // 更新上一轮的路径流量
            }
            new_demand.push_back(new_q);
        }
        dem__.updateDemand(od.first, new_demand, 1);
    }
    for (Link* lk : net_->getLinks()) {
        double x_aux = aux.count(lk) ? aux.at(lk) : 0.0;
        double x_old = lk->getVolume();
        lk->setVolume((1 - alpha) * x_old + alpha * x_aux);
        if (lk->getCapacity() > 0 && dynamic_cast<Tlink*>(lk)) {
            double shadow_price = lk->getShadowPrice();
            double viol = -lk->getVacancy();
            double new_price = std::max(0.0, shadow_price + 0.05 * alpha_ * viol);
            lk->updateShadowPrice(new_price);  // 更新影子价格
        }

        if (heter) {
            auto hv = auxH.count(lk) ? auxH.at(lk) : std::vector<double>(B, 0.0);
            hv.resize(B, 0.0);
            for (int b = 0; b < B; ++b) {
                double oldH = lk->getHeterVolume(types[b]);
                double newH = (1 - alpha) * oldH + alpha * hv[b];
                lk->setHeterVolume(types[b], newH);
            }
        }
    }
}

void AssignmentSolver::updatePathCosts()
{
    for (auto& [key, vec] : pathPool_) {
        net_->setCurrentUser(user_config::heter_user ? user_config::heter_types[key.b] : user_config::current_user);
        for (auto& p : vec) {
            p.cost = computePathCost(p.links);   // 用最新边成本
        }
    }
}

void AssignmentSolver::updateBundlePathCosts()
{
    for (auto& [key, vec] : pathPool_) {
        net_->setCurrentUser(user_config::heter_user ? user_config::heter_types[key.b] : user_config::current_user);
        // Path path = net_->shortestPath(key.o, key.d, "Dijkstra", "Layer", 2);
        // double newCost = path.cost + sbpCost_.at({key.o, key.d, key.b});   // 用最新边成本
        // if (newCost < spCost_.at({key.o, key.d, key.b})) {
        //     spCost_[{key.o, key.d, key.b}] = newCost;  // 更新最短路径成本
        // }
        for (auto& p : vec) {
            p.cost = computePathCost(p.links);   // 用最新边成本
            // std::cerr << " cost: " << spCost_.at({key.o, key.d, key.b}) << std::endl;
            p.cost += sbpCost_.at({key.o, key.d, key.b});  // 加上 bundle 成本
        }
    }
}

/* ---------- (D) 顶层 solveFW ---------- */
void AssignmentSolver::solveFW(int maxIter, double tol)
{
    FlowMap auxVol;
    HeterFlowMap auxHVol;
    PathMap auxPathFlow;
    buildAuxFlow(auxVol, auxHVol, auxPathFlow);
    updateLinkVolumeMSA(auxVol, auxHVol, 1, auxPathFlow);
    auxPathFlow.clear();  // 清空辅助路径流量
    std::ofstream paramFile("D:\\MyCode\\C++\\multi_net\\result\\solveFW.txt");

    for (int it = 1; it <= maxIter; ++it) {
        buildAuxFlow(auxVol, auxHVol, auxPathFlow);            // AON
        updatePathCosts();

        double gap = calcPathGap();         // Gap
        // double gap = calcRelGap(auxVol);
        std::ofstream paramFile("D:\\MyCode\\C++\\multi_net\\result\\solveFW.txt", std::ios::app);
        paramFile << "Iter " << it << " Gap=" << gap << '\n';
        // std::cout << "Iter " << it << " Gap=" << gap << '\n';
        if (gap < tol) break;

        updateLinkVolumeMSA(auxVol, auxHVol, it, auxPathFlow);     // MSA
        auxPathFlow.clear();  // 清空辅助路径流量
        // 可选：边成本更新
        // for(Link* lk : net_->getLinks()) lk->updateCost();
    }
}

void AssignmentSolver::solveBundleFW(int maxIter, double tol)
{
    FlowMap auxVol;
    HeterFlowMap auxHVol;
    PathMap auxPathFlow, prevPathFlow, prevAuxPathFlow;
    std::ofstream paramFile("D:\\MyCode\\C++\\multi_net\\result\\solveBundleFW.txt");
    buildBundleAuxFlow(auxVol, auxHVol, auxPathFlow);
    // prevPathFlow = auxPathFlow;  // 保存初始辅助路径流量
    // prevAuxPathFlow = auxPathFlow;  // 保存初始辅助路径流量
    // double h_k = 0.0;  // 初始步长
    updateBundleLinkVolumeMSA(auxVol, auxHVol, 1, auxPathFlow/*, prevPathFlow, prevAuxPathFlow, h_k*/);
    net_->getLayer("RHRoad")->updateFleetStats();  // 更新 Road 层车队统计
    updateBundlePathCosts();
    auxPathFlow.clear();  // 清空辅助路径流量

    for (int it = 1; it <= maxIter; ++it) {
        buildBundleAuxFlow(auxVol, auxHVol, auxPathFlow);            // AON

        double gap = calcPathGap();         // Gap
        // double gap = calcRelGap(auxVol);
        std::ofstream paramFile("D:\\MyCode\\C++\\multi_net\\result\\solveBundleFW.txt", std::ios::app);
        paramFile << "Iter " << it << " Gap=" << gap << '\n';
        // std::cout << "Iter " << it << " Gap=" << gap << '\n';
        if (gap < tol && it > 15) break;

        updateBundleLinkVolumeMSA(auxVol, auxHVol, it, auxPathFlow/*, prevPathFlow, prevAuxPathFlow, h_k*/);     // MSA
        net_->getLayer("RHRoad")->updateFleetStats();  // 更新 RHRoad 层车队统计
        updateBundlePathCosts();
        auxPathFlow.clear();  // 清空辅助路径流量
        // net_->getCongestionData();
        // getModeChoice();  // 获取交通方式选择统计
    }
}

void AssignmentSolver::resetNetwork(int lel)
{
    // 影子价格与流量重置
    for (Link* lk : net_->getLinks()) {
        if (lk->getCapacity() > 0) {
            lk->updateShadowPrice(0.0);  // 重置影子价格
        }
        if (lel == 1) {
            lk->setVolume(0.0);  // 重置流量
            if (user_config::heter_user) {
                for (const auto& type : user_config::heter_types) {
                    lk->setHeterVolume(type, 0.0);  // 重置异构流量
                }
            }
        }
    }
    if (lel == 1) {
        // 清空路径池
        pathPool_.clear();
        // 清空最短路径成本缓存
        spCost_.clear();
        // 清空 bundle 成本缓存
        sbpCost_.clear();
        // 需求重置
        dem__ = *dem_;  // 恢复初始需求
    }
    h_k_ = 1.0;  // 恢复初始步长
    h_k = 1.0;  // 恢复初始步长
}

void AssignmentSolver::getModeChoice()
{
    std::unordered_map<std::string, double> modeCount;
    std::unordered_map<std::string, double> modeLength;
    // 初始化五种交通方式的计数
    modeCount["Car"] = 0.0;
    modeCount["Bus"] = 0.0;
    modeCount["Bike"] = 0.0;
    modeCount["Walk"] = 0.0;
    modeCount["Ride-Hailing"] = 0.0;
    modeLength["Car"] = 0.0;
    modeLength["Bus"] = 0.0;
    modeLength["Bike"] = 0.0;
    modeLength["Walk"] = 0.0;
    modeLength["Ride-Hailing"] = 0.0;

    for (const auto& [key, vec] : pathPool_) {
        for (const auto& p : vec) {
            if (p.flow > 1e-6) {
                double* current_length = nullptr;  // <-- 修改类型
                for (const Link* lk : p.links) {
                    if (dynamic_cast<const ATlink*>(lk)) {
                        modeCount["Bus"] += p.flow;
                        current_length = &modeLength["Bus"];
                    } else if (dynamic_cast<const ARHlink*>(lk)) {  // <-- 修正类型名
                        modeCount["Ride-Hailing"] += p.flow;
                        current_length = &modeLength["Ride-Hailing"];
                    } else if (dynamic_cast<const Wlink*>(lk) &&
                               lk->getTail()->getParentLayer()->getMode() == "Walk") {
                        modeCount["Walk"] += p.flow;
                        current_length = &modeLength["Walk"];
                    } else if (dynamic_cast<const ARlink*>(lk) &&
                               lk->getTail()->getParentLayer()->getMode() == "Road") {
                        modeCount["Car"] += p.flow;
                        current_length = &modeLength["Car"];
                    } else if (dynamic_cast<const ARlink*>(lk) &&
                               lk->getTail()->getParentLayer()->getMode() == "Bike") {
                        modeCount["Bike"] += p.flow;
                        current_length = &modeLength["Bike"];
                    }
                    if (current_length) {
                        *current_length += lk->getLength() * p.flow;
                    }
                }
            }
        }
    }
    for (const auto& [mode, count] : modeCount) {
        std::cout << "Mode: " << mode
                  << ", Count: " << count
                  << ", Length: " << modeLength[mode] << '\n';
    }
    double syscost = 0.0;
    for (const auto& link : net_->getLinks()) {
        syscost += link->getLength() * link->getVolume();
    }
    std::cout << "System Cost: " << syscost << '\n';
    std::vector<double> bundle_adoption_rate(user_config::heter_types.size(), 0.0);
    for (const auto& [od, demands] : dem__.table()) {
        for (size_t i = 0; i < demands.size(); ++i) {
            if (demands[i] > 1e-6) {
                bundle_adoption_rate[i] += demands[i];
            }
        }
    }
    std::cout << "Bundle Adoption Rate: ";
    double total_demand = accumulate(bundle_adoption_rate.begin(), bundle_adoption_rate.end(), 0.0);
    for (const auto& rate : bundle_adoption_rate) {
        std::cout << rate << ' ';
        if (total_demand > 0) {
            std::cout << "(" << rate / total_demand * 100 << "%) ";
        }
    }
    std::cout << '\n';
    // 打印出行方式选择占比
    std::cerr << "Mode Choice Distribution:\n";
    double total_count = std::accumulate(modeCount.begin(), modeCount.end(), 0.0,
        [](double sum, const std::pair<std::string, double>& p) { return sum + p.second; });
    std::cerr << "Car: " << modeCount["Car"] << " (" 
              << (modeCount["Car"] / total_count) * 100 << "%)\n";
    std::cerr << "Walk: " << modeCount["Walk"] << " ("
              << (modeCount["Walk"] / total_count) * 100 << "%)\n";
    std::cerr << "PT&BI&RH: " << modeCount["Bike"] + modeCount["Ride-Hailing"] + modeCount["Bus"] << " ("
              << ((modeCount["Bike"] + modeCount["Ride-Hailing"] + modeCount["Bus"]) / total_count) * 100 << "%)\n";
    std::ofstream File("D:\\MyCode\\C++\\multi_net\\result\\sensitivity.txt", std::ios::app);
    File << (modeCount["Car"] / total_count) * 100 << " "
         << (modeCount["Bus"] / total_count) * 100 << " "
         << modeCount["Ride-Hailing"] / total_count * 100 << " "
         << bundle_adoption_rate[0] / total_demand * 100 << " ";
    std::vector<std::unordered_map<std::string, double>> bundleModeCount(user_config::heter_types.size());
    for (const auto& [key, vec] : pathPool_) {
        int b = key.b; // bundle编号
        for (const auto& p : vec) {
            if (p.flow > 1e-6) {
                for (const Link* lk : p.links) {
                    if (dynamic_cast<const ATlink*>(lk)) {
                        bundleModeCount[b]["Bus"] += p.flow;
                    } else if (dynamic_cast<const ARHlink*>(lk)) {
                        bundleModeCount[b]["Ride-Hailing"] += p.flow;
                    } else if (dynamic_cast<const Wlink*>(lk) &&
                            lk->getTail()->getParentLayer()->getMode() == "Walk") {
                        bundleModeCount[b]["Walk"] += p.flow;
                    } else if (dynamic_cast<const ARlink*>(lk) &&
                            lk->getTail()->getParentLayer()->getMode() == "Road") {
                        bundleModeCount[b]["Car"] += p.flow;
                    } else if (dynamic_cast<const ARlink*>(lk) &&
                            lk->getTail()->getParentLayer()->getMode() == "Bike") {
                        bundleModeCount[b]["Bike"] += p.flow;
                    }
                }
            }
        }
    }

    // 2. 输出每个 bundle 下的比例
    for (size_t b = 0; b < bundleModeCount.size(); ++b) {
        double total = 0.0;
        for (const auto& kv : bundleModeCount[b]) total += kv.second;
        std::cout << "Bundle " << b << " (" << user_config::heter_types[b] << ") Mode Choice:\n";
        for (const auto& mode : {"Car", "Bus", "Bike", "Walk", "Ride-Hailing"}) {
            double cnt = bundleModeCount[b][mode];
            double ratio = (total > 0) ? (cnt / total * 100.0) : 0.0;
            std::cout << "  " << mode << ": " << cnt << " (" << ratio << "%)\n";
        }
        std::cout << "  PT&BI&RH: " << bundleModeCount[b]["Bus"] + bundleModeCount[b]["Bike"] + bundleModeCount[b]["Ride-Hailing"]
                << " (" << ((bundleModeCount[b]["Bus"] + bundleModeCount[b]["Bike"] + bundleModeCount[b]["Ride-Hailing"]) / (total > 0 ? total : 1) * 100.0) << "%)\n";
    }
    for (size_t b = 0; b < user_config::heter_types.size(); ++b) {
        double total_flow = 0.0;
        double weighted_transfer = 0.0;
        for (const auto& [key, vec] : pathPool_) {
            if (key.b != b) continue;
            for (const auto& p : vec) {
                if (p.flow > 1e-6) {
                    // 统计换乘次数：层变化次数
                    int transfer_count = 0;
                    Layer* last_layer = nullptr;
                    for (const Link* lk : p.links) {
                        Layer* cur_layer = lk->getTail()->getParentLayer();
                        if (last_layer && cur_layer != last_layer) {
                            ++transfer_count;
                        }
                        last_layer = cur_layer;
                    }
                    weighted_transfer += transfer_count * p.flow;
                    total_flow += p.flow;
                }
            }
        }
        double avg_transfer = (total_flow > 0) ? (weighted_transfer / total_flow) : 0.0;
        std::cout << "Bundle " << b << " (" << user_config::heter_types[b]
                << ") 平均换乘次数: " << avg_transfer << std::endl;
    }
    // 统计整体平均换乘次数（流量加权）
    double total_flow_all = 0.0;
    double weighted_transfer_all = 0.0;
    for (const auto& [key, vec] : pathPool_) {
        for (const auto& p : vec) {
            if (p.flow > 1e-6) {
                int transfer_count = 0;
                Layer* last_layer = nullptr;
                for (const Link* lk : p.links) {
                    Layer* cur_layer = lk->getTail()->getParentLayer();
                    if (last_layer && cur_layer != last_layer) {
                        ++transfer_count;
                    }
                    last_layer = cur_layer;
                }
                weighted_transfer_all += transfer_count * p.flow;
                total_flow_all += p.flow;
            }
        }
    }
    double avg_transfer_all = (total_flow_all > 0) ? (weighted_transfer_all / total_flow_all) : 0.0;
    File << avg_transfer_all << " ";
    std::cout << "整体平均换乘次数: " << avg_transfer_all << std::endl;
    total_flow_all = 0.0;
    double weighted_time_all = 0.0;
    double weighted_cost_all = 0.0;
    for (size_t b = 0; b < user_config::heter_types.size(); ++b) {
        net_->setCurrentUser(user_config::heter_types[b]);
        double total_flow = 0.0;
        double weighted_time = 0.0;
        double weighted_cost = 0.0;
        for (const auto& [key, vec] : pathPool_) {
            if (key.b != b) continue;
            for (const auto& p : vec) {
                if (p.flow > 1e-6) {
                    weighted_time += (computePathCost(p.links, 0) - computePathCost(p.links, 1)) * p.flow;
                    weighted_cost += computePathCost(p.links, 1) * p.flow;
                    total_flow += p.flow;
                    weighted_time_all += (computePathCost(p.links, 0) - computePathCost(p.links, 1)) * p.flow;
                    weighted_cost_all += computePathCost(p.links, 1) * p.flow;
                    total_flow_all += p.flow;
                }
            }
        }
        double avg_time = (total_flow > 0) ? (weighted_time / total_flow) : 0.0;
        double avg_cost = (total_flow > 0) ? (weighted_cost / total_flow) : 0.0;
        std::cout << "Bundle " << b << " (" << user_config::heter_types[b]
                << ") 平均出行时间: " << avg_time
                << "，平均出行费用: " << avg_cost << std::endl;
    }
    double avg_time_all = (total_flow_all > 0) ? (weighted_time_all / total_flow_all) : 0.0;
    double avg_cost_all = (total_flow_all > 0) ? (weighted_cost_all / total_flow_all) : 0.0;
    std::cout << "整体平均出行时间: " << avg_time_all
            << "，整体平均出行费用: " << avg_cost_all << std::endl;
    File << avg_time_all << " " << avg_cost_all << " ";
    File.close();
    // 保存路网流量到文件
    std::ofstream flowFile("D:\\MyCode\\C++\\multi_net\\result\\links_flows.txt", std::ios::app);
    for (const auto& link : net_->getLinks()) {
        flowFile << link->getId() << " " << link->getVolume() << " ";
    }
    flowFile << "\n";
    flowFile.close();
    std::ofstream pathFile("D:\\MyCode\\C++\\multi_net\\result\\paths_flows.txt", std::ios::app);
    for (const auto& [key, vec] : pathPool_) {
        // 使用路径作为匹配的id
        for (const auto& p : vec) {
            if (p.flow > 1e-6) {
                for (const Link* lk : p.links) {
                    pathFile << lk->getId() << "-";
                }
                pathFile << key.b << " ";  // 输出套餐标识符
                pathFile << ":" << p.flow << " ";  // 输出路径流量
                pathFile << " ";
            }
        }
        pathFile << "; ";
    }
    pathFile << "\n";
    pathFile.close();
}
void AssignmentSolver::getAverageTransferTime() // 当前路径池中的平均换乘次数
{

}

void AssignmentSolver::printMuCosts()
{
    std::ofstream File("D:\\MyCode\\C++\\multi_net\\result\\mu.txt");
    if (!File) {
        std::cerr << "无法创建文件 mu.txt，请检查路径和权限！" << std::endl;
        return;
    }
    for (const auto& od : dem__.table()) {
        File << od.first.first->getId() << ' ' << od.first.second->getId();
        for (double q : od.second) File << ' ' << q;
        File << '\n';
        for (int b = 0; b < user_config::heter_types.size(); ++b) {
            net_->setCurrentUser(user_config::heter_user ? user_config::heter_types[b] : user_config::current_user);
            double cost = net_->shortestPath(od.first.first, od.first.second, "Dijkstra", "Layer", 2).cost;
            File << ' ' << cost;
        }
        File << '\n';
    }
}

double AssignmentSolver::calculateUniquenessRatio() const
{
    // 1. 建立路径到矩阵索引的映射
    std::unordered_map<int, int> pathId_to_matrixIdx;
    int matrix_dim = 0;
    for (const auto& [key, path_vec] : pathPool_) {
        for (const auto& path : path_vec) {
            if (path.flow > 1e-6) { // 只考虑有流量的路径
                pathId_to_matrixIdx[path.pid] = matrix_dim++;
            }
        }
    }
    std::cout << "[DEBUG] Active paths (matrix_dim) = " << matrix_dim << std::endl;

    if (matrix_dim == 0) {
        std::cerr << "Warning: No active paths found to calculate uniqueness ratio." << std::endl;
        return -1.0;
    }

    // 初始化雅可比矩阵
    Eigen::MatrixXd jacobian_link = Eigen::MatrixXd::Zero(matrix_dim, matrix_dim);
    Eigen::MatrixXd jacobian_bundle = Eigen::MatrixXd::Zero(matrix_dim, matrix_dim);
    size_t varLinkCnt = 0, fixedLinkCnt = 0;
    double minPosDeriv = std::numeric_limits<double>::max();
    for (auto* l : net_->getLinks()) {
        double d = l->getLengthDerivative();
        if (d > 1e-10) { ++varLinkCnt; minPosDeriv = std::min(minPosDeriv, d); }
        else           { ++fixedLinkCnt; }
    }
    std::cout << "[DEBUG] Variable links = " << varLinkCnt
            << ", Fixed links = " << fixedLinkCnt
            << ", Min positive derivative = " << minPosDeriv << '\n';
    // 2. 构建拥堵成本雅可比矩阵 (S_link)
    // S_link(i, j) = ∂(cost_path_i) / ∂(flow_path_j)
    static size_t nzLinkJac = 0;   // 仅第一次声明
    for (const auto& [key_i, path_vec_i] : pathPool_) {
        for (const auto& path_i : path_vec_i) {
            if (path_i.flow < 1e-6) continue;
            int idx_i = pathId_to_matrixIdx.at(path_i.pid);

            // 遍历所有链路，计算成本对其他路径流量的偏导
            for (Link* link_k : path_i.links) {
                // 只考虑可变成本的链路
                auto r_link = dynamic_cast<Rlink*>(link_k);
                auto at_link = dynamic_cast<ATlink*>(link_k);

                if (r_link || at_link) {
                    // 获取成本导数 d(cost_link_k) / d(vol_link_k)
                    double cost_derivative = r_link ? r_link->getLengthDerivative() : at_link->getLengthDerivative();
                    // cost_derivative *= (user_config::heter_user ? user_config::heter_VOT.at(user_config::current_user) : user_config::VOT);

                    // 遍历所有路径 j，检查其是否经过 link_k
                    for (const auto& [key_j, path_vec_j] : pathPool_) {
                        for (const auto& path_j : path_vec_j) {
                            if (path_j.flow < 1e-6) continue;
                            int idx_j = pathId_to_matrixIdx.at(path_j.pid);

                            for (Link* link_m : path_j.links) {
                                if (link_m == link_k) {
                                    jacobian_link(idx_i, idx_j) += cost_derivative;
                                    ++nzLinkJac;                   // 放在 jacobian_link(idx_i,idx_j)+= 的那行下
                                    break; // 一条路径最多经过一个链路一次
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    std::cout << "[DEBUG] Non-zero entries in S_link ≈ " << nzLinkJac << '\n';

    // 3. 构建套餐选择雅可比矩阵 (S_bundle)
    // F_k(f) = E(μ_upper) + E(μ_lower) = (e_k + τ/μ_upper) + (e_k + τ/μ_lower)
    // ∂F_i/∂f_j = ∂e_i/∂f_j (from S_link) + τ_i * [∂(1/μ_upper)/∂f_j + ∂(1/μ_lower)/∂f_j]
    for (const auto& [key_i, path_vec_i] : pathPool_) {
        Node* o = key_i.o;
        Node* d = key_i.d;
        int b_i = key_i.b;

        // 计算该OD对的总需求和各bundle的需求
        double q_w = dem__.getTotalDemand({o, d});
        if (q_w < 1e-6) continue;

        double cumulative_demand = 0;
        const auto& all_demands = dem__.getDemand(o, d);

        for (int b = 0; b < b_i; ++b) {
            cumulative_demand += all_demands[b];
        }

        double p_lower = cumulative_demand / q_w;
        double p_upper = (cumulative_demand + all_demands[b_i]) / q_w;

        double mu_lower = user_config::inverseCDF(p_lower);
        double mu_upper = user_config::inverseCDF(p_upper);

        // 如果mu值为0或无穷，跳过避免计算错误
        if (mu_lower <= 0 || std::isinf(mu_lower) || mu_upper <= 0 || std::isinf(mu_upper)) continue;

        double h_lower = user_config::getPDF(mu_lower);
        double h_upper = user_config::getPDF(mu_upper);
        
        double tau_i = bundle_price[b_i];

        for (const auto& path_i : path_vec_i) {
            if (path_i.flow < 1e-6) continue;
            int idx_i = pathId_to_matrixIdx.at(path_i.pid);
            
            // 计算 ∂(1/μ)/∂f_j
            for (const auto& [key_j, path_vec_j] : pathPool_) {
                 for (const auto& path_j : path_vec_j) {
                    if (path_j.flow < 1e-6) continue;
                    int idx_j = pathId_to_matrixIdx.at(path_j.pid);
                    
                    double d_inv_mu_lower_df_j = 0;
                    if (h_lower > 1e-9 && key_j.o == o && key_j.d == d) {
                        int b_j = key_j.b;
                        if (b_j < b_i) {
                            d_inv_mu_lower_df_j = (1.0 / (q_w * h_lower)) * (1.0 / (mu_lower * mu_lower));
                        }
                    }

                    double d_inv_mu_upper_df_j = 0;
                    if (h_upper > 1e-9 && key_j.o == o && key_j.d == d) {
                        int b_j = key_j.b;
                        if (b_j <= b_i) {
                           d_inv_mu_upper_df_j = (1.0 / (q_w * h_upper)) * (1.0 / (mu_upper * mu_upper));
                        }
                    }

                    jacobian_bundle(idx_i, idx_j) += tau_i * (d_inv_mu_lower_df_j + d_inv_mu_upper_df_j);
                 }
            }
        }
    }

    // ==== DEBUG 4：S_bundle 非零元素计数（构造完 jacobian_bundle 后） ====
    size_t nzBund = (jacobian_bundle.array().abs() > 1e-12).count();
    std::cout << "[DEBUG] Non-zero entries in S_bundle = " << nzBund << '\n';
    // 4. 计算对称部分、特征值和谱范数
    Eigen::MatrixXd s_link = 0.5 * (jacobian_link + jacobian_link.transpose());
    Eigen::MatrixXd s_bundle = 0.5 * (jacobian_bundle + jacobian_bundle.transpose());

    // 计算 S_link 的最小特征值
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_link(s_link);
    double min_eigenvalue_link = es_link.eigenvalues().minCoeff();

    // 计算 S_bundle 的谱范数 (最大特征值的绝对值)
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_bundle(s_bundle);
    double spectral_norm_bundle = es_bundle.eigenvalues().cwiseAbs().maxCoeff();


    // ==== DEBUG 5：特征值摘要（求 eigenvalues 后） ====
    auto evals_link = es_link.eigenvalues();
    std::cout << "[DEBUG] S_link eigen [min, median, max] = ["
            << evals_link.minCoeff() << ", "
            << evals_link(evals_link.size()/2) << ", "
            << evals_link.maxCoeff() << "]\n";

    auto evals_bund = es_bundle.eigenvalues();
    std::cout << "[DEBUG] S_bundle eigen [min, median, max] = ["
            << evals_bund.minCoeff() << ", "
            << evals_bund(evals_bund.size()/2) << ", "
            << evals_bund.maxCoeff() << "]\n";
    if (spectral_norm_bundle < 1e-9) {
        std::cerr << "Warning: Spectral norm of bundle Jacobian is near zero. Ratio is unstable." << std::endl;
        return std::numeric_limits<double>::infinity();
    }
    std::cout << "Min Eigenvalue (Link): " << min_eigenvalue_link << ", Spectral Norm (Bundle): " << spectral_norm_bundle << std::endl;
    return min_eigenvalue_link / spectral_norm_bundle;
}
