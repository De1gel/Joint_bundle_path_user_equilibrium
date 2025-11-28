#include "Network.h"
#include "Node.h"
#include "Link.h"
#include "Layer.h"

std::random_device rd;
std::mt19937 gen(rd());

/*
2.设置文件对费用toll的读取；
3.添加异质客流的处理；
4.添加按层计费的最短路径查找；
*/
// Network 类的实现

int Path::pathNum = 0;

Network::Network() {
    std::cout << "[Network] Created" << std::endl;
}

Network::~Network() {
    std::cout << "[Network] Destroyed" << std::endl;
    for (auto& link : all_links) {
        delete link;
    }
    all_links.clear();
    for (auto& node : all_nodes) {
        delete node;
    }
    all_nodes.clear();
}

ODKey Path::get_OD() const noexcept {
    if (links.empty()) return {nullptr, nullptr};
    return {links.front()->getHead(), links.back()->getTail()};
}

void Network::setHeterUser(const std::vector<std::string>& heter_types) {
    user_config::heter_user = true;
    user_config::heter_types = heter_types;
}

void Network::setHeterVOT(const std::unordered_map<std::string, double>& heter_VOT) {
    user_config::heter_VOT = heter_VOT;
}

double Network::getHeterVOT(const std::string& type) const {
    auto it = user_config::heter_VOT.find(type);
    if (it != user_config::heter_VOT.end()) {
        return it->second;
    }
    return 0.15; // 如果没有找到，返回默认值
}

void Network::setHeterTollBike(const std::unordered_map<std::string, double>& heter_toll_bike) {
    user_config::heter_toll_bike = heter_toll_bike;
}

void Network::setHeterTollRH(const std::unordered_map<std::string, double>& heter_toll_RH) {
    user_config::heter_toll_RH = heter_toll_RH;
}

void Network::setHeterTollPT(const std::unordered_map<std::string, double>& heter_toll_PT) {
    user_config::heter_toll_PT = heter_toll_PT;
}

void Network::setHeterTollDiscount(const std::vector<std::vector<double>>& heter_toll_discount) {
    for (int b = 0; b < user_config::heter_types.size(); ++b) {
        for (int m = 0; m < 3; ++m) {
            if (m == 0) {
                user_config::heter_toll_bike[user_config::heter_types[b]] = heter_toll_discount[b][m];
            } else if (m == 1) {
                user_config::heter_toll_RH[user_config::heter_types[b]] = heter_toll_discount[b][m];
            } else if (m == 2) {
                user_config::heter_toll_PT[user_config::heter_types[b]] = heter_toll_discount[b][m];
            }
        }
    }
}

void Network::setVOT(double VOT) {
    user_config::VOT = VOT;
}

void Network::setCurrentUser(const std::string& user) {
    user_config::current_user = user;
}

void Network::setTripDistributionTriangular(double a, double b, double m) {
    user_config::trip_dist_type = user_config::DistributionType::Triangular;
    user_config::dist_param_a = a;
    user_config::dist_param_b = b;
    user_config::dist_param_m = m;
}

void Network::setTripDistributionNormal(double mu, double sigma) {
    user_config::trip_dist_type = user_config::DistributionType::Normal;
    user_config::dist_mu = mu;
    user_config::dist_sigma = sigma;
}

void Network::setDistributionType(const std::string& type) {
    // static const std::set<std::string> supported = {"lognormal", "gamma", "empirical"};
    // if (supported.count(type) == 0) {
    //     throw std::invalid_argument("Unsupported distribution type: " + type);
    // }
    if (type == "Triangular"){
        user_config::trip_dist_type = user_config::DistributionType::Triangular;
    }else if (type == "Normal"){
        user_config::trip_dist_type = user_config::DistributionType::Normal;
    }else{
        throw std::invalid_argument("Unsupported distribution type: " + type);
    }
}

void Network::addLayer(const std::string& layerName, const std::string& type) {
    if (type == "TransitLine") {
        layers[layerName] = std::make_unique<TransitLine>(this, layerName);
    } else if (type == "Walk") { 
        layers[layerName] = std::make_unique<WalkLayer>(this, layerName);
    } else {
        layers[layerName] = std::make_unique<Layer>(this, layerName);
    }
}

Layer* Network::getLayer(const std::string& layerName) const{
    auto it = layers.find(layerName);
    if (it != layers.end()) {
        return it->second.get();
    }
    return nullptr;
}

std::vector<TransitLine*> Network::getTransitLines() const {
    std::vector<TransitLine*> transitLines;
    for (const auto& layer : layers) {
        TransitLine* transitLine = dynamic_cast<TransitLine*>(layer.second.get());
        if (transitLine != nullptr) {
            transitLines.push_back(transitLine);
        }
    }
    return transitLines;
}

int Network::getLayerNum() const {
    return layers.size();
}

void Network::addGlobalLink(Link* link) {
    all_links.push_back(link);
}

void Network::addGlobalNode(Node* node) {
    all_nodes.push_back(node);
}

std::vector<Node*> Network::getNodes() const {
    return all_nodes;
}

std::vector<Link*> Network::getLinks() const {
    return all_links;
}

// 读取数据文件创建网络
void Network::createNetwork(const std::string& directoryPath) {
    cerr << "Creating network..." << endl;
    // 注册各种 Link 类型
    LinkGenerator::getInstance().registerLinkType("Rlink", new RlinkCreator());
    LinkGenerator::getInstance().registerLinkType("Tlink", new TlinkCreator());
    LinkGenerator::getInstance().registerLinkType("Wlink", new WlinkCreator());
    LinkGenerator::getInstance().registerLinkType("ATlink", new ATlinkCreator());
    LinkGenerator::getInstance().registerLinkType("ARlink", new ARlinkCreator());
    LinkGenerator::getInstance().registerLinkType("ARHlink", new ARHlinkCreator());
    LinkGenerator::getInstance().registerLinkType("Elink", new ElinkCreator());
    std::vector<std::string> expectedFiles = {
        "route.txt", "shape.txt", "transit.txt", "road.tntp", "walk.txt", "trip.txt", "flow.tntp"
    };

    for (const auto& filename : expectedFiles) {
        std::string filepath = directoryPath + "/" + filename;
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "Cannot open file: " << filepath << std::endl;
            continue;
        }
        if (filename == "transit.txt") {
            readTransitFile(file);
        } else if (filename == "route.txt") {
            readRouteFile(file);
        } else if (filename == "shape.txt") {
            readShapeFile(file);
        } else if (filename == "road.tntp") {
            readRoadFile(file);
        // } else if (filename == "trip.txt") {
        //     readTripFile(file);
        } else if (filename == "walk.txt") {
            readWalkFile(file);
        } else if (filename == "flow.tntp") {
            readFlowFile(file);
        }
        file.close();
    }
    generateTransferLinks();
    // 生成可达矩阵
    // for (const auto& layer : layers) {
    //     layer.second->getReachabilityMatrix();
    // }
}

void Network::readRouteFile(std::ifstream& file) {
    std::string line;
    // 跳过第一行
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;

        // 按逗号分割每一行
        std::getline(iss, token, ',');
        addLayer("Line" + token, "TransitLine");
    }
}

void Network::readTransitFile(std::ifstream& file) {
    std::string line;
    // 跳过第一行
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;

        // 按逗号分割每一行
        while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }

        TransitLine* layer = dynamic_cast<TransitLine*>(getLayer("Line" + tokens[0]));
        if (layer == nullptr) {
            std::cerr << "Layer not found: " << tokens[0] << std::endl;
            continue;
        }
        Node* head = layer->addNode(layer->getMode() + "-" + tokens[1]);
        Node* tail = layer->addNode(layer->getMode() + "-" + tokens[2]);
        double time_ = std::stod(tokens[3]);
        Link* link = LinkGenerator::getInstance().generateLink("Tlink", layer->getLinkNum(), head, tail, time_, {});
        layer->addLink(link);
    }
}
void Network::readShapeFile(std::ifstream& file) {
    std::string line;
    // 跳过第一行
    std::getline(file, line);
    while(std::getline(file, line)){
        if (line.empty()) {
            continue;
        }
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        while(std::getline(iss, token, ',')){
            tokens.push_back(token);
        }
        if (!tokens.empty()) {
            TransitLine* layer = dynamic_cast<TransitLine*>(getLayer("Line" + tokens[0]));
            if (layer == nullptr) {
                std::cerr << "Layer not found: " << tokens[0] << std::endl;
                continue;
            }
            layer->setFrequency(std::stod(tokens[2]));
            layer->setCapacity(std::stod(tokens[3]));
        }
    }
}
void Network::readRoadFile(std::ifstream& file) {
    std::string line;
    // 跳过前九行
    for (int i = 0; i < 9; i++) {
        std::getline(file, line);
    }
    addLayer("Road", "Road");
    addLayer("RHRoad", "Road");
    Layer* layer = getLayer("Road");
    Layer* rhLayer = getLayer("RHRoad");
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(iss, token, '\t')) {
            tokens.push_back(token);
        }
        Node* head = layer->addNode(layer->getMode() + tokens[1]);
        Node* tail = layer->addNode(layer->getMode() + tokens[2]);
        Node* rhHead = rhLayer->addNode(rhLayer->getMode() + tokens[1]);
        Node* rhTail = rhLayer->addNode(rhLayer->getMode() + tokens[2]);
        double time_ = std::stod(tokens[5]);
        Link* link = LinkGenerator::getInstance().generateLink("Rlink", layer->getLinkNum(), head, tail, time_, {tokens[3], tokens[10]});
        Link* rhLink = LinkGenerator::getInstance().generateLink("Rlink", rhLayer->getLinkNum(), rhHead, rhTail, time_, {tokens[3], tokens[10]});
        layer->addLink(link);
        rhLayer->addLink(rhLink);
        // 关联两个子网络流量
        link->setcorrelink(dynamic_cast<Rlink*>(rhLink));
        rhLink->setcorrelink(dynamic_cast<Rlink*>(link));
    }
}
void Network::readWalkFile(std::ifstream& file) {
    std::string line;
    // 跳过第一行
    std::getline(file, line);
    addLayer("Walk", "Walk");
    addLayer("Bike", "Bike");
    WalkLayer* layer = dynamic_cast<WalkLayer*>(getLayer("Walk"));
    Layer* bikeLayer = getLayer("Bike");
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }
        Node* head = layer->addNode(layer->getMode() + tokens[0]);
        Node* tail = layer->addNode(layer->getMode() + tokens[1]);
        Node* bikeHead = bikeLayer->addNode(bikeLayer->getMode() + tokens[0]);
        Node* bikeTail = bikeLayer->addNode(bikeLayer->getMode() + tokens[1]);
        double time_ = std::stod(tokens[2]);
        Link* link = LinkGenerator::getInstance().generateLink("Wlink", layer->getLinkNum(), head, tail, time_, {});
        Link* bikeLink = LinkGenerator::getInstance().generateLink("Wlink", bikeLayer->getLinkNum(), bikeHead, bikeTail, time_ / 2, {});
        layer->addLink(link);
        bikeLayer->addLink(bikeLink);
    }
    
}
// void Network::readTripFile(std::ifstream& file) {
//     std::string line;
//     // 跳过第一行
//     std::getline(file, line);
//     WalkLayer* layer = dynamic_cast<WalkLayer*>(getLayer("Walk"));
//     while (std::getline(file, line)) {
//         if (line.empty()) {
//             continue;
//         }
//         std::istringstream iss(line);
//         std::string token;
//         std::vector<std::string> tokens;
//         while (std::getline(iss, token, '\t')) {
//             tokens.push_back(token);
//         }
//         Centroid* des = layer->addCentroid('C' + layer->getMode() + tokens[1]);
//         Node* Wnode = layer->getNode(layer->getMode() + tokens[1]);
//         Link* link = LinkGenerator::getInstance().generateLink("Wlink", layer->getLinkNum(), Wnode, des, 0.0, {});
//         std::getline(file, line);
//         std::istringstream iss2(line);
//         std::string token2;
//         while (std::getline(iss2, token2, ';')) {
//             std::istringstream tokenStream(token2);
//             std::string originStr, demandStr;
//             // 再按冒号分割
//             if (std::getline(tokenStream, originStr, ':') && std::getline(tokenStream, demandStr)) {
//                 std::vector<int> demands;
//                 if (user_config::heter_user) {
//                     std::istringstream demandStream(demandStr);
//                     std::string demandToken;
//                     for (int i = 0; i < user_config::heter_types.size(); i++) {
//                         std::getline(demandStream, demandToken, ',');
//                         if (demandToken.empty()) {
//                             cerr << "number of demand is not equal to number of types" << endl;
//                         }
//                         demands.push_back(std::stoi(demandToken));
//                     }
//                     des->addOrigin(layer->getNode(layer->getMode() + originStr), demands);
//                 } else {
//                     demands.push_back(std::stoi(demandStr));
//                     des->addOrigin(layer->getNode(layer->getMode() + originStr), demands);
//                 }
//             }
//         }
//         }
// }

void Network::readFlowFile(std::ifstream& file) {
    std::string line;
    std::getline(file, line);
    Layer* layer = getLayer("Road");
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(iss, token, '\t')) {
            auto pos = token.find_last_not_of(" ");
            if (pos != std::string::npos) {
                token = token.erase(pos + 1);
            }
            tokens.push_back(token);
        }
        Link* link = layer->getLink(layer->getMode() + tokens[0], layer->getMode() + tokens[1]);
        if (link == nullptr) {
            std::cerr << "Link not found: " << tokens[0] << " -> " << tokens[1] << std::endl;
            continue;
        }
        // link->setBackgroundVolume(std::stod(tokens[2]));
        // std::uniform_real_distribution<double> dist(0.7, 0.9);
        // link->setBackgroundVolume(link->getCapacity() * dist(gen));
    }
    for (auto& link : layer->getLinks()) {
        std::uniform_real_distribution<double> dist(0.7, 0.9);
        link->setBackgroundVolume(link->getCapacity() * dist(gen));
    }
}

void Network::generateTransferLinks() {
    Layer* roadLayer = getLayer("Road");
    Layer* rhLayer = getLayer("RHRoad");
    Layer* walkLayer = getLayer("Walk");
    Layer* bikeLayer = getLayer("Bike");
    for (Node* walkNode : walkLayer->getNodes()) {
        std::string walkNodeId = walkNode->getId();
        // 将walk替换为road
        std::string roadNodeId = "Road" + walkNodeId.substr(4);
        std::string rhNodeId = "RHRoad" + walkNodeId.substr(4);
        std::string bikeNodeId = "Bike" + walkNodeId.substr(4);
        if (roadLayer->getNode(roadNodeId) != nullptr) {
            Node* roadNode = roadLayer->getNode(roadNodeId);
            Link* alink = LinkGenerator::getInstance().generateLink("ARlink", roadLayer->getLinkNum(), walkNode, roadNode, 0.0, {});
            walkLayer->addLink(alink);
            Link* elink = LinkGenerator::getInstance().generateLink("Elink", roadLayer->getLinkNum(), roadNode, walkNode, 0.0, {});
            roadLayer->addLink(elink);
        }
        if (rhLayer->getNode(rhNodeId) != nullptr) {
            Node* rhNode = rhLayer->getNode(rhNodeId);
            Link* alink = LinkGenerator::getInstance().generateLink("ARHlink", rhLayer->getLinkNum(), walkNode, rhNode, 0.0, {});
            walkLayer->addLink(alink);
            Link* elink = LinkGenerator::getInstance().generateLink("Elink", rhLayer->getLinkNum(), rhNode, walkNode, 0.0, {});
            rhLayer->addLink(elink);
        }
        if (bikeLayer->getNode(bikeNodeId) != nullptr) {
            Node* bikeNode = bikeLayer->getNode(bikeNodeId);
            Link* alink = LinkGenerator::getInstance().generateLink("ARlink", bikeLayer->getLinkNum(), walkNode, bikeNode, 0.0, {});
            walkLayer->addLink(alink);
            Link* elink = LinkGenerator::getInstance().generateLink("Elink", bikeLayer->getLinkNum(), bikeNode, walkNode, 0.0, {});
            bikeLayer->addLink(elink);
        }

        for (const auto& layer: getTransitLines()) {
            for (const auto& node: layer->getNodes()) {
                std::string transitNodeId = node->getId();
                // 分割字符串
                std::vector<std::string> tokens;
                std::istringstream iss(transitNodeId);
                std::string token;
                while (std::getline(iss, token, '-')) {
                    tokens.push_back(token);
                }
                if (tokens[1] == walkNodeId.substr(4)) {
                    Link* alink = LinkGenerator::getInstance().generateLink("ATlink", roadLayer->getLinkNum(), walkNode, node, 0.0, {layer->getMode(), std::to_string(layer->getFrequency())});
                    walkLayer->addLink(alink);
                    Link* elink = LinkGenerator::getInstance().generateLink("Elink", roadLayer->getLinkNum(), node, walkNode, 0.0, {});
                    layer->addLink(elink);
                }
            }
        }
    }
}

Path Network::shortestPath(Node* s, Node* t,
                                         const std::string& algo,
                                         const std::string& cost_mode,
                                         int K_maxTransfer) const
{
    if (algo == "Dijkstra")     return Dijkstra(s, t, cost_mode, K_maxTransfer + 1);
    // else if (algo == "AStar")   return aStarSP  (s, t, cost_mode, K_maxTransfer);
    // else if (algo == "BellmanFord") return bellmanSP(s, t, cost_mode, K_maxTransfer);
    else
        throw std::invalid_argument("Unknown algorithm: " + algo);
}

/*------------------ Dijkstra：按“出层一次性计费” ------------------*/
Path Network::Dijkstra(Node* s, Node* t,
                                       const std::string& cost_mode,
                                       int K_max) const
{
    struct Label {
        double dist;  int trans;  Node* node; 
        std::vector<Node*> seg;           // 当前仍在 *本层* 内的节点序列
        bool operator>(const Label& o) const { return dist > o.dist; }
    };
    using PQ = std::priority_queue<Label,
                 std::vector<Label>, std::greater<Label>>;
    PQ Q;
    std::unordered_map<Node*, double> best;
    std::unordered_map<Node*, Link*>  prev;

    Q.push({0, 0, s, {s}});
    best[s] = 0;

    while (!Q.empty()) {
        Label cur = Q.top(); Q.pop();
        // if (cur.seg[0] == nullptr)
        //     std::cerr << "Warning: Segment path is empty." << std::endl;
        if (cur.node == t) {
            best[t] = cur.dist;  // 最终到达目标节点
            break;
        }
        if (cur.trans > K_max) continue;

        Layer* Lcur = cur.node->getParentLayer();

        for (Link* lk : cur.node->getOutLinks()) {
            Node* v     = lk->getTail();
            Layer* Lnext= v->getParentLayer();

            /*------------- 换乘计数（Walk→非 Walk）-------------*/
            int nextTrans = cur.trans +
                ((Lcur->getMode()=="Walk" && Lnext->getMode()!="Walk") ? 1 : 0);
            if (nextTrans > K_max) continue;

            /*------------- 更新 segPath -------------*/
            bool leaving = (cost_mode=="Layer" && Lcur!=Lnext);   // ★ 只看换层
            std::vector<Node*> seg = cur.seg;

            if (Lcur == Lnext) {
                seg.push_back(v);                 // 层内继续累积
            }

            /*------------- 计算边权 -------------*/
            double w = 0.0;
            if (cost_mode == "Link") {
                w = lk->getCost();                // 逐段
            } else { /* layer 计费 */
                if(user_config::heter_user) {
                    w = lk->getLength() * user_config::heter_VOT.at(user_config::current_user);
                } else {
                    w = lk->getLength() * user_config::VOT; // 按长度计费
                }        // link 时间
                if (leaving) {
                    double fare_ = Lcur->farematrix(seg);
                    w += fare_;
                    seg.clear();  // 换层后清空
                    if (dynamic_cast<ARHlink*>(lk)) {
                        seg.push_back(lk->getHead());  // 加入空节点表示换层
                    }
                    seg.push_back(v); // 换层后重新开始
                }
            }
            if (lk->getCapacity() > 0.0) // 如果有影子价格
                w += lk->getShadowPrice(); // 叠加影子价格

            double alt = cur.dist + w;
            if (!best.count(v) || alt < best[v]) {
                best[v] = alt;
                prev[v] = lk;
                Q.push({alt, nextTrans, v, std::move(seg)});
            }
        }
    }

    /* 回溯路径 */
    Path path;
    for (Node* n=t; n!=s; ) {
        auto it=prev.find(n);
        if (it==prev.end()) return {};
        path.links.push_back(it->second);
        n = it->second->getHead();
    }
    std::reverse(path.links.begin(), path.links.end());
    path.cost = best[t];
    return path;
}

// /*------------------ A*（启发式 = 欧氏距离） ------------------*/
// double heuristic(Node* a, Node* b)
// {
//     double dx = a->x() - b->x();     // 假设有经纬 or 投影坐标
//     double dy = a->y() - b->y();
//     return std::sqrt(dx*dx + dy*dy) / 15.0; // 估算 15 m/s 上界
// }

// std::vector<Link*> Network::aStarSP(Node* s, Node* t,
//                                     const std::string& cost_mode,
//                                     int K_maxTransfer)
// {
//     struct Label {
//         double f, g; int trans; Node* node; Node* entry;
//         bool operator>(const Label& o) const { return f > o.f; }
//     };
//     std::priority_queue<Label, std::vector<Label>, std::greater<Label>> Q;
//     std::unordered_map<Node*, double> gbest;
//     std::unordered_map<Node*, Link*>  prev;
//     std::unordered_map<Node*, Node*>  entrySave;

//     Q.push({heuristic(s,t), 0, 0, s, nullptr});
//     gbest[s] = 0;

//     while (!Q.empty()) {
//         auto cur = Q.top(); Q.pop();
//         if (cur.node == t) break;
//         if (cur.trans > K_maxTransfer) continue;

//         for (Link* lk : cur.node->getOutLinks()) {
//             Node* v = lk->getTail();
//             Layer* Lcur = cur.node->getParentLayer();
//             Layer* Lnext= v->getParentLayer();

//             int nextTrans = cur.trans +
//                 ((Lcur->getMode()=="Walk" && Lnext->getMode()!="Walk") ? 1 : 0);
//             if (nextTrans > K_maxTransfer) continue;

//             Node* nextEntry = cur.entry;
//             if (cost_mode=="layer" && Lcur!=Lnext)
//                 nextEntry = v;

//             double w = edgeCost(lk, cur.entry, cost_mode);
//             double g = cur.g + w;
//             double f = g + heuristic(v, t);

//             if (!gbest.count(v) || g < gbest[v]) {
//                 gbest[v] = g;
//                 prev[v] = lk;
//                 entrySave[v] = nextEntry;
//                 Q.push({f, g, nextTrans, v, nextEntry});
//             }
//         }
//     }
//     // 回溯同上
//     std::vector<Link*> path;
//     for (Node* cur = t; cur != s; ) {
//         Link* lk = prev[cur];
//         if(!lk) break;
//         path.push_back(lk);
//         cur = lk->getHead();
//     }
//     std::reverse(path.begin(), path.end());
//     return path;
// }

// /*------------------ Bellman-Ford (演示) ------------------*/
// std::vector<Link*> Network::bellmanSP(Node* s, Node* t,
//                                       const std::string& cost_mode,
//                                       int K_maxTransfer)
// {
//     std::vector<Node*> V = getNodes();       // 获取网络全部节点
//     std::vector<Link*> E = getLinks();

//     std::unordered_map<Node*, double> dist;
//     std::unordered_map<Node*, Link*>  prev;
//     std::unordered_map<Node*, Node*>  entry;
//     dist[s] = 0;

//     for (size_t k=0; k<V.size()-1; ++k) {
//         bool updated = false;
//         for (Link* lk : E) {
//             Node* u = lk->getHead();
//             Node* v = lk->getTail();
//             if (!dist.count(u)) continue;

//             double w = edgeCost(lk, entry[u], cost_mode);
//             if (!dist.count(v) || dist[u] + w < dist[v]) {
//                 dist[v] = dist[u] + w;
//                 prev[v] = lk;
//                 entry[v]= entry[u];
//                 updated = true;
//             }
//         }
//         if (!updated) break;      // 早停
//     }
//     // 简化：省略负环检测
//     std::vector<Link*> path;
//     for (Node* cur = t; cur != s; ) {
//         Link* lk = prev[cur];
//         if(!lk) break;
//         path.push_back(lk);
//         cur = lk->getHead();
//     }
//     std::reverse(path.begin(), path.end());
//     return path;
// }

// 路径集生成
// void Network::generateTransferPlanSet(int max_transfer_num) {
//     transfer_plan_set.clear();
//     std::vector<Centroid*> dess;
//     WalkLayer* walkLayer = dynamic_cast<WalkLayer*>(getLayer("Walk"));
//     if (walkLayer == nullptr) {
//         std::cerr << "Walk layer not found!" << std::endl;
//         return;
//     }
//     dess = walkLayer->getCentroids();
//     for (const auto& des : dess) {
//         std::string des_id = des->getId();
//         //去掉"C"前缀
//         if (des_id[0] == 'C') des_id = des_id.substr(1);
//         std::vector<Node*> origins = des->getOrigins();
//         Node* node_end = walkLayer->getNode(des_id);
//         for (const auto& origin : origins) {
//             Node* node_start = walkLayer->getNode(origin->getId());
//             ODKey key = {node_start, node_end};
//             if (transfer_plan_set.find(key) == transfer_plan_set.end()) {
//                 transfer_plan_set[key] = {};
//             }
//             // 遍历所有换乘路径
//             FindTransferPath(node_start, node_end, max_transfer_num);
//         }
//     }
// }

// void Network::FindTransferPath(Node* start, Node* end, int max_transfer_num)
// {
//     //—— 写入位置 ——//
//     ODKey key{ start, end };
//     auto& planBucket = transfer_plan_set[key];   // 若 key 不存在会自动插入空 vector

//     //—— 状态定义 ——//
//     struct State {
//         Node* node;
//         std::string curMode;
//         int transfers;                 // 已换乘次数
//         std::vector<Node*> transferpath;       // 节点序列
//         std::vector<std::string> modes;// 段-模式序列
//         bool useRoad = false; // 是否使用过 Road 层
//     };

//     //—— DFS 栈 ——//
//     std::stack<State> st;
//     std::string initMode = start->getParentLayer()->getMode();
//     st.push({ start, initMode, 0, { start }, { initMode } });

//     while (!st.empty()) {
//         State cur = std::move(st.top());
//         st.pop();

//         // —— 抵达终点：生成 TransferPlan —— //
//         if (cur.node == end) {
//             TransferPlan tp;
//             tp.path          = std::move(cur.transferpath);
//             tp.transferCount = cur.transfers;
//             tp.modes         = std::move(cur.modes);
//             planBucket.emplace_back(std::move(tp));
//             continue;
//         }

//         // —— 剪枝：已超换乘上限 —— //
//         if (cur.transfers > max_transfer_num)
//             continue;

//         // —— 遍历后继 —— //
//         Layer* curLayer = cur.node->getParentLayer();
//         std::string curMode = curLayer->getMode();
//         for (Node* next : curLayer->getNodes()) {
//             // 剪枝
//             if (next == nullptr)  // 自环
//                 continue;
//             if (!curLayer->isReachable(cur.node, next))  // 不可达
//                 continue;
//             if (next != end) {
//                 for (Link* link : next->getOutLinks()) {
//                     if (link->getTail()->getParentLayer() != curLayer) {
//                         next = link->getTail(); //换层
//                         Layer* nextLayer = next->getParentLayer();
//                         std::string nextMode = nextLayer->getMode();
//                         if (nextMode == "Road" && cur.useRoad) {
//                             continue; // 已经进过一次 Road 层，剪枝
//                         }
//                         // 3) 计算是否换乘
//                         int nextTransfers = cur.transfers;
//                         if (curLayer->getMode() == "Walk" && nextMode != curMode)
//                             nextTransfers += 1;
//                         if (nextTransfers > max_transfer_num)
//                             continue;

//                         // 4) 入栈下一状态
//                         State nxt = cur;                 // 复制当前状态
//                         nxt.node       = next;
//                         nxt.curMode    = nextMode;
//                         nxt.transfers  = nextTransfers;
//                         nxt.transferpath.push_back(next);
//                         nxt.modes.push_back(nextMode);
//                         if (nextMode == "Road") {
//                             nxt.useRoad = true; // 标记使用过 Road 层
//                         }

//                         st.push(std::move(nxt));
//                     }
//                 }
//             } else {
//                 // 直接到达终点
//                 State nxt = cur;                 // 复制当前状态
//                 nxt.node       = next;
//                 nxt.transferpath.push_back(next);
//                 st.push(std::move(nxt));
//             }
//         }
//     }
// }

void Network::getCongestionData() const {
    // 统计路网数据
    std::cout << "[Network] Congestion Data:" << std::endl;
    int num_links_low = 0;
    int num_links_medium = 0;
    int num_links_high = 0;
    int num_links_overload = 0;
    int num_links_total = 0;
    double total_saturation = 0.0;
    double total_saturation_ = 0.0;
    double total_volume = 0.0;
    for (const auto& link : all_links) {
        // // 起始输出
        // std::cout << "Link ID: " << link->getId()
        //         << ", Head Node: " << link->getHead()->getId()
        //         << ", Tail Node: " << link->getTail()->getId();

        // // 根据条件输出不同的字段
        // if (user_config::heter_user) {
        //     std::cout << ", Heterogeneous Volume: " << std::endl;
        //     for (const auto& type : user_config::heter_types) {
        //         std::cout << "  " << type << ": "
        //                   << link->getHeterVolume(type);
        //     }
        // } else {
        //     std::cout << ", Volume: " << link->getVolume();
        // }

        // // 其他通用输出
        // std::cout << ", Capacity: " << link->getCapacity()
        //         << std::endl;
        if (link->getCapacity() == 0 || dynamic_cast<Rlink*>(link)) {
            continue;
        }
        //计算平均饱和率
        total_saturation += link->getVolume() * link->getVolume() / (link->getCapacity() * link->getTail()->getParentLayer()->getFrequency());
        // if (link->getVolume() > link->getCapacity()) {
        //     std::cerr << "Warning: Link " << link->getId() 
        //               << " is overloaded! Volume: " << link->getVolume() 
        //               << ", Capacity: " << link->getCapacity() << std::endl;
        // }
        total_saturation_ += link->getVolume() / (link->getCapacity() * link->getTail()->getParentLayer()->getFrequency());
        total_volume += link->getVolume();
        num_links_total++;
        // if (link->getVolume() < 0.5 * link->getCapacity()) {
        //     num_links_low++;
        // } else if (link->getVolume() < 0.8 * link->getCapacity()) {
        //     num_links_medium++;
        // } else if (link->getVolume() < 1.1 * link->getCapacity()) {
        //     num_links_high++;
        // } else {
        //     num_links_overload++;
        // }
    }
    // // 输出不同饱和率线路占比
    // std::cerr << "Link Saturation Ratios:";
    // if (num_links_total > 0) {
    //     std::cerr << "  Low: " << (double)num_links_low / num_links_total * 100 << "%";
    //     std::cerr << "  Medium: " << (double)num_links_medium / num_links_total * 100 << "%";
    //     std::cerr << "  High: " << (double)num_links_high / num_links_total * 100 << "%";
    //     std::cerr << "  Overloaded: " << (double)num_links_overload / num_links_total * 100 << "%" << std::endl;
    // }
    // 输出平均饱和率
    if (num_links_total > 0) {
        std::cerr << "Effective Average Link Saturation: " << (total_saturation / total_volume) * 100 << "%" << std::endl;
        std::cerr << "Average Link Saturation: " << (total_saturation_ / num_links_total) * 100 << "%" << std::endl;
    }
}

void Network::modifyCapacity(double ratio, std::vector<std::string>& modified_lines) const {
    // 对线路按照最大饱和率进行排序
    std::vector<Link*> sorted_links = all_links;
    std::sort(sorted_links.begin(), sorted_links.end(),
        [](Link* a, Link* b) {
            if (a->getShadowPrice() != b->getShadowPrice())
                return a->getShadowPrice() > b->getShadowPrice(); // shadowPrice优先，降序
            else
                return a->getVacancy() < b->getVacancy(); // vacancy相同时，按vacancy升序
        });
    // 修改前 num_lines 条线路的容量
    for (int i = 0; i < sorted_links.size(); ++i) {
        Link* link = sorted_links[i];
        if (!dynamic_cast<Tlink*>(link)) {
            continue;
        }
        std::string mode = link->getTail()->getParentLayer()->getMode();
        if (std::find(modified_lines.begin(), modified_lines.end(), mode) != modified_lines.end()) {
            continue;
        }
        Layer* layer = getLayer(mode);
        // for (const auto& link_ : layer->getLinks()) {
        //     double old_capacity = link_->getCapacity();
        //     double new_capacity = old_capacity * ratio;
        //     link_->setCapacity(new_capacity);
        // }
        if (auto tl = dynamic_cast<TransitLine*>(layer)) {
            tl->setFrequency(tl->getFrequency() * ratio);
        }
        for (const auto& node_ : layer->getNodes()) {
            for (const auto& link_ : node_->getInLinks()) {
                if (auto atlink = dynamic_cast<ATlink*>(link_)) {
                    atlink->frequency *= ratio;
                }
            }
        }
        // 输出修改信息
        modified_lines.push_back(mode);
        break; // 只修改一条线路
    }
}
