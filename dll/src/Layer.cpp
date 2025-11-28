#include "Layer.h"
#include "Node.h"
#include "Link.h"
#include "Network.h"

// Layer 类的实现
Layer::Layer(Network* parent, const std::string& modeName)
    : mode(modeName), parentNetwork(parent) {
        Layer::layerNum++;
}

Node* Layer::addNode(const std::string& nodeId) {
    if (getNode(nodeId) != nullptr) {
        return getNode(nodeId);
    }
    else {
        Node* newNode = new Node(nodeId, this);
        nodes.push_back(newNode);
        if (parentNetwork) {
            parentNetwork->addGlobalNode(newNode);
        }
        Layer::nodeNum++;
        return newNode;
    }
}


Node* Layer::getNode(const std::string& nodeId) {
    for (Node* node : nodes) {
        if (node->getId() == nodeId) {
            return node;
        }
    }
    return nullptr;
}

double Layer::getCost(Node* from, Node* to) const {
    return 0.0;
}

void Layer::updateFleetStats() {
    double busyTime = 0.0;
    D_ = 0.0;
    double D__ = 0.0;

    for (Link* lk : parentNetwork->getLinks()) {           // links 来自基类 Layer:contentReference[oaicite:1]{index=1}
        if (dynamic_cast<ARHlink*>(lk))           // 登车链路 ⇒ 统计需求
            D_ += lk->getVolume();               // x_a + \tilde{x}_a

        else if (dynamic_cast<ARlink*>(lk))
            D__ += lk->getVolume();
    }
    double ratio = D_ / std::max(D__, 1e-6); // 车队需求占比
    for (Link* lk : parentNetwork->getLinks()) { // links 来自基类 Layer:contentReference[oaicite:1]{index=1}
        if (auto rl = dynamic_cast<Rlink*>(lk)) {
            double occ = rl->getVolume();   // 载客流
            busyTime += rl->getLength() * occ * ratio / 60;             // t_a × 载客流
        }
    }
    V_ = std::max(1e-6, Fleetsize_ - busyTime);                  // 避免 V≤0
}

void Layer::addLink(Link* link) {
    links.push_back(link);
    if (parentNetwork) {
        parentNetwork->addGlobalLink(link);
    }
    Node* u = link->getHead();
    Node* v = link->getTail();
    adj[u][v] = link;
    Layer::linkNum++;
}

Link* Layer::Linkhash(Node* u, Node* v) const {
    auto it = adj.find(u);
    if (it != adj.end()) {
        auto jt = it->second.find(v);
        if (jt != it->second.end()) {
            return jt->second;
        }
    }
    return nullptr;
}

Link* Layer::getLink(const std::string& from, const std::string& to) {
    for (Link* link : links) {
        if (link->getHead()->getId() == from && link->getTail()->getId() == to) {
            return link;
        }
    }
    return nullptr;
}

int Layer::layerNum = 0;
int Layer::nodeNum  = 0;
int Layer::linkNum  = 0;

int Layer::getLayerNum() const {
    return layerNum;
}

int Layer::getNodeNum() const {
    return nodeNum;
}

int Layer::getLinkNum() const {
    return linkNum;
}

std::string Layer::getMode() const {
    return mode;
}

const std::vector<Node*>& Layer::getNodes() const {
    return nodes;
}

TransitLine::TransitLine(Network* parentNetwork, const std::string& modeName)
    : Layer(parentNetwork, modeName) {}

double TransitLine::getFrequency() const {
    return frequency;
}

void TransitLine::setFrequency(double frequency) {
    this->frequency = frequency;
}

double TransitLine::getCapacity() const {
    return capacity;
}

void TransitLine::setCapacity(double capacity) {
    this->capacity = capacity;
    for (Link* link : links) {
        link->setCapacity(capacity);
    }
}

double TransitLine::getCost(Node* from, Node* to) const {
    return 0.0;
}

WalkLayer::WalkLayer(Network* parent, const std::string& modeName)
    : Layer(parent, modeName) {}

std::vector<Centroid*> WalkLayer::getCentroids() const {
    return centroids;
}

Centroid* WalkLayer::addCentroid(const std::string& nodeId) {
    Node* existingNode = getNode(nodeId);
    if (existingNode != nullptr) {
        return dynamic_cast<Centroid*>(existingNode);
    } else {
        Centroid* newCentroid = new Centroid(nodeId, this);
        nodes.push_back(newCentroid);
        centroids.push_back(newCentroid);
        ++Layer::nodeNum;
        return newCentroid;
    }
}

void Layer::getReachabilityMatrix() {
    int n = nodes.size();
    reachabilityMatrix.resize(n, std::vector<bool>(n, false));

    // Map node pointer to index
    for (int i = 0; i < n; ++i) {
        nodeIndex[nodes[i]] = i;
    }

    // BFS for each node
    for (int i = 0; i < n; ++i) {
        Node* start = nodes[i];
        std::queue<Node*> q;
        std::unordered_set<Node*> visited;

        q.push(start);
        visited.insert(start);
        reachabilityMatrix[i][i] = true;

        while (!q.empty()) {
            Node* current = q.front(); q.pop();
            int from = nodeIndex[current];

            for (Link* link : current->getOutLinks()) {
                Node* neighbor = link->getTail();
                if (nodeIndex.find(neighbor) == nodeIndex.end()) continue; // safety

                if (!visited.count(neighbor)) {
                    visited.insert(neighbor);
                    q.push(neighbor);
                    int to = nodeIndex[neighbor];
                    reachabilityMatrix[i][to] = true;
                }
            }
        }
    }
}

bool Layer::isReachable(Node* from, Node* to) const
{
    // ---------- 1) 空指针保护 ----------
    if (from == nullptr || to == nullptr) {
        return false;
    }

    // ---------- 2) “节点是否属于本层” 快速判定 ----------
    // 若 Node 里有 parentLayer 指针，用它来检查最省事
    if (from->getParentLayer() != this || to->getParentLayer() != this) {
        return false;
    }

    // ---------- 3) 正式查索引并返回 reachability ----------
    auto itFrom = nodeIndex.find(from);
    auto itTo   = nodeIndex.find(to);
    if (itFrom == nodeIndex.end() || itTo == nodeIndex.end()) {
        return false;                       // 仍做一次兜底检查
    }
    return reachabilityMatrix[itFrom->second][itTo->second];
}

double Layer::farematrix(std::vector<Node*> path) const {
    // 默认实现：根据距离计费
    if (path.empty()) {
        return 0.0; // 如果路径为空，返回0费用
    }
    if (mode == "Walk") {
        return 0.0; // 步行不收费
    }
    bool flag = false; // 标记是否是非 Road 层
    if (mode == "RHRoad") {
        path.erase(path.begin());
        flag = true; // 标记为RH层
    }
    double totalFare = 0.0;
    double total_toll = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        Node* from = path[i];
        Node* to = path[i + 1];
        Link* link = Linkhash(from, to);
        if (link == nullptr) {
            continue; // 如果没有链接，跳过
        }
        if (from == nullptr || to == nullptr) {
            continue; // 如果节点为空，跳过
        }
        if (from->getParentLayer() != this || to->getParentLayer() != this) {
            continue; // 如果节点不属于当前层，跳过
        }
        // 根据距离计算费用
        totalFare += link->getLength();
        total_toll += link->getToll();
    }
    if (mode == "Bike") {
        return 2.0 * totalFare / 15 * user_config::heter_toll_bike.at(user_config::current_user); // 自行车按次收费
    }
    if (!flag) // 选择私家车
        return total_toll + path.back()->getParkingTime() * user_config::VOT; // 停车时间按 VOT 计费
    totalFare = (totalFare >= 10) ? 5 + (totalFare - 10) * 0.2 : 5;
    // 考虑bundle的情况下，价格不同
    if (user_config::heter_user) {
        totalFare *= user_config::heter_toll_RH.at(user_config::current_user);
    }
    return totalFare;
}

double TransitLine::farematrix(std::vector<Node*> path) const {
    // 公交线路的 fare 计算根据站点数量
    double totalFare = 0.0;
    if (path.size() < 4) {
        totalFare = 2; // 假设小于8个站点的费用为2
    } else if (path.size() < 7) {
        totalFare = 3;
    } else if (path.size() < 10) {
        totalFare = 4;
    } else {
        totalFare = 6;
    }
    // 考虑bundle的情况下，价格不同
    if (user_config::heter_user) {
        totalFare *= user_config::heter_toll_PT.at(user_config::current_user);
    }
    return totalFare;
}
