#include "Node.h"
#include "Link.h"

bool user_config::heter_user = false;
std::string user_config::current_user = "";
double user_config::VOT = 0.1;
std::vector<std::string> user_config::heter_types = {};
std::unordered_map<std::string, double> user_config::heter_VOT = {};
std::unordered_map<std::string, double> user_config::heter_toll_bike = {};
std::unordered_map<std::string, double> user_config::heter_toll_RH = {};
std::unordered_map<std::string, double> user_config::heter_toll_PT = {};
user_config::DistributionType user_config::trip_dist_type = user_config::DistributionType::Triangular;
double user_config::dist_param_a = 5.0;  // 三角分布下限
double user_config::dist_param_b = 80.0; // 三角分布上
double user_config::dist_param_m = 45.6; // 三角分布众数
double user_config::dist_mu = 40.0;      // 正态分布均
double user_config::dist_sigma = 10.0;   // 正态分布标准差
double user_config::getPDF(double x) {
    if (trip_dist_type == DistributionType::Triangular)
        return triangularPDF(x, dist_param_a, dist_param_b, dist_param_m);
    else
        return normalPDF(x, dist_mu, dist_sigma);
}

double user_config::getCDF(double x) {
    if (trip_dist_type == DistributionType::Triangular)
        return triangularCDF(x, dist_param_a, dist_param_b, dist_param_m);
    else
        return normalCDF(x, dist_mu, dist_sigma);
}

double user_config::integrate(double x1, double x2) {
    const int n = 1000;
    double h = (x2 - x1) / n;
    double sum = 0.5 * (getPDF(x1) + getPDF(x2));
    for (int i = 1; i < n; ++i)
        sum += getPDF(x1 + i * h);
    return sum * h;
}

double user_config::meanOverInterval(double x1, double x2) {
    if (x1 == x2)
        return x1;
    const int n = 1000;
    double h = (x2 - x1) / n;
    double sum_fx = 0.0;
    double sum_f = 0.0;
    for (int i = 0; i <= n; ++i) {
        double x = x1 + i * h;
        double fx = getPDF(x);
        sum_fx += x * fx;
        sum_f += fx;
    }
    return sum_f > 0 ? sum_fx / sum_f : 0.0;
}

double user_config::inverseCDF(double p) {
    if (trip_dist_type == DistributionType::Triangular)
        return inverseTriangularCDF(p, dist_param_a, dist_param_b, dist_param_m);
    else
        return inverseNormalCDF(p, dist_mu, dist_sigma);
}

double user_config::triangularPDF(double z, double a, double b, double m) {
    if (z < a || z > b) return 0.0;
    return (z < m)
        ? 2 * (z - a) / ((b - a) * (m - a))
        : 2 * (b - z) / ((b - a) * (b - m));
}

double user_config::triangularCDF(double z, double a, double b, double m) {
    if (z <= a) return 0.0;
    if (z >= b) return 1.0;
    return (z < m)
        ? sqr(z - a) / ((b - a) * (m - a))
        : 1 - sqr(b - z) / ((b - a) * (b - m));
}

double user_config::normalPDF(double x, double mu, double sigma) {
    return (1.0 / (sigma * sqrt(2 * M_PI))) * exp(-0.5 * sqr((x - mu) / sigma));
}

double user_config::normalCDF(double x, double mu, double sigma) {
    return 0.5 * erfc(-(x - mu) / (sigma * sqrt(2)));
}

double user_config::inverseTriangularCDF(double p, double a, double b, double m) {
    if (p < 0 || p > 1) throw std::out_of_range("Probability must be in [0, 1]");
    if (p == 0) return a;
    if (p == 1) return b;

    double range = b - a;
    if (p < (m - a) / range) {
        return a + sqrt(p * range * (m - a));
    } else {
        return b - sqrt((1 - p) * range * (b - m));
    }
}

double user_config::erfInv(double x) {
    // Abramowitz and Stegun formula 7.1.26 approximation
    double a = 0.147; // magic constant
    double ln = log(1.0 - x * x);
    double sign = x < 0 ? -1.0 : 1.0;
    return sign * sqrt(
        sqrt( (2.0 / (M_PI * a) + ln / 2.0) * (2.0 / (M_PI * a) + ln / 2.0) - ln / a ) -
        (2.0 / (M_PI * a) + ln / 2.0)
    );
}

double user_config::inverseNormalCDF(double p, double mu, double sigma) {
    if (p <= 0.0) return -std::numeric_limits<double>::infinity();
    if (p >= 1.0) return std::numeric_limits<double>::infinity();
    return mu + sigma * sqrt(2.0) * erfInv(2.0 * p - 1.0);
}

// Node 类的实现
Node::Node(const std::string& id, Layer* parentLayer)
    : id(id), parentLayer(parentLayer), xCord(0), yCord(0), scanStatus(0), tmpNumOfIn(0), m_tmpdata(0.0), m_wait(0.0), buffer(nullptr) {}

Node::~Node() {}

std::string Node::getId() const {
    return id;
}

Layer* Node::getParentLayer() const {
    return parentLayer;
}

void Node::addInLink(Link* link) {
    inLinks.push_back(link);
}

void Node::addOutLink(Link* link) {
    outLinks.push_back(link);
}


const std::vector<Link*>& Node::getInLinks() const {
    return inLinks;
}

const std::vector<Link*>& Node::getOutLinks() const {
    return outLinks;
}

double Node::getParkingTime() const {
    double stop_flow = 0.0;
    for (const auto& link : outLinks) {
        if (dynamic_cast<Rlink*>(link)) {
            stop_flow -= link->getBackgroundVolume(); // 停车时间与背景流量相关
        }
    }
    for (const auto& link : inLinks) {
        if (dynamic_cast<Rlink*>(link)) {
            stop_flow += link->getBackgroundVolume();
        }
    }
    if (stop_flow < 0) {
        stop_flow = 0.0; // 确保停车时间不为负
    }
    // 参数设定
    const double T_base = 3.0; // 基础停车时间 (分钟)
    const double alpha = 0.25; // 拥挤系数
    const double beta = 4.0;   // 拥挤指数

    // 避免除零错误
    if (s_parking < 1e-9) {
        return 1e6; // 如果没有停车位，返回一个极大值
    }

    // 计算占用率 (demand-to-supply ratio)
    // 注意：这里需要将流量D_parking转换成占用车辆数。
    // 一个简化的假设是，车辆平均停放时间为 T_avg_dwell。
    // 那么占用的车位数约等于 D_parking * T_avg_dwell
    // 为了简化，很多模型直接用 D_parking / S_parking 作为代理指标。
    // 我们这里也采用简化形式，假设S_parking的单位已经和D_parking匹配
    double ratio = stop_flow / s_parking;

    // BPR函数计算
    return T_base * (1.0 + alpha * std::pow(ratio, beta));
}

Centroid::Centroid(const std::string& id, Layer* parentLayer)
    : Node(id, parentLayer), totalDemand(0) {}

void Centroid::addOrigin(Node* origin, std::vector<int> demand) {
    origins.push_back(origin);
    originDemands[origin] = demand;
    for (int d : demand) {
        totalDemand += d;
    }
}

const std::vector<Node*>& Centroid::getOrigins() const {
    return origins;
}

std::vector<int> Centroid::getOriginDemand(Node* origin) const {
    auto it = originDemands.find(origin);
    if (it != originDemands.end()) {
        return it->second;
    }
    return {};
}

int Centroid::getTotalDemand() const {
    return totalDemand;
}

void Centroid::setTotalDemand(int demand) {
    totalDemand = demand;
}
