#include "Link.h"
#include "Node.h"
#include "Layer.h"

// Link 类的实现
Link::Link(int id, Node* head, Node* tail, double time_)
    : id(id), head(head), tail(tail), volume(0.0), capacity(0.0), length(time_), toll_(0.0), buffer(nullptr) {
    head->addOutLink(this);
    tail->addInLink(this);
    if (user_config::heter_user) {
        for (const auto& type : user_config::heter_types) {
            heter_volume[type] = 0.0;
        }
    }
}

double Link::getCost() const {
    if (user_config::heter_user) {
        double VOT = 0.0;
        try {
            VOT = user_config::heter_VOT.at(user_config::current_user);
        } catch (const std::out_of_range& e) {
            throw std::invalid_argument("VOT not found for user: " + user_config::current_user);
        }
        return VOT * getLength() + getToll();
    }
    return user_config::VOT * getLength() + getToll();
}

// Rlink 类的实现
Rlink::Rlink(int id, Node* head, Node* tail, double time_, double capacity, double toll_)
    : Link(id, head, tail, time_) {
    this->capacity = capacity;
    this->toll_ = toll_;
}

double Rlink::getLength() const {
    return length * (1 + 0.15 * pow((background_volume + volume + correlink->getVolume()) / capacity, 4));
}

void Rlink::setBackgroundVolume(double volume) {
    background_volume = volume;
    if (correlink != nullptr && correlink->getBackgroundVolume() != volume) {
        correlink->setBackgroundVolume(volume);
    }
}

double Rlink::getLengthDerivative() const {
    double x = background_volume + volume + correlink->getVolume();
    if (capacity < 1e-6) return 0.0; // 避免除零
    double ratio = x / capacity;
    if (getVolume() > 0.0) {
        std::cout << "[Rlink] id=" << id
                << " x=" << x
                << "background_volume=" << background_volume
                << " cap=" << capacity
                << " ratio=" << ratio
                << " grad=" << (length * 0.6 * pow(ratio,3) / capacity)
                << std::endl;
    }
    return length * 0.6 * std::pow(ratio, 3) / capacity;
    // return 0;
}

double Rlink::getToll() const {
    // if (user_config::heter_user) {
    //     return toll_ * user_config::heter_toll_RH.at(user_config::current_user);
    // }
    return toll_;
}

// Tlink 类的实现
Tlink::Tlink(int id, Node* head, Node* tail, double time_)
    : Link(id, head, tail, time_) {
        this->capacity = tail->getParentLayer()->getCapacity();
    }

double Tlink::getToll() const {
    if (user_config::heter_user) {
        return toll_ * user_config::heter_toll_PT.at(user_config::current_user);
    }
    return toll_;
}

double Tlink::getVacancy(double volume_) const {
    double frequency = tail->getParentLayer()->getFrequency();
    return frequency * capacity - (volume_ == -1 ? volume : volume_);
}

// Wlink 类的实现
Wlink::Wlink(int id, Node* head, Node* tail, double time_)
    : Link(id, head, tail, time_) {}


double Wlink::getLength() const {
    // if (user_config::heter_user) {
    //     if (user_config::heter_toll_bike[user_config::current_user] == 0.0)
    //         return time_bike;
    // }
    return length;
}

double Wlink::getToll() const {
    // if (user_config::heter_user) {
    //     if (user_config::heter_toll_bike[user_config::current_user] == 0.0)
    //         return 0.0;
    // }
    return toll_;
}

// ATlink 类的实现
ATlink::ATlink(int id, Node* head, Node* tail, double time_, std::string line_id, double frequency)
    : Link(id, head, tail, time_), line_id(line_id), frequency(frequency) {
    }
std::string ATlink::getLineId() const {
    return line_id;
}

double ATlink::getLength() const
{
    static constexpr double BETA_PT  = 0.40;   // β_pt
    static constexpr double GAMMA_PT = 0.15;   // γ_pt
    static constexpr double Z_PT     = 1.30;   // z  (>1)
    const double x = getVolume();          // 乘客登车流量 x_a   [pax·h⁻¹]
    const double f = std::max(1e-6, frequency);   // 发车频率 f_a  [veh·h⁻¹]
    const double capacity = getTail()->getParentLayer()->getCapacity();

    double x_plus = 0.0; // x⁺_a 取值
    /* —— 精确的 x⁺_a 取法 —— */
    for (const Link* outLink : getTail()->getOutLinks()) {
        if (outLink->getTail()->getParentLayer()->getMode() != "Walk") {
            // 取第一个出站的 PT link 的流量作为 x⁺_a
            x_plus = outLink->getVolume();
        }
    }

    const double ratio_raw  = ((1.0 - GAMMA_PT) * x
                          + GAMMA_PT * x_plus) / (capacity * f);

    /* 数值保护：ratio→0 时 pow 下溢 */
    const double ratio     = std::max(ratio_raw, 1e-12);
    const double core   = std::pow(std::max(ratio, 1e-12), Z_PT);

    return 1.0 / f + BETA_PT * core;
}


double ATlink::getLengthDerivative() const
{
    static constexpr double BETA_PT  = 0.40;   // β_pt
    static constexpr double GAMMA_PT = 0.15;   // γ_pt
    static constexpr double Z_PT     = 1.30;   // z  (>1)
    const double x = getVolume();                                   // 当前该 ATlink 上乘客流量 x_a
    const double f = std::max(1e-6, frequency);                     // 发车频率 f_a
    const double capacity = getTail()->getParentLayer()->getCapacity();

    double x_plus = 0.0; // x⁺_a 取值
    /* —— 精确的 x⁺_a 取法 —— */
    for (const Link* outLink : getTail()->getOutLinks()) {
        if (outLink->getTail()->getParentLayer()->getMode() != "Walk") {
            // 取第一个出站的 PT link 的流量作为 x⁺_a
            x_plus = outLink->getVolume();
        }
    }

    /* —— ratio 及其导数 —— */
    // N = x + γ (x⁺_a − x) = (1−γ)·x + γ·x⁺_a
    const double N         = (1.0 - GAMMA_PT) * x + GAMMA_PT * x_plus;
    const double ratio     = N / (capacity * f);

    // ∂N/∂x = 1 − γ   （x⁺_a 与 x 无关）
    const double dN_dx     = 1.0 - GAMMA_PT;
    const double dratio_dx = dN_dx / (capacity * f);

    /* —— ∂t/∂x = β·z·ratio^{z-1}·dratio_dx —— */
    const double grad =
        BETA_PT * Z_PT *
        std::pow(std::max(ratio, 1e-12), Z_PT - 1.0) *
        dratio_dx;
    if (getVolume() > 0.0) {
        std::cout << "[ATlink] id=" << id
                << " x=" << x
                << " cap=" << capacity * f
                << " ratio=" << ratio
                << " grad=" << grad
                << std::endl;
    }
    return grad;   // 1/f 项导数为 0
}

// ARlink 类的实现
ARlink::ARlink(int id, Node* head, Node* tail, double time_)
    : Link(id, head, tail, time_) {}

ARHlink::ARHlink(int id, Node* head, Node* tail, double time_)
    : Link(id, head, tail, time_) {}

double ARHlink::getLength() const {
    double kappa  = 1.0,
           a1     = 0.6,
           a2     = 0.4,
           psi    = 0.0,
           rho    = 0.1,
           lambda = 20.0;
    Layer* layer = tail->getParentLayer();
    const double D = layer->getFleetDemand();
    const double V = layer->getFleetVacant();

    /* 零需求直接返回 0（避免 log/除零） */
    if (D < 1e-9) return 0.0;

    /* ------------------------------------------------------------
    * 2) 基础等待时间项  t_base = κ^(−1/α₂) · V^(−α₁/α₂) · D^((1−α₂)/α₂)
    * ------------------------------------------------------------ */
    const double base =
        std::pow(kappa, -1.0 / a2) *
        std::pow(V,     -a1   / a2) *
        std::pow(D,     (1.0 - a2) / a2);

    /* ------------------------------------------------------------
    * 3) 软阈值惩罚：penalty = ψ/λ · ln(1+e^{λ(ρ − V/D)})
    * ------------------------------------------------------------ */
    const double penalty =
        psi / lambda *
        std::log1p(std::exp(lambda * (rho - V / D)));

    return base + penalty;
}

double ARHlink::getLengthDerivative() const
{
    /* ——— 常量 / 参数（可改为 class 成员） ——— */
    static constexpr double kappa  = 1.0;
    static constexpr double a1     = 0.6;
    static constexpr double a2     = 0.4;
    static constexpr double psi    = 0.0;
    static constexpr double rho    = 0.1;
    static constexpr double lambda = 20.0;

    /* ——— 系统级指标 ——— */
    Layer*  layer = tail->getParentLayer();
    const double D = layer->getFleetDemand();   // Σ boarding-flows
    const double V = layer->getFleetVacant();   // 空驶车辆 (veh)

    if (D < 1e-9)  return 0.0;                  // 避免除零

    /* ——— ① 基础项导数  d(base)/dD ———
       base = κ^(−1/α₂) · V^(−α₁/α₂) · D^((1−α₂)/α₂)
       => d(base)/dD = base · (1−α₂)/(α₂·D)                             */
    const double base =
        std::pow(kappa, -1.0 / a2) *
        std::pow(V,     -a1   / a2) *
        std::pow(D,     (1.0 - a2) / a2);
    const double dBase =
        base * (1.0 - a2) / (a2 * D);

    /* ——— ② 软阈值惩罚导数  d(penalty)/dD ———
       penalty = ψ/λ · ln(1+e^{λ(ρ−V/D)})
       let u = ρ−V/D  ⇒  du/dD =  V / D²
       d(pen)/dD = ψ * σ(λu) * V / D²            (σ 为 logistic)       */
    const double u       = rho - V / D;
    const double sigma   = 1.0 / (1.0 + std::exp(-lambda * u));   // 数值稳定
    const double dPen    = psi * sigma * V / (D * D);

    return dBase + dPen;
}


// Elink 类的实现
Elink::Elink(int id, Node* head, Node* tail, double time_)
    : Link(id, head, tail, time_) {}

// LinkCreator 类的实现
Link* RlinkCreator::create(int id, Node* head, Node* tail, double time_, const std::vector<std::string>& params) const {
    double capacity = params.empty() ? 0.0 : std::stod(params[0]);
    double toll = params.size() > 1 ? std::stod(params[1]) : 0.0;
    return new Rlink(id, head, tail, time_, capacity, toll);
}

Link* TlinkCreator::create(int id, Node* head, Node* tail, double time_, const std::vector<std::string>& params) const {
    return new Tlink(id, head, tail, time_);
}

Link* WlinkCreator::create(int id, Node* head, Node* tail, double time_, const std::vector<std::string>& params) const {
    return new Wlink(id, head, tail, time_);
}

Link* ATlinkCreator::create(int id, Node* head, Node* tail, double time_, const std::vector<std::string>& params) const {
    if (params.size() < 2) {
        throw std::invalid_argument("ATlink requires line_id and frequency");
    }
    std::string line_id = params[0];
    double frequency = std::stod(params[1]);
    return new ATlink(id, head, tail, time_, line_id, frequency);
}

Link* ARlinkCreator::create(int id, Node* head, Node* tail, double time_, const std::vector<std::string>& params) const {
    return new ARlink(id, head, tail, time_);
}

Link* ARHlinkCreator::create(int id, Node* head, Node* tail, double time_, const std::vector<std::string>& params) const {
    return new ARHlink(id, head, tail, time_);
}

Link* ElinkCreator::create(int id, Node* head, Node* tail, double time_, const std::vector<std::string>& params) const {
    return new Elink(id, head, tail, time_);
}

// LinkGenerator 类的实现
LinkGenerator& LinkGenerator::getInstance() {
    static LinkGenerator instance;
    return instance;
}

void LinkGenerator::registerLinkType(const std::string& type, LinkCreator* creator) {
    creators[type] = creator;
}

Link* LinkGenerator::generateLink(const std::string& type, int id, Node* head, Node* tail, double time_, const std::vector<std::string>& params) {
    auto it = creators.find(type);
    if (it != creators.end()) {
        return it->second->create(id, head, tail, time_, params);
    }
    throw std::invalid_argument("Unknown link type: " + type);
}
