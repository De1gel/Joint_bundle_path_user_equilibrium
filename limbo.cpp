#include <limbo/limbo.hpp>
using namespace limbo;

struct Params {
    struct bayes_opt_bobachk {
        BO_PARAM(double, noise, 1e-3);          // 观测噪声先验
        BO_PARAM(int,   init_samples, 8);       // LHS ×8
        BO_PARAM(int,   iterations,   100);     // 总迭代 ≤120
    };
    struct kernel_squared_exp_ard {
        BO_PARAM(double, sigma, 1.0);           // 输出方差
        BO_PARAM(double, l,     1.0);           // 初始长度尺度
    };
    struct stop_maxeval { BO_PARAM(int, value, 120); };
};

using Kernel = kernel::SquaredExpARD<Params>;
using Mean   = mean::Data<Params>;
using GP_t   = model::GP<Params, Kernel, Mean>;
using Acq_t  = acqui::EI<Params, GP_t>;         // 期望改进

// ------------ 你的评价函数包装 ---------------
struct Eval {
    double operator()(const Eigen::VectorXd& x) const {
        std::array<double,3> tau = transform(x);           // 变量变换
        double welfare = run_assignment_and_welfare(tau);  // 调你已有的 evaluate()
        return welfare;                                    // BO 默认“最大化”
    }
};

// ------------ 运行优化 ----------------------
int main() {
    bop::BOptimizer<Params, GP_t, Acq_t> opt;
    Eigen::VectorXd x_best = opt.optimize(Eval());
    auto tau_best = transform(x_best);
    std::cout << "Best τ = " << tau_best[1] << ' '
              << tau_best[2] << ' ' << tau_best[3] << std::endl;
}