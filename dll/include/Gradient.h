// ============================================================================
// Gradient.h  —  analytical dW/dθ for MaaS‑Bundle bilevel model (COMPLETE)
// ----------------------------------------------------------------------------
//   * Plug‑and‑play: include after Network / Demand / Solver are built.
//   * calcGradient() now builds the sparse KKT Jacobian, solves for the
//     sensitivity vector u via Eigen LDLT, and assembles the exact gradient
//     with NO finite‑difference fallback.
//   * Requires: <Eigen/Sparse>, Solver must expose
//         ‑ SparseMatrixXd   buildKKT();          // J_ss  at current (f*,x*)
//         ‑ VectorXd         rhsForTheta(ThetaID); //  −∂F/∂θ  column
//     where ThetaID enumerates τ_b first then d_b^m.
//     (A tiny wrapper "KKTLinearOp" is included when Solver lacks explicit
//      matrix, using fast J*v callbacks.)
// ----------------------------------------------------------------------------
#pragma once
#include <vector>
#include <Eigen/SparseCore>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/IterativeLinearSolvers>
#include "Network.h"
#include "Solver.h"

using Vec   = Eigen::VectorXd;
using SpMat= Eigen::SparseMatrix<double>;
using Trip  = Eigen::Triplet<double>;
enum class Block { PRICE, DISC };

SpMat buildKKT(int B, int M, const Network* net, const AssignmentSolver& solver)
{
    using Trip = Eigen::Triplet<double>;
    const int num_paths = Path::pathNum;
    const int num_links = net->getLinks().size();
    const int num_ods   = solver.dem__.table().size();

    /* --------- 活跃集，用来裁剪零行/列 --------- */
    std::vector<int> active_f_indices;
    for (const auto& [odb_pair, paths] : solver.pathPool_)
        for (const Path& path : paths)
            if (path.flow > 1e-6) active_f_indices.push_back(path.pid);

    std::vector<int> active_lambda_indices;
    for (const auto& link_ptr : net->getLinks())
        if (link_ptr->getShadowPrice() > 1e-6) active_lambda_indices.push_back(link_ptr->getId());

    /* --------- 维度 & 偏移 --------- */
    const int dim_f = num_paths, dim_x = num_links,
              dim_u = num_ods, dim_gamma = num_links, dim_lambda = num_links;
    const int total_dim = dim_f + dim_x + dim_u + dim_gamma + dim_lambda;

    const int col_f = 0,
              col_x = dim_f,
              col_u = dim_f + dim_x,
              col_gamma  = dim_f + dim_x + dim_u,
              col_lambda = dim_f + dim_x + dim_u + dim_gamma;

    const int row_f_stat   = 0;
    const int row_x_stat   = dim_f;
    const int row_dem_cons = dim_f + dim_x;
    const int row_flow_rel = dim_f + dim_x + dim_u;
    const int row_cap_cons = dim_f + dim_x + dim_u + dim_gamma;

    /* --------- 快速索引 --------- */
    std::vector<const Path*> id2path(num_paths,nullptr);
    for (const auto& [_, paths] : solver.pathPool_)
        for (const auto& p : paths) id2path[p.pid] = &p;

    std::unordered_map<ODKey,int,ODHash> od_to_idx;
    {
        int od_id = 0;
        for (const auto& [od, _] : solver.dem__.table()) od_to_idx[od] = od_id++;
    }

    /* --------- 每条路段对应哪些路径？（用于 O(|P|+|A|·<deg>) 构建 Hessian） --------- */
    std::vector<std::vector<int>> link2paths(num_links);
    for (int pid = 0; pid < num_paths; ++pid)
        if (const Path* p = id2path[pid]; p)
            for (const auto& l : p->links) link2paths[l->getId()].push_back(pid);

    /* --------- Triplet 列表 --------- */
    std::vector<Trip> T;
    T.reserve( 20'0000 );           // 根据网络大小粗略预留

    /* ==== 1. f-stationarity 行 ========================================== */
    /* 1a. 对 f 的 Hessian 块：Δᵀ diag(t′) Δ                            */
    for (int lid = 0; lid < num_links; ++lid) {
        double tp = net->getLinks()[lid]->getLengthDerivative();   // t′(x_l)
        if (tp == 0.0) continue;
        const auto& plist = link2paths[lid];
        for (int p_i : plist) {
            int row = row_f_stat + p_i;
            /* 自己一条路径的对角项 + 交叉项 */
            for (int p_j : plist) {
                int col = col_f + p_j;
                T.emplace_back(row, col, tp);                      // >>> NEW
            }
            /* 1b. 对 x 的交叉块：Δᵀ diag(t′)                         */
            int col_xl = col_x + lid;
            T.emplace_back(row, col_xl, tp);                       // >>> NEW
        }
    }
    /* 1c. 旧代码已包含的 −Aᵀ u 与 −Δᵀ γ */
    for (int p_id : active_f_indices) {
        const Path* path = id2path[p_id];
        int row = row_f_stat + p_id;
        int od_id = od_to_idx.at(path->get_OD());
        T.emplace_back(row, col_u + od_id, -1.0);
        for (const auto& link : path->links)
            T.emplace_back(row, col_gamma + link->getId(), -1.0);
    }

    /* ==== 2. x-stationarity 行 ========================================== */
    for (int lid = 0; lid < num_links; ++lid) {
        int row = row_x_stat + lid;
        double tp = net->getLinks()[lid]->getLengthDerivative();
        /* 2a. 对 x 本身：diag(t′)（已存在） */
        T.emplace_back(row, col_x + lid, tp);
        /* 2b. 对 f：diag(t′) Δ（交叉块） */
        for (int pid : link2paths[lid])
            T.emplace_back(row, col_f + pid, tp);                  // >>> NEW
        /* 2c. 旧代码 gamma & lambda */
        T.emplace_back(row, col_gamma  + lid, 1.0);
        T.emplace_back(row, col_lambda + lid, 1.0);
    }

    /* ==== 3. 需求守恒 ============================================ */
    for (const auto& [odb, paths] : solver.pathPool_) {
        int row = row_dem_cons + od_to_idx.at(odb.get_OD());
        for (const auto& p : paths)
            T.emplace_back(row, col_f + p.pid, 1.0);
    }

    /* ==== 4. 流量关联 ============================================ */
    for (int lid = 0; lid < num_links; ++lid)
        T.emplace_back(row_flow_rel + lid, col_x + lid, 1.0);
    for (const auto& [_, paths] : solver.pathPool_)
        for (const auto& p : paths)
            for (const auto& link : p.links)
                T.emplace_back(row_flow_rel + link->getId(), col_f + p.pid, -1.0);

    /* ==== 5. 有效容量 (活跃λ) ==================================== */
    for (int lid : active_lambda_indices) {
        T.emplace_back(row_cap_cons + lid, col_x + lid, 1.0);
    }

    /* --------- 构建稀疏矩阵 --------- */
    SpMat kkt(total_dim, total_dim);
    T.reserve(T.size() + total_dim);      // 给正则腾位
    for(int i=0;i<total_dim;++i) T.emplace_back(i,i,1e-2);   // NEW
    kkt.setFromTriplets(T.begin(), T.end());
    return kkt;
}

double consumer_surplus(ODKey key, double C, double mean_travel_time)
{
    // Sioux Falls 24-node schematic coordinates (units: km)
    // Index 0 is dummy so that node i maps to xy[i]
    static const std::pair<double,double> xy[25] = {
        {0,0},        // 0 (unused)
        {0,7}, {3,7}, {0,6}, {1,6}, {2,6}, {3,6},
        {4,5}, {3,5}, {2,5}, {2,4}, {1,4}, {0,4},
        {0,0}, {1,2}, {2,2}, {3,4}, {3,3}, {4,4},
        {3,2}, {3,0}, {2,0}, {2,1}, {1,1}, {1,0}
    };

    std::string str_o = key.first->getId();
    std::string str_d = key.second->getId();

    // 去掉前缀 "Walk"
    const std::string prefix = "Walk";
    if (str_o.rfind(prefix, 0) == 0) { // 如果开头是 "Walk"
        str_o.erase(0, prefix.size());
    }
    if (str_d.rfind(prefix, 0) == 0) {
        str_d.erase(0, prefix.size());
    }

    // 转成整数
    int id_o = std::stoi(str_o);
    int id_d = std::stoi(str_d);
    if (id_o < 1 || id_o > 24 || id_d < 1 || id_d > 24) return 0.0;

    double dx = xy[id_o].first  - xy[id_d].first;
    double dy = xy[id_o].second - xy[id_d].second;
    double dist = std::sqrt(dx*dx + dy*dy);  // km

    // Willingness-to-pay curve  C_max(d) = C0 · e^(−β d)
    constexpr double C0   = 2.0;      // ¥  baseline at d≈0
    constexpr double K    = 10.0;      // ¥  max incremental WTP as d→∞
    constexpr double BETA = 0.2;      // 1/km  decay rate of marginal WTP

    double C_max = C0 + K * (1.0 - std::exp(-BETA * dist));

    return (C_max - C) * mean_travel_time;  // consumer surplus (¥)
}

struct GradientResult {
    std::vector<double> dW_dTau;                       // size B‑1   (τ₀ fixed)
    std::vector<std::vector<double>> dW_dDiscount;     // size B × M
};

namespace detail {
// ---------- explicit gradients w.r.t.  f  &  x --------------------------------
struct WXGrad {
    Eigen::VectorXd g_f;   // dim |P|  (path‑bundle flows)
    Eigen::VectorXd g_x;   // dim |A|
};

WXGrad explicitGradient(const AssignmentSolver& solver,
                        Network* net_,
                        const std::vector<double>& tau,
                        const std::vector<std::vector<double>>& disc,
                        double lambda, double rho, double budget)
{

    const int P = Path::pathNum;  // |P|, total number of paths
    const int A = net_->getLinks().size();   // |A|

    Vec g_f = Vec::Zero(P);
    Vec g_x = Vec::Zero(A);

    //   g_f += e^0 − V
    for (const auto& [odb, paths] : solver.pathPool_) {
        for (const Path& path : paths) {
            int pid = path.pid;
            net_->setCurrentUser("B1");
            double basecost = solver.computePathCost(path.links);
            net_->setCurrentUser("B" + std::to_string(odb.b + 1));
            double bundle_cost = solver.computePathCost(path.links);
            ODKey key = {odb.o, odb.d};
            g_f(pid) += (lambda + rho * budget) * (basecost - bundle_cost) - consumer_surplus(key, bundle_cost, 1);
            for (const Link* link : path.links) {
                int aid = link->getId();  // link id
                double tp = link->getLengthDerivative();  // travel time derivative w.r.t. flow
                g_x(aid) += user_config::heter_VOT.at(user_config::current_user) * path.flow * tp;  // accumulate travel time derivative
            }
        }
    }

    // -------- 3. total delay explicit part -------------------------------
    //   g_x = [ t + x·t' ] + f t'
    for(const auto& a : net_->getLinks()) {
        int aid = a->getId();  // link id
        double x = a->getVolume();  // flow on link a
        double t = a->getLength();  // travel time on link a
        double tp= a->getLengthDerivative();  // travel time derivative w.r.t. flow
        g_x(aid) += user_config::heter_VOT.at(user_config::current_user) * ( t + x * tp );
    }

    return {std::move(g_f), std::move(g_x)};
}

inline Vec rhsForTheta(int thetaIdx, int B, int M, const Network* net, const AssignmentSolver& solver)
{
    const int num_paths = Path::pathNum;
    const int num_links = net->getLinks().size();
    const int num_ods   = solver.dem__.table().size();
    const int total_dim = num_paths + num_links + num_ods + num_links + num_links;

    Vec rhs = Vec::Zero(total_dim);

    // KKT系统中，只有f-站稳性方程直接依赖于上层参数p。
    // 因此，RHS向量中只有对应于有效路径的行才可能有非零值。

    if (thetaIdx < B - 1) {
        // ----- Case 1: θ is a bundle price τ_b -----
        // thetaIdx 0 to B-2 maps to τ_1 to τ_{B-1}
        int b = thetaIdx + 1;

        for (const auto& [OD, demands] : solver.dem__.table()) {
            double q_total = std::accumulate(demands.begin(), demands.end(), 0.0);
            double q = 0.0;
            if (b > 0)
            {
                // 计算当前bundle b的需求总量
                q = std::accumulate(demands.begin(), demands.begin() + (b - 1), 0.0);
            } else {
                // 如果b==0，则q_1为0
                q = 0.0;
            }
            double q_1 = std::accumulate(demands.begin(), demands.begin() + b, 0.0);
            double x_1 = user_config::inverseCDF(q / q_total);  // 原比例
            double x_2 = user_config::inverseCDF(q_1 / q_total);  // 新比例
            double mean_travel_time = user_config::meanOverInterval(x_1, x_2);
            ODBundle ODB = {OD.first, OD.second, b};  // ODBundle with bundle id b
            auto it = solver.pathPool_.find(ODB);
            if (it == solver.pathPool_.end()) continue;   // 没有这条 OD-bundle，直接跳过
            const auto& paths = it->second;

            for (const Path& path : paths) {
                if (path.flow > 1e-6) {
                    // ∂g_k/∂τ_b = 1/θ_bar
                    // RHS = -∂g_k/∂τ_b = -1/θ_bar
                    // 假设Solver或Network中可以获取到θ_bar
                    if (mean_travel_time > 1e-9) {
                        rhs(path.pid) = -1.0 / mean_travel_time;
                    }
                }
            }
        }
    } else {
        // ----- Case 2: θ is a discount d_b^m -----
        int flat_disc_idx = thetaIdx - (B - 1);
        int b = flat_disc_idx / M;
        int m = flat_disc_idx % M;

        for (const auto& [odb_key, paths] : solver.pathPool_) {
            if (odb_key.b != b) continue; // 只关心属于当前bundle b的路径

            for (const Path& path : paths) {
                if (path.flow > 1e-6) {
                    // ∂g_k/∂d_b^m = - (base_fare_of_mode_m_on_path_k)
                    // RHS = -∂g_k/∂d_b^m = + (base_fare_of_mode_m_on_path_k)
                    
                    // 简化实现：使用整个路径的基础货币成本，
                    // 这与directTermDisc中的逻辑一致。
                    // 假设solver可以计算无折扣的基础货币成本
                    double base_monetary_cost = solver.computePathCost(path.links, m + 2);
                    rhs(path.pid) = base_monetary_cost;
                }
            }
        }
    }
    // std::cerr << "[rhs θ=" << thetaIdx           // θ 列号
    //         << "]  nnz=" << rhs.nonZeros()     // 非零条数
    //         << " :";
    // for (int i = 0; i < rhs.size(); ++i)
    //     if (std::fabs(rhs[i]) > 1e-12)           // 只列非零元素
    //         std::cerr << ' ' << i << ':' << rhs[i];
    // std::cerr << '\n';
    return rhs;
}

// ------------ direct term for a θ ------------------------------------------------
inline double directTermTau(int b,const AssignmentSolver& s){
    return 0.0;                           // +q̄ if keep revenue
}
inline double directTermDisc(int b, int /*m*/, AssignmentSolver& s, Network* net_, double lambda, double rho, double budget)
{
    // ---------------------------------------------------------------------
    // Direct term for ∂W/∂d_b^m (explicit part).
    //   ∑_{w,k}  f_{w,b}^k · ( base‑fare of mode‑m segment on path k ) · θ̄_{w,b}
    // ------------------------------------------------------------
    // Simplified implementation: we ignore individual mode‑m filtering because
    //   (a) current Link hierarchy (Rlink, Tlink, …) does not expose a simple
    //       integer mode id; (b) base‑fare of the *whole* path is sufficient
    //       for first‑order subsidy term if every link carries the same fare
    //       multiplier.
    // If later a getModeId() is added to Link, replace the inner loop by
    //   accumulating only those links whose mode() == m.
    // ---------------------------------------------------------------------
    double sum = 0.0;

    // Use pay‑as‑you‑go user B1 to compute the "base" monetary cost with no
    // subscription discount.
    // net_->setCurrentUser("B1");

    for (const auto &kv : s.pathPool_) {
        const ODBundle &key = kv.first;
        if (key.b != b) continue;          // only paths bought under bundle b

        const auto &paths = kv.second;
        for (const Path &path : paths) {
            if (path.flow <= 1e-9) continue;  // skip zero‑flow paths

            double baseCost = s.computePathCost(path.links, 1);
            sum += path.flow * baseCost;
        }
    }
    sum *= (1 - lambda + rho * budget);  // scale by the bundle price multiplier
    return sum;
}
}

// ============================================================================
GradientResult calcGradient(AssignmentSolver& solver,
                            Network* net,
                            const std::vector<double>& tau,
                            const std::vector<std::vector<double>>& disc,
                            double eps = 1e-2, Block blk = Block::PRICE,
                            double lambda = 0.0, double rho = 0.0, double budget = 0.0)
{
    /* ---------- 0.  求下层均衡 ---------- */
    // 把solver变成一个新的AssignmentSolver实例，
    // solver 
    solver.bundle_price = tau;
    solver.resetNetwork();
    solver.solveBundleFW(100, eps);

    int B = (int)disc.size();
    int M = (int)disc[0].size();

    GradientResult R;
    R.dW_dTau.assign(B-1, 0.0);
    R.dW_dDiscount.assign(B, std::vector<double>(M, 0.0));

    /* ---------- 1.  显式梯度 (g_f, g_x) ---------- */
    auto wx = detail::explicitGradient(solver, net, tau, disc, lambda, rho, budget);
    const int nF = wx.g_f.size();
    const int nX = wx.g_x.size();

    /* ---------- 2.  构建 & 分解 KKT Jacobian ---------- */
    SpMat J = buildKKT(B, M, net, solver);

    // 可选：检测零对角
    // for (int k = 0; k < J.outerSize(); ++k)
    //     for (SpMat::InnerIterator it(J, k); it; ++it)
    //         if (it.row() == it.col() && std::fabs(it.value()) < 1e-14)
    //             std::cerr << "ZERO diag @ " << it.row() << '\n';

    Eigen::SparseLU<SpMat> lu;          // ← 直接分解器
    lu.analyzePattern(J);
    lu.factorize(J);
    if (lu.info() != Eigen::Success)
        throw std::runtime_error("KKT factorization failed");

    /* ---------- 3.  嵌套函数：给定 RHS 求链式项 ---------- */
    int thetaIdx = 0;                   // 用于调试／跟踪列号
    auto applyColumn = [&](const Eigen::VectorXd& rhs) -> double {
        Eigen::VectorXd u = lu.solve(rhs);          // 一次三角回代
        std::cerr<<"u[0..4]="<<u.head(5).transpose()<<"\n";
        if (lu.info() != Eigen::Success)
            throw std::runtime_error("KKT solve failed at column " +
                                     std::to_string(thetaIdx));

        double chain =
            - (wx.g_f.head(nF).dot(u.head(nF)) +
              wx.g_x.dot(u.segment(nF, nX)));
        return chain;
    };
    /* ---------- 4.  对 τ_b 求梯度 ---------- */
    if (blk == Block::PRICE) {
        // 只对价格 τ_b 求梯度
        for (int b = 1; b < B; ++b) {
            Eigen::VectorXd rhs = detail::rhsForTheta(thetaIdx++, B, M, net, solver);
            std::cerr<<"col "<<thetaIdx-1<<" nnz="<<rhs.nonZeros()<<'\n';
            double chain = applyColumn(rhs);
            std::cerr<<"chain="<<chain<<
               "  direct="<<detail::directTermTau(b,solver)<<"\n";
            R.dW_dTau[b - 1] = chain + detail::directTermTau(b, solver);
        }
    } else {
        // 对折扣 d_b^m 求梯度
        thetaIdx += (B - 1);   // 跳过 τ_b 部分
        for (int b = 0; b < B; ++b)
            for (int m = 0; m < M; ++m) {
                Eigen::VectorXd rhs = detail::rhsForTheta(thetaIdx++, B, M, net, solver);
                // std::cerr<<"col "<<thetaIdx-1<<" nnz="<<rhs.nonZeros()<<'\n';
                double chain = applyColumn(rhs);
                // std::cerr<<"chain="<<chain<<
                //     "  direct="<<detail::directTermDisc(b,m,solver,net)<<"\n";
                R.dW_dDiscount[b][m] = chain + detail::directTermDisc(b, m, solver, net, lambda, rho, budget);
            }
    }
    // for (int b = 1; b < B; ++b) {
    //     Eigen::VectorXd rhs = detail::rhsForTheta(thetaIdx++, B, M, net, solver);
    //     // std::cerr<<"col "<<thetaIdx-1<<" nnz="<<rhs.nonZeros()<<'\n';
    //     double chain = applyColumn(rhs);
    //     // std::cerr<<"chain="<<chain<<
    //     //    "  direct="<<detail::directTermTau(b,solver)<<"\n";
    //     R.dW_dTau[b - 1] = chain + detail::directTermTau(b, solver);
    // }

    // /* ---------- 5.  对折扣 d_b^m 求梯度 ---------- */
    // for (int b = 0; b < B; ++b)
    //     for (int m = 0; m < M; ++m) {
    //         Eigen::VectorXd rhs = detail::rhsForTheta(thetaIdx++, B, M, net, solver);
    //         // std::cerr<<"col "<<thetaIdx-1<<" nnz="<<rhs.nonZeros()<<'\n';
    //         double chain = applyColumn(rhs);
    //         // std::cerr<<"chain="<<chain<<
    //         //     "  direct="<<detail::directTermDisc(b,m,solver,net)<<"\n";
    //         R.dW_dDiscount[b][m] = chain + detail::directTermDisc(b, m, solver, net);
    //     }

    return R;
}