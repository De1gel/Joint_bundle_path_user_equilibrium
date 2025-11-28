#pragma once
#include "head.h"

class Network;
class Node;
class Link;
class Layer;
class DemandStore;
class AssignmentSolver;
class user_config;
struct Path;
using ODKey = std::pair<Node*, Node*>;

struct ODBundle {
    Node* o;
    Node* d;
    int   b;  // bundle id
    bool operator==(const ODBundle& rhs) const noexcept {
        return o==rhs.o && d==rhs.d && b==rhs.b;
    }
    // 获取对应的 ODKey
    inline ODKey get_OD() const noexcept {
        return {o, d};
    }
};
struct ODBundleHash {
    size_t operator()(const ODBundle& k) const noexcept {
        return (reinterpret_cast<uintptr_t>(k.o)>>3) ^
               (reinterpret_cast<uintptr_t>(k.d)<<5) ^ k.b;
    }
};
using PathPool = std::unordered_map<ODBundle, std::vector<Path>, ODBundleHash>;
using FlowMap       = std::unordered_map<Link*, double>;
using PathMap = std::unordered_map<ODBundle, std::unordered_map<size_t,double>, ODBundleHash>;

/*  ODKey 用指针对，便于后续直接拿来求最短路 */
struct ODHash {
    size_t operator()(const ODKey& p) const noexcept {
        return std::hash<Node*>{}(p.first) ^ (std::hash<Node*>{}(p.second)<<1);
    }
};

/* -------------------------------------------------- */
class multiNet_API DemandStore {
public:
    /* 每条记录存 vector<double>: bundle0,bundle1,...   or   单用户仅 size()==1 */
    void loadTripFile(const std::string& filename, const Network* net, int user_count);
    void updateDemand(const ODKey& key, const std::vector<double>& demand, int iter=1);
    void updateDemand(const ODKey& key, const std::vector<double>& demand, int iter=1) const;
    /* 访问接口 */
    const std::vector<double>& getDemand(Node* o, Node* d) const;
    const double getTotalDemand(const ODKey& key) const;
    const auto& table() const { return od_; }

private:
    std::unordered_map<ODKey, std::vector<double>, ODHash> od_;
};


class multiNet_API AssignmentSolver {
public:
    AssignmentSolver(Network* net, const DemandStore* dem)
        : net_(net), dem_(dem), dem__(*dem){
        }
    DemandStore dem__;
    void solveFW(int maxIter=100, double tol=1e-6);
    void solveBundleFW(int maxIter=100, double tol=1e-6);
    double computePathCost(const std::vector<Link*>& links, int mode = 0) const;
    PathPool pathPool_;
    std::vector<double> bundle_price;  // bundle flow weights
    void getModeChoice(); // 当前路径池中不同交通方式占比
    void resetNetwork(int lel = 0); // 重置网络状态，lel=1时重置流量和影子价格，lel=0时仅重置影子价格
    void getAverageTransferTime(); // 当前路径池中的平均换乘次数
    void printMuCosts();
    double calculateUniquenessRatio() const;
private:
    double h_k_ = 1.0;  // SRA 步长
    double h_k = 1.0;
    Network*     net_;
    const DemandStore* dem_;
    double computeGap(const std::unordered_map<Link*,double>& auxFlow);
    size_t addOrAccumulatePath(const ODBundle&, const std::vector<Link*>&);
    void updatePathCosts();
    void updateBundlePathCosts();
    double calcPathGap() const;
    double calcBundlePathGap() const;
    using HeterFlowMap  = std::unordered_map<Link*, std::vector<double>>;
    void buildAuxFlow(FlowMap& aux, HeterFlowMap& auxH, 
                      PathMap& auxPathFlow);
    void buildBundleAuxFlow(FlowMap& aux, HeterFlowMap& auxH, 
                      PathMap& auxPathFlow);
    double calcRelGap(const FlowMap& aux) const;
    void updateLinkVolumeMSA(const FlowMap& aux, const HeterFlowMap& auxH, int iter,
                              PathMap& auxPathFlow);
    void updateBundleLinkVolumeMSA(const FlowMap& aux, const HeterFlowMap& auxH, int iter,
                              PathMap& auxPathFlow/*, PathMap& prevPathFlow, PathMap& prevAuxPathFlow, double& h_k*/);
    std::unordered_map<ODBundle,double,ODBundleHash> spCost_;
    std::unordered_map<ODBundle,double,ODBundleHash> sbpCost_;
    std::unordered_map<ODBundle,double,ODBundleHash> bundleMeanTrips_;
};


