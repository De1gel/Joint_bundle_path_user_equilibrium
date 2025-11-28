#pragma once
#include "head.h"
class Centroid; // od centroid
class Node;
class Link;
class Rlink;
class Layer;

class multiNet_API user_config {
public:
    static std::string current_user;
    static double VOT;
    static bool heter_user;
    static std::vector<std::string> heter_types;
    static std::unordered_map<std::string, double> heter_VOT;
    static std::unordered_map<std::string, double> heter_toll_bike;
    static std::unordered_map<std::string, double> heter_toll_RH;
    static std::unordered_map<std::string, double> heter_toll_PT;

    // 出行次数分布配置
    enum class DistributionType { Triangular, Normal };
    static DistributionType trip_dist_type;
    static double dist_param_a, dist_param_b, dist_param_m; // 用于三角分布：a=下限, b=上限, m=众数
    static double dist_mu, dist_sigma;                      // 用于正态分布：mu=均值, sigma=标准差

    // 分布计算接口
    static double getPDF(double x);
    static double getCDF(double x);
    static double integrate(double x1, double x2);
    static double meanOverInterval(double x1, double x2);
    static double inverseCDF(double p);

private:
    static double sqr(double x) { return x * x; }
    static double triangularCDF(double z, double a, double b, double m);
    static double triangularPDF(double z, double a, double b, double m);
    static double normalPDF(double z, double mu, double sigma);
    static double normalCDF(double z, double mu, double sigma);
    static double inverseTriangularCDF(double p, double a, double b, double m);
    static double inverseNormalCDF(double p, double mu, double sigma);
    static double erfInv(double x); // 误差函数的反函数
};

class multiNet_API Node {
    public:
        Node(const std::string& id, Layer* parentLayer);
        virtual ~Node();
    
        std::string getId() const;
        Layer* getParentLayer() const;
        void addInLink(Link* link);
        void addOutLink(Link* link);
        const std::vector<Link*>& getInLinks() const;
        const std::vector<Link*>& getOutLinks() const;
        double getParkingTime() const;
    
    private:
        std::vector<Link*> inLinks;
        std::vector<Link*> outLinks;
        std::string id;
        Layer* parentLayer;
        int xCord;
        int yCord;
        int scanStatus;
        int tmpNumOfIn;
        double m_tmpdata;
        double m_wait;
        double s_parking = 100.0; // 停车位 
        void* buffer;
};

class multiNet_API Centroid : public Node {
    public:
        // 构造函数依然保留已有参数
        Centroid(const std::string& id, Layer* parentLayer);
    
        // 添加带有需求量的来源节点
        void addOrigin(Node* origin, std::vector<int> demand);
    
        // 获取所有来源节点
        const std::vector<Node*>& getOrigins() const;
    
        // 获取特定起源节点的需求量
        std::vector<int> getOriginDemand(Node* origin) const;

        // 获取或设置总体需求量（如果有需要）
        int getTotalDemand() const;
        void setTotalDemand(int demand);
    
    private:
        int totalDemand;  // 可选：质心总体需求量
        std::vector<Node*> origins;  // 存储所有起源节点
        std::unordered_map<Node*, std::vector<int>> originDemands; // 每个起源节点对应的需求量
};