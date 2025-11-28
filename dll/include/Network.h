#pragma once
#include "head.h"
// #include "..\trafficNet\stdafx.h"

class Network;
class Layer;
class TransitLine;
class Node;
class Link;
using ODKey = std::pair<Node*, Node*>;

/* 一个简化 Path 对象：只关心 link 列表 & 流量 & 最新成本 */
struct multiNet_API Path {
    std::vector<Link*> links;
    double             flow  = 0.0;  // 总流（多用户可用 map）
    double             cost  = 0.0;  // Σ c_e(links)
    int                pid     = -1;    // id

    static int pathNum;  // 全局路径计数器

    void assignId() {
        if (pid == -1) {
            pid = pathNum++;
        }
    }

    // 获取路径的 OD 对应关系
    ODKey get_OD() const noexcept;
};

class multiNet_API Network {
    public:
        Network();
        ~Network();
        void setHeterUser(const std::vector<std::string>& heter_types);
        void setHeterVOT(const std::unordered_map<std::string, double>& heter_VOT);
        double getHeterVOT(const std::string& type) const;
        void setHeterTollBike(const std::unordered_map<std::string, double>& heter_toll_bike);
        void setHeterTollRH(const std::unordered_map<std::string, double>& heter_toll_RH);
        void setHeterTollPT(const std::unordered_map<std::string, double>& heter_toll_PT);
        void setHeterTollDiscount(const std::vector<std::vector<double>>& heter_toll_discount);
        void setVOT(double VOT);
        void setCurrentUser(const std::string& user);
        void setTripDistributionTriangular(double a, double b, double m);
        void setTripDistributionNormal(double mu, double sigma);
        void setDistributionType(const std::string& type);
        // 向网络中添加一层
        void addLayer(const std::string& layerName, const std::string& type);
        void addGlobalLink(Link* link);
        void addGlobalNode(Node* node);
        void modifyCapacity(double ratio, std::vector<std::string>& modified_lines) const;
        std::vector<Node*> getNodes() const;
        std::vector<Link*> getLinks() const;

        int getLayerNum() const;

        // 获取某一层
        Layer* getLayer(const std::string& layerName) const;

        // 获取所有公交线路层
        std::vector<TransitLine*> getTransitLines() const;

        // 创建网络
        void createNetwork(const std::string& filepath);

        // 最短路径查找
        Path shortestPath(Node* from, Node* to, const std::string& algo, const std::string& cost_mode, int K_maxTransfer = 0) const;

        // 统计路网数据
        void getCongestionData() const;
    private:
        // 将每个层以 layerName -> Layer 的形式储存
        std::unordered_map<std::string, std::unique_ptr<Layer>> layers;
        std::vector<Link*> all_links;
        std::vector<Node*> all_nodes;
        void readRouteFile(std::ifstream& file);
        void readShapeFile(std::ifstream& file);
        void readRoadFile(std::ifstream& file);
        void readTransitFile(std::ifstream& file);
        void readWalkFile(std::ifstream& file);
        void readTripFile(std::ifstream& file);
        void readFlowFile(std::ifstream& file);
        void generateTransferLinks();
        Path Dijkstra(Node* from, Node* to, const std::string& cost_mode,
                      int K_maxTransfer) const;
};