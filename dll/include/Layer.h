#pragma once
#include "head.h"

class Node;
class Centroid;
class Link;
class Layer;
class TransitLine;
class Network;

class multiNet_API Layer {
    public:
        Layer(Network* parent, const std::string& modeName);
        int getLayerNum() const;
        int getNodeNum() const;
        int getLinkNum() const;
        void getReachabilityMatrix();
        virtual double getCapacity() const { return 0.0; }  // 基类给缺省 0
        bool isReachable(Node* from, Node* to) const;
        virtual double farematrix(std::vector<Node*> path) const;
        virtual double getFrequency() const { return 0.0; } // 基类给缺省 0

        virtual double getCost(Node* from, Node* to) const;

        void updateFleetStats();

        inline double getFleetDemand() const  { return D_; }
        inline double getFleetVacant() const  { return V_; }

        // 新增一个添加节点的方法
        Node* addNode(const std::string& nodeId);
        Node* getNode(const std::string& nodeId);
        void addLink(Link* link);
        Link* getLink(const std::string& from, const std::string& to);
        std::string getMode() const;
        const std::vector<Node*>& getNodes() const;
        const std::vector<Link*>& getLinks() const { return links; }
        Link* Linkhash(Node* u, Node* v) const;

    protected:
        Network* parentNetwork = nullptr; // 所属网络
        std::string mode; // 标识这一层所属的交通模式或线路
        static int layerNum;
        static int nodeNum;
        static int linkNum;
        std::vector<Node*> nodes;
        std::vector<Link*> links;
        std::unordered_map<Node*, int> nodeIndex;
        std::vector<std::vector<bool>> reachabilityMatrix; // 可达性矩阵
        std::unordered_map<Node*, std::unordered_map<Node*, Link*>> adj;
        double Fleetsize_ = 1000; // 车队规模
        double D_ = 0.0; // 车队需求量
        double V_ = 1.0; // 车队空闲量
};

class TransitLine : public Layer {
    public:
        TransitLine(Network* parent, const std::string& modeName);
        double getFrequency() const;
        void setFrequency(double frequency);
        double getCapacity() const override;
        void setCapacity(double capacity);
        double getCost(Node* from, Node* to) const override;
        double farematrix(std::vector<Node*> path) const override;
    private:
        double frequency;
        double capacity;
};

class WalkLayer : public Layer {
    public:
        WalkLayer(Network* parent, const std::string& modeName);
        Centroid* addCentroid(const std::string& nodeId);
        std::vector<Centroid*> getCentroids() const;
    private:
        std::vector<Centroid*> centroids; // 存储所有的中心点
};