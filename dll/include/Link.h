#pragma once
#include "head.h"

class Node;
class Link;
class LinkCreator;
class Rlink;
class Tlink;
class Wlink;
class ATlink;
class ARlink;
class ARHlink;
class Elink;

class multiNet_API Link {
    public:
        Link(int id, Node* head, Node* tail, double length);
    
        int getId() const;
        Node* getHead() const;
        Node* getTail() const;
        void setCapacity(double capacity);
        void setVolume(double volume);
        void setHeterVolume(const std::string& type, double volume);
        virtual double getLength() const;
        virtual double getToll() const;
        virtual double getCost() const;
        double getVolume() const;
        double getHeterVolume(const std::string& type) const;
        double getCapacity() const;
        double getShadowPrice() const;
        void updateShadowPrice(double new_price);
        virtual double getVacancy(double volume_ = -1) const {
            if (capacity > 0) {
                return capacity - (volume_ == -1 ? volume : volume_);
            }
            return 0.0;
        }
        virtual void setBackgroundVolume(double volume) {} // 仅 Rlink 使用
        virtual double getBackgroundVolume() const { return 0.0; } // 仅 Rlink 使用
        virtual void setcorrelink(Rlink* correlink) {}
        virtual double getLengthDerivative() const;

    protected:
        int id;
        Node* head;
        Node* tail;
        double volume;
        std::unordered_map<string, double> heter_volume;
        double capacity;
        double length;
        double toll_;
        double shadow_price = 0.0; // Shadow price for capacity constraints
        void* buffer;
};

inline int Link::getId() const {
    return id;
}

inline Node* Link::getHead() const {
    return head;
}

inline Node* Link::getTail() const {
    return tail;
}

inline double Link::getLength() const {
    return length;
}

inline double Link::getToll() const {
    return toll_;
}

inline double Link::getVolume() const {
    return volume;
}

inline double Link::getHeterVolume(const std::string& type) const {
    auto it = heter_volume.find(type);
    return (it != heter_volume.end()) ? it->second : 0.0;
}

inline void Link::setCapacity(double capacity) {
    this->capacity = capacity;
}

inline void Link::setVolume(double volume) {
    this->volume = volume;
}

inline void Link::setHeterVolume(const std::string& type, double v) {
    heter_volume[type] = v;
    /* 重新累计 —— 若性能敏感可改成增量加减 */
    volume = 0.0;
    for (auto& kv : heter_volume) volume += kv.second;
}

inline double Link::getCapacity() const {
    return capacity;
}

inline double Link::getShadowPrice() const {
    return shadow_price;
}

inline void Link::updateShadowPrice(double new_price) {
    shadow_price = new_price;
}

inline double Link::getLengthDerivative() const {
    return 0.0;  // Default implementation (can be overridden)
}

class LinkCreator {
    public:
        virtual ~LinkCreator() {}
        virtual Link* create(int id, Node* head, Node* tail, double length, const std::vector<std::string>& params) const = 0;
};

class Rlink : public Link {
public:
    Rlink(int id, Node* head, Node* tail, double length, double capacity, double toll_ = 0.0);
    // BPR function
    double getLength() const override;
    double getToll() const override;
    void setBackgroundVolume(double volume) override;
    double getLengthDerivative() const override;
    double getBackgroundVolume() const override { return background_volume; }
    void setcorrelink(Rlink* correlink) override { this->correlink = correlink; }
    double getVacancy(double volume_ = -1) const override {
        if (capacity > 0) {
            return capacity - (volume_ == -1 ? volume : volume_) - background_volume - (correlink ? correlink->getVolume() : 0.0);
        }
        return 0.0; // No vacancy if capacity is zero
    }
protected:
    double background_volume = 0.0; // Background traffic volume
    Rlink* correlink = nullptr; // RH traffic volume
};

class Tlink : public Link {
public:
    Tlink(int id, Node* head, Node* tail, double length);
    double getToll() const override;
    double getVacancy(double volume_ = -1) const override;
};

class Wlink : public Link {
public:
    Wlink(int id, Node* head, Node* tail, double length);
    double volume_bike = 0.0;
    double getToll() const override;
    double getLength() const override;
};

class ATlink : public Link {
public:
    ATlink(int id, Node* head, Node* tail, double length, std::string line_id, double frequency);
    std::string getLineId() const;
    double getLength() const override;
    double getLengthDerivative() const override;
    std::string line_id;
    double frequency;
};

class ARlink : public Link {
public:
    ARlink(int id, Node* head, Node* tail, double length);
};

class ARHlink : public Link {
public:
    ARHlink(int id, Node* head, Node* tail, double length);
    double getLength() const override;
    double getLengthDerivative() const override;
};

class Elink : public Link {
public:
    Elink(int id, Node* head, Node* tail, double length);
};

class RlinkCreator : public LinkCreator {
public:
    Link* create(int id, Node* head, Node* tail, double length, const std::vector<std::string>& params) const override;
};

class TlinkCreator : public LinkCreator {
public:
    Link* create(int id, Node* head, Node* tail, double length, const std::vector<std::string>& params) const override;
};

class WlinkCreator : public LinkCreator {
public:
    Link* create(int id, Node* head, Node* tail, double length, const std::vector<std::string>& params) const override;
};

class ATlinkCreator : public LinkCreator {
public:
    Link* create(int id, Node* head, Node* tail, double length, const std::vector<std::string>& params) const override;
};

class ARlinkCreator : public LinkCreator {
public:
    Link* create(int id, Node* head, Node* tail, double length, const std::vector<std::string>& params) const override;
};

class ARHlinkCreator : public LinkCreator {
public:
    Link* create(int id, Node* head, Node* tail, double length, const std::vector<std::string>& params) const override;
};

class ElinkCreator : public LinkCreator {
public:
    Link* create(int id, Node* head, Node* tail, double length, const std::vector<std::string>& params) const override;
};

class LinkGenerator {
    public:
        static LinkGenerator& getInstance();
    
        void registerLinkType(const std::string& type, LinkCreator* creator);
    
        Link* generateLink(const std::string& type, int id, Node* head, Node* tail, double length, const std::vector<std::string>& params);
    
    private:
        LinkGenerator() {}
        std::unordered_map<std::string, LinkCreator*> creators;
};