#include "Network.h"
#include "Node.h"
#include "Link.h"
#include "Layer.h"
#include "Solver.h"
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef byte
#include <cstdint>
#include <iostream>
constexpr double aTrips = 5.0;
constexpr double bTrips = 80.0;
constexpr double mTrips = 45.6;  // = c
// 调用时传入已创建好的 Network 和 AssignmentSolver 对象指针
using OKey = std::pair<Node*, Node*>;
double consumer_surplus(OKey key, double C, double mean_travel_time)
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

void loadTauDeltaFromFile(const std::string& filepath, Network* net, AssignmentSolver* solver) {
    std::ifstream in("best_params.txt");
    std::string tag;
    if (in >> tag && tag == "Tau") {
        solver->bundle_price.clear();
        for (int i = 0; i < 4 && in; ++i) {
            double v; in >> v;
            solver->bundle_price.push_back(v);
        }
    }

    /* ---------- Delta ---------- */
    if (in >> tag && tag == "Delta") {
        std::vector<double> d(12);
        for (double& v : d) in >> v;     // 读取 12 个数

        const std::vector<std::string> usr = {"B1","B2","B3","B4"};
        std::unordered_map<std::string,double> bike, rh, pt;

        for (int i = 0; i < 4; ++i) {
            bike[usr[i]] = d[i*3 + 0];   // B1: d0, d1, d2   …
            rh  [usr[i]] = d[i*3 + 1];
            pt  [usr[i]] = d[i*3 + 2];
        }

        net->setHeterTollBike(bike);
        net->setHeterTollRH(rh);
        net->setHeterTollPT(pt);
    }
}

void deleteTxtFiles(const std::string& dir)
{
    std::string pattern = dir + "\\*.txt";

    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA(pattern.c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) {
        std::cerr << "目录不存在或无 .txt 文件\n";
        return;
    }

    do {
        std::string filePath = dir + "\\" + fd.cFileName;
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
            if (!DeleteFileA(filePath.c_str()))
                std::cerr << "删除失败: " << filePath << '\n';
        }
    } while (FindNextFileA(h, &fd));
    FindClose(h);
}
void printBundleSurplusAndSubsidy(Network& net, AssignmentSolver& solver) {
    std::vector<double> avgSurplus(4, 0.0), avgSubsidy(4, 0.0), bundle_adoption(4, 0.0);
    for (const auto& kv : solver.dem__.table()) {
        const auto& key = kv.first;
        double q_total = std::accumulate(kv.second.begin(), kv.second.end(), 0.0);
        std::vector<double> pathC;
        double q = 0.0;
        for (int b = 0; b < 4; ++b) {
            bundle_adoption[b] += kv.second[b];
            double q_1 = q + kv.second[b];
            double x_1 = user_config::inverseCDF(q / q_total);  // 原比例
            double x_2 = user_config::inverseCDF(q_1 / q_total);  // 新比例
            q = q_1;
            net.setCurrentUser("B" + std::to_string(b + 1));
            double Cb = net.shortestPath(key.first, key.second, "Dijkstra", "Layer", 2).cost;
            pathC.push_back(Cb);
            avgSurplus[b] += consumer_surplus(key, Cb, 1) * kv.second[b];
            avgSubsidy[b] += (pathC[0] - pathC[b]) * kv.second[b]; // 假设补贴等于成本
        }
    }
    for (int b = 0; b < 4; ++b) {
        if (bundle_adoption[b] > 0) {
            std::cout << "Bundle " << b + 1 << ": "
                      << "人均剩余 = " << avgSurplus[b] / bundle_adoption[b]
                      << ", 人均补贴 = " << avgSubsidy[b] / bundle_adoption[b] << std::endl;
        } else {
            std::cout << "Bundle " << b + 1 << ": "
                      << "人均剩余 = 0, 人均补贴 = 0" << std::endl;
        }
    }
    double totalSurplus = std::accumulate(avgSurplus.begin(), avgSurplus.end(), 0.0);
    double totalSubsidy = std::accumulate(avgSubsidy.begin(), avgSubsidy.end(), 0.0);
    std::ofstream File("D:\\MyCode\\C++\\multi_net\\result\\sensitivity.txt", std::ios::app);
    File << totalSurplus << " " << totalSubsidy << std::endl;
    File.close();
    double totalAdoption = std::accumulate(bundle_adoption.begin(), bundle_adoption.end(), 0.0);
    if (totalAdoption > 0) {
        std::cout << "总人均剩余 = " << totalSurplus / totalAdoption
                  << ", 总人均补贴 = " << totalSubsidy / totalAdoption << std::endl;
    } else {
        std::cout << "总人均剩余 = 0, 总人均补贴 = 0" << std::endl;
    }
}

int main() {
    std::cerr << "Starting program..." << std::endl;
    // 删除结果目录下的所有 .txt 文件
    deleteTxtFiles("D:\\MyCode\\C++\\multi_net\\result");

    Network net;
    // net.createNetwork("winnipeg");
    net.createNetwork("siouxfalls");
    std::vector<std::string> heter = {"B1", "B2", "B3", "B4"};
    net.setHeterUser(heter);
    // net.setHeterVOT({{"B1", 0.10}, {"B2", 0.10}, {"B3", 0.10}, {"B4", 0.10}});
    net.setHeterVOT({{"B1", 0.15}, {"B2", 0.15}, {"B3", 0.15}, {"B4", 0.15}});
    // net.setHeterTollBike({{"B1", 1}, {"B2", 0.767886}, {"B3", 0}, {"B4", 0}});
    // net.setHeterTollRH({{"B1", 1}, {"B2", 0.804712}, {"B3", 0.747172}, {"B4", 0.652242}});
    // net.setHeterTollPT({{"B1", 1}, {"B2", 0.738492}, {"B3", 0.455068}, {"B4", 0.0}});

    net.setHeterTollBike({{"B1", 1}, {"B2", 0.5}, {"B3", 0}, {"B4", 0}});
    net.setHeterTollRH({{"B1", 1}, {"B2", 0.8}, {"B3", 0.5}, {"B4", 0.3}});
    net.setHeterTollPT({{"B1", 1}, {"B2", 0.5}, {"B3", 0.3}, {"B4", 0}});
    DemandStore dem;
    // dem.loadTripFile("winnipeg/trip.txt", &net, 4);
    dem.loadTripFile("siouxfalls/trip.txt", &net, 4);

    AssignmentSolver solver(&net, &dem);
    // loadTauDeltaFromFile("best_params.txt", &net, &solver);
    // solver.bundle_price = {0, 24.1925, 49.468, 141.725};
    // solver.bundle_price = {0, 0, 0, 0};
    solver.bundle_price = {0, 30, 80, 200};
    // solver.bundle_price = {0, 2000, 2000, 2000};
    // solver.solveBundleFW(100);
    // solver.resetNetwork(0);
    // solver.solveBundleFW(100);
    // solver.resetNetwork(0);
    // solver.solveBundleFW(100);
    // solver.resetNetwork(1);
    solver.solveBundleFW(50);
    // solver.resetNetwork(0);
    // solver.solveBundleFW(100);
    // solver.resetNetwork(0);
    // solver.solveBundleFW(100);
    // solver.getModeChoice();
    // solver.printMuCosts();
    // 统计路网拥堵数据
    // net.getCongestionData();
    // printBundleSurplusAndSubsidy(net, solver);
    // // 计算并打印唯一性比值
    // double uniqueness_ratio = solver.calculateUniquenessRatio();
    // std::cout << "Uniqueness Condition Ratio for this scenario: " 
    //         << uniqueness_ratio << std::endl;
    // 灵敏度分析
    // std::vector<std::string> modified_lines;
    // for (int i = 0; i < 10; ++i) {
    //     // std::vector<std::string> modified_lines;
    //     net.modifyCapacity(1.3, modified_lines); // 恢复原始容量
    //     solver.resetNetwork(1);
    //     solver.solveBundleFW(100);
    //     solver.resetNetwork(0);
    //     solver.solveBundleFW(100);
    //     solver.resetNetwork(0);
    //     solver.solveBundleFW(100);
    //     solver.getModeChoice();
    //     printBundleSurplusAndSubsidy(net, solver);
    // }
    // 扰动实验，多次更换初始解
    // for (int run = 1; run < 31; ++run) {
    //     std::cout << "----- Run " << run + 1 << " -----" << std::endl;
    //     double ratio = run / 30.0;
    //     net.setHeterTollRH({{"B1", 1}, {"B2", 1.0 * ratio}, {"B3", 0.9 * ratio}, {"B4", 0.8 * ratio}});
    //     net.setHeterTollPT({{"B1", 1}, {"B2", 0.7 * ratio}, {"B3", 0.6 * ratio}, {"B4", 0.5 * ratio}});
    //     net.setHeterTollBike({{"B1", 1}, {"B2", 0.4 * ratio}, {"B3", 0.3 * ratio}, {"B4", 0.2 * ratio}});
    //     solver.bundle_price = {0, 30 * ratio, 80 * ratio, 200 * ratio};
    //     solver.resetNetwork(1);
    //     solver.solveBundleFW(10,0.001);
    //     loadTauDeltaFromFile("best_params.txt", &net, &solver);
    //     solver.resetNetwork(0);
    //     solver.solveBundleFW(500,0.00001);
    //     solver.getModeChoice();
    //     printBundleSurplusAndSubsidy(net, solver);
    // }
}

