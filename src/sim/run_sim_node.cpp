/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 */

// src/run_sim_node.cpp
#include <ros/ros.h>
#include "ElevatorTrajectory.hpp"
#include "ObjectRelativeTrajectory.hpp"
#include "ObjectWorldTrajectory.hpp"
#include "SimSensorPublisher.hpp"

#include <filesystem>


// int main(int argc, char** argv) {
//     std::string cfg_path = std::string(PACKAGE_ROOT_DIR) + "/src/sim/config.jsonc";
//     if (argc > 1) cfg_path = argv[1];
//
//     // 1) load json
//     nlohmann::json cfg;
//     {
//         std::ifstream f(cfg_path);
//         if (!f.is_open()) {
//             std::cerr << "Failed to open config: " << cfg_path << "\n";
//             return -1;
//         }
//         f >> cfg;
//     }
//
//     // 2) build trajectory
//     ElevatorTrajectory traj;
//     if (!traj.loadConfig(cfg)) {
//         std::cerr << "traj.loadConfig failed.\n";
//         return -1;
//     }
//
//     // 3) export elevator intermediate CSV if configured
//     if (!traj.exportElevatorCSVIfConfigured(cfg)) {
//         std::cerr << "traj.exportElevatorCSVIfConfigured failed.\n";
//         return -1;
//     }
//
//     // 4) quick sanity prints
//     double T = traj.getTotalDuration();
//     std::cout << "Total duration = " << T << " s\n";
//
//     auto print_state = [&](double t) {
//         auto s = traj.getState(t);
//         std::cout << "t=" << s.t
//                   << "  p=" << s.p
//                   << "  v=" << s.v
//                   << "  a=" << s.a << "\n";
//     };
//
//     print_state(0.0);
//     print_state(std::min(1.0, T));
//     print_state(std::min(5.0, T));
//     print_state(std::min(10.0, T));
//     print_state(T);
//
//     return 0;
// }

// int main(int argc, char** argv) {
//     std::string cfg_path = std::string(PACKAGE_ROOT_DIR) + "/src/sim/config.jsonc";
//     if (argc > 1) cfg_path = argv[1];
//
//     nlohmann::json cfg;
//     {
//         std::ifstream f(cfg_path);
//         if (!f.is_open()) {
//             std::cerr << "Failed to open config: " << cfg_path << "\n";
//             return -1;
//         }
//         f >> cfg;
//     }
//
//     ObjectRelativeTrajectory objTraj;
//     if (!objTraj.loadConfig(cfg)) {
//         std::cerr << "ObjectRelativeTrajectory.loadConfig failed.\n";
//         return -1;
//     }
//
//     // optional export
//     if (!objTraj.exportObjectCSVIfConfigured(cfg)) {
//         std::cerr << "export object CSV failed.\n";
//         return -1;
//     }
//
//     double T = objTraj.getTotalDuration();
//     std::cout << "ObjectRel total duration = " << T << " s\n";
//
//     auto quat_to_yaw = [](double w, double x, double y, double z) {
//         double siny_cosp = 2.0 * (w*z + x*y);
//         double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
//         return std::atan2(siny_cosp, cosy_cosp); // rad
//     };
//
//
//     auto print_state = [&](double t) {
//         auto s = objTraj.getState(t);
//         double yaw = quat_to_yaw(s.qw, s.qx, s.qy, s.qz);
//
//         std::cout << "t=" << t
//                   << " p=(" << s.x << "," << s.y << "," << s.z << ")"
//                   << " v=(" << s.vx << "," << s.vy << "," << s.vz << ")"
//                   << " a=(" << s.ax << "," << s.ay << "," << s.az << ")"
//                   << " yaw_deg=" << yaw * 180.0 / 3.14159265358979323846
//                   << " yaw_rate_deg_s=" << s.wz * 180.0 / 3.14159265358979323846
//                   << "\n";
//     };
//
//
//     print_state(0.0);
//     print_state(std::min(6.0, T));
//     print_state(std::min(10.0, T));
//     print_state(T);
//
//     return 0;
// }


// int main(int argc, char** argv) {
//     std::string cfg_path = std::string(PACKAGE_ROOT_DIR) + "/src/sim/config.jsonc";
//     if (argc > 1) cfg_path = argv[1];
//
//     nlohmann::json cfg;
//     {
//         std::ifstream f(cfg_path);
//         if (!f.is_open()) {
//             std::cerr << "Failed to open config: " << cfg_path << "\n";
//             return -1;
//         }
//         f >> cfg;
//     }
//
//     ElevatorTrajectory elev;
//     if (!elev.loadConfig(cfg)) return -1;
//     if (!elev.exportElevatorCSVIfConfigured(cfg)) return -1;
//
//     ObjectRelativeTrajectory objRel;
//     if (!objRel.loadConfig(cfg)) return -1;
//     if (!objRel.exportObjectCSVIfConfigured(cfg)) return -1;
//
//     ObjectWorldTrajectory objWorld;
//     if (!objWorld.loadConfig(cfg)) return -1;
//     if (!objWorld.exportWorldCSVIfConfigured(cfg, elev, objRel)) return -1;
//
//     // quick sanity print
//     double T = std::max(elev.getTotalDuration(), objRel.getTotalDuration());
//     auto s0 = objWorld.getState(0.0, elev, objRel);
//     auto sT = objWorld.getState(T, elev, objRel);
//
//     std::cout << "World first: z=" << s0.z << " qw=" << s0.qw << "\n";
//     std::cout << "World last : z=" << sT.z << " qw=" << sT.qw << "\n";
//
//     return 0;
// }

int main(int argc, char** argv) {
    // ----------------------------------------------------------------
    // 阶段 1: 离线生成轨迹 CSV
    // ----------------------------------------------------------------

    // 获取配置文件路径
    std::string cfg_path = std::string(PACKAGE_ROOT_DIR) + "/src/sim/config.jsonc";
    if (argc > 1) cfg_path = argv[1];

    std::cout << "[Step 1] Loading Configuration: " << cfg_path << "\n";

    nlohmann::json cfg;
    {
        std::ifstream f(cfg_path);
        if (!f.is_open()) {
            std::cerr << "Failed to open config: " << cfg_path << "\n";
            return -1;
        }
        std::string raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        cfg = nlohmann::json::parse(strip_jsonc_comments(raw));
    }

    auto resolve_path = [&](const std::string& p) -> std::string {
        std::filesystem::path path(p);
        if (path.is_absolute()) {
            return path.string();
        }
        std::filesystem::path base = std::filesystem::path(cfg_path).parent_path();
        return (base / path).string();
    };

    auto patch_path = [&](const char* a, const char* b) {
        if (!cfg.contains(a) || !cfg[a].contains(b) || !cfg[a][b].is_string()) return;
        cfg[a][b] = resolve_path(cfg[a][b].get<std::string>());
    };

    patch_path("simulation", "output_file");
    patch_path("elevator", "output_csv");
    patch_path("object", "output_csv");
    patch_path("object", "world_output_csv");

    std::cout << "[Step 2] Generating Trajectories...\n";

    // 生成电梯轨迹
    ElevatorTrajectory elev;
    if (!elev.loadConfig(cfg)) return -1;
    if (!elev.exportElevatorCSVIfConfigured(cfg)) return -1;

    // 生成物体相对轨迹
    ObjectRelativeTrajectory objRel;
    if (!objRel.loadConfig(cfg)) return -1;
    if (!objRel.exportObjectCSVIfConfigured(cfg)) return -1;

    // 生成物体世界系轨迹 (SimSensorNode 将会读取这个 CSV)
    ObjectWorldTrajectory objWorld;
    if (!objWorld.loadConfig(cfg)) return -1;
    if (!objWorld.exportWorldCSVIfConfigured(cfg, elev, objRel)) return -1;

    // 简单打印结果
    double T = std::max(elev.getTotalDuration(), objRel.getTotalDuration());
    auto sT = objWorld.getState(T, elev, objRel);
    std::cout << "Trajectory Generation Complete. Total T=" << T << "s\n";
    std::cout << "--------------------------------------------------------\n";

    // ----------------------------------------------------------------
    // 阶段 2: 启动 ROS 仿真节点
    // ----------------------------------------------------------------

    std::cout << "[Step 3] Starting ROS Simulation Node...\n";

    // 1. 初始化 ROS
    ros::init(argc, argv, "elevator_sim_combined");
    ros::NodeHandle nh("~");

    // 2. 将配置文件路径设置到参数服务器
    //    SimSensorNode 内部会读取这个参数来找到 json，再从中找到 csv 路径
    nh.setParam("config_path", cfg_path);

    // 3. 实例化仿真节点 (类定义在 SimSensorPublisher.hpp 中)
    //    构造函数执行时会自动加载 CSV
    SimSensorNode simNode(nh);

    ROS_INFO("Simulation running. Use Rviz to visualize.");

    // 4. 循环等待回调
    ros::spin();

    return 0;
}
