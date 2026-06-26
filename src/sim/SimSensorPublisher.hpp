/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 */

/**
 * @file SimSensorPublisher.hpp
 * @brief 仿真传感器发布类的定义（无 main 函数，供外部调用）
 */

#pragma once // 防止头文件重复包含

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <random>
#include <cmath>
#include <filesystem>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static std::string strip_jsonc_comments(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    bool in_string = false;
    bool escape = false;
    for (size_t i = 0; i < s.size(); ++i) {
        char c = s[i];
        if (in_string) {
            out.push_back(c);
            if (escape) {
                escape = false;
            } else if (c == '\\') {
                escape = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }
        if (c == '"') {
            in_string = true;
            out.push_back(c);
            continue;
        }
        if (c == '/' && i + 1 < s.size()) {
            char n = s[i + 1];
            if (n == '/') {
                i += 2;
                while (i < s.size() && s[i] != '\n') ++i;
                if (i < s.size()) out.push_back('\n');
                continue;
            }
            if (n == '*') {
                i += 2;
                while (i + 1 < s.size() && !(s[i] == '*' && s[i + 1] == '/')) {
                    if (s[i] == '\n') out.push_back('\n');
                    ++i;
                }
                ++i;
                continue;
            }
        }
        out.push_back(c);
    }
    return out;
}

// ---------------------------------------------------------
// CSV 解析与状态结构体
// ---------------------------------------------------------

/**
 * @brief 定义某一时刻的仿真状态
 */
struct SimState {
    double t; // 时间戳

    // --- 世界坐标系 (World Frame) 下的运动学参数 ---
    Eigen::Vector3d p_w;    // 位置
    Eigen::Vector3d v_w;    // 速度
    Eigen::Vector3d a_w;    // 加速度
    Eigen::Quaterniond q_wb; // 姿态四元数

    // --- 机体坐标系 (Body Frame) 下的角速度 ---
    Eigen::Vector3d w_b;     // 角速度
    Eigen::Vector3d alpha_b; // 角加速度

    // --- 相对坐标系 (相对于电梯厢) ---
    Eigen::Vector3d p_rel;   // 物体相对于电梯系的位置
    double elev_z;           // 电梯在世界系下的当前高度
};

/**
 * @brief 数据播放器类
 */
class DataPlayer {
public:
    std::vector<SimState> states;

    bool loadCSV(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            ROS_ERROR_STREAM("Could not open CSV: " << path);
            return false;
        }
        std::string line;
        std::getline(file, line); // Skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<double> vals;
            while (std::getline(ss, cell, ',')) {
                vals.push_back(std::stod(cell));
            }

            if (vals.size() < 26) continue;

            SimState s;
            // vals: timestamp,w_x,w_y,w_z,w_vx,w_vy,w_vz,w_ax,w_ay,w_az,qw,qx,qy,qz,wx,wy,wz,alphax,alphay,alphaz,elev_z,elev_vz,elev_az,rel_x,rel_y,rel_z
            //          0       1   2   3   4    5    6    7    8    9   10 11 12 13 14 15 16   17     18     19     20     21      22     23    24    25
            s.t = vals[0];
            s.p_w = Eigen::Vector3d(vals[1], vals[2], vals[3]);
            s.v_w = Eigen::Vector3d(vals[4], vals[5], vals[6]);
            s.a_w = Eigen::Vector3d(vals[7], vals[8], vals[9]);
            s.q_wb = Eigen::Quaterniond(vals[10], vals[11], vals[12], vals[13]).normalized();
            s.w_b = Eigen::Vector3d(vals[14], vals[15], vals[16]);
            s.alpha_b = Eigen::Vector3d(vals[17], vals[18], vals[19]);
            s.elev_z = vals[20];
            s.p_rel = Eigen::Vector3d(vals[23], vals[24], vals[25]);

            states.push_back(s);
        }
        ROS_INFO_STREAM("Loaded " << states.size() << " trajectory points.");
        return !states.empty();
    }

    /**
     * @brief 根据给定时间 t 获取离散系统状态（不做插值）
     * @param t 目标时间戳
     * @return SimState 对应时刻的离散状态
     */
    SimState getState(double t) {
        if (states.empty()) return SimState();
        if (t <= states.front().t) return states.front();
        if (t >= states.back().t) return states.back();

        auto it = std::lower_bound(states.begin(), states.end(), t,
            [](const SimState& s, double val) { return s.t < val; });

        if (it == states.begin()) return states.front();
        if (it == states.end()) return states.back();
        if (it->t == t) return *it;

        const SimState& s2 = *it;
        const SimState& s1 = *(it - 1);

        double dt1 = std::abs(t - s1.t);
        double dt2 = std::abs(s2.t - t);
        double eps = 1e-4;
        if (dt1 <= eps || dt2 <= eps) {
            return (dt1 <= dt2) ? s1 : s2;
        }
        return (dt1 <= dt2) ? s1 : s2;
    }
};

// ---------------------------------------------------------
// 仿真节点主类
// ---------------------------------------------------------
class SimSensorNode {
public:
    // 构造函数
    SimSensorNode(ros::NodeHandle& nh) : nh_(nh) {
        // 1. 获取 config_path 参数 (由 run_sim_node 的 main 设置)
        std::string config_path;
        if (!nh_.getParam("config_path", config_path)) {
            ROS_WARN("Param 'config_path' not found, using default 'config.jsonc'");
            config_path = "config.jsonc";
        }
        loadConfig(config_path);

        // 2. 加载 CSV (由 loadConfig 获取路径)
        if (!player_.loadCSV(csv_path_)) {
            ROS_ERROR("Failed to load CSV data. Simulation will not publish.");
            return;
        }

        // 3. 设置发布者
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 200);
        lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(lidar_topic_, 20);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/ground_truth/odom", 200);
        elevator_flag_pub_ = nh_.advertise<std_msgs::Bool>(elevator_flag_topic_, 1, false);

        prepareElevatorFlagSchedule();
        prepareGravityDriftSchedule();

        // 4. 定时器
        imu_timer_ = nh_.createTimer(ros::Duration(1.0/imu_rate_), &SimSensorNode::imuCallback, this);
        lidar_timer_ = nh_.createTimer(ros::Duration(1.0/lidar_rate_), &SimSensorNode::lidarCallback, this);

        start_wall_time_ = ros::WallTime::now().toSec();
        total_duration_ = player_.states.empty() ? 0 : player_.states.back().t;
        if (initial_gravity_misalignment_enable_) {
            total_duration_ += initial_gravity_misalignment_duration_s_;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_, lidar_pub_, odom_pub_, elevator_flag_pub_;
    ros::Timer imu_timer_, lidar_timer_;
    tf::TransformBroadcaster tf_br_;

    DataPlayer player_;
    double start_wall_time_;
    double total_duration_;
    bool finished_ = false;

    // 配置参数
    std::string csv_path_;
    double imu_rate_;
    std::string imu_topic_;
    double acc_noise_, gyro_noise_;
    Eigen::Vector3d gravity_base_w_;
    Eigen::Vector3d gravity_w_;
    double lidar_rate_;
    std::string lidar_topic_, imu_frame_, lidar_frame_;
    double lidar_range_noise_, lidar_angle_noise_rad_, lidar_min_, lidar_max_;
    int lidar_points_, lidar_beams_;
    double lidar_fov_;
    double box_w_, box_d_, box_h_;
    std::string elevator_flag_topic_;
    bool elevator_flag_value_ = true;
    double elevator_flag_lead_time_s_ = 0.0;
    std::vector<double> elevator_flag_times_;
    size_t elevator_flag_index_ = 0;
    double tilt_pitch_deg_ = 0.0;
    Eigen::Quaterniond q_wc_{1.0, 0.0, 0.0, 0.0}; // cabin frame C -> world W
    Eigen::Quaterniond q_cw_{1.0, 0.0, 0.0, 0.0}; // world W -> cabin frame C
    bool time_varying_gravity_drift_enable_ = false;
    double gravity_drift_step_deg_ = 0.0;
    std::vector<double> gravity_drift_times_;
    size_t gravity_drift_index_ = 0;
    Eigen::Quaterniond gravity_drift_q_{1.0, 0.0, 0.0, 0.0};
    bool initial_gravity_misalignment_enable_ = false;
    double initial_gravity_misalignment_duration_s_ = 0.0;
    double initial_gravity_misalignment_angle_deg_ = 0.0;
    double initial_gravity_misalignment_azimuth_deg_ = 0.0;
    bool initial_gravity_misalignment_random_direction_ = false;
    Eigen::Vector3d initial_misaligned_gravity_w_{0.0, 0.0, -9.81};
    unsigned int noise_seed_ = 0;

    std::default_random_engine rng_;
    std::normal_distribution<double> norm_dist_{0.0, 1.0};
    std::uniform_real_distribution<double> uni_azimuth_dist_{-M_PI, M_PI};

    void loadConfig(const std::string& path) {
        std::ifstream f(path);
        if(!f.is_open()) {
             ROS_ERROR_STREAM("Cannot open config: " << path);
             return;
        }
        std::string raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        json j = json::parse(strip_jsonc_comments(raw));

        if (j.contains("object") && j["object"].contains("world_output_csv")) {
            std::string raw = j["object"]["world_output_csv"].get<std::string>();
            std::filesystem::path p(raw);
            if (p.is_absolute()) {
                csv_path_ = p.string();
            } else {
                std::filesystem::path base = std::filesystem::path(path).parent_path();
                csv_path_ = (base / p).string();
            }
        }

        tilt_pitch_deg_ = 0.0;
        time_varying_gravity_drift_enable_ = false;
        gravity_drift_step_deg_ = 0.0;
        initial_gravity_misalignment_enable_ = false;
        initial_gravity_misalignment_duration_s_ = 0.0;
        initial_gravity_misalignment_angle_deg_ = 0.0;
        initial_gravity_misalignment_azimuth_deg_ = 0.0;
        initial_gravity_misalignment_random_direction_ = false;
        if (j.contains("perturbations")) {
            const auto& p = j["perturbations"];
            if (p.contains("cabin_inclination_error")) {
                const auto& c = p["cabin_inclination_error"];
                tilt_pitch_deg_ = c.value("pitch_deg", 0.0);
            }
            if (p.contains("time_varying_gravity_drift")) {
                const auto& d = p["time_varying_gravity_drift"];
                time_varying_gravity_drift_enable_ = d.value("enable", false);
                gravity_drift_step_deg_ = d.value("step_deg", 0.0);
            }
            if (p.contains("initial_gravity_misalignment")) {
                const auto& gmis = p["initial_gravity_misalignment"];
                initial_gravity_misalignment_enable_ = gmis.value("enable", false);
                initial_gravity_misalignment_duration_s_ = gmis.value("duration_s", 1.0);
                initial_gravity_misalignment_angle_deg_ = gmis.value("angle_deg", 0.0);
                initial_gravity_misalignment_azimuth_deg_ = gmis.value("azimuth_deg", 0.0);
                initial_gravity_misalignment_random_direction_ = gmis.value("random_direction", false);
            }
        }
        if (j.contains("elevator")) {
            const auto& e = j["elevator"];
            if (!(j.contains("perturbations") && j["perturbations"].contains("cabin_inclination_error"))) {
                tilt_pitch_deg_ = e.value("tilt_pitch_deg", 0.0);
            }
        }
        double pitch_rad = tilt_pitch_deg_ * (M_PI / 180.0);
        q_wc_ = Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY());
        q_cw_ = q_wc_.conjugate();

        if (j.contains("simulation")) {
            const auto& sim = j["simulation"];
            noise_seed_ = sim.value("seed", 0u);
        } else {
            noise_seed_ = 0u;
        }
        if (j.contains("sensors")) {
            const auto& sensors = j["sensors"];
            noise_seed_ = sensors.value("seed", noise_seed_);
        }
        rng_.seed(noise_seed_);
        ROS_INFO_STREAM("Simulation noise seed: " << noise_seed_);

        auto& imu = j["sensors"]["imu"];
        imu_rate_ = imu.value("rate", 200.0);
        imu_topic_ = imu.value("topic", "/sim/imu");
        imu_frame_ = imu.value("frame_id", "sim_imu");
        acc_noise_ = imu.value("acc_noise_std", 0.01);
        gyro_noise_ = imu.value("gyro_noise_std", 0.001);
        auto g = imu["gravity_w"];
        gravity_base_w_ = Eigen::Vector3d(g[0], g[1], g[2]);
        gravity_w_ = gravity_base_w_;

        auto& lid = j["sensors"]["lidar"];
        lidar_rate_ = lid.value("rate", 10.0);
        lidar_topic_ = lid.value("topic", "/sim/lidar");
        lidar_frame_ = lid.value("frame_id", "sim_lidar");
        lidar_points_ = lid.value("num_points", 720);
        lidar_beams_ = lid.value("vertical_beams", 16);
        lidar_fov_ = lid.value("vertical_fov_deg", 30.0) * M_PI / 180.0;
        lidar_min_ = lid.value("range_min", 0.1);
        lidar_max_ = lid.value("range_max", 30.0);
        lidar_range_noise_ = lid.value("range_noise_std", lid.value("noise_std", 0.01));
        if (lid.contains("angle_noise_std_rad")) {
            lidar_angle_noise_rad_ = lid["angle_noise_std_rad"].get<double>();
        } else {
            lidar_angle_noise_rad_ = lid.value("angle_noise_std_deg", 0.0) * M_PI / 180.0;
        }

        if (j.contains("sensors") && j["sensors"].contains("elevator_flag")) {
            const auto& flag = j["sensors"]["elevator_flag"];
            elevator_flag_topic_ = flag.value("topic", "/LIO/set_elevator_flag");
            elevator_flag_value_ = flag.value("value", true);
            elevator_flag_lead_time_s_ = flag.value("lead_time_s", 0.0);
        } else {
            elevator_flag_topic_ = "/LIO/set_elevator_flag";
            elevator_flag_value_ = true;
            elevator_flag_lead_time_s_ = 0.0;
        }

        auto& dim = j["elevator"]["dimensions"];
        box_w_ = dim.value("width", 2.0);
        box_d_ = dim.value("depth", 2.0);
        box_h_ = dim.value("height", 2.5);

        gravity_drift_q_.setIdentity();
        gravity_w_ = gravity_base_w_;
        double init_azimuth_deg = initial_gravity_misalignment_azimuth_deg_;
        if (initial_gravity_misalignment_random_direction_) {
            init_azimuth_deg = uni_azimuth_dist_(rng_) * 180.0 / M_PI;
        }
        initial_misaligned_gravity_w_ =
            buildGravityTiltQuaternion(initial_gravity_misalignment_angle_deg_, init_azimuth_deg) * gravity_base_w_;

        ROS_INFO_STREAM("Cabin inclination pitch_deg: " << tilt_pitch_deg_);
        ROS_INFO_STREAM("Time-varying gravity drift: enable=" << (time_varying_gravity_drift_enable_ ? "true" : "false")
                         << " step_deg=" << gravity_drift_step_deg_);
        ROS_INFO_STREAM("Initial gravity misalignment: enable=" << (initial_gravity_misalignment_enable_ ? "true" : "false")
                         << " duration_s=" << initial_gravity_misalignment_duration_s_
                         << " angle_deg=" << initial_gravity_misalignment_angle_deg_);
    }

    void prepareElevatorFlagSchedule() {
        elevator_flag_times_.clear();
        elevator_flag_index_ = 0;
        if (player_.states.size() < 2) return;

        const double z_eps = 1e-6;
        bool moving_prev = std::abs(player_.states[1].elev_z - player_.states[0].elev_z) > z_eps;
        for (size_t i = 1; i < player_.states.size(); ++i) {
            double dz = player_.states[i].elev_z - player_.states[i - 1].elev_z;
            bool moving_now = std::abs(dz) > z_eps;
            if (!moving_prev && moving_now) {
                double trigger_time = player_.states[i].t - elevator_flag_lead_time_s_;
                if (trigger_time < 0.0) trigger_time = 0.0;
                elevator_flag_times_.push_back(trigger_time);
            }
            moving_prev = moving_now;
        }

        // 若全程无移动（仅 WAIT），设置一个很大的触发时间，避免误触发
        if (elevator_flag_times_.empty()) {
            elevator_flag_times_.push_back(1e12);
        }
    }

    Eigen::Quaterniond buildGravityTiltQuaternion(double angle_deg, double azimuth_deg) const {
        if (std::abs(angle_deg) < 1e-9) {
            return Eigen::Quaterniond::Identity();
        }
        double azimuth_rad = azimuth_deg * M_PI / 180.0;
        Eigen::Vector3d axis(std::sin(azimuth_rad), -std::cos(azimuth_rad), 0.0);
        if (axis.norm() < 1e-9) {
            axis = Eigen::Vector3d::UnitY();
        } else {
            axis.normalize();
        }
        return Eigen::Quaterniond(Eigen::AngleAxisd(angle_deg * M_PI / 180.0, axis));
    }

    void prepareGravityDriftSchedule() {
        gravity_drift_times_.clear();
        gravity_drift_index_ = 0;
        if (!time_varying_gravity_drift_enable_ || gravity_drift_step_deg_ <= 0.0 || player_.states.size() < 2) {
            return;
        }

        const double z_eps = 1e-6;
        bool moving_prev = std::abs(player_.states[1].elev_z - player_.states[0].elev_z) > z_eps;
        for (size_t i = 1; i < player_.states.size(); ++i) {
            double dz = player_.states[i].elev_z - player_.states[i - 1].elev_z;
            bool moving_now = std::abs(dz) > z_eps;
            if (moving_prev && !moving_now) {
                gravity_drift_times_.push_back(player_.states[i].t);
            }
            moving_prev = moving_now;
        }
    }

    void applyGravityDriftIfDue(double sim_t) {
        if (!time_varying_gravity_drift_enable_ || gravity_drift_step_deg_ <= 0.0) {
            gravity_w_ = gravity_base_w_;
            return;
        }

        while (gravity_drift_index_ < gravity_drift_times_.size() &&
               sim_t >= gravity_drift_times_[gravity_drift_index_]) {
            double azimuth_deg = uni_azimuth_dist_(rng_) * 180.0 / M_PI;
            Eigen::Quaterniond step_q = buildGravityTiltQuaternion(gravity_drift_step_deg_, azimuth_deg);
            gravity_drift_q_ = (step_q * gravity_drift_q_).normalized();
            gravity_w_ = gravity_drift_q_ * gravity_base_w_;
            gravity_drift_index_++;
            ROS_INFO_STREAM("Applied gravity drift step #" << gravity_drift_index_
                             << " at sim_t=" << sim_t
                             << " angle_deg=" << gravity_drift_step_deg_
                             << " azimuth_deg=" << azimuth_deg
                             << " gravity_w=[" << gravity_w_.x() << ", " << gravity_w_.y() << ", " << gravity_w_.z() << "]");
        }
    }

    double getElapsedTime() const {
        return ros::WallTime::now().toSec() - start_wall_time_;
    }

    bool inInitialGravityWarmup(double elapsed) const {
        return initial_gravity_misalignment_enable_ && elapsed < initial_gravity_misalignment_duration_s_;
    }

    double getTrajectoryTime(double elapsed) const {
        if (!initial_gravity_misalignment_enable_) {
            return elapsed;
        }
        return std::max(0.0, elapsed - initial_gravity_misalignment_duration_s_);
    }

    void publishElevatorFlagIfDue(double t) {
        if (elevator_flag_index_ >= elevator_flag_times_.size()) return;
        if (t < elevator_flag_times_[elevator_flag_index_]) return;

        std_msgs::Bool flag_msg;
        flag_msg.data = elevator_flag_value_;
        elevator_flag_pub_.publish(flag_msg);
        elevator_flag_index_++;
    }

    /**
     * @brief IMU 数据发布回调函数
     * @param event ROS 定时器事件（通常由 createTimer 触发，用于控制发布频率，如 200Hz）
     */
    void imuCallback(const ros::TimerEvent&) {
        // 1. 安全检查：如果没有仿真状态数据，直接返回，避免崩溃
        if(player_.states.empty()) return;

        // 2. 获取当前时刻的真实状态 (Ground Truth)
        double elapsed = getElapsedTime();
        bool warmup = inInitialGravityWarmup(elapsed);
        double sim_t = getTrajectoryTime(elapsed);
        if (!warmup) {
            applyGravityDriftIfDue(sim_t);
        }
        SimState s = player_.getState(sim_t);
        ros::Time stamp(elapsed);
        if (!warmup) {
            publishElevatorFlagIfDue(sim_t);
        }

        // 3. 初始化 ROS IMU 消息
        sensor_msgs::Imu msg;
        msg.header.stamp = stamp; // 使用 CSV 中的仿真时间作为时间戳
        msg.header.frame_id = imu_frame_;         // 设置坐标系 ID，对应 TF 树中的 imu link

        // 4. 模拟陀螺仪 (Gyroscope) 数据
        // 逻辑：真实机体角速度 (w_b) + 高斯白噪声
        // norm_dist_(rng_) 用于生成标准正态分布随机数，乘以 gyro_noise_ 得到噪声幅值
        msg.angular_velocity.x = s.w_b.x() + norm_dist_(rng_) * gyro_noise_;
        msg.angular_velocity.y = s.w_b.y() + norm_dist_(rng_) * gyro_noise_;
        msg.angular_velocity.z = s.w_b.z() + norm_dist_(rng_) * gyro_noise_;

        // 5. 模拟加速度计 (Accelerometer) 数据 [关键物理逻辑]
        // 步骤 A: 计算世界坐标系下的“比力” (Specific Force) / 固有加速度
        // 加速度计测量的是物体受到的合外力（不包含引力），而不是运动学加速度。
        // 公式: a_sensor = a_kinematic - g
        // 例如：静止在桌面上，运动加速度为0，但加速度计显示 +9.8m/s² (抵抗重力)
        const Eigen::Vector3d gravity_for_imu_w = warmup ? initial_misaligned_gravity_w_ : gravity_w_;
        Eigen::Vector3d a_proper_w = s.a_w - gravity_for_imu_w;

        // 步骤 B: 将世界坐标系的加速度旋转到机体坐标系 (Body Frame)
        // q_wb 表示从 Body 到 World 的旋转，因此使用 inverse() 将向量从 World 转回 Body
        Eigen::Vector3d a_body = s.q_wb.inverse() * a_proper_w;

        // 步骤 C: 叠加加速度计噪声
        msg.linear_acceleration.x = a_body.x() + norm_dist_(rng_) * acc_noise_;
        msg.linear_acceleration.y = a_body.y() + norm_dist_(rng_) * acc_noise_;
        msg.linear_acceleration.z = a_body.z() + norm_dist_(rng_) * acc_noise_;

        // 6. 填充姿态真值 (Orientation Ground Truth)
        // 注意：实际 IMU 硬件通常不直接输出无漂移的四元数，这里通常用于调试或作为 AHRS 的输出
        msg.orientation.w = s.q_wb.w();
        msg.orientation.x = s.q_wb.x();
        msg.orientation.y = s.q_wb.y();
        msg.orientation.z = s.q_wb.z();

        // 7. 发布消息
        imu_pub_.publish(msg);           // 发布 sensor_msgs::Imu 消息
        publishTF(s, stamp);
        publishOdom(s, stamp);

        if (total_duration_ > 0.0 && elapsed >= total_duration_ && !finished_) {
            finished_ = true;
            ros::shutdown();
        }
    }

    void publishTF(const SimState& s, ros::Time stamp) {
        // World -> Elevator
        tf::Transform tx_elev;
        tx_elev.setOrigin(tf::Vector3(0, 0, s.elev_z));
        tx_elev.setRotation(tf::Quaternion(q_wc_.x(), q_wc_.y(), q_wc_.z(), q_wc_.w()));
        tf_br_.sendTransform(tf::StampedTransform(tx_elev, stamp, "world", "elevator_frame"));

        // World -> IMU
        tf::Transform tx_wb;
        tx_wb.setOrigin(tf::Vector3(s.p_w.x(), s.p_w.y(), s.p_w.z()));
        tx_wb.setRotation(tf::Quaternion(s.q_wb.x(), s.q_wb.y(), s.q_wb.z(), s.q_wb.w()));
        tf_br_.sendTransform(tf::StampedTransform(tx_wb, stamp, "world", imu_frame_));

        // IMU -> LiDAR
        tf::Transform tx_sensor;
        tx_sensor.setIdentity();
        tf_br_.sendTransform(tf::StampedTransform(tx_sensor, stamp, imu_frame_, lidar_frame_));
    }

    void publishOdom(const SimState& s, ros::Time stamp) {
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "world";
        odom.child_frame_id = imu_frame_;
        odom.pose.pose.position.x = s.p_w.x();
        odom.pose.pose.position.y = s.p_w.y();
        odom.pose.pose.position.z = s.p_w.z();
        odom.pose.pose.orientation.w = s.q_wb.w();
        odom.pose.pose.orientation.x = s.q_wb.x();
        odom.pose.pose.orientation.y = s.q_wb.y();
        odom.pose.pose.orientation.z = s.q_wb.z();
        odom_pub_.publish(odom);
    }

    /**
     * @brief 激光雷达 (LiDAR) 数据仿真回调函数
     * @details 通过几何射线投射 (Ray Casting) 模拟雷达点云数据，包含噪声模型与量程过滤
     */
    void lidarCallback(const ros::TimerEvent&) {
        // 1. 安全检查：若无状态数据，直接返回，防止处理空数据
        if(player_.states.empty()) return;

        // 2. 获取当前仿真时刻及对应的系统状态 (Ground Truth)
        double elapsed = getElapsedTime();
        if (inInitialGravityWarmup(elapsed)) {
            if (total_duration_ > 0.0 && elapsed >= total_duration_ && !finished_) {
                finished_ = true;
                ros::shutdown();
            }
            return;
        }
        double sim_t = getTrajectoryTime(elapsed);
        applyGravityDriftIfDue(sim_t);
        SimState s = player_.getState(sim_t);
        ros::Time stamp(elapsed);
        publishElevatorFlagIfDue(sim_t);

        // 3. 初始化 PCL 点云容器
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.header.frame_id = lidar_frame_; // 设置 TF 坐标系
        // PCL 库的时间戳通常单位为微秒 (us)
        cloud.header.stamp = stamp.toNSec() / 1000;

        // 4. 准备射线投射的起始状态
        // 使用 p_rel (相对位置) 暗示碰撞检测是在局部环境（如电梯井/箱体）坐标系下进行的
        // q_le: 从雷达/机体 (Body) 到环境 (Elevator) 的旋转四元数
        Eigen::Quaterniond q_le = q_cw_ * s.q_wb;
        Eigen::Vector3d p_le = s.p_rel;

        // 5. 模拟激光扫描过程
        // 外层循环：遍历垂直维度的线束 (Elevation/Rings)
        for (int i = 0; i < lidar_beams_; ++i) {
            // 计算当前线束的仰角，均匀分布在垂直 FOV 范围内
            double elev_angle = -lidar_fov_/2.0 + (lidar_fov_ / (lidar_beams_>1?lidar_beams_-1:1)) * i;
            if (lidar_beams_ == 1) elev_angle = 0; // 单线雷达特殊处理

            // 内层循环：遍历水平维度的采样点 (Azimuth)
            for (int j = 0; j < lidar_points_; ++j) {
                // 水平角覆盖 [-pi, pi) 全周
                double azimuth = -M_PI + (2.0*M_PI / lidar_points_) * j;

                // 6. 计算射线方向向量
                // A. 叠加角度噪声（仰角/方位角分别扰动）
                double elev_noisy = elev_angle + norm_dist_(rng_) * lidar_angle_noise_rad_;
                double azimuth_noisy = azimuth + norm_dist_(rng_) * lidar_angle_noise_rad_;

                // B. 将球坐标 (仰角, 方位角) 转换为雷达坐标系下的单位方向向量
                double cr = cos(elev_noisy), sr = sin(elev_noisy);
                double ca = cos(azimuth_noisy), sa = sin(azimuth_noisy);
                Eigen::Vector3d dir_body(cr*ca, cr*sa, sr);

                // C. 坐标系转换：将方向向量旋转至环境坐标系 (用于计算几何交点)
                Eigen::Vector3d dir_elev = q_le * dir_body;

                double dist;
                // 7. 几何碰撞检测 (Ray Casting)
                // intersectBox: 计算射线(起点 p_le, 方向 dir_elev)与环境包围盒(AABB)的最近交点
                if (intersectBox(p_le, dir_elev, dist)) {
                    // 8. 传感器特性模拟
                    // 叠加高斯测距噪声 (Range Noise)
                    dist += norm_dist_(rng_) * lidar_range_noise_;

                    // 距离过滤：仅保留在有效量程内的回波 (Min/Max Range)
                    if (dist > lidar_min_ && dist < lidar_max_) {
                        // 9. 生成最终点云
                        // 将测得的距离还原为雷达坐标系下的 3D 坐标 (x, y, z)
                        // 注意：发布的点云数据是相对于 lidar_frame_ 的，因此使用 dir_body 即可
                        Eigen::Vector3d p_body = dir_body * dist;
                        cloud.points.emplace_back(p_body.x(), p_body.y(), p_body.z());
                    }
                }
            }
        }

        // 10. 转换格式并发布
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);           // 将 PCL 格式转换为 ROS 消息
        output.header.stamp = stamp; // 使用 CSV 中的仿真时间作为时间戳
        lidar_pub_.publish(output);

        if (total_duration_ > 0.0 && elapsed >= total_duration_ && !finished_) {
            finished_ = true;
            ros::shutdown();
        }
    }

    /**
     * @brief 计算射线与环境包围盒 (AABB) 的交点
     * @details 使用 "Slab Method" 算法，通过计算射线在三个轴向平面的进入和离开时间来判断相交
     * @param origin 射线起点 (雷达/传感器位置)
     * @param dir    射线单位方向向量
     * @param dist   [输出] 射线起点到最近交点的距离
     * @return true 表示相交，false 表示未相交
     */
    bool intersectBox(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir, double& dist) {
        // 1. 定义包围盒 (AABB) 的物理边界
        // 假设盒子中心在 X/Y 原点，Z轴从 0 开始向上
        Eigen::Vector3d min_bound(-box_w_/2, -box_d_/2, 0.0);
        Eigen::Vector3d max_bound(box_w_/2, box_d_/2, box_h_);

        // 2. 初始化射线的有效区间 [t_min, t_max]
        // t 代表距离，初始范围设为 [0, 最大量程]
        double t_min = 0.0, t_max = lidar_max_;

        // 3. 遍历 X, Y, Z 三个维度进行 "Slab" 切割
        for (int i = 0; i < 3; ++i) {
            // --- 情况 A: 射线平行于当前坐标轴 ---
            if (std::abs(dir[i]) < 1e-6) {
                // 如果射线在当前维度没有速度（平行），且起点不在边界范围内
                // 说明射线永远不可能进入这个维度的平板范围，直接判定为不相交
                if (origin[i] < min_bound[i] || origin[i] > max_bound[i]) return false;
            }
            // --- 情况 B: 射线与当前坐标轴有夹角 ---
            else {
                double ood = 1.0 / dir[i]; // 计算方向的倒数，避免除法优化速度

                // 计算射线到达该维度 "左/下边界" (t1) 和 "右/上边界" (t2) 的距离
                // 公式: t = (TargetPosition - StartPosition) / Velocity
                double t1 = (min_bound[i] - origin[i]) * ood;
                double t2 = (max_bound[i] - origin[i]) * ood;

                // 确保 t1 是进入点 (较小值/Near)，t2 是离开点 (较大值/Far)
                if (t1 > t2) std::swap(t1, t2);

                // 核心逻辑：缩小射线的有效重叠区间
                // 射线必须要在所有维度的区间内才算相交，所以：
                // 最终进入时间 = 所有维度进入时间的最大值
                // 最终离开时间 = 所有维度离开时间的最小值
                t_min = std::max(t_min, t1);
                t_max = std::min(t_max, t2);

                // 如果 进入时间 > 离开时间，说明各维度的平板区间没有交集 -> 射线错过了盒子
                if (t_min > t_max) return false;
            }
        }

        // 4. 计算最终距离
        // 如果 t_min > 0，说明射线从外部射向盒子，交点是进入点 t_min
        // 如果 t_min == 0 (或 < 0)，说明射线起点就在盒子内部，此时交点应该是离开点 t_max (打到内壁)
        dist = (t_min > 0) ? t_min : t_max;

        return true;
    }
};
