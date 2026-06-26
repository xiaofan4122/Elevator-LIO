/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 *
 * Elevator motion detection and world-z acceleration implementation.
 */

#include "elevator/ElevatorProcess.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include "support/common_lib.h"

void ElevatorProcess::processDoorDetector(const PointCloudXYZI& cloud,
                                          double timestamp,
                                          const State& state,
                                          bool lidar_process_active)
{
    if (!elevator_enable || !elevator_door_detector_enable) return;
    if (!lidar_process_active) return;

    if (!door_first_packet_received_) {
        door_start_bag_time_ = timestamp;
        door_first_packet_received_ = true;
        LOG_INFO(Elevator, "[door_detector] start time synced: " << door_start_bag_time_ << " s");
    }

    std::vector<double> distances;
    distances.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        if (p.x == 0.0f && p.y == 0.0f && p.z == 0.0f) continue;
        distances.push_back(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z));
    }
    if (distances.empty()) return;

    std::sort(distances.begin(), distances.end());
    int index = static_cast<int>(distances.size() * (1.0 - elevator_door_filter_percent));
    if (index < 0) index = 0;
    if (index >= static_cast<int>(distances.size())) index = static_cast<int>(distances.size()) - 1;
    const double filtered_max_d = distances[static_cast<size_t>(index)];
    door_last_filtered_max_distance_ = filtered_max_d;

    const bool door_closed_like = filtered_max_d > 0.1 && filtered_max_d < elevator_door_dist_threshold;

    door_low_distance_active_ = door_closed_like;

    if (!state.in_elevator && !ELEVATOR_TRIGGER) {
        if ((timestamp - door_last_enter_trigger_time_) < elevator_door_cooldown_time) return;

        if (door_closed_like) {
            if (door_low_dist_start_time_ < 0.0) {
                door_low_dist_start_time_ = timestamp;
            }

            if (!door_closed_triggered_ &&
                (timestamp - door_low_dist_start_time_) >= elevator_door_time_threshold) {
                ELEVATOR_TRIGGER = true;
                door_closed_triggered_ = true;
                door_last_enter_trigger_time_ = timestamp;
                LOG_WARN(Elevator, "[door_detector] elevator door closed detected"
                                       << " (filtered_d=" << filtered_max_d
                                       << ", rel_t=" << (timestamp - door_start_bag_time_) << " s)");
            }
        } else {
            door_low_dist_start_time_ = -1.0;
            door_closed_triggered_ = false;
        }
    }
}

double ElevatorProcess::doorCooldownRemaining(double timestamp) const
{
    double remaining = 0.0;
    if (std::isfinite(door_last_enter_trigger_time_)) {
        remaining = std::max(remaining, elevator_door_cooldown_time - (timestamp - door_last_enter_trigger_time_));
    }
    return std::max(0.0, remaining);
}

/* ============================== 进入/退出模式 ============================== */

void ElevatorProcess::ElevatorModeEnter(State& S, const Eigen::Vector3d& acc_b)
{
    (void)acc_b;
    // 电梯 1D 子状态表示“本段相对位移”，进入时必须从 0 重新开始。
    S.z  = 0.0;
    S.vz = 0.0;
    S.az = 0.0;
    S.in_elevator = true;

    // 1D 子块方差初值。这里只能操作 3x3 子块，4x4 会越界写坏 P。
    const int idx_elev_start = StateIndex::Z;
    const int dim_elev = 3;
    const int dim_robot = StateIndex::STATE_TOTAL - dim_elev;
    S.P.block(idx_elev_start, 0, dim_elev, dim_robot).setZero();
    S.P.block(0, idx_elev_start, dim_robot, dim_elev).setZero();
    S.P.block(idx_elev_start, idx_elev_start, dim_elev, dim_elev).setZero();
    S.P(StateIndex::Z,  StateIndex::Z ) = 1e-4;
    S.P(StateIndex::VZ, StateIndex::VZ) = 1e-4;
    S.P(StateIndex::AZ, StateIndex::AZ) = 1e-3;

    S.P = 0.5 * (S.P + S.P.transpose()); // 可选：保证数值对称
}

void ElevatorProcess::ElevatorModeExit(State& S)
{
    S.in_elevator = false;

    const int idx_elev_start = StateIndex::Z;
    const int dim_elev = 3;
    const int dim_robot = StateIndex::STATE_TOTAL - dim_elev;
    S.P.block(idx_elev_start, 0, dim_elev, dim_robot).setZero();
    S.P.block(0, idx_elev_start, dim_robot, dim_elev).setZero();
    S.P.block(idx_elev_start, idx_elev_start, dim_elev, dim_elev).setZero();
    S.P(StateIndex::Z,  StateIndex::Z ) = 1e-4;
    S.P(StateIndex::VZ, StateIndex::VZ) = 1e-3;
    S.P(StateIndex::AZ, StateIndex::AZ) = 1e-2;

    S.P = 0.5 * (S.P + S.P.transpose()); // 可选
}

/* ============================== 1D 传播与绑定 ============================== */
/**
 * @brief 在电梯模式下进行电梯状态的传播，并填充到协方差子块中
 * @param S 状态
 * @param dt 时间步长
 */
void ElevatorProcess::stepProcess(State& S, double dt)
{
    if (!S.in_elevator || dt <= 0) return;

    // 1) 均值（常加速度）
    S.z  += S.vz * dt + 0.5 * S.az * dt * dt;
    S.vz += S.az * dt;

    // 2) Φe 与 Qe
    Eigen::Matrix3d Phi, Qe;
    Phi << 1, dt, 0.5*dt*dt,
           0,  1, dt,
           0,  0, 1;

    const double dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt, dt5 = dt4*dt;
    Qe << dt5/20.0, dt4/8.0, dt3/6.0,
          dt4/8.0,  dt3/3.0, dt2/2.0,
          dt3/6.0,  dt2/2.0, dt;
    Qe *= kJerkNoise;

    // 3) 协方差：EE 子块
    auto P_EE = S.P.block<3,3>(StateIndex::Z, StateIndex::Z);
    P_EE = Phi * P_EE * Phi.transpose() + Qe;
    S.P.block<3,3>(StateIndex::Z, StateIndex::Z) = P_EE;

    // 4) 协方差：IE / EI 交叉块（保持整体预测一致性）
    constexpr int N_INS = StateIndex::Z;            // 主块维度（Z 之前）
    Eigen::Matrix<double, N_INS, 3> P_IE = S.P.block<N_INS,3>(0, StateIndex::Z);
    S.P.block<N_INS,3>(0, StateIndex::Z) = P_IE * Phi.transpose();
    S.P.block<3, N_INS>(StateIndex::Z, 0) = (P_IE * Phi.transpose()).transpose();

    // 5) 可选：数值对称化
    S.P = 0.5 * (S.P + S.P.transpose());
}



/**
 * @brief 应用零速更新（ZUPT）伪量测.
 * * @param S   需要更新的系统状态 (State&)，函数将直接修改S和S.P。
 * @param prof 包含噪声参数的配置文件，这里主要使用 R_vel_xy。
 */
void ElevatorProcess::applyZeroVelocityUpdateZ(State& S)
{

    // cout << "[before]S.vz: " << S.vz << " S.z:" << S.z <<endl;
    //
    // Eigen::Matrix<double, 1, StateIndex::STATE_TOTAL> H;
    // H.setZero();
    // H(StateIndex::VZ) = 1;
    // Eigen::Matrix<double, 1, 1> r (-S.vz);
    // Eigen::Matrix<double, 1, 1> R (1E-4);
    // ekfUpdate<1>(S, H, r, R);
    //
    // cout << "[after]S.vz: " << S.vz << " S.z:" << S.z <<endl;

    std::ostringstream before_oss;
    before_oss << "[before]S.vz: " << S.vz << " S.v(2):" << S.v(2) << " S.z:" << S.z << " S.az:" << S.az;
    LOG_INFO(Elevator, before_oss.str());
    runtimeLogger().writeLine(LogFileChannel::ElevatorZupt, "", before_oss.str());

    Eigen::Matrix<double, 2, StateIndex::STATE_TOTAL> H;
    H.setZero();
    H(0,StateIndex::VZ) = 1;
    H(1,StateIndex::AZ) = 1;
    Eigen::Matrix<double, 2, 1> r ;
    r(0) = -S.vz;
    r(1) = -S.az;
    Eigen::Matrix<double, 2, 2> R ;
    R.setZero();
    R(0, 0) = 1E-5; // 速度观测噪声
    R(1, 1) = 1E-4; // 加速度观测噪声
    ekfUpdate<2>(S, H, r, R);

    // cout << "[before]S.vz: " << S.vz << " S.z:" << S.z <<endl;

    // Eigen::Matrix<double, 1, StateIndex::STATE_TOTAL> H;
    // H.setZero();
    // H(StateIndex::VZ) = 1;
    // Eigen::Matrix<double, 1, 1> r (-S.vz);
    // Eigen::Matrix<double, 1, 1> R (1E-3);
    // ekfUpdate<1>(S, H, r, R);

    // Eigen::Matrix<double, 2, StateIndex::STATE_TOTAL> H;
    // H.setZero();
    // H(0,StateIndex::VZ) = 1;
    // H(0,StateIndex::V+2) = 1;
    // H(1,StateIndex::AZ) = 1;
    // Eigen::Matrix<double, 2, 1> r ;
    // r(0) = -S.vz - S.v(2);
    // r(1) = -S.az;
    // Eigen::Matrix<double, 2, 2> R ;
    // R.setZero();
    // R(0, 0) = 1E-9; // 速度观测噪声
    // R(1, 1) = 1E-6; // 加速度观测噪声
    // ekfUpdate<2>(S, H, r, R);

    // Eigen::Matrix<double, 3, StateIndex::STATE_TOTAL> H;
    // H.setZero();
    // H(0,StateIndex::VZ) = 1;
    // H(1,StateIndex::V + 2) = 1;
    // H(2,StateIndex::AZ) = 1;
    // Eigen::Matrix<double, 3, 1> r ;
    // r(0) = -S.vz;
    // r(1) = -S.v(2);
    // r(2) = -S.az;
    // Eigen::Matrix<double, 3, 3> R ;
    // R.setZero();
    // R(0, 0) = 1E-9; // 电梯速度观测噪声
    // R(1, 1) = 1E-9; // 相对速度观测噪声
    // R(2, 2) = 1E-6; // 加速度观测噪声
    // ekfUpdate<3>(S, H, r, R);

    const double pzz = S.P(StateIndex::Z, StateIndex::Z);
    std::ostringstream after_oss;
    after_oss << "[after]S.vz: " << S.vz << " S.v(2):" << S.v(2) << " S.z:" << S.z << " S.az:" << S.az
              << " Pzz:" << pzz;
    LOG_INFO(Elevator, after_oss.str());
    runtimeLogger().writeLine(LogFileChannel::ElevatorZupt, "", after_oss.str());
}

void ElevatorProcess::finalizeExitCovariance(State& S)
{
    const int idx_elev_start = StateIndex::Z;
    const int dim_elev = 3;
    const int dim_robot = StateIndex::STATE_TOTAL - dim_elev; // 剩余的维度
    S.P.block(idx_elev_start, 0, dim_elev, dim_robot).setZero();
    S.P.block(0, idx_elev_start, dim_robot, dim_elev).setZero();
    S.P.block(idx_elev_start, idx_elev_start, dim_elev, dim_elev).setZero();

    // 再分别设置对角线
    S.P(StateIndex::Z, StateIndex::Z) = 1e-4;
    S.P(StateIndex::VZ, StateIndex::VZ) = 1e-3;
    S.P(StateIndex::AZ, StateIndex::AZ) = 1e-2;
    S.P = 0.5 * (S.P + S.P.transpose());
}
