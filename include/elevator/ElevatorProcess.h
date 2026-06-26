/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 *
 * Elevator motion detection and world-z acceleration helpers.
 */

#ifndef ELEVATORPROCESS_H
#define ELEVATORPROCESS_H

#include <limits>
#include <cmath>
#include <cassert>
#include <Eigen/Dense>
#include "support/type.h"
#include "support/common_lib.h"
#include "support/LIONode.h"

/**
 * @brief 电梯门触发、模式切换辅助和 1D 电梯状态传播
 *
 * 约定：
 *  - 世界 z 轴向上为正，重力 g 指向下（模≈9.81）。因此 “向上单位向量”
 *    ezu = -g / ||g||。所有“水平/竖直”计算均以 ezu 为基准。
 *  - 进入/退出只会切换 S.in_elevator 开关，不改变状态维度；STATE_TOTAL 固定 22。
 *  - 1D 子状态 (z, vz, az) 的名义传播与 Q 注入在 stepProcess() 内完成。
 */
class ElevatorProcess {
public:
    ElevatorProcess() = default;

    void applyZeroVelocityUpdateZ(State& S);
    void finalizeExitCovariance(State& S);

    // 进入/退出电梯模式（不改变矩阵尺寸，只赋值 1D 子状态与子块方差）
    void ElevatorModeEnter(State& S, const Eigen::Vector3d& acc_b);
    void ElevatorModeExit(State& S);

    // LiDAR door-distance trigger used only to enter elevator mode.
    void processDoorDetector(const PointCloudXYZI& cloud,
                             double timestamp,
                             const State& state,
                             bool lidar_process_active);
    double doorCooldownRemaining(double timestamp) const;
    double doorLastFilteredMaxDistance() const { return door_last_filtered_max_distance_; }
    bool doorLowDistanceActive() const { return door_low_distance_active_; }

    // 电梯内：1D 传播 + Q 注入
    void stepProcess(State& S, double dt);

    // 世界“向上”单位向量：由 g（向下）得到 ezu = -g/||g||
    static inline Eigen::Vector3d ezu_from_g(const Eigen::Vector3d& g){
        const double n = g.norm();
        Eigen::Vector3d ezu;
        if (std::isfinite(n) && n > 1e-6) {
            ezu = (-1.0 / n) * g;      // = -g / n
        } else {
            ezu = Eigen::Vector3d(0, 0, 1);  // 等价于 UnitZ()，但类型是 Vector3d
        }
        return ezu;
    }


private:
    static constexpr double kJerkNoise = 0.5; // 竖直 jerk 噪声 [m/s^3/Hz]

    bool door_first_packet_received_ = false;
    bool door_closed_triggered_ = false;
    double door_start_bag_time_ = 0.0;
    double door_low_dist_start_time_ = -1.0;
    double door_last_enter_trigger_time_ = -std::numeric_limits<double>::infinity();
    double door_last_filtered_max_distance_ = -1.0;
    bool door_low_distance_active_ = false;
};

/**
 * @brief 你项目里的 ESKF 量测更新模板（标量/小维量测）
 *
 * 需要配合列数 = StateIndex::STATE_TOTAL(=22) 的 H 使用：
 *   K = P Hᵀ (H P Hᵀ + R)^{-1}
 *   注入 + Joseph 形式更新协方差
 * 约定：这是“约束残差”写法 r(x)≈H δx，目标是 r→0，因此 δx = -K r
 *
 */
template<int M>
inline void ekfUpdate(State& S,
                      const Eigen::Matrix<double, M, StateIndex::STATE_TOTAL>& H,
                      const Eigen::Matrix<double, M, 1>& r,
                      const Eigen::Matrix<double, M, M>& R)
{
    constexpr int NX = StateIndex::STATE_TOTAL; // 22
    using MatXX = Eigen::Matrix<double, NX, NX>;
    using MatXM = Eigen::Matrix<double, NX, M >;
    using MatMM = Eigen::Matrix<double, M , M >;
    using VecX  = Eigen::Matrix<double, NX, 1 >;

    // --- 0) 早退与断言 ---
    if (H.rows() == 0) return;
    assert(H.cols() == NX && "H must have 22 columns");

    // --- 1) K = P Hᵀ (H P Hᵀ + R)^{-1}  （用分解+solve，避免显式逆）---
    const MatXM PHt = S.P * H.transpose();   // 22×M
    MatMM S_innov   = H * PHt + R;           // M×M

    Eigen::LDLT<MatMM> ldlt(S_innov);
    if (ldlt.info() != Eigen::Success) {
        // 轻微阻尼，增强稳健性
        const double eps = 1e-9;
        S_innov.diagonal().array() += eps;
        ldlt.compute(S_innov);
    }
    const MatXM K = PHt * ldlt.solve(MatMM::Identity()); // 22×M

    // --- 2) 状态增量（约束残差：δx = K r）---
    const VecX dx = K * r;                // 22×1

    // --- 3) 在流形上注入（姿态左乘，其余加性）---
    const Eigen::Vector3d dtheta = dx.template segment<3>(StateIndex::R);
    // S.q = (S.q * so3Exp(dtheta)).normalized();
    S.q = S.q * so3Exp(dtheta);

    S.p  += dx.template segment<3>(StateIndex::P);
    S.v  += dx.template segment<3>(StateIndex::V);
    S.bw += dx.template segment<3>(StateIndex::BW);
    S.ba += dx.template segment<3>(StateIndex::BA);
    S.g  += dx.template segment<3>(StateIndex::G);

    // 电梯 1D 子状态：需要生效就给 H 的这些列非零；不想影响就把相应列设 0
    S.z  += dx(StateIndex::Z );
    S.vz += dx(StateIndex::VZ);
    S.az += dx(StateIndex::AZ);

    // --- 4) 协方差（Joseph 形式，数值稳健）---
    const MatXX I  = MatXX::Identity();
    const MatXX KH = K * H;                 // 22×22
    S.P = (I - KH) * S.P * (I - KH).transpose() + K * R * K.transpose();

    // --- 5) ESKF reset（姿态误差重中心化）---
    MatXX Gamma = MatXX::Identity();
    Gamma.block<3,3>(StateIndex::R, StateIndex::R) -= 0.5 * skew3d(dtheta);
    S.P = Gamma * S.P * Gamma.transpose();

    // 可选：对称化
    S.P = 0.5 * (S.P + S.P.transpose());
}


#endif // ELEVATORPROCESS_H
