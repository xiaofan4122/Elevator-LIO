/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * IMU propagation and elevator-prior integration.
 */

#ifndef IMUPROCESS_H
#define IMUPROCESS_H

#include <iostream>
#include <deque>
#include "estimator/ESEKF.h"
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "elevator/ElevatorProcess.h"

using namespace std;
using namespace Eigen;

class IMUProcess {
public:
    // 必须添加这个宏，以确保 new 操作符正确对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUProcess() {states_imu.resize(MAX_LEN);}
    /********** 电梯模式控制与处理 ***********/
    ElevatorProcess elev_process;
    /************ 固定大小的数组 ************/
    static constexpr int MAX_LEN = 1000;
    // IMU 数组
    array<Vector3d, MAX_LEN> imu_acc_buffer;
    array<Vector3d, MAX_LEN> imu_gyr_buffer;
    // 时间戳数组
    array<double, MAX_LEN> time_buffer;
    // 状态数组
    std::vector<State, Eigen::aligned_allocator<State>> states_imu;
    /************ 对外接口函数 ************/
    void set_init_state(State state_init, double time_stamp); // 设置出输出位姿
    bool init(ImuMsgConst &msg, int init_imu_num,
              Eigen::Vector3d &mean_acc_, Eigen::Vector3d &mean_gyr_,
              State &state); // IMU 初始化：收集样本、计算均值、初始化状态
    void integration(ImuMsgConst imu_msg, Eigen::Vector3d mean_acc);
    void integration(Vector3d IMU_acc, Vector3d IMU_gyr, double time_stamp);  // 1.更新：IMU新测的数据传入到IMU_Buffer 2.预积分：IMU通过测得的加速度数据更新States
    void set_state_at_t(State &state_in, double time_stamp);   // 3.状态更新：更新指定时间戳的状态，并重积分之后的状态
    void get_state_at_t(State &state_out, double time_stamp);  // 4.状态获取：获取指定时间戳的状态，使用插值获取状态
    void get_imu_pose_from_t1_to_t2(std::vector<Pose> &IMU_Pose_vec, double time_stamp_1, double time_stamp_2); // 5.状态获取：获取指定时间段内的状态，插值获取两端状态
    /************* 变量 ************/

private:
    bool initialized = false;
    void initialize(double t0);

    State predictOnce(const State &prev,
                  const Eigen::Vector3d &acc_b,
                  const Eigen::Vector3d &gyr_b,
                  double dt,
                  double stamp) const;

    // 环形缓冲索引映射：保证返回值在 [0, MAX_LEN)
    inline int wrapIndex(int idx) const;
    inline Pose get_pose(int index , double base_time);

    Eigen::MatrixXd Q_;
    double ACC_NOISE_VAR = 1E-4;
    double GYRO_NOISE_VAR = 1E-5;
    double ACC_RANDOM_WALK_VAR = 1E-6;
    double GYRO_RANDOM_WALK_VAR = 1E-6;
    double GRAVITY_NOISE_VAR = 1E-9;
    double ELEVATOR_ACC_NOISE_VAR = 1E-1;

    int head_  = 0;                         // 下一次写入索引
    int count_ = 0;                         // 已写入总帧数

    // InitImu 收集缓冲
    std::deque<Eigen::Vector3d> init_acc_buf_;
    std::deque<Eigen::Vector3d> init_gyr_buf_;
};

#endif //IMUPROCESS_H
