/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Common state, sensor, and point-cloud data types.
 */

#ifndef TYPE_H
#define TYPE_H

#include <sensor_msgs/Imu.h>  
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>

// 触发器枚举（判断来了哪一个数据）
enum class Trigger {
    Lidar = 0,
    Image = 1,
};

// 轮速数据结构体，转化为了阿克曼模型，原有4个角度和4个速度均被缩减到的2个
struct WheelData
{
    double timestamp; // 时间戳
    double angle_L; // 左前轮角度
    double angle_R; // 右前轮角度
    double v_L ; // 左前轮速度
    double v_R ; // 右前轮速度
};


enum StateIndex : int {
    R = 0,                   // (3 dimention) rotation in world frame
    P = 3,                   // (3 dimention) position in world frame
    V = 6,                   // (3 dimention) velocity in world frame
    BW = 9,                 // (3 dimention) IMU gyroscope bias
    BA = 12,                 // (3 dimention) IMU acceleration bias
    G = 15,                 // (3 dimention) Gravity in world frame
    // Elevator 1D substate (world gravity axis)  (scalars)
    Z  = 18,  // z position (1)
    VZ = 19,  // z velocity (1)
    AZ = 20,  // z acceleration (1)

    // ---- Total state dimension ----
    STATE_TOTAL = 21
};

enum StateNoiseIndex : int {
    GYRO_NOISE = 0,        // angular velocity change
    ACC_NOISE = 3,         // linear acceleration change
    GYRO_RANDOM_WALK = 6,  // IMU gyroscope bias random walk
    ACC_RANDOM_WALK = 9,   // IMU aceleration bias random walk
    GRAVITY_NOISE = 12,
    // Elevator 1D noise (scalar)
    ELEVATOR_ACC_NOISE = 15,       // az jerk noise (1)  (dot(az) = w_j)
    // Total noise dimension
    NOISE_TOTAL = 16
};

// imu frame
struct State {
    // ------------ Nominal (world imu) ------------
    Eigen::Quaterniond q;  // w_q_imu
    Eigen::Vector3d p; // w_p_imu
    Eigen::Vector3d v; // w_v_imu
    Eigen::Vector3d bw;
    Eigen::Vector3d ba;
    Eigen::Vector3d g; // world
    // ------------ Elevator 1D substate (world z-axis) -----------
    // z-axis is defined by n_z = normalize(g)
    double z  = 0.0;   // 1D position along n_z
    double vz = 0.0;   // 1D velocity along n_z
    double az = 0.0;   // 1D acceleration along n_z
    // ------------ Book-keeping / flags ------------
    bool in_elevator = false;   // mode flag (enter/exit elevator)
    // Full error-state covariance (dynamic sized)
    Eigen::MatrixXd P;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


// 对于 IMU 预积分中的 IMU_POSE 来说，offset_time 是相对于 lidar_begin_time 的偏移时间，可能为负值
struct Pose {
    Eigen::Quaterniond q; // world
    Eigen::Vector3d p; // world
    Eigen::Vector3d v; // world
    Eigen::Vector3d gyr; // body
    Eigen::Vector3d acc; // world
    double offset_time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/* =====  常用点云类型别名 ===== */
using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;
using PointCloudXYZIPtr = PointCloudXYZI::Ptr; // 共享指针，不必担心局部定义而被被释放
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
/* =====  其他消息 / 图像别名（可按需增加） ===== */
using ImuMsg      = sensor_msgs::Imu;
using ImuMsgConst = sensor_msgs::Imu::ConstPtr;


using StateVec = std::vector<State, Eigen::aligned_allocator<State>>;



#endif //TYPE_H
