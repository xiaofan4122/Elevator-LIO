/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * ROS subscription callbacks for elevator-lio.
 */

#include "support/LIONode.h"
#include "support/common_lib.h"
#include "support/type.h"

extern std::shared_ptr<TopicProcess> p_topic_process;

/********************************************** Callback Function **********************************************/

/**
 * @brief 接收到点云后的回调函数, 支持livox自定义消息类型
 * @param msg 点云数据
 */
void pcl_cbk_custom(const lio_ros::CustomMsgConstPtr& msg) {
    // Keep incoming scans during IMU initialization. The ikd-tree frontend
    // accumulates startup scans, while IVox waits for initialized IMU coverage
    // before selecting its first scan.
    p_topic_process->receive_points(msg);
}

/**
 * @brief 接收到点云后的回调函数, 支持标准点云消息类型
 * @param msg 点云数据
 */
void pcl_cbk_pc2(const lio_ros::PointCloud2ConstPtr &msg)  {
    switch (lidar_type) {
        case LidarType::Livox:
            break;
        case LidarType::Ouster:
            p_topic_process->receive_points_ntu(msg);
            break;
        case LidarType::Velodyne:
            p_topic_process->receive_points_velodyne(msg);
            break;
        case LidarType::XT32:
            p_topic_process->receive_points_xt32(msg);
            break;
        default:
            LOG_ERROR(Core, "[LIO] Unknown lidar type.");
    }
}

// #include "estimator/IMUProcess.h"
// IMUProcess imu_process;

void imu_cbk(const lio_ros::ImuConstPtr& msg) {
    static bool first_imu = true;
    if (first_imu) { FSM::dispatch(FSM::Event::IMUTriger); first_imu = false; }
    p_topic_process->receive_imu(msg);
}

void wheel_cbk(const lio_ros::WheelInfoConstPtr& msg) {
    p_topic_process->receive_wheel(msg);
    if (!(wheel_data_log_enable & log_save_enable)) return;

    static uint64_t wheel_seq = 0;
    const double stamp = get_time_sec(msg->header.stamp);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9)
        << stamp << ","
        << msg->lf_wheel_fb_velocity << ","
        << msg->lr_wheel_fb_velocity << ","
        << msg->rf_wheel_fb_velocity << ","
        << msg->rr_wheel_fb_velocity << ","
        << msg->front_angle_fb_l << ","
        << msg->front_angle_fb_r << ","
        << msg->rear_angle_fb_l << ","
        << msg->rear_angle_fb_r;
    runtimeLogger().writeLine(
            LogFileChannel::WheelData,
            "stamp,lf_v,lr_v,rf_v,rr_v,ang_fl,ang_fr,ang_rl,ang_rr",
            oss.str(),
            ++wheel_seq,
            stamp);
}
