/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * ROS LiDAR and IMU message conversion for elevator-lio.
 */

#ifndef TOPICPROCESS_H
#define TOPICPROCESS_H

#include <deque>
#include <pcl/common/common.h>
#include <memory>
#include <sensor_msgs/PointCloud2.h>  // 替代 sensor_msgs/msg/point_cloud2.hpp
#include <sensor_msgs/Imu.h>          // 替代 sensor_msgs/msg/imu.hpp
#include <ros/ros.h>                  // 替代 rclcpp/rclcpp.hpp
#include <pcl_conversions/pcl_conversions.h>
// #include <livox_ros_driver2/CustomMsg.h>  // 替代 livox_ros_driver2/msg/custom_msg.hpp
#include <lio/CustomMsg.h>
#include "support/common_lib.h"
#include "support/type.h"
#include <condition_variable>  // 包含条件变量的头文件
#include "support/SharedBuffers.h"


enum LidarType {
    Livox = 1,
    Ouster = 2,
    Velodyne = 3,
    XT32 = 4,
};

/*** Velodyne ***/
namespace velodyne_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        std::uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))
/****************/

/*** Ouster ***/
namespace ouster_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        std::uint32_t t;
        std::uint16_t reflectivity;
        uint8_t ring;
        std::uint16_t ambient;
        std::uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };
} // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (std::uint32_t, t, t)(std::uint16_t, reflectivity,
                                                        reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)(std::uint32_t, range, range))
/****************/

/*** Hesai_XT32 ***/
namespace xt32_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        std::uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };
} // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/*** Hesai_Pandar128 ***/
namespace Pandar128_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float timestamp;
        uint8_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };
} // namespace Pandar128_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(Pandar128_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, timestamp, timestamp))
/*****************/


/**
 * @brief
 * TopicProcess
 * 负责接收各类 Topic 数据（LiDAR / IMU / Wheel / Image）
 * 将其同步组织到共享缓冲区 SharedBuffers 中
 */
class TopicProcess {

public:
    explicit TopicProcess(std::shared_ptr<SharedBuffers> buffers)
    : buffers_(std::move(buffers)) {}
    /* topic reader */
    void receive_points(const lio::CustomMsgConstPtr &msg);
    void receive_points_ntu(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void receive_points_velodyne(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void receive_points_xt32(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void receive_imu(const sensor_msgs::Imu::ConstPtr& msg);
    void receive_wheel(const lio::wheel_infoConstPtr& msg);
    /* data synchronization and organization */
    // bool sync_packages(MeasureGroup &meas);
    /* get method */
    void get_current_points(PointCloudXYZI::Ptr& cloud_out);
    std::deque<WheelData> get_wheel_buffer() const;


private:
    // 使用 share buffer
    std::shared_ptr<SharedBuffers> buffers_;

    // temporary variables
    double last_timestamp_imu_ = 0; // 最近一次接收到imu topic的时间戳
    double last_timestamp_lidar_ = 0; // 最近一次接收到lidar topic的时间戳
    double last_timestamp_img_ = 0;
    // count
    unsigned int cloud_topic_count_ = 0; // 接收到lidar话题的次数
    unsigned int imu_topic_count_ = 0; // 接收到imu话题的次数
    unsigned int wheel_topic_count_ = 0; // 接收到imu话题的次数
    unsigned int scan_num_ = 0; // 正确组织数据次数
    // reset
    bool img_en_ = true; // 启用图像
};

#endif //TOPICPROCESS_H
