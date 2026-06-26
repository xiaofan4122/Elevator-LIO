/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * ROS node interface and runtime state declarations for elevator-lio.
 */

#ifndef LIO_NODE_H
#define LIO_NODE_H

// 相比于 ros2 头文件的变化
#include <ros/ros.h>  // 替代 rclcpp/rclcpp.hpp 和 rclcpp/node.hpp
#include <sensor_msgs/PointCloud2.h>  // 替代 sensor_msgs/msg/point_cloud2.hpp
#include <sensor_msgs/Imu.h>  // 替代 sensor_msgs/msg/imu.hpp
#include <nav_msgs/Odometry.h>  // 替代 nav_msgs/msg/odometry.hpp
#include <tf2_ros/transform_broadcaster.h>  // 保持不变
#include <tf2_ros/static_transform_broadcaster.h>  // 保持不变
#include <pcl_conversions/pcl_conversions.h>  // 保持不变
#include "support/YamlReader.h"
#include "support/TopicProcess.h"
#include "estimator/ESEKF.h"
#include "node/LidarPipeline.h"
#include <nav_msgs/Path.h>
#include <csignal>  // 包含信号处理相关的头文件

#include <functional>
#include <map>
#include <std_msgs/Bool.h>

struct TimeRecord {
    double lidar_end_time;
    double lidar_beg_time;
    double img_beg_time;
    double img_end_time;
    double imu_beg_time;
    double imu_end_time;
};

namespace FSM {
    enum class State { Waitting, Initializing, IMUProcess, LidarProcess, ImgProcess /*…*/ };
    enum class Event { IMUTriger, InitEnd, LidarTriger, LidarEnd, ImgTriger, ImgEnd /*…*/ };
    struct Transition {
        State next_state;
        std::function<void()> action;
    };
    using Key = std::pair<State,Event>;
    // 状态表，和 dispatch() 声明
    extern const std::map<Key, Transition> kTransitionTable;
    extern State current_state;
    void dispatch(Event e);
    // 辅助打印函数
    inline const char* to_string(State s) {
        switch(s) {
            case State::Waitting:      return "Waitting";
            case State::Initializing:  return "Initializing";
            case State::IMUProcess:    return "IMUProcess";
            case State::LidarProcess:  return "LidarProcess";
            case State::ImgProcess:    return "ImgProcess";
            default:                   return "UnknownState";
        }
    }
    inline const char* to_string(Event e) {
        switch(e) {
            case Event::IMUTriger:   return "IMUTriger";
            case Event::InitEnd:     return "InitEnd";
            case Event::LidarTriger: return "LidarTriger";
            case Event::LidarEnd:    return "LidarEnd";
            case Event::ImgTriger:   return "ImgTriger";
            case Event::ImgEnd:      return "ImgEnd";
            default:                 return "UnknownEvent";
        }
    }
}

class LIONode {
public:
    LIONode();

    void publish_imu_odometry(State state, double stamp_sec = -1.0);
    void publish_body_odometry(State state, double stamp_sec = -1.0);
    void publish_Dedistort_clouds_lidar(const PointCloudXYZI::Ptr &cloud);
    void publish_Effect_clouds_lidar(const PointCloudXYZI::Ptr &cloud);
    void publish_Rejected_clouds_lidar(const PointCloudXYZI::Ptr &cloud);
    void publishStaticTransform();
    void publishGlobalMap();
    void publishIKDTree();
    void save_odometry(State state,double time, const string& save_path = "");
    void save_singel_clouds_world(PointCloudXYZI::Ptr clouds_lidar);
    static void lasermap_fov_segment();
    static void map_incremental(PointCloudXYZI & lidar_clouds);
    void timer_1HZ_callback(const ros::TimerEvent& );
    void timer_10HZ_callback(const ros::TimerEvent& );
    void timer_500HZ_callback(const ros::TimerEvent& );
    void timer_2000HZ_callback(const ros::TimerEvent& );


private:


    ros::NodeHandle nh_;
    ros::Publisher clouds_lidar_pub_;
    ros::Publisher clouds_lidar_effect_pub_;
    ros::Publisher clouds_lidar_reject_pub_;
    ros::Publisher global_map_pub_;
    ros::Publisher ikdtree_pub_;
    ros::Publisher odom_imu_pub_;
    ros::Publisher odom_body_pub_;
    ros::Publisher odom_path_pub_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber wheel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    ros::Timer timer_2000HZ_;
    ros::Timer timer_500HZ_;
    ros::Timer timer_10HZ_;
    ros::Timer timer_1HZ_;
    ros::Subscriber elevator_flag_sub_;
    ros::Publisher ele_state_pub_;

    void elevatorFlagCallback(const std_msgs::Bool::ConstPtr& msg);

    LidarPipeline lidar_pipeline_;
};

// Callback Function
void pcl_cbk_custom(const lio::CustomMsgConstPtr& msg);
void pcl_cbk_pc2(const sensor_msgs::PointCloud2::ConstPtr &msg);
void imu_cbk(const sensor_msgs::ImuConstPtr& msg);
void wheel_cbk(const lio::wheel_infoConstPtr& msg);

void convert_traj_csv_to_tum();

#endif // LIO_NODE_H
