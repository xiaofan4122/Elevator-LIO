/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * ROS publishers for odometry, paths, maps, and debug clouds.
 */

#include "support/LIONode.h"
#include "support/common_lib.h"

extern PointCloudXYZI::Ptr map_world;
extern std::shared_ptr<EskfEstimator> p_eskf_estimator;
extern vector<string> incremental_paths;

/********************************************** Publish Function **********************************************/

void LIONode::publish_imu_odometry(State state, double stamp_sec) {
    nav_msgs::Odometry odom_imu;
    const double odom_stamp = stamp_sec >= 0.0 ? stamp_sec : lidar_end_time;
    // odom.header.stamp = ros::Time::now();  // 使用 ROS 1 的时间戳
    odom_imu.header.stamp = get_ros_time(odom_stamp);
    odom_imu.header.frame_id = "world";
    odom_imu.child_frame_id = "IMU";

    // 设置位置
    odom_imu.pose.pose.position.x = state.p(0);
    odom_imu.pose.pose.position.y = state.p(1);
    odom_imu.pose.pose.position.z = state.p(2);

    if (state.in_elevator) {
        odom_imu.pose.pose.position.z += state.z;
    }

    // 设置四元数姿态
    odom_imu.pose.pose.orientation.x = state.q.x();
    odom_imu.pose.pose.orientation.y = state.q.y();
    odom_imu.pose.pose.orientation.z = state.q.z();
    odom_imu.pose.pose.orientation.w = state.q.w();

    // 设置线速度
    odom_imu.twist.twist.linear.x = state.v(0);
    odom_imu.twist.twist.linear.y = state.v(1);
    odom_imu.twist.twist.linear.z = state.v(2);

    // 发布里程计消息
    odom_imu_pub_.publish(odom_imu);

    //Path
    static nav_msgs::Path odom_path;
    geometry_msgs::PoseStamped this_pose_stamped;

    this_pose_stamped.pose = odom_imu.pose.pose; // 使用已有的里程计姿态信息
    this_pose_stamped.header.stamp = odom_imu.header.stamp;
    this_pose_stamped.header.frame_id = "world"; // 确保frame_id与您的应用相匹配

    odom_path.poses.push_back(this_pose_stamped);
    odom_path.header.stamp = odom_imu.header.stamp;
    odom_path.header.frame_id = "world"; // 同样确保frame_id正确

    odom_path_pub_.publish(odom_path);

    // 发布tf变换
    geometry_msgs::TransformStamped trans;
    trans.header.frame_id = "world";
    trans.header.stamp = odom_imu.header.stamp;
    trans.child_frame_id = "IMU";
    trans.transform.translation.x = odom_imu.pose.pose.position.x;
    trans.transform.translation.y = odom_imu.pose.pose.position.y;
    trans.transform.translation.z = odom_imu.pose.pose.position.z;
    trans.transform.rotation.w = odom_imu.pose.pose.orientation.w;
    trans.transform.rotation.x = odom_imu.pose.pose.orientation.x;
    trans.transform.rotation.y = odom_imu.pose.pose.orientation.y;
    trans.transform.rotation.z = odom_imu.pose.pose.orientation.z;
    tf_broadcaster_->sendTransform(trans);
}

void LIONode::publish_body_odometry(State state, double stamp_sec) {
    nav_msgs::Odometry odom_body;
    const double odom_stamp = stamp_sec >= 0.0 ? stamp_sec : lidar_end_time;
    odom_body.header.stamp = get_ros_time(odom_stamp);
    odom_body.header.frame_id = "world";
    odom_body.child_frame_id = "IMU";

    // 将位姿从雷达中心转换到小车中心
    Eigen::Matrix4d imu_T_lidar = Eigen::Matrix4d::Identity();
    imu_T_lidar.block<3, 3>(0, 0) = imu_R_lidar;
    imu_T_lidar.block<3, 1>(0, 3) = imu_t_lidar;
    Eigen::Matrix4d world_T_imu = Eigen::Matrix4d::Identity();
    world_T_imu.block<3, 3>(0, 0) = state.q.toRotationMatrix();
    world_T_imu.block<3, 1>(0, 3) = state.p;
    Eigen::Matrix4d lidar_T_body = Eigen::Matrix4d::Identity();
    lidar_T_body.block<3, 3>(0, 0) = lidar_R_body;
    lidar_T_body.block<3, 1>(0, 3) = lidar_t_body;
    // 对变换本身操作使用右乘
    Eigen::Matrix4d world_T_body = world_T_imu * imu_T_lidar * lidar_T_body;
    Eigen::Quaterniond body_q = Eigen::Quaterniond(world_T_body.block<3, 3>(0, 0));
    Eigen::Vector3d body_p = world_T_body.block<3, 1>(0, 3);
    odom_body.pose.pose.position.x = body_p(0);
    odom_body.pose.pose.position.y = body_p(1);
    odom_body.pose.pose.position.z = body_p(2);
    odom_body.pose.pose.orientation.x = body_q.x();
    odom_body.pose.pose.orientation.y = body_q.y();
    odom_body.pose.pose.orientation.z = body_q.z();
    odom_body.pose.pose.orientation.w = body_q.w();

    if (state.in_elevator) {
        odom_body.pose.pose.position.z += state.z;
    }

    odom_body_pub_.publish(odom_body);

    // 发布tf变换
    geometry_msgs::TransformStamped trans;
    trans.header.frame_id = "world";
    trans.header.stamp = odom_body.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odom_body.pose.pose.position.x;
    trans.transform.translation.y = odom_body.pose.pose.position.y;
    trans.transform.translation.z = odom_body.pose.pose.position.z;
    trans.transform.rotation.w = odom_body.pose.pose.orientation.w;
    trans.transform.rotation.x = odom_body.pose.pose.orientation.x;
    trans.transform.rotation.y = odom_body.pose.pose.orientation.y;
    trans.transform.rotation.z = odom_body.pose.pose.orientation.z;
    tf_broadcaster_->sendTransform(trans);
}

void LIONode::publish_Dedistort_clouds_lidar(const PointCloudXYZI::Ptr &cloud) {
    auto Dedistort_clouds_world = p_eskf_estimator -> transLidar2World(*cloud);
    static int publish_frame_counter = 0;
    ++publish_frame_counter;
    if (clouds_lidar_pub_every_n > 1 && (publish_frame_counter % clouds_lidar_pub_every_n) != 0) {
        return;
    }
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*Dedistort_clouds_world, cloud_msg);
    cloud_msg.header.frame_id = "world";
    // cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.stamp = get_ros_time(lidar_end_time);
    clouds_lidar_pub_.publish(cloud_msg);
}

void LIONode::publish_Effect_clouds_lidar(const PointCloudXYZI::Ptr &cloud) {
    if (!cloud || cloud->empty()) return;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "lidar";
    cloud_msg.header.stamp = get_ros_time(lidar_end_time);
    clouds_lidar_effect_pub_.publish(cloud_msg);
}

void LIONode::publish_Rejected_clouds_lidar(const PointCloudXYZI::Ptr &cloud) {
    if (!cloud || cloud->empty()) return;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "lidar";
    cloud_msg.header.stamp = get_ros_time(lidar_end_time);
    clouds_lidar_reject_pub_.publish(cloud_msg);
}

/**
 * @brief 发布 IMU 系到 lidar 系的静态 tf 变换
 */
void LIONode::publishStaticTransform() {
    geometry_msgs::TransformStamped trans;
    trans.header.frame_id = "IMU";
    // trans.header.stamp = ros::Time::now();
    trans.header.stamp = get_ros_time(lidar_end_time);
    trans.child_frame_id = "lidar";
    Eigen::Vector3d t (imu_t_lidar);
    Eigen::Quaterniond q (imu_R_lidar);
    trans.transform.translation.x = t(0);
    trans.transform.translation.y = t(1);
    trans.transform.translation.z = t(2);
    trans.transform.rotation.w = q.w();
    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    static_broadcaster_->sendTransform(trans);
}

/**
 * @brief 积累 down_effect_cloud_lidar 到 map_world （全局地图）并发布（发布受 global_map_pub_enable 控制）
 * @note  全局地图和 ikdtree 的降采样参数不同，实际 ikdtree 中的点云会更加稀疏
 */
void LIONode::publishGlobalMap() {
    if (relocation_enable) return; // 重定位时不发布全局地图，防止内存占用过大
    if (!global_map_save_enable) return; // 不保存全局地图时，不累计全局地图，防止内存占用过大
    /************* 当非重定位模式并保存全局点云时 ***************/
    // static float filter_size = 0.1;
    static double last_time = 0;
    static int frame_idx = 0;
    if (!down_effect_cloud_lidar->points.empty() && last_time != lidar_end_time) {
        // 计算程序耗时
        auto start = std::chrono::system_clock::now();
        auto effect_down_cloud_world = p_eskf_estimator -> transLidar2World(*down_effect_cloud_lidar);
        auto time2 = std::chrono::system_clock::now();
        /********* 如果需要发布全局地图，则将点云累加到 map_world 中 *********/
        if (global_map_pub_enable) {
            *map_world += *effect_down_cloud_world;
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*map_world, cloud_msg);
            cloud_msg.header.frame_id = "world";
            // cloud_msg.header.stamp = ros::Time::now();
            cloud_msg.header.stamp = get_ros_time(lidar_end_time);
            // 发布点云
            global_map_pub_.publish(cloud_msg);
        }
        /********* 如果需要不发布全局地图，先存储在文件中，最后再合并 *********/
        else{
            // 构造文件名，准备先记录到文件中
            std::stringstream ss;
            ss << PACKAGE_ROOT_DIR << "/PCD/Temp/incremental_cloud_" << frame_idx++ << ".pcd";
            // 保存本帧
            pcl::io::savePCDFileBinary(ss.str(), *effect_down_cloud_world);
            // 路径保存起来
            incremental_paths.push_back(ss.str());
        }
        // 计算程序耗时
        auto end = std::chrono::system_clock::now();
        auto duration_ms = std::chrono::duration<double, std::milli>(end - start);
        // 每行代码的耗时记录到文件中
        if(log_save_enable && pub_time_log_enable) {
            auto duration_ms1 = std::chrono::duration<double, std::milli>(time2 - start);
            auto duration_ms2 = std::chrono::duration<double, std::milli>(end - time2);
            static uint64_t pub_time_seq = 0;
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6)
                << duration_ms.count() << ","
                << duration_ms1.count() << ","
                << duration_ms2.count();
            runtimeLogger().writeLine(
                    LogFileChannel::PublishTime,
                    "publish_total_ms,publish_prepare_ms,publish_finish_ms",
                    oss.str(),
                    ++pub_time_seq);
        }
        if(duration_ms.count() > 10){
            cerr << "GlobalMap pub cost more than 10ms ! stop publish ! " << endl;
            global_map_pub_enable = false;
        }
        last_time = lidar_end_time;
    }
}

void LIONode::publishIKDTree() {
    if (ikdtree.Root_Node != nullptr && ikdtree_pub_enable) {
        // 计算程序耗时
        auto start = std::chrono::system_clock::now();
        PointVector points;
        ikdtree.flatten(ikdtree.Root_Node, points, NOT_RECORD);
        // 将提取的点赋值给点云对象
        pcl::PointCloud<PointType>::Ptr cloud_ikdtree(new pcl::PointCloud<PointType>);
        cloud_ikdtree->points = points;
        cloud_ikdtree->width = points.size();
        cloud_ikdtree->height = 1;
        cloud_ikdtree->is_dense = true;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_ikdtree, cloud_msg);
        cloud_msg.header.frame_id = "world";
        // cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.stamp = get_ros_time(lidar_end_time);
        // 发布点云
        ikdtree_pub_.publish(cloud_msg);
        // 计算程序耗时
        auto end = std::chrono::system_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start)/1000.0;
        if(duration_ms.count() > 10){
            // ikdtree_pub_enable = false;
            cerr << "[warning] IKDTree pub cost more than 10ms!" << endl;
        }
    }
}
