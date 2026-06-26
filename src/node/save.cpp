/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Trajectory and point-cloud persistence helpers.
 */

#include "support/LIONode.h"
#include "support/common_lib.h"
#include <pcl/filters/voxel_grid.h>
extern std::shared_ptr<EskfEstimator> p_eskf_estimator;
extern KD_TREE<PointType> ikdtree;
extern PointCloudXYZI::Ptr map_world;
extern vector<string> incremental_paths;

/********************************************** Save Function **********************************************/

void LIONode::save_odometry(State state, double time, const string& save_path) {
    const LogFileChannel channel = save_path.empty() ? LogFileChannel::TrajectoryPost
                                                     : LogFileChannel::TrajectoryPrior;
    if (!save_path.empty() && save_path != runtimeLogger().resolvePath(LogFileChannel::TrajectoryPrior)) {
        return;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9)
        << time << ","
        << state.p(0) << "," << state.p(1) << "," << state.p(2) << ","
        << state.q.x() << "," << state.q.y() << "," << state.q.z() << "," << state.q.w() << ","
        << state.z << "," << state.vz << "," << state.az << ","
        << state.ba(0) << "," << state.ba(1) << "," << state.ba(2) <<  ","
        << state.v(0) << "," << state.v(1) << "," << state.v(2) << ","
        << state.g(0) << "," << state.g(1) << "," << state.g(2);
    runtimeLogger().writeLine(channel,
                              "time,x,y,z,qx,qy,qz,qw,Ez,Evz,Eaz,ba_x,ba_y,ba_z,vx,vy,vz,gx,gy,gz",
                              oss.str(), 0, time);
}

/**
 * @brief 保存单帧点云到指定文件夹，文件夹名为第一次调用函数时的时间，文件名为当前帧点云时间戳
 * 注意传入为lidar系，函数中会转换为world系再输出
 * @param clouds_lidar 激光雷达坐标系下点云
 */
void LIONode::save_singel_clouds_world(PointCloudXYZI::Ptr clouds_lidar) {
    /*** 保存单帧点云 ****/
    if (every_n_frame_save > 0) {
        // lidar系转为world系
        auto Dedistort_clouds_world = p_eskf_estimator -> transLidar2World(*clouds_lidar);
        static int cnt = 0;
        static bool first = true;
        if (cnt++ % every_n_frame_save == 0) {
            static std::string folder_name;
            if (first) {
                // 当前时间作为文件夹名
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream  folder_name_ss;
                folder_name_ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d-%H%M%S");
                folder_name = string(PACKAGE_ROOT_DIR) + "/PCD/" + folder_name_ss.str();
                // 以文件夹名创建文件夹
                mkdir(folder_name.c_str(), 0777);
                first = false;
            }
            // 时间戳保留9位小数
            std::stringstream ss;
            auto time_stamp = get_ros_time(lidar_end_time);
            ss << std::fixed << std::setprecision(9) << time_stamp.toSec();
            // 补充完整路径
            std::string file_path = folder_name + "/" + ss.str()+".pcd";
            // 只有在保存时才打印，避免频繁IO
            static double last_print_time = -1.0;
            if (last_print_time < 0.0 || (lidar_end_time - last_print_time) > 5.0) {
                cout << "[save_singel_clouds_world] save pcd file: " << file_path << std::endl;
                last_print_time = lidar_end_time;
            }
            pcl::io::savePCDFileBinary(file_path, *Dedistort_clouds_world);
        }
    }
}

void shutdown_save_maps() {
    if (relocation_enable || !global_map_save_enable) return;

    std::string pcd_save_path;
    if (timestamp_prefix) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d-%H%M%S");
        pcd_save_path = string(PACKAGE_ROOT_DIR) + "/PCD/" + ss.str() + "_" + pcd_save_name;
    } else {
        pcd_save_path = string(PACKAGE_ROOT_DIR) + "/PCD/" + pcd_save_name;
    }

    if (global_map_pub_enable) {
        if (!map_world->empty()) {
            pcl::io::savePCDFileBinary(pcd_save_path, *map_world);
            std::cout << "save pcd file: " << pcd_save_path << std::endl;
        }
    } else {
        PointCloudXYZI::Ptr merged(new PointCloudXYZI);
        for (auto &pcd_path : incremental_paths) {
            PointCloudXYZI::Ptr tmp(new PointCloudXYZI);
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *tmp) == -1) continue;
            *merged += *tmp;
        }
        if (!merged->empty()) {
            PointCloudXYZI::Ptr cleaned(new PointCloudXYZI);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*merged, *cleaned, indices);
            PointCloudXYZI::Ptr downsampled(new PointCloudXYZI);
            pcl::VoxelGrid<PointType> vg;
            vg.setLeafSize(0.1, 0.1, 0.1);
            vg.setInputCloud(cleaned);
            vg.filter(*downsampled);
            if (!downsampled->empty()) {
                pcl::io::savePCDFileBinary(pcd_save_path, *downsampled);
                std::cout << "save pcd file: " << pcd_save_path << std::endl;
            }
        }
        for (auto &pcd_path : incremental_paths) std::remove(pcd_path.c_str());
    }

    if (ikdtree.Root_Node != nullptr) {
        PointVector points;
        ikdtree.flatten(ikdtree.Root_Node, points, NOT_RECORD);
        if (!points.empty()) {
            pcl::PointCloud<PointType>::Ptr cloud_ikdtree(new pcl::PointCloud<PointType>);
            cloud_ikdtree->points = points;
            cloud_ikdtree->width = points.size();
            cloud_ikdtree->height = 1;
            cloud_ikdtree->is_dense = true;
            std::string ikdtree_path;
            if (timestamp_prefix) {
                ikdtree_path = string(PACKAGE_ROOT_DIR) + "/PCD/" + std::to_string(std::time(nullptr)) + "_ikdtree_" + pcd_save_name;
            } else {
                ikdtree_path = string(PACKAGE_ROOT_DIR) + "/PCD/ikdtree_" + pcd_save_name;
            }
            pcl::io::savePCDFileBinary(ikdtree_path, *cloud_ikdtree);
            std::cout << "save ikdtree pcd file: " << ikdtree_path << std::endl;
        }
        runtimeLogger().flushIfDue(true);
        if (runtimeLogger().config().enabled &&
            runtimeLogger().config().file.enabled &&
            runtimeLogger().config().file.ikdtree_snapshot.enabled) {
            PointVector pts;
            ikdtree.flatten(ikdtree.Root_Node, pts, NOT_RECORD);
            if (!pts.empty()) {
                pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
                cloud->points = pts;
                cloud->width = pts.size();
                cloud->height = 1;
                cloud->is_dense = true;
                std::string snap_path = runtimeLogger().makePrefixedPath(
                    LogFileChannel::IkdTreeSnapshot,
                    "_" + std::to_string(std::time(nullptr)) + ".pcd");
                pcl::io::savePCDFileBinary(snap_path, *cloud);
            }
        }
    }
}
