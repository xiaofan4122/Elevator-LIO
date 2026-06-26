/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Encapsulates the LiDAR processing pipeline.
 * Extracted from timer_500HZ_callback in main.cpp for clarity.
 */

#ifndef LIDAR_PIPELINE_H
#define LIDAR_PIPELINE_H

#include "support/type.h"
#include "support/SharedBuffers.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <deque>
#include <limits>

#include "AdaptiveFilter/AdaptiveVoxelPController.hpp"

class EskfEstimator;
class IMUProcess;
class ElevatorSelfExit;
template<typename PointT> class IkdMap;

class LidarPipeline {
public:
    struct Result {
        bool processed = false;
        State post_state;
        PointCloudXYZI::Ptr dedistorted_cloud;
    };

    LidarPipeline(ros::NodeHandle& nh,
                   std::shared_ptr<SharedBuffers> shared_bufs,
                   std::shared_ptr<EskfEstimator> estimator,
                   IMUProcess& imu_process,
                   ElevatorSelfExit& elevator_self_exit,
                   std::shared_ptr<IkdMap<PointType>> ikd_map);

    Result process();

private:
    struct FrameData {
        PointCloudXYZI::Ptr raw_cloud;
        double base_time = 0.0;
        double end_time = 0.0;
    };

    bool dequeueFrame(FrameData& out);
    void initAdaptiveVoxel();
    void updateAdaptiveConfig(double end_time);
    void applyVoxelFilter(const PointCloudXYZI::Ptr& in, PointCloudXYZI::Ptr& out, double blind_dist);
    bool handleFirstLidarInit(const FrameData& frame);
    bool handlePostElevatorRebuild(const FrameData& frame);
    void resolveFrameState(const FrameData& frame, State& out_state, bool& out_in_elevator);
    bool undistortFrame(const FrameData& frame, State& lidar_end_state,
                        PointCloudXYZI::Ptr& out_cloud);
    void downsampleFrame(PointCloudXYZI::Ptr& cloud);
    void runIeskfUpdate(State& post_state);
    void handleElevatorStop(State& post_state);
    void handleElevatorExit(State& post_state);
    void commitFrame(const FrameData& frame, const State& post_state);

    ros::Publisher ele_state_pub_;
    ros::Publisher elevator_estimate_pub_;

    std::shared_ptr<SharedBuffers> shared_bufs_;
    std::shared_ptr<EskfEstimator> estimator_;
    IMUProcess& imu_process_;
    ElevatorSelfExit& elevator_self_exit_;
    std::shared_ptr<IkdMap<PointType>> ikd_map_;

    PointCloudXYZI::Ptr raw_clouds_lidar_;
    PointCloudXYZI::Ptr cloud_init_accum_lidar_;
    PointCloudXYZI::Ptr cloud_init_lidar_;
    PointCloudXYZI::Ptr cloud_init_world_;
    PointCloudXYZI::Ptr rebuild_map_accum_lidar_;
    PointCloudXYZI::Ptr downsampled_cloud_;
    PointCloudXYZI::Ptr blind_filtered_;

    pcl::VoxelGrid<PointType> uniform_voxel_;
    AdaptiveVoxelPController voxel_ctrl_;
    bool voxel_ctrl_initialized_ = false;
    double current_voxel_ = 0.0;
    double current_target_points_per_frame_ = 0.0;

    double last_lidar_stamp_for_adaptive_ = -1.0;
    double smoothed_lidar_dt_for_adaptive_ = 0.0;
    bool adaptive_dt_warmed_up_ = false;

    bool first_lidar_ = true;
    int init_lidar_accum_count_ = 0;

    bool exit_from_elevator_ = false;
    bool rebuild_map_pending_ = false;
    int rebuild_map_accum_count_ = 0;

    static constexpr int kInitLidarAccumFrames = 5;
};

#endif // LIDAR_PIPELINE_H
