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
#include <pcl/filters/voxel_grid.h>
#include "support/ros_compat.h"
#include <cstdint>
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

    LidarPipeline(lio_ros::Node& nh,
                   std::shared_ptr<SharedBuffers> shared_bufs,
                   std::shared_ptr<EskfEstimator> estimator,
                   IMUProcess& imu_process,
                   ElevatorSelfExit& elevator_self_exit,
                   std::shared_ptr<IkdMap<PointType>> ikd_map);

    Result process();
    double lastCommittedTime() const {
        return temporal_anchor_valid_ ? temporal_anchor_state_.time : -1.0;
    }

private:
    struct FrameData {
        PointCloudXYZI::Ptr raw_cloud;
        double base_time = 0.0;
        double end_time = 0.0;
    };

    bool recoverAfterSensorGap(const FrameData& frame, Result& result);
    bool gap_recovery_pending_ = false;
    double handled_imu_gap_end_ = 0.0;
    double recovery_data_start_ = 0.0;
    double trusted_scan_span_s_ = 0.1;
    bool dequeueFrame(FrameData& out);
    void initAdaptiveVoxel();
    void updateAdaptiveConfig(double end_time);
    void applyVoxelFilter(const PointCloudXYZI::Ptr& in, PointCloudXYZI::Ptr& out,
                          double blind_dist, double voxel_size);
    bool handleFirstLidarInit(const FrameData& frame);
    bool handlePostElevatorRebuild(const FrameData& frame);
    void resolveFrameState(const FrameData& frame, State& out_state, bool& out_in_elevator);
    bool undistortFrame(const FrameData& frame, State& lidar_end_state,
                        PointCloudXYZI::Ptr& out_cloud);
    void downsampleFrame(PointCloudXYZI::Ptr& cloud);
    void runIeskfUpdate(State& post_state);
    bool runImuGroupedUpdate(const FrameData& frame, State& post_state,
                             int over_range_samples);
    void handleElevatorStop(State& post_state);
    void handleElevatorExit(State& post_state);
    void commitFrame(const FrameData& frame, const State& post_state,
                     bool temporal_update_done);

    lio_ros::Publisher<lio_ros::Bool> tracking_valid_pub_;
    lio_ros::Publisher<lio_ros::Bool> ele_state_pub_;
    lio_ros::Publisher<lio_ros::ElevatorState> elevator_estimate_pub_;

    std::shared_ptr<SharedBuffers> shared_bufs_;
    std::shared_ptr<EskfEstimator> estimator_;
    IMUProcess& imu_process_;
    ElevatorSelfExit& elevator_self_exit_;
    std::shared_ptr<IkdMap<PointType>> ikd_map_;

    PointCloudXYZI::Ptr raw_clouds_lidar_;
    PointCloudXYZI::Ptr cloud_init_accum_lidar_{new PointCloudXYZI};
    int init_lidar_accum_count_ = 0;
    static constexpr int kInitLidarAccumFrames = 5;
    PointCloudXYZI::Ptr cloud_init_lidar_;
    PointCloudXYZI::Ptr cloud_init_world_;
    PointCloudXYZI::Ptr rebuild_map_accum_lidar_;
    PointCloudXYZI::Ptr downsampled_cloud_;
    PointCloudXYZI::Ptr blind_filtered_;
    PointCloudXYZI::Ptr point_group_cloud_;
    PointCloudXYZI::Ptr point_world_cloud_;
    std::vector<PointVector> point_insertion_neighbors_;

    pcl::VoxelGrid<PointType> uniform_voxel_;
    AdaptiveVoxelPController voxel_ctrl_;
    bool voxel_ctrl_initialized_ = false;
    double current_voxel_ = 0.0;
    double current_target_points_per_frame_ = 0.0;
    double frame_voxel_ = 0.0;
    bool downsample_controller_updated_this_frame_ = false;
    bool downsample_logged_this_frame_ = false;

    double last_lidar_stamp_for_adaptive_ = -1.0;
    double smoothed_lidar_dt_for_adaptive_ = 0.0;
    bool adaptive_dt_warmed_up_ = false;

    // Detector input is stored before the in-place deskew mutates a scan.
    std::deque<FrameData> door_detector_window_;
    bool first_lidar_ = true;

    bool exit_from_elevator_ = false;
    bool rebuild_map_pending_ = false;
    int rebuild_map_accum_count_ = 0;

    // The on-demand IMU-group update keeps one monotonically increasing filter
    // timeline across scan boundaries. Raw Livox scan intervals can overlap
    // slightly, so the latest committed posterior is retained as its anchor.
    bool temporal_anchor_valid_ = false;
    State temporal_anchor_state_;
    double temporal_anchor_covariance_time_ = 0.0;
    double pending_temporal_covariance_time_ = 0.0;
    std::uint64_t temporal_skipped_overlap_points_ = 0;
    std::uint64_t temporal_processed_points_ = 0;

};

#endif // LIDAR_PIPELINE_H
