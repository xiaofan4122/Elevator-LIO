/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * LiDAR processing pipeline implementation.
 * Extracted from timer_500HZ_callback in main.cpp.
 */

#include "node/LidarPipeline.h"
#include "support/common_lib.h"
#include "support/LIONode.h"
#include "estimator/IMUProcess.h"
#include "estimator/ESEKF.h"
#include "elevator/ElevatorSelfExit.h"
#include "ikd_tree/IkdMap.hpp"
#include "AdaptiveFilter/AdaptiveVoxelPController.hpp"
#include <pcl/filters/voxel_grid.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <cmath>

// ===================== LidarPipeline =====================

LidarPipeline::LidarPipeline(lio_ros::Node& nh,
                               std::shared_ptr<SharedBuffers> shared_bufs,
                               std::shared_ptr<EskfEstimator> estimator,
                               IMUProcess& imu_process,
                               ElevatorSelfExit& elevator_self_exit,
                               std::shared_ptr<IkdMap<PointType>> ikd_map)
    : shared_bufs_(std::move(shared_bufs))
    , estimator_(std::move(estimator))
    , imu_process_(imu_process)
    , elevator_self_exit_(elevator_self_exit)
    , ikd_map_(std::move(ikd_map))
    , raw_clouds_lidar_(new PointCloudXYZI)
    , cloud_init_lidar_(new PointCloudXYZI)
    , cloud_init_world_(new PointCloudXYZI)
    , rebuild_map_accum_lidar_(new PointCloudXYZI)
    , downsampled_cloud_(new PointCloudXYZI)
    , blind_filtered_(new PointCloudXYZI)
    , point_group_cloud_(new PointCloudXYZI)
    , point_world_cloud_(new PointCloudXYZI)
{
    tracking_valid_pub_ = nh.advertise<lio_ros::Bool>("/LIO/tracking_valid", 1, true);
    lio_tracking_valid.store(false);
    lio_ros::Bool tracking; tracking.data = false;
    tracking_valid_pub_.publish(tracking);
    ele_state_pub_ = nh.advertise<lio_ros::Bool>("/LIO/in_elevator", 1, true);
    elevator_estimate_pub_ = nh.advertise<lio_ros::ElevatorState>("/LIO/elevator_state", 1, true);
}

// ===================== Helper: adaptive voxel init/update =====================

void LidarPipeline::initAdaptiveVoxel() {
    if (!downsample_adaptive_enabled) {
        current_voxel_ = filter_size;
        return;
    }
    AdaptiveVoxelPController::Config cfg;
    cfg.target_points = downsample_target_points;
    cfg.alpha = downsample_alpha;
    cfg.min_voxel = downsample_min_voxel;
    cfg.max_voxel = downsample_max_voxel;
    voxel_ctrl_.setConfig(cfg);
    if (!voxel_ctrl_initialized_) {
        voxel_ctrl_.setVoxel(filter_size);
        current_voxel_ = voxel_ctrl_.voxel();
        voxel_ctrl_initialized_ = true;
    }

}

void LidarPipeline::updateAdaptiveConfig(double end_time) {
    // A temporal-update attempt can fall back to the ordinary frame path.
    // Both calls must use the same voxel and advance the controller at most
    // once for this LiDAR scan.
    frame_voxel_ = current_voxel_;
    downsample_controller_updated_this_frame_ = false;
    downsample_logged_this_frame_ = false;
    if (!downsample_adaptive_enabled) return;

    double lidar_dt = 0.0;
    if (last_lidar_stamp_for_adaptive_ > 0.0 && end_time > last_lidar_stamp_for_adaptive_) {
        lidar_dt = end_time - last_lidar_stamp_for_adaptive_;
        if (adaptive_dt_warmed_up_ && smoothed_lidar_dt_for_adaptive_ > 1e-4) {
            lidar_dt = std::clamp(lidar_dt,
                                  smoothed_lidar_dt_for_adaptive_ * 0.25,
                                  smoothed_lidar_dt_for_adaptive_ * 4.0);
        }
    }
    if (lidar_dt <= 1e-4) {
        lidar_dt = smoothed_lidar_dt_for_adaptive_ > 1e-4 ? smoothed_lidar_dt_for_adaptive_ : 0.1;
    }
    if (smoothed_lidar_dt_for_adaptive_ <= 1e-4) {
        smoothed_lidar_dt_for_adaptive_ = lidar_dt;
    } else {
        constexpr double kAdaptiveDtEma = 0.2;
        smoothed_lidar_dt_for_adaptive_ =
            (1.0 - kAdaptiveDtEma) * smoothed_lidar_dt_for_adaptive_ + kAdaptiveDtEma * lidar_dt;
    }
    adaptive_dt_warmed_up_ = adaptive_dt_warmed_up_ || (last_lidar_stamp_for_adaptive_ > 0.0);
    last_lidar_stamp_for_adaptive_ = end_time;

    current_target_points_per_frame_ =
        std::max(1.0, downsample_target_points * smoothed_lidar_dt_for_adaptive_);
    AdaptiveVoxelPController::Config cfg = voxel_ctrl_.config();
    cfg.target_points = current_target_points_per_frame_;
    voxel_ctrl_.setConfig(cfg);
}

void LidarPipeline::applyVoxelFilter(const PointCloudXYZI::Ptr& in,
                                      PointCloudXYZI::Ptr& out,
                                      double blind_dist,
                                      double voxel_size) {
    out->clear();
    if (!in || in->empty()) return;
    blind_filtered_->clear();
    const double blind_sq = blind_dist * blind_dist;
    blind_filtered_->points.reserve(in->points.size());
    for (const auto& pt : in->points) {
        double r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (r2 < blind_sq) continue;
        blind_filtered_->points.push_back(pt);
    }
    if (blind_filtered_->empty()) return;
    const float leaf_size = static_cast<float>(voxel_size);
    uniform_voxel_.setLeafSize(leaf_size, leaf_size, leaf_size);
    uniform_voxel_.setInputCloud(blind_filtered_);
    uniform_voxel_.filter(*out);
}

// ===================== Frame dequeue =====================

bool LidarPipeline::dequeueFrame(FrameData& out) {
    auto frame_opt = shared_bufs_->getFrontLidarFrameSafe();
    if (!frame_opt.has_value()) return false;
    const auto& frame = *frame_opt;
    out.raw_cloud = frame.cloud;
    out.base_time = frame.base_time;
    out.end_time = frame.end_time;
    return true;
}

// ===================== Step 1: Initial map build =====================

bool LidarPipeline::handleFirstLidarInit(const FrameData& frame) {
    if (!first_lidar_ || relocation_enable) return false;

    if (!ikd_map_->usesIvox()) {
        cloud_init_accum_lidar_->points.insert(
            cloud_init_accum_lidar_->points.end(),
            frame.raw_cloud->points.begin(), frame.raw_cloud->points.end());
        cloud_init_accum_lidar_->width = static_cast<uint32_t>(cloud_init_accum_lidar_->points.size());
        cloud_init_accum_lidar_->height = 1;
        cloud_init_accum_lidar_->is_dense = false;
        init_lidar_accum_count_++;

        if (init_lidar_accum_count_ < kInitLidarAccumFrames) {
            shared_bufs_->popFrontLidarFrameSafe();
            return true; // still accumulating
        }

        ikdtree.set_downsample_param(min_filter_size);
        applyVoxelFilter(cloud_init_accum_lidar_, cloud_init_lidar_, blind, min_filter_size);
        cloud_init_world_ = estimator_->transLidar2World(*cloud_init_lidar_);
        ikd_map_->add(cloud_init_world_);
        std::cout << "[LidarPipeline] Build Map Model with "
                  << kInitLidarAccumFrames << " lidar frames" << std::endl;

        first_lidar_ = false;
        init_lidar_accum_count_ = 0;
        cloud_init_accum_lidar_->clear();
        shared_bufs_->popFrontLidarFrameSafe();
        return true; // handled
    }

    // Point-LIO's MID360 launch seeds IVox with one full, undistorted scan.
    // LiDAR frames can have accumulated while the IMU was initializing. Drop
    // those stale frames just as Point-LIO does until a whole scan is covered.
    if (!imu_process_.coversTimeRange(frame.base_time, frame.end_time)) {
        shared_bufs_->popFrontLidarFrameSafe();
        return true;
    }
    State init_state;
    bool init_in_elevator = false;
    resolveFrameState(frame, init_state, init_in_elevator);
    if (!undistortFrame(frame, init_state, cloud_init_lidar_)) {
        shared_bufs_->popFrontLidarFrameSafe();
        return true;
    }
    estimator_->set_cur_state(init_state);
    cloud_init_world_ = estimator_->transLidar2World(*cloud_init_lidar_);
    PointCloudXYZI::Ptr initial_map_cloud = cloud_init_world_;
    PointCloudXYZI::Ptr fused_initial_map;
    if (ivox_initial_voxel_size > 0.0) {
        fused_initial_map.reset(new PointCloudXYZI);
        pcl::VoxelGrid<PointType> initial_voxel;
        const float leaf = static_cast<float>(ivox_initial_voxel_size);
        initial_voxel.setLeafSize(leaf, leaf, leaf);
        initial_voxel.setInputCloud(cloud_init_world_);
        initial_voxel.filter(*fused_initial_map);
        initial_map_cloud = fused_initial_map;
    }
    ikd_map_->add(initial_map_cloud);
    std::cout << "[LidarPipeline] Build Point-LIO initial map with "
              << initial_map_cloud->size() << " points from one full lidar frame, initial voxel="
              << ivox_initial_voxel_size << " m" << std::endl;

    first_lidar_ = false;
    shared_bufs_->popFrontLidarFrameSafe();
    return true; // handled
}

// ===================== Step 2: Post-elevator map rebuild =====================

bool LidarPipeline::handlePostElevatorRebuild(const FrameData& frame) {
    if (!rebuild_map_pending_ || relocation_enable) return false;

    rebuild_map_accum_lidar_->points.insert(
        rebuild_map_accum_lidar_->points.end(),
        frame.raw_cloud->points.begin(), frame.raw_cloud->points.end());
    rebuild_map_accum_lidar_->width = static_cast<uint32_t>(rebuild_map_accum_lidar_->points.size());
    rebuild_map_accum_lidar_->height = 1;
    rebuild_map_accum_lidar_->is_dense = false;
    ++rebuild_map_accum_count_;

    if (rebuild_map_accum_count_ < elevator_rebuild_map_on_exit_frames ||
        static_cast<int>(rebuild_map_accum_lidar_->points.size()) < elevator_rebuild_map_on_exit_min_points) {
        shared_bufs_->popFrontLidarFrameSafe();
        return true; // still accumulating
    }

    applyVoxelFilter(rebuild_map_accum_lidar_, cloud_init_lidar_, blind, current_voxel_);
    cloud_init_world_ = estimator_->transLidar2World(*cloud_init_lidar_);
    if (map_world) map_world->clear();
    incremental_paths.clear();
    cub_needrm.clear();
    if (ikd_map_) {
        ikd_map_->reset();
        ikd_map_->add(cloud_init_world_);
    }
    if (global_map_pub_enable && map_world) {
        *map_world = *cloud_init_world_;
    }
    rebuild_map_pending_ = false;
    rebuild_map_accum_count_ = 0;
    rebuild_map_accum_lidar_->clear();
    LOG_WARN(Map, "[LidarPipeline] rebuilt map after elevator exit with "
                      << cloud_init_world_->points.size() << " points from "
                      << elevator_rebuild_map_on_exit_frames << " lidar frames");
    shared_bufs_->popFrontLidarFrameSafe();
    return true; // handled
}

// ===================== Step 3: Resolve frame state =====================

void LidarPipeline::resolveFrameState(const FrameData& frame, State& out_state, bool& out_in_elevator) {
    imu_process_.get_state_at_t(out_state, frame.end_time);
    if (!elevator_door_accumulate_enable) {
        door_detector_window_.clear();
        imu_process_.elev_process.processDoorDetector(
            *frame.raw_cloud, frame.end_time, out_state,
            FSM::current_state == FSM::State::LidarProcess);
    } else if (!out_state.in_elevator) {
        FrameData snapshot = frame;
        snapshot.raw_cloud.reset(new PointCloudXYZI(*frame.raw_cloud));
        door_detector_window_.push_back(std::move(snapshot));
        constexpr double tolerance = 1e-3;
        while (door_detector_window_.size() > 1 &&
               frame.end_time - door_detector_window_[1].base_time >=
                   elevator_door_accumulate_window_s - tolerance) {
            door_detector_window_.pop_front();
        }
        if (frame.end_time - door_detector_window_.front().base_time >=
                elevator_door_accumulate_window_s - tolerance) {
            PointCloudXYZI accumulated;
            for (const auto& sample : door_detector_window_) {
                accumulated.points.insert(accumulated.points.end(),
                    sample.raw_cloud->points.begin(), sample.raw_cloud->points.end());
            }
            // Ranges stay in each scan's own sensor frame. Detector input is
            // independent of estimated poses and uses a fixed data-time span.
            imu_process_.elev_process.processDoorDetector(
                accumulated, frame.end_time, out_state,
                FSM::current_state == FSM::State::LidarProcess);
        }
    } else {
        door_detector_window_.clear();
    }
    out_in_elevator = out_state.in_elevator;
}

// ===================== Step 4: Undistortion =====================

bool LidarPipeline::undistortFrame(const FrameData& frame, State& lidar_end_state,
                                    PointCloudXYZI::Ptr& out_cloud) {
    *out_cloud = *frame.raw_cloud;
    if (exit_from_elevator_) return true;

    std::vector<Pose> imu_pose_vec;
    imu_process_.get_imu_pose_from_t1_to_t2(imu_pose_vec, frame.base_time, frame.end_time);
    if (imu_pose_vec.empty()) return false;

    auto it_pcl = out_cloud->points.begin();
    for (auto it_pose = imu_pose_vec.begin(); it_pose + 1 != imu_pose_vec.end(); ++it_pose) {
        const Pose& head = *it_pose;
        const Pose& tail = *(it_pose + 1);
        const Eigen::Quaterniond q_imu = head.q;
        const Eigen::Vector3d vel_imu = head.v;
        const Eigen::Vector3d pos_imu = head.p;
        const Eigen::Vector3d acc_imu = tail.acc;
        const Eigen::Vector3d gyr_imu = tail.gyr;

        for (; it_pcl != out_cloud->points.end(); ++it_pcl) {
            double point_time = it_pcl->curvature / 1000.0;
            if (point_time > tail.offset_time + 1e-6) break;
            double dt = point_time - head.offset_time;
            Eigen::Vector3d P_L(it_pcl->x, it_pcl->y, it_pcl->z);
            Eigen::Vector3d P_I = imu_R_lidar * P_L + imu_t_lidar;
            Eigen::Matrix3d R_I_W(q_imu * Exp(gyr_imu, dt));
            Eigen::Vector3d T_I_W(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt);
            Eigen::Vector3d P_W = R_I_W * P_I + T_I_W;
            Eigen::Vector3d P_IE = lidar_end_state.q.inverse() * (P_W - lidar_end_state.p);
            Eigen::Vector3d P_LE = imu_R_lidar.transpose() * (P_IE - imu_t_lidar);
            it_pcl->x = static_cast<float>(P_LE.x());
            it_pcl->y = static_cast<float>(P_LE.y());
            it_pcl->z = static_cast<float>(P_LE.z());
        }
        if (it_pcl == out_cloud->points.end()) break;
    }
    return true;
}

// ===================== Step 5: Downsampling =====================

void LidarPipeline::downsampleFrame(PointCloudXYZI::Ptr& cloud) {
    if (!cloud) {
        downsampled_cloud_->clear();
        return;
    }
    const double target_points_per_frame = current_target_points_per_frame_;
    const std::size_t points_before = cloud->points.size();
    const double voxel_used = frame_voxel_;
    // Every update schedule uses the same continuous controller and the same
    // single downsampled cloud for this scan.  Filtering even a tiny/empty
    // scan is intentional: reusing the previous scan here would feed a stale
    // LiDAR observation into the filter.
    applyVoxelFilter(cloud, downsampled_cloud_, blind, voxel_used);
    if (cloud->points.size() > 10) {
        static double last_downsample_print_time = -1.0;
        if (last_downsample_print_time < 0.0 || (lidar_end_time - last_downsample_print_time) > 5.0) {
            std::cout << "[LidarPipeline] downsample: in=" << cloud->points.size()
                      << " out=" << downsampled_cloud_->points.size() << std::endl;
            last_downsample_print_time = lidar_end_time;
        }
    } else {
        static double last_no_downsample_print_time = -1.0;
        if (last_no_downsample_print_time < 0.0 || (lidar_end_time - last_no_downsample_print_time) > 5.0) {
            LOG_INFO(Map, "[LidarPipeline] sparse scan: in=" << cloud->points.size()
                          << " out=" << downsampled_cloud_->points.size());
            last_no_downsample_print_time = lidar_end_time;
        }
    }
    const std::size_t points_after = downsampled_cloud_->points.size();

    if (!downsample_controller_updated_this_frame_) {
        if (downsample_adaptive_enabled && !downsampled_cloud_->points.empty()) {
            current_voxel_ = voxel_ctrl_.update(points_after);
        }
        downsample_controller_updated_this_frame_ = true;
    }

    if (log_save_enable && !downsample_logged_this_frame_) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << lidar_end_time << "," << voxel_used << ","
            << points_after << "," << points_before << ","
            << target_points_per_frame << ","
            << smoothed_lidar_dt_for_adaptive_ << ","
            << downsample_target_points << ","
            << current_voxel_;
        runtimeLogger().writeLine(LogFileChannel::AdaptiveDownsample,
                                  "time,voxel_size,points_after,points_before,target_points_frame,lidar_dt,target_points_per_second,next_voxel",
                                  oss.str(), 0, lidar_end_time);
        downsample_logged_this_frame_ = true;
    }
}

// ===================== Step 6: IESKF update =====================

void LidarPipeline::runIeskfUpdate(State& post_state) {
    estimator_->state_vector_prior_.clear();
    estimator_->state_vector_prior_.push_back(post_state);
    estimator_->set_cur_state(post_state);
    LIONode::lasermap_fov_segment();
    estimator_->UpdateByLidarIESKF(*downsampled_cloud_);
    post_state = estimator_->get_cur_state();
}

bool LidarPipeline::runImuGroupedUpdate(const FrameData& frame, State& post_state,
                                        int over_range_samples) {
    raw_clouds_lidar_ = frame.raw_cloud;
    downsampleFrame(raw_clouds_lidar_);
    if (!downsampled_cloud_ || downsampled_cloud_->empty()) return false;

    std::stable_sort(downsampled_cloud_->points.begin(), downsampled_cloud_->points.end(),
                     [](const PointType &lhs, const PointType &rhs) {
                         return lhs.curvature < rhs.curvature;
                     });
    const auto pointTime = [&](const PointType &point) {
        const double offset_s =
                std::max(0.0, static_cast<double>(point.curvature) / 1000.0);
        return std::clamp(frame.base_time + offset_s,
                          frame.base_time, frame.end_time);
    };

    const bool have_committed_anchor = temporal_anchor_valid_;
    const double committed_time = have_committed_anchor
            ? temporal_anchor_state_.time
            : pointTime(downsampled_cloud_->points.front());
    const double last_point_time = pointTime(downsampled_cloud_->points.back());
    if (last_point_time <= committed_time + 1e-9) {
        LOG_WARN(Core, "IMU-group scan has no point newer than committed state: scan_last="
                 << std::fixed << std::setprecision(9) << last_point_time
                 << " committed=" << committed_time);
        return false;
    }

    IMUProcess::ReplayCursor replay;
    if (!imu_process_.beginReplay(committed_time, last_point_time, replay, true)) {
        return false;
    }
    if (have_committed_anchor) {
        replay.state = temporal_anchor_state_;
        replay.state.time = committed_time;
        replay.covariance_time = temporal_anchor_covariance_time_;
    }
    // A scan straddling the elevator exit can have an outside end state even
    // though its committed anchor is still inside.  Never start the augmented
    // IMU-group LiDAR path from such an anchor; use the normal frame path.
    if (!imu_update_enable || replay.state.in_elevator ||
        !imu_process_.activateOutputReplay(replay)) return false;

    struct TimedPose {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double time = 0.0;
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
    };
    std::vector<TimedPose, Eigen::aligned_allocator<TimedPose>> pose_history;
    pose_history.reserve(downsampled_cloud_->size() + 64);
    pose_history.push_back({replay.state.time, replay.state.q, replay.state.p});

    point_world_cloud_->clear();
    point_world_cloud_->points.reserve(downsampled_cloud_->size());
    point_insertion_neighbors_.clear();
    point_insertion_neighbors_.reserve(downsampled_cloud_->size());
    if (down_effect_cloud_lidar) down_effect_cloud_lidar->clear();
    if (down_reject_cloud_lidar) down_reject_cloud_lidar->clear();

    estimator_->state_vector_prior_.clear();
    estimator_->state_vector_prior_.push_back(replay.state);
    estimator_->set_cur_state(replay.state);
    LIONode::lasermap_fov_segment();

    const auto begin_tp = std::chrono::steady_clock::now();
    const Eigen::Vector3d omega_before = replay.state.omg;
    const Eigen::Vector3d acc_before = replay.state.acc;
    std::size_t group_begin = 0;
    int group_count = 0;
    int matched_points = 0;
    int updated_points = 0;
    int skipped_overlap_points = 0;

    while (group_begin < downsampled_cloud_->size() &&
           pointTime(downsampled_cloud_->points[group_begin]) <=
                   committed_time + 1e-9) {
        ++group_begin;
        ++skipped_overlap_points;
    }

    while (group_begin < downsampled_cloud_->size()) {
        const double first_point_time =
                pointTime(downsampled_cloud_->points[group_begin]);
        double group_time = last_point_time;
        for (const auto &sample : replay.samples) {
            if (sample.time > replay.state.time + 1e-9 &&
                sample.time >= first_point_time - 1e-9) {
                group_time = std::min(sample.time, last_point_time);
                break;
            }
        }

        std::size_t group_end = group_begin;
        while (group_end < downsampled_cloud_->size() &&
               pointTime(downsampled_cloud_->points[group_end]) <=
                       group_time + 1e-9) {
            ++group_end;
        }
        if (group_end == group_begin) {
            LOG_ERROR(Core, "failed to assign LiDAR point to IMU interval at t="
                      << std::fixed << std::setprecision(9) << first_point_time);
            return false;
        }

        std::vector<TimedPose, Eigen::aligned_allocator<TimedPose>> acquisition_poses;
        acquisition_poses.reserve(group_end - group_begin);
        for (std::size_t i = group_begin; i < group_end; ++i) {
            const double stamp = pointTime(downsampled_cloud_->points[i]);
            if (!imu_process_.propagateReplay(replay, stamp)) return false;
            acquisition_poses.push_back({stamp, replay.state.q, replay.state.p});
            pose_history.push_back(acquisition_poses.back());
        }
        if (!imu_process_.propagateReplay(replay, group_time)) return false;

        point_group_cloud_->clear();
        point_group_cloud_->points.reserve(group_end - group_begin);
        for (std::size_t i = group_begin; i < group_end; ++i) {
            const PointType &raw_point = downsampled_cloud_->points[i];
            const TimedPose &acquisition = acquisition_poses[i - group_begin];
            const Eigen::Vector3d p_l(raw_point.x, raw_point.y, raw_point.z);
            const Eigen::Vector3d p_i = imu_R_lidar * p_l + imu_t_lidar;
            const Eigen::Vector3d p_w = acquisition.q * p_i + acquisition.p;
            const Eigen::Vector3d p_i_group =
                    replay.state.q.inverse() * (p_w - replay.state.p);
            const Eigen::Vector3d p_l_group =
                    imu_R_lidar.transpose() * (p_i_group - imu_t_lidar);
            PointType undistorted = raw_point;
            undistorted.x = static_cast<float>(p_l_group.x());
            undistorted.y = static_cast<float>(p_l_group.y());
            undistorted.z = static_cast<float>(p_l_group.z());
            undistorted.curvature = static_cast<float>(
                    std::max(0.0, group_time - frame.base_time) * 1000.0);
            point_group_cloud_->points.push_back(undistorted);
        }
        point_group_cloud_->width =
                static_cast<std::uint32_t>(point_group_cloud_->size());
        point_group_cloud_->height = 1;
        point_group_cloud_->is_dense = true;

        estimator_->set_cur_state(replay.state);
        if (!replay.output_model_active || replay.state.in_elevator) {
            LOG_WARN(Core, "IMU-group replay reached elevator mode; fallback to frame update at t="
                     << std::fixed << std::setprecision(9) << group_time);
            return false;
        }
        EskfEstimator::PointUpdateResult update =
                estimator_->UpdateByLidarImuGroup(*point_group_cloud_);
        matched_points += update.matched_points;
        updated_points += update.updated_points;
        point_insertion_neighbors_.insert(
                point_insertion_neighbors_.end(),
                std::make_move_iterator(update.insertion_neighbors.begin()),
                std::make_move_iterator(update.insertion_neighbors.end()));
        replay.state = estimator_->get_cur_state();
        replay.state.time = group_time;
        pose_history.push_back({group_time, replay.state.q, replay.state.p});

        for (const PointType &point_lidar : point_group_cloud_->points) {
            const Eigen::Vector3d p_l(point_lidar.x, point_lidar.y, point_lidar.z);
            const Eigen::Vector3d p_i = imu_R_lidar * p_l + imu_t_lidar;
            const Eigen::Vector3d p_w = replay.state.q * p_i + replay.state.p;
            PointType point_world = point_lidar;
            point_world.x = static_cast<float>(p_w.x());
            point_world.y = static_cast<float>(p_w.y());
            point_world.z = static_cast<float>(p_w.z());
            point_world_cloud_->points.push_back(point_world);
        }
        ++group_count;
        group_begin = group_end;
    }

    if (group_count == 0) return false;

    post_state = replay.state;
    imu_process_.deactivateOutputReplay(post_state);
    pending_temporal_covariance_time_ = replay.covariance_time;
    estimator_->set_cur_state(post_state);
    estimator_->state_vector_post_.clear();
    estimator_->state_vector_post_.push_back(post_state);

    temporal_skipped_overlap_points_ +=
            static_cast<std::uint64_t>(skipped_overlap_points);
    temporal_processed_points_ +=
            static_cast<std::uint64_t>(point_world_cloud_->size());
    point_world_cloud_->width =
            static_cast<std::uint32_t>(point_world_cloud_->size());
    point_world_cloud_->height = 1;
    point_world_cloud_->is_dense = true;

    downsampled_cloud_->clear();
    downsampled_cloud_->points.reserve(point_world_cloud_->size());
    for (const PointType &point_world : point_world_cloud_->points) {
        const Eigen::Vector3d p_w(point_world.x, point_world.y, point_world.z);
        const Eigen::Vector3d p_i = post_state.q.inverse() * (p_w - post_state.p);
        const Eigen::Vector3d p_l =
                imu_R_lidar.transpose() * (p_i - imu_t_lidar);
        PointType point_final = point_world;
        point_final.x = static_cast<float>(p_l.x());
        point_final.y = static_cast<float>(p_l.y());
        point_final.z = static_cast<float>(p_l.z());
        downsampled_cloud_->points.push_back(point_final);
    }
    downsampled_cloud_->width =
            static_cast<std::uint32_t>(downsampled_cloud_->size());
    downsampled_cloud_->height = 1;
    downsampled_cloud_->is_dense = true;

    const auto transform_cloud_to_group_end = [&](PointCloudXYZI &cloud) {
        for (PointType &point : cloud.points) {
            const double offset_s =
                    std::max(0.0, static_cast<double>(point.curvature) / 1000.0);
            const double stamp = std::clamp(frame.base_time + offset_s,
                                            frame.base_time, frame.end_time);
            const auto upper = std::upper_bound(
                    pose_history.begin(), pose_history.end(), stamp + 1e-9,
                    [](double time, const TimedPose &pose) {
                        return time < pose.time;
                    });
            const TimedPose &pose = upper == pose_history.begin()
                    ? pose_history.front() : *(upper - 1);
            const Eigen::Vector3d p_l(point.x, point.y, point.z);
            const Eigen::Vector3d p_i = imu_R_lidar * p_l + imu_t_lidar;
            const Eigen::Vector3d p_w = pose.q * p_i + pose.p;
            const Eigen::Vector3d p_i_end =
                    post_state.q.inverse() * (p_w - post_state.p);
            const Eigen::Vector3d p_l_end =
                    imu_R_lidar.transpose() * (p_i_end - imu_t_lidar);
            point.x = static_cast<float>(p_l_end.x());
            point.y = static_cast<float>(p_l_end.y());
            point.z = static_cast<float>(p_l_end.z());
        }
    };

    PointCloudXYZI::Ptr published_cloud(new PointCloudXYZI(*frame.raw_cloud));
    transform_cloud_to_group_end(*published_cloud);
    if (down_effect_cloud_lidar) transform_cloud_to_group_end(*down_effect_cloud_lidar);
    if (down_reject_cloud_lidar) transform_cloud_to_group_end(*down_reject_cloud_lidar);
    raw_clouds_lidar_ = published_cloud;

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin_tp).count();
    LOG_WARN(Core, "IMU-group LiDAR update: over_range_samples="
             << over_range_samples << " groups=" << group_count
             << " points=" << point_world_cloud_->size()
             << " matched=" << matched_points
             << " updated=" << updated_points
             << " skipped_overlap=" << skipped_overlap_points
             << " domega=" << (post_state.omg - omega_before).norm()
             << " dacc=" << (post_state.acc - acc_before).norm()
             << " filter_time=" << std::fixed << std::setprecision(9)
             << post_state.time << " elapsed_ms=" << elapsed_ms);
    return true;
}

// ===================== Step 7: Elevator stop handling =====================

void LidarPipeline::handleElevatorStop(State& post_state) {
    if (!elevator_enable) return;
    if (!post_state.in_elevator || exit_from_elevator_) return;
    if (!elevator_self_exit_.stopCheckConfirmed()) return;

    int zupt_count = elevator_self_exit_.stopCheckLidarZuptCount();
    if (elevator_zupt_enable && zupt_count < 3) {
        imu_process_.elev_process.applyZeroVelocityUpdateZ(post_state);
        estimator_->set_cur_state(post_state);
        elevator_self_exit_.setStopCheckLidarZuptCount(++zupt_count);
        LOG_INFO(Elevator, "applyZeroVelocityUpdateZ after lidar update, count=" << zupt_count);
        if (zupt_count >= 3) {
            ELEVATOR_TRIGGER = false;
            elevator_self_exit_.reset();
            LOG_INFO(Elevator, "full stop confirmed + 3x lidar ZUPT done, exiting mode");
        }
    } else if (!elevator_zupt_enable) {
        ELEVATOR_TRIGGER = false;
        elevator_self_exit_.reset();
        LOG_INFO(Elevator, "full stop confirmed, exiting elevator mode without ZUPT update");
    }
}

// ===================== Step 8: Elevator exit =====================

void LidarPipeline::handleElevatorExit(State& post_state) {
    if (!elevator_enable) return;
    if (!exit_from_elevator_) return;
    exit_from_elevator_ = false;
    EXIT_FROM_ELEVATOR = false;
    elevator_self_exit_.reset();

    const double world_z_before_commit = post_state.p(2);
    const double segment_z_before_zupt = post_state.z;

    if (elevator_zupt_enable) {
        imu_process_.elev_process.applyZeroVelocityUpdateZ(post_state);
        LOG_INFO(Elevator, "applyZeroVelocityUpdateZ");
    } else {
        LOG_INFO(Elevator, "skip exit ZeroVelocityUpdateZ because elevator.zupt.enable=false");
    }

    const double delta_z_commit = post_state.z;
    post_state.p(2) += delta_z_commit;
    if (elevator_exit_cov_jacobian_enable) {
        // Re-anchor p_z <- p_z + z before either reset clears the cross terms:
        // P <- J P J^T, J = I + e_pz e_z^T (the revised paper's reset).
        const int pz = StateIndex::P + 2;
        post_state.P.row(pz) += post_state.P.row(StateIndex::Z);
        post_state.P.col(pz) += post_state.P.col(StateIndex::Z);
    }

    if (elevator_exit_icp_z_enable) {
        const EskfEstimator::ZIcpResult icp_result =
                estimator_->matchZByIcpAgainstIkdTree(*downsampled_cloud_, post_state);
        if (icp_result.accepted) {
            post_state.p(2) += icp_result.correction_z;
            LOG_INFO(Elevator, "[exit_icp_z] accepted correction_z=" << icp_result.correction_z
                       << " world_z=" << post_state.p(2)
                       << " points=" << icp_result.effective_points
                       << " info_ratio=" << icp_result.information_ratio
                       << " rmse=" << icp_result.rmse
                       << " iterations=" << icp_result.iterations);
        } else {
            LOG_WARN(Elevator, "[exit_icp_z] rejected reason=" << icp_result.reason
                       << " correction_z=" << icp_result.correction_z
                       << " degenerate=" << icp_result.degenerate
                       << " points=" << icp_result.effective_points
                       << " info_ratio=" << icp_result.information_ratio
                       << " rmse=" << icp_result.rmse
                       << " iterations=" << icp_result.iterations);
        }
    }

    std::ostringstream exit_commit_oss;
    exit_commit_oss << std::fixed << std::setprecision(6)
                    << "[exit_commit] time:" << lidar_end_time
                    << " world_z_before:" << world_z_before_commit
                    << " segment_z_before_zupt:" << segment_z_before_zupt
                    << " delta_z_commit:" << delta_z_commit
                    << " world_z_after:" << post_state.p(2)
                    << " vz_after:" << post_state.vz
                    << " az_after:" << post_state.az
                    << " Pzz_after:" << post_state.P(StateIndex::Z, StateIndex::Z);
    LOG_INFO(Elevator, exit_commit_oss.str());
    runtimeLogger().writeLine(LogFileChannel::ElevatorZupt, "", exit_commit_oss.str());

    post_state.z = 0;
    post_state.vz = 0;
    post_state.az = 0;
    imu_process_.elev_process.ElevatorModeExit(post_state);
    imu_process_.elev_process.finalizeExitCovariance(post_state);

    estimator_->set_cur_state(post_state);
    LOG_INFO(Elevator, "[LidarPipeline] EXIT_FROM_ELEVATOR commit done");

    if (elevator_rebuild_map_on_exit_enable && !relocation_enable) {
        rebuild_map_pending_ = true;
        rebuild_map_accum_count_ = 0;
        rebuild_map_accum_lidar_->clear();
        LOG_WARN(Map, "[LidarPipeline] schedule map rebuild after elevator exit"
                          << " frames=" << elevator_rebuild_map_on_exit_frames
                          << " min_points=" << elevator_rebuild_map_on_exit_min_points);
    } else {
        applyVoxelFilter(raw_clouds_lidar_, cloud_init_lidar_, blind, current_voxel_);
        cloud_init_world_ = estimator_->transLidar2World(*cloud_init_lidar_);
        ikd_map_->add(cloud_init_world_);
    }
}

// ===================== Step 9: Commit frame (state update, map, pop) =====================

void LidarPipeline::commitFrame(const FrameData& frame, const State& post_state,
                                bool temporal_update_done) {
    lidar_end_time = frame.end_time;

    State committed_state = post_state;
    const double state_commit_time = temporal_update_done
            ? post_state.time
            : frame.end_time;
    committed_state.time = state_commit_time;
    imu_process_.set_state_at_t(
            committed_state, state_commit_time,
            temporal_update_done ? pending_temporal_covariance_time_ : state_commit_time);
    // Keep the latest committed posterior as the causal anchor even when this
    // scan used the ordinary frame update.  A later on-demand temporal update
    // must never restart from an overlapping raw scan time before this state.
    const double scan_span = frame.end_time - frame.base_time;
    if (scan_span > 1e-4 && scan_span < 1.0) {
        trusted_scan_span_s_ = temporal_anchor_valid_
                ? 0.9 * trusted_scan_span_s_ + 0.1 * scan_span : scan_span;
    }
    temporal_anchor_state_ = committed_state;
    temporal_anchor_covariance_time_ = temporal_update_done
            ? pending_temporal_covariance_time_ : state_commit_time;
    temporal_anchor_valid_ = true;
    lio_tracking_valid.store(true);
    lio_ros::Bool tracking; tracking.data = true;
    tracking_valid_pub_.publish(tracking);

    lio_ros::Bool msg;
    msg.data = post_state.in_elevator;
    ele_state_pub_.publish(msg);

    lio_ros::ElevatorState elevator_msg;
    elevator_msg.header.stamp = get_ros_time(state_commit_time);
    elevator_msg.header.frame_id = "world";
    elevator_msg.in_elevator = post_state.in_elevator;
    elevator_msg.displacement = post_state.z;
    elevator_msg.velocity = post_state.vz;
    elevator_msg.acceleration = post_state.az;
    elevator_estimate_pub_.publish(elevator_msg);

    shared_bufs_->popFrontLidarFrameSafe();

    if (!rebuild_map_pending_) {
        if (temporal_update_done && !relocation_enable && ikd_map_) {
            // Transform with the final committed pose, but preserve the
            // insertion decision based on the neighbors seen by each point's
            // measurement update.
            estimator_->set_cur_state(post_state);
            PointCloudXYZI::Ptr world_cloud =
                    estimator_->transLidar2World(*downsampled_cloud_);
            ikd_map_->addWithMeasurementNeighbors(
                    world_cloud, point_insertion_neighbors_);
        } else {
            LIONode::map_incremental(*downsampled_cloud_);
        }
    }
}

// ===================== Main process entry =====================

LidarPipeline::Result LidarPipeline::process() {
    Result result;
    initAdaptiveVoxel();

    if (!elevator_enable) {
        ELEVATOR_TRIGGER = false;
        EXIT_FROM_ELEVATOR = false;
    }
    exit_from_elevator_ = elevator_enable && EXIT_FROM_ELEVATOR;

    // Dispatch by FSM state
    switch (FSM::current_state) {
        case FSM::State::Waitting:
        case FSM::State::Initializing:
        case FSM::State::IMUProcess:
            return result;
        case FSM::State::ImgProcess:
            FSM::dispatch(FSM::Event::ImgEnd);
            return result;
        case FSM::State::LidarProcess:
            break; // proceed below
    }

    // [1] Dequeue frame
    FrameData frame;
    if (!dequeueFrame(frame)) {
        FSM::dispatch(FSM::Event::LidarEnd);
        return result;
    }

    if (sensor_gap_recovery_enable && recoverAfterSensorGap(frame, result)) {
        FSM::dispatch(FSM::Event::LidarEnd);
        return result;
    }

    // [2] Adaptive voxel config
    updateAdaptiveConfig(frame.end_time);

    // [3] Handle initial map build (first N frames)
    if (handleFirstLidarInit(frame)) {
        FSM::dispatch(FSM::Event::LidarEnd);
        return result;
    }

    // [4] Handle post-elevator map rebuild
    if (handlePostElevatorRebuild(frame)) {
        FSM::dispatch(FSM::Event::LidarEnd);
        return result;
    }

    // [5] Resolve elevator state
    State lidar_end_state;
    bool in_elevator = false;
    resolveFrameState(frame, lidar_end_state, in_elevator);

    // [6] Outside elevator mode, an over-range scan switches to one accumulated
    // LiDAR update per adjacent IMU interval. Elevator scans always remain on
    // the original frame-wise update path.
    raw_clouds_lidar_ = frame.raw_cloud;
    State post_state = lidar_end_state;
    int over_range_samples = 0;
    const bool request_imu_group_update = imu_update_enable &&
            !ELEVATOR_TRIGGER &&
            !lidar_end_state.in_elevator &&
            imu_process_.hasSaturationBetween(
                    frame.base_time, frame.end_time, &over_range_samples);
    bool imu_group_update_done = false;
    if (request_imu_group_update) {
        imu_group_update_done = runImuGroupedUpdate(
                frame, post_state, over_range_samples);
        if (!imu_group_update_done) {
            LOG_WARN(Core, "IMU-group replay unavailable; fallback to frame update at t="
                     << frame.end_time);
        }
    }

    if (!imu_group_update_done) {
        // Preserve the original frame-mode preprocessing behavior when the
        // grouped update is disabled, unnecessary, or cannot be replayed.
        if (in_elevator && !exit_from_elevator_) {
            // Legacy frame mode skips undistortion while the cabin is moving.
        } else if (!undistortFrame(frame, lidar_end_state, raw_clouds_lidar_)) {
            shared_bufs_->popFrontLidarFrameSafe();
            FSM::dispatch(FSM::Event::LidarEnd);
            std::cerr << "[LidarPipeline] IMU Data Not Enough" << std::endl;
            return result;
        }
        downsampleFrame(raw_clouds_lidar_);
        runIeskfUpdate(post_state);
    }

    // [9] Elevator stop handling
    handleElevatorStop(post_state);

    // [10] Elevator exit handling
    handleElevatorExit(post_state);

    // [11] Commit frame state, update map, pop buffer
    commitFrame(frame, post_state, imu_group_update_done);

    // [12] Return results for caller to publish
    result.processed = true;
    result.post_state = post_state;
    result.dedistorted_cloud = raw_clouds_lidar_;

    FSM::dispatch(FSM::Event::LidarEnd);
    return result;
}

// Recover only against the last trusted map. Candidate scans are never inserted
// before registration is accepted; missing motion is not filled with poses.
bool LidarPipeline::recoverAfterSensorGap(const FrameData& frame, Result& result) {
    if (!temporal_anchor_valid_) return false;
    const double gap_end = imu_process_.latest_gap_end;
    if (gap_end > handled_imu_gap_end_ && gap_end > temporal_anchor_state_.time &&
        frame.end_time > gap_end - imu_process_.latest_gap_duration) {
        gap_recovery_pending_ = true;
        recovery_data_start_ = std::max(recovery_data_start_, gap_end);
    }
    if (!gap_recovery_pending_ && !temporal_anchor_state_.in_elevator &&
        !ELEVATOR_TRIGGER &&
        frame.base_time - temporal_anchor_state_.time > lidar_gap_threshold_s) {
        gap_recovery_pending_ = true;
        recovery_data_start_ = frame.base_time;
        LOG_WARN(Core, "[lidar_gap] duration="
                 << frame.base_time - temporal_anchor_state_.time);
    }
    if (!gap_recovery_pending_) return false;
    lio_tracking_valid.store(false);
    lio_ros::Bool tracking; tracking.data = false;
    tracking_valid_pub_.publish(tracking);
    auto reject = [&]() { shared_bufs_->popFrontLidarFrameSafe(); return true; };
    const double nominal_period = trusted_scan_span_s_;
    const double scan_span = frame.end_time - frame.base_time;
    if (frame.base_time < recovery_data_start_ || scan_span < 0.75 * nominal_period ||
        scan_span > 1.5 * nominal_period || frame.raw_cloud->size() < 500 ||
        !imu_process_.coversTimeRange(frame.base_time, frame.end_time)) {
        LOG_WARN(Core, "[gap_recovery] discard invalid scan t="
                 << std::setprecision(16) << frame.end_time);
        return reject();
    }
    if (frame.end_time - temporal_anchor_state_.time > gap_recovery_max_window_s ||
        temporal_anchor_state_.in_elevator || ELEVATOR_TRIGGER) {
        LOG_WARN(Core, "[gap_recovery] tracking lost; restart or external relocalization required");
        return reject();
    }
    const bool missing_imu = gap_end > temporal_anchor_state_.time;
    State seed = temporal_anchor_state_;
    if (missing_imu) {
        seed.p += seed.v * (frame.base_time - seed.time);
    } else {
        // With LiDAR-only loss, retain the real IMU rotation and motion as the
        // registration seed instead of replacing them with constant velocity.
        imu_process_.get_state_at_t(seed, frame.base_time);
    }
    seed.time = frame.base_time;
    seed.P.setZero();
    seed.P.diagonal().setConstant(1e-3);
    seed.P.block<3,3>(StateIndex::R,StateIndex::R).setIdentity();
    seed.P.block<3,3>(StateIndex::R,StateIndex::R) *= 0.01;
    seed.P.block<3,3>(StateIndex::P,StateIndex::P).setIdentity();
    seed.P.block<3,3>(StateIndex::P,StateIndex::P) *= 0.04;
    seed.P.block<3,3>(StateIndex::V,StateIndex::V).setIdentity();
    seed.P.block<3,3>(StateIndex::BW,StateIndex::BW) =
            temporal_anchor_state_.P.block<3,3>(StateIndex::BW,StateIndex::BW);
    seed.P.block<3,3>(StateIndex::BA,StateIndex::BA) =
            temporal_anchor_state_.P.block<3,3>(StateIndex::BA,StateIndex::BA);
    imu_process_.set_state_at_t(seed, frame.base_time);
    State candidate;
    imu_process_.get_state_at_t(candidate, frame.end_time);
    PointCloudXYZI::Ptr scan(new PointCloudXYZI(*frame.raw_cloud));
    if (!undistortFrame(frame, candidate, scan)) return reject();
    // A sensor outage does not change the nominal scan or point-budget period.
    last_lidar_stamp_for_adaptive_ = frame.end_time - nominal_period;
    updateAdaptiveConfig(frame.end_time);
    downsampleFrame(scan);
    double fitness = 0.0, rmse = 0.0;
    if (!estimator_->recoverPoseAfterGap(*downsampled_cloud_, candidate, fitness, rmse)) {
        LOG_WARN(Core, "[gap_recovery] registration rejected fitness=" << fitness << " rmse=" << rmse);
        return reject();
    }
    if (missing_imu) candidate.v = temporal_anchor_state_.v;
    estimator_->set_cur_state(candidate);
    raw_clouds_lidar_ = scan;
    runIeskfUpdate(candidate);
    commitFrame(frame, candidate, false);
    handled_imu_gap_end_ = gap_end;
    gap_recovery_pending_ = false;
    result.processed = true;
    result.post_state = candidate;
    result.dedistorted_cloud = scan;
    LOG_WARN(Core, "[gap_recovery] accepted t=" << std::setprecision(16) << frame.end_time
             << " fitness=" << fitness << " rmse=" << rmse);
    return true;
}
