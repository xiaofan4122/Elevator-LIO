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
#include <lio/ElevatorState.h>
#include <pcl/filters/voxel_grid.h>
#include <sstream>
#include <iomanip>

// ===================== LidarPipeline =====================

LidarPipeline::LidarPipeline(ros::NodeHandle& nh,
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
    , cloud_init_accum_lidar_(new PointCloudXYZI)
    , cloud_init_lidar_(new PointCloudXYZI)
    , cloud_init_world_(new PointCloudXYZI)
    , rebuild_map_accum_lidar_(new PointCloudXYZI)
    , downsampled_cloud_(new PointCloudXYZI)
    , blind_filtered_(new PointCloudXYZI)
{
    ele_state_pub_ = nh.advertise<std_msgs::Bool>("/LIO/in_elevator", 1, true);
    elevator_estimate_pub_ = nh.advertise<lio::ElevatorState>("/LIO/elevator_state", 1, true);
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
                                      double blind_dist) {
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
    uniform_voxel_.setLeafSize(current_voxel_, current_voxel_, current_voxel_);
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
    applyVoxelFilter(cloud_init_accum_lidar_, cloud_init_lidar_, blind);
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

    applyVoxelFilter(rebuild_map_accum_lidar_, cloud_init_lidar_, blind);
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
    imu_process_.elev_process.processDoorDetector(
        *frame.raw_cloud, frame.end_time, out_state,
        FSM::current_state == FSM::State::LidarProcess);
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
    const std::size_t points_before = cloud->points.size();
    if (cloud->points.size() > 10) {
        applyVoxelFilter(cloud, downsampled_cloud_, blind);
        static double last_downsample_print_time = -1.0;
        if (last_downsample_print_time < 0.0 || (lidar_end_time - last_downsample_print_time) > 5.0) {
            std::cout << "[LidarPipeline] downsample: in=" << cloud->points.size()
                      << " out=" << downsampled_cloud_->points.size() << std::endl;
            last_downsample_print_time = lidar_end_time;
        }
    } else {
        static double last_no_downsample_print_time = -1.0;
        if (last_no_downsample_print_time < 0.0 || (lidar_end_time - last_no_downsample_print_time) > 5.0) {
            LOG_INFO(Map, "[LidarPipeline] no downsample, reuse: " << downsampled_cloud_->points.size());
            last_no_downsample_print_time = lidar_end_time;
        }
    }
    const std::size_t points_after = downsampled_cloud_->points.size();

    if (downsample_adaptive_enabled && !downsampled_cloud_->points.empty()) {
        current_voxel_ = voxel_ctrl_.update(downsampled_cloud_->points.size());
    }

    if (log_save_enable) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << lidar_end_time << "," << current_voxel_ << ","
            << points_after << "," << points_before << ","
            << current_target_points_per_frame_ << ","
            << smoothed_lidar_dt_for_adaptive_;
        runtimeLogger().writeLine(LogFileChannel::AdaptiveDownsample,
                                  "time,voxel_size,points_after,points_before,target_points_frame,lidar_dt",
                                  oss.str(), 0, lidar_end_time);
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
        applyVoxelFilter(raw_clouds_lidar_, cloud_init_lidar_, blind);
        cloud_init_world_ = estimator_->transLidar2World(*cloud_init_lidar_);
        ikd_map_->add(cloud_init_world_);
    }
}

// ===================== Step 9: Commit frame (state update, map, pop) =====================

void LidarPipeline::commitFrame(const FrameData& frame, const State& post_state) {
    lidar_end_time = frame.end_time;

    imu_process_.set_state_at_t(const_cast<State&>(post_state), frame.end_time);

    std_msgs::Bool msg;
    msg.data = post_state.in_elevator;
    ele_state_pub_.publish(msg);

    lio::ElevatorState elevator_msg;
    elevator_msg.header.stamp = get_ros_time(frame.end_time);
    elevator_msg.header.frame_id = "world";
    elevator_msg.in_elevator = post_state.in_elevator;
    elevator_msg.displacement = post_state.z;
    elevator_msg.velocity = post_state.vz;
    elevator_msg.acceleration = post_state.az;
    elevator_estimate_pub_.publish(elevator_msg);

    shared_bufs_->popFrontLidarFrameSafe();

    if (!rebuild_map_pending_) {
        LIONode::map_incremental(*downsampled_cloud_);
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
    bool in_elevator;
    resolveFrameState(frame, lidar_end_state, in_elevator);

    // [6] Undistort
    raw_clouds_lidar_ = frame.raw_cloud;
    if (in_elevator && !exit_from_elevator_) {
        // Skip undistortion in elevator mode
    } else {
        if (!undistortFrame(frame, lidar_end_state, raw_clouds_lidar_)) {
            shared_bufs_->popFrontLidarFrameSafe();
            FSM::dispatch(FSM::Event::LidarEnd);
            std::cerr << "[LidarPipeline] IMU Data Not Enough" << std::endl;
            return result;
        }
    }

    // [7] Downsample
    downsampleFrame(raw_clouds_lidar_);

    // [8] IESKF update
    State post_state = lidar_end_state;
    runIeskfUpdate(post_state);

    // [9] Elevator stop handling
    handleElevatorStop(post_state);

    // [10] Elevator exit handling
    handleElevatorExit(post_state);

    // [11] Commit frame state, update map, pop buffer
    commitFrame(frame, post_state);

    // [12] Return results for caller to publish
    result.processed = true;
    result.post_state = post_state;
    result.dedistorted_cloud = raw_clouds_lidar_;

    FSM::dispatch(FSM::Event::LidarEnd);
    return result;
}
