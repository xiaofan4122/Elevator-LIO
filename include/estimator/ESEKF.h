/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Error-state Kalman filter estimator interface.
 */

#ifndef ESEKF_H
#define ESEKF_H

#define G_m_s2 (9.80665)

#include "support/common_lib.h"
#include "support/TopicProcess.h"
 

Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, double dt);

using namespace Eigen;

struct NormVec {
    Eigen::Vector3d norm_vec;
    Eigen::Vector3d point_imu_xyz;
    double d = 0.0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class EskfEstimator {
public:
    using PoseJacobianMatrix = Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>;

    struct ZIcpResult {
        bool accepted = false;
        bool degenerate = false;
        bool converged = false;
        double correction_z = 0.0;
        double information_ratio = 0.0;
        double rmse = 0.0;
        int effective_points = 0;
        int iterations = 0;
        std::string reason;
    };

    EskfEstimator();
    ~EskfEstimator();
    void UpdateByLidarIESKF(PointCloudXYZI &Dedistort_clouds_lidar);
    ZIcpResult matchZByIcpAgainstIkdTree(const PointCloudXYZI &clouds_lidar,
                                         const State &initial_state) const;
    void UpdateByWheel(std::deque<WheelData> buf);
    bool InitImu(std::deque<ImuMsgConst> imu_msgs , int init_imu_num);
    std::vector<State, Eigen::aligned_allocator<State>> get_prior_states();
    std::vector<State, Eigen::aligned_allocator<State>> get_post_states();

    State get_cur_state();
    void set_cur_state(State state_now);

    PointType transLidar2World(const PointType &point);
    PointCloudXYZIPtr transLidar2Elevator(const PointCloudXYZI &points_lidar);
    PointCloudXYZIPtr transLidar2World(const PointCloudXYZI &points_lidar); // 使用指针避免内存分配，不然耗时很长
    vector<PointVector>  Nearest_Points; //每个点的最近点序列
    double last_lidar_end_time_{}; //上一帧结束时间戳

    Eigen::Vector3d mean_acc_;//加速度均值,用于计算方差
    Eigen::Vector3d mean_gyr_;//角速度均值，用于计算方差

    std::vector<State, Eigen::aligned_allocator<State>> state_vector_prior_; // 先验状态向量组，大小与一帧中IMU数据数量相同（预积分得到的状态）
    std::vector<State, Eigen::aligned_allocator<State>> state_vector_post_; // 后验状态向量组，目前大小为1，保存了一帧中最后时刻的状态（更新后得到的状态）

private:
    bool calculate(const State&state, PointCloudXYZI &clouds_lidar, Eigen::MatrixXd & Z,Eigen::MatrixXd & H, bool kd_research_en);
    bool calculatePose(const State &state, PointCloudXYZI &clouds_lidar,
                       Eigen::VectorXd &Z, PoseJacobianMatrix &H_pose, bool kd_research_en);
    void prepareFrameCache(const PointCloudXYZI &clouds_lidar);
    void evaluatePointMatches(const State &state, PointCloudXYZI &clouds_lidar, bool kd_research_en);
    void packMeasurementMatrices(const State &state, PointCloudXYZI &clouds_lidar,
                                 Eigen::MatrixXd &Z, Eigen::MatrixXd &H);
    void packPoseMeasurementMatrices(const State &state, PointCloudXYZI &clouds_lidar,
                                     Eigen::VectorXd &Z, PoseJacobianMatrix &H_pose);
    static Eigen::Matrix<double,StateIndex::STATE_TOTAL,1> getErrorState(const State &s1, const  State &s2);
    State state_; // 当前状态（位置和姿态基于IMU系）

    std::vector<Pose> IMU_Pose_vector_;
    // Eigen::MatrixXd Q_; // variance matrix of imu

    Eigen::Matrix3d Lidar_R_wrt_IMU;// lidar到IMU的旋转外参
    Eigen::Vector3d Lidar_T_wrt_IMU;// lidar到IMU的位置外参
    Eigen::Vector3d angvel_last;//上一帧角速度
    Eigen::Vector3d acc_s_last;//上一帧加速度

    double start_timestamp_{}; //开始时间戳
    
    int    vaild_points_num_ = 0; // 更新过程中有效点数量
    bool   b_first_frame_ = true; //是否是第一帧的第一个数据
    bool   imu_need_init_ = true; //是否需要初始化imu
    std::vector<double> effect_meas_inv_var_; // 有效残差对应的逆方差（用于加权求解）
    size_t cached_point_num_ = 0;
    std::vector<Eigen::Vector3d> point_lidar_vec_;
    std::vector<Eigen::Vector3d> point_imu_vec_;
    std::vector<double> point_lidar_range_;
    std::vector<Eigen::Vector3d> point_world_cache_;
    std::vector<NormVec, Eigen::aligned_allocator<NormVec>> norm_vector_;
    std::vector<uint8_t> is_effect_point_;
    std::vector<int> effective_indices_;
    std::vector<int> reject_indices_;
    std::vector<std::vector<float>> knn_distance_sq_;
    PoseJacobianMatrix pose_jacobian_cache_;
    Eigen::VectorXd pose_residual_cache_;

    // double ACC_NOISE_VAR = 1E-4;
    // double GYRO_NOISE_VAR = 1E-5;
    // double ACC_RANDOM_WALK_VAR = 1E-6;
    // double GYRO_RANDOM_WALK_VAR = 1E-9;
};

#endif //ESEKF_H
