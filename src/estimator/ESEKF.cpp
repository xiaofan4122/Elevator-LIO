//
// Created by xiaofan on 25-1-7.
//

#include "estimator/ESEKF.h"

#include "support/common_lib.h"
#include "support/LIONode.h"
#include "ikd_tree/IkdNearestQuery.hpp"
#include "support/type.h"
#include "omp.h"

static Eigen::Vector3d SO3Log(const Eigen::Matrix3d&SO3 ){
    double theta = (SO3.trace()>3-1e6)?0:acos((SO3.trace()-1)/2);
    Eigen::Vector3d so3(SO3(2,1)-SO3(1,2),SO3(0,2)-SO3(2,0),SO3(1,0)-SO3(0,1));
    return fabs(theta)<0.001?(0.5*so3):(0.5*theta/sin(theta)*so3);
}


static Eigen::Matrix3d A_T(const Eigen::Vector3d& v) {
    const double theta_sq = v.squaredNorm();
    const double theta = std::sqrt(theta_sq);
    const Eigen::Matrix3d K = skew3d(v);

    if (theta < 1e-6) {
        // 使用泰勒展开：I + 1/2 [v]x + 1/6 [v]x^2
        return Eigen::Matrix3d::Identity() + 0.5 * K + (1.0/6.0) * K * K;
    }

    const Eigen::Matrix3d K2 = K * K;
    return Eigen::Matrix3d::Identity()
           + ((1.0 - std::cos(theta)) / theta_sq) * K
           + ((theta - std::sin(theta)) / (theta_sq * theta)) * K2;
}

namespace {
    constexpr int kLidarPoseDim = 6;
    constexpr double kLidarPriorScale = 1E-3;

    using SteadyClock = std::chrono::steady_clock;

    /**
     * @brief 辅助函数：将字符串转换为全小写并返回副本
     * 使用 std::string 传值调用自动创建副本，适合需要返回新字符串的场景
     */
    std::string normalizeLowerCopy(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        return value;
    }

    /**
     * @brief 判断 Lidar 更新模式
     */
    bool usePose6LidarUpdate() {
        // 1. 统一转换为小写副本
        const std::string mode = normalizeLowerCopy(lidar_update_mode);

        // 2. 使用容器或直接比较逻辑
        // 如果后续模式变多，可以使用 std::unordered_set<std::string> 来加速匹配
        return mode.empty() || mode == "pose6" || mode == "pose_6";
    }

    /**
     * @brief 判断是否启用 Fast-LIO 风格的评分闸门（Score Gate）
     * @return 若模式匹配则返回 true，用于决定是否采用基于残差评分的匹配过滤
     */
    bool useFastLioScoreGate() {
        // 将配置字符串标准化为小写，增强容错性
        const std::string mode = normalizeLowerCopy(ikdtree_gate_mode);
        // 支持空字符串（默认）、"fastlio_score" 或简写 "fastlio"
        return mode.empty() || mode == "fastlio_score" || mode == "fastlio";
    }

    /**
     * @brief 从完整的状态误差向量中提取位姿相关的误差（旋转+平移）
     * @param error_s 完整的误差状态向量（含旋转、平移、速度、Bias等）
     * @return Eigen::Matrix 包含 6 自由度位姿误差的列向量 [Rotation, Position]^T
     */
    Eigen::Matrix<double, kLidarPoseDim, 1> packPoseError(
            const Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> &error_s) {
        Eigen::Matrix<double, kLidarPoseDim, 1> pose_error;
        // 提取旋转误差（通常是 Lie Algebra \so(3) 上的扰动）
        pose_error.template segment<3>(0) = error_s.template segment<3>(StateIndex::R);
        // 提取平移（位置）误差
        pose_error.template segment<3>(3) = error_s.template segment<3>(StateIndex::P);
        return pose_error;
    }

    /**
     * @brief 计算动态大小矩阵中所有元素绝对值的最大值
     * @param x 输入矩阵
     * @return 返回最大绝对值，若矩阵为空则返回 0.0
     */
    double maxAbsCoeff(const Eigen::MatrixXd &x) {
        return x.size() > 0 ? x.cwiseAbs().maxCoeff() : 0.0;
    }

    /**
     * @brief 计算固定大小状态向量中所有元素绝对值的最大值
     * @note 常用于检查整个状态向量是否发生数值爆炸或越界
     */
    double maxAbsCoeff(const Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> &x) {
        return x.cwiseAbs().maxCoeff();
    }

    using FullStateVector = Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1>;
    using FullStateMatrix = Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>;

    void applyPoseIterationCorrection(const FullStateVector &dx,
                                      FullStateVector &dx_new,
                                      FullStateMatrix &P) {
        const Eigen::Matrix3d so3_correction = A_T(dx.segment<3>(StateIndex::R)).transpose();
        dx_new.segment<3>(StateIndex::R) = so3_correction * dx_new.segment<3>(StateIndex::R);
        for (int c = 0; c < StateIndex::STATE_TOTAL; ++c) {
            P.block<3, 1>(StateIndex::R, c) = so3_correction * P.block<3, 1>(StateIndex::R, c);
        }
        for (int r = 0; r < StateIndex::STATE_TOTAL; ++r) {
            P.block<1, 3>(r, StateIndex::R) = P.block<1, 3>(r, StateIndex::R) * so3_correction.transpose();
        }
    }

    void applyPoseFinalCovarianceCorrection(const FullStateVector &dx,
                                            FullStateMatrix &L,
                                            FullStateMatrix &P_base,
                                            FullStateMatrix &K_x) {
        const Eigen::Matrix3d so3_correction = A_T(dx.segment<3>(StateIndex::R)).transpose();
        for (int c = 0; c < StateIndex::STATE_TOTAL; ++c) {
            L.block<3, 1>(StateIndex::R, c) = so3_correction * P_base.block<3, 1>(StateIndex::R, c);
        }
        for (int c = 0; c < kLidarPoseDim; ++c) {
            K_x.block<3, 1>(StateIndex::R, c) = so3_correction * K_x.block<3, 1>(StateIndex::R, c);
        }
        for (int r = 0; r < StateIndex::STATE_TOTAL; ++r) {
            L.block<1, 3>(r, StateIndex::R) = L.block<1, 3>(r, StateIndex::R) * so3_correction.transpose();
            P_base.block<1, 3>(r, StateIndex::R) = P_base.block<1, 3>(r, StateIndex::R) * so3_correction.transpose();
        }
    }


    /**
     * @brief 判断 IESKF 迭代是否收敛
     * @param update_x 当前迭代步的状态增量
     * @return true 表示旋转和平移的增量均小于设定阈值，可以停止迭代
     */
    bool poseConverged(const Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> &update_x) {
        // 分别计算旋转部分和平移部分的最大绝对值增量
        const double rot_max = update_x.segment<3>(StateIndex::R).cwiseAbs().maxCoeff();
        const double pos_max = update_x.segment<3>(StateIndex::P).cwiseAbs().maxCoeff();

        // 检查是否满足收敛阈值（弧度/米）
        return rot_max <= lidar_ieskf_converge_rot_rad && pos_max <= lidar_ieskf_converge_pos_m;
    }

    /**
     * @brief 判断在迭代过程中是否需要重新寻找最近邻点（Rematch）
     * @param update_x 当前迭代步的状态增量
     * @param iter_idx 当前迭代索引
     * @param iter_num 总迭代次数限制
     * @return 返回 true 表示当前位姿变化已足够小，可以进行最后一次精确的近邻搜索
     */
    bool shouldRematchPose(const Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> &update_x, int iter_idx, int iter_num) {
        const double rot_max = update_x.segment<3>(StateIndex::R).cwiseAbs().maxCoeff();
        const double pos_max = update_x.segment<3>(StateIndex::P).cwiseAbs().maxCoeff();

        // 情况1：增量足够小，此时更新位姿后的匹配会更准
        // 情况2：倒数第二次迭代（iter_num - 2），强制进行一次重匹配以保证最后一步迭代的质量
        return (rot_max <= lidar_ieskf_rematch_rot_rad && pos_max <= lidar_ieskf_rematch_pos_m)
               || iter_idx == iter_num - 2;
    }

struct RollingPerfWindow {
    static constexpr size_t kWindow = 30;
    std::array<double, kWindow> values{};
    size_t size = 0;
    size_t next = 0;
    double max_ms = 0.0;
    double last_ms = 0.0;

    void add(double ms) {
        last_ms = ms;
        max_ms = std::max(max_ms, ms);
        values[next] = ms;
        next = (next + 1) % kWindow;
        size = std::min(kWindow, size + 1);
    }

    double avgMs() const {
        if (size == 0) return 0.0;
        double sum = 0.0;
        for (size_t i = 0; i < size; ++i) sum += values[i];
        return sum / static_cast<double>(size);
    }

    double maxWindowMs() const {
        double max_value = 0.0;
        for (size_t i = 0; i < size; ++i) max_value = std::max(max_value, values[i]);
        return max_value;
    }
};

struct LidarPerfSnapshot {
    double update_total_ms = 0.0;
    double calculate_ms = 0.0;
    double solve_ms = 0.0;
    int points = 0;
    int valid_points = 0;
    int iterations = 0;
    bool pose6 = true;
    double avg_abs_residual = 0.0;
    double max_abs_residual = 0.0;
    double pose_rot_update = 0.0;
    double pose_pos_update = 0.0;
    bool early_stop = false;
    bool full_iter = false;
    int rematch_count = 0;
};

void recordAndMaybeReportLidarPerf(const LidarPerfSnapshot &snap) {
    if (!time_log_enable) return;

    static RollingPerfWindow update_total;
    static RollingPerfWindow calculate_total;
    static RollingPerfWindow solve_total;
    static RollingPerfWindow residual_avg;
    static RollingPerfWindow residual_max;
    static RollingPerfWindow pose_rot;
    static RollingPerfWindow pose_pos;
    static RollingPerfWindow valid_ratio;
    static RollingPerfWindow iter_window;
    static RollingPerfWindow input_points_window;
    static RollingPerfWindow valid_points_window;
    static RollingPerfWindow rematch_window;
    static int report_count = 0;
    static int early_stop_count = 0;
    static int full_iter_count = 0;

    update_total.add(snap.update_total_ms);
    calculate_total.add(snap.calculate_ms);
    solve_total.add(snap.solve_ms);
    residual_avg.add(snap.avg_abs_residual);
    residual_max.add(snap.max_abs_residual);
    pose_rot.add(snap.pose_rot_update);
    pose_pos.add(snap.pose_pos_update);
    valid_ratio.add(snap.points > 0 ? static_cast<double>(snap.valid_points) / static_cast<double>(snap.points) : 0.0);
    iter_window.add(static_cast<double>(snap.iterations));
    input_points_window.add(static_cast<double>(snap.points));
    valid_points_window.add(static_cast<double>(snap.valid_points));
    rematch_window.add(static_cast<double>(snap.rematch_count));
    ++report_count;
    if (snap.early_stop) ++early_stop_count;
    if (snap.full_iter) ++full_iter_count;

    if (report_count % static_cast<int>(RollingPerfWindow::kWindow) != 0) return;

    const double calc_ratio = update_total.avgMs() > 1e-9 ? (100.0 * calculate_total.avgMs() / update_total.avgMs()) : 0.0;
    const double solve_ratio = update_total.avgMs() > 1e-9 ? (100.0 * solve_total.avgMs() / update_total.avgMs()) : 0.0;

    LOG_INFO(Perf,
             "[lidar_perf][" << (snap.pose6 ? "pose6" : "full21") << "] "
             << "frames=" << report_count
             << " win_avg_total_ms=" << std::fixed << std::setprecision(3) << update_total.avgMs()
             << " win_max_total_ms=" << update_total.maxWindowMs()
             << " hist_max_total_ms=" << update_total.max_ms
             << " avg_calc_ms=" << calculate_total.avgMs()
             << " avg_solve_ms=" << solve_total.avgMs()
             << " calc_ratio=" << calc_ratio << "%"
             << " solve_ratio=" << solve_ratio << "%"
             << " input_points=" << input_points_window.avgMs()
             << " valid_points=" << valid_points_window.avgMs()
             << " valid_ratio=" << valid_ratio.avgMs()
             << " rematch_count=" << rematch_window.avgMs()
             << " last_valid=" << snap.valid_points
             << " avg_iters=" << iter_window.avgMs()
             << " avg_abs_res=" << residual_avg.avgMs()
             << " max_abs_res=" << residual_max.avgMs()
             << " avg_pose_rot=" << pose_rot.avgMs()
             << " avg_pose_pos=" << pose_pos.avgMs()
             << " early_stop_ratio=" << (100.0 * early_stop_count / static_cast<double>(report_count)) << "%"
             << " full_iter_ratio=" << (100.0 * full_iter_count / static_cast<double>(report_count)) << "%"
             << " last_total_ms=" << update_total.last_ms
             << " last_calc_ms=" << calculate_total.last_ms
             << " last_solve_ms=" << solve_total.last_ms);
}
} // namespace


Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, double dt)
{
    double ang_vel_norm = ang_vel.norm();
    Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();

    if (ang_vel_norm < 0.0000001)
    {
        cerr << "[Exp] ang_vel_norm is too small" << endl;
        return Eye3;

    }
    Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix3d K;

    K << skew3d(r_axis);
    double r_ang = ang_vel_norm * dt;

    /// Roderigues Transformation
    return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
}

template<typename PointType,typename T>
static PointType transformPoint(PointType point,const Eigen::Quaternion<T> &q,const Eigen::Matrix<T,3,1> &t) {
    Eigen::Matrix<T,3,1> ep = {point.x,point.y,point.z};
    ep = q * ep + t;
    point.x = ep.x();
    point.y = ep.y();
    point.z = ep.z();
    return point;
}
template<typename PointType,typename T>
static PointType transformPoint(PointType point,const Eigen::Matrix<T,3,3> &transMatrix,const Eigen::Matrix<T,3,1> &t){
    Eigen::Matrix<T,3,1> ep = {point.x,point.y,point.z};
    ep = transMatrix * ep + t;
    point.x = ep.x();
    point.y = ep.y();
    point.z = ep.z();
    return point;
}



EskfEstimator::EskfEstimator() {
    state_.q = Eigen::Quaterniond(1, 0, 0, 0);
    state_.p = Eigen::Vector3d(0, 0, 0);
    state_.v = Eigen::Vector3d(0, 0, 0);
    state_.bw = Eigen::Vector3d(0, 0, 0);
    state_.ba = Eigen::Vector3d(0, 0, 0);
    state_.g = Eigen::Vector3d(0, 0, -G_m_s2);
    state_.P = Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
    state_.P.block<3,3>(StateIndex::P, StateIndex::P) = Eigen::Matrix3d::Identity() * 0.01;
    state_.P.block<3,3>(StateIndex::V, StateIndex::V) = Eigen::Matrix3d::Identity() * 0.01;
    state_.time = 0;
    mean_acc_ = Eigen::Vector3d(0, 0, 0);
    mean_gyr_ = Eigen::Vector3d(0, 0, 0);
    // Q_ = Eigen::Matrix<double, StateNoiseIndex::NOISE_TOTAL, StateNoiseIndex::NOISE_TOTAL>::Identity();
    // Q_.block<3, 3>(StateNoiseIndex::ACC_NOISE, StateNoiseIndex::ACC_NOISE) =
    //         ACC_NOISE_VAR * Eigen::Matrix3d::Identity();
    // Q_.block<3, 3>(StateNoiseIndex::GYRO_NOISE, StateNoiseIndex::GYRO_NOISE) =
    //         GYRO_NOISE_VAR * Eigen::Matrix3d::Identity();
    // Q_.block<3, 3>(StateNoiseIndex::ACC_RANDOM_WALK, StateNoiseIndex::ACC_RANDOM_WALK) =
    //         ACC_RANDOM_WALK_VAR * Eigen::Matrix3d::Identity();
    // Q_.block<3, 3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK) =
    //         GYRO_RANDOM_WALK_VAR * Eigen::Matrix3d::Identity();
}

EskfEstimator::~EskfEstimator() = default;


#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

std::vector<std::string> createStateLabels() {
    std::vector<std::string> labels;
    // q (error-state is 3D delta-theta)
    labels.push_back("dth_x"); labels.push_back("dth_y"); labels.push_back("dth_z");
    // p
    labels.push_back("dp_x"); labels.push_back("dp_y"); labels.push_back("dp_z");
    // v
    labels.push_back("dv_x"); labels.push_back("dv_y"); labels.push_back("dv_z");
    // bw
    labels.push_back("dbw_x"); labels.push_back("dbw_y"); labels.push_back("dbw_z");
    // ba
    labels.push_back("dba_x"); labels.push_back("dba_y"); labels.push_back("dba_z");
    // g
    labels.push_back("dg_x"); labels.push_back("dg_y"); labels.push_back("dg_z");
    // Elevator 1D substate
    labels.push_back("dz");
    labels.push_back("dvz");
    labels.push_back("daz");

    return labels;
}

/**
 * @brief 可视化一个Eigen协方差矩阵，并在后续调用中更新它。
 * (版本 4: 支持行列标题)
 *
 * @param P 要可视化的 n x n 协方差矩阵 (Eigen::MatrixXd)。
 * @param labels 对应于行和列的标题列表。
 * @param window_name 显示窗口的名称。
 * @param cell_width 每个矩阵元素在图像中显示的像素宽度。
 * @param cell_height 每个矩阵元素在图像中显示的像素高度。
 */
void visualizeCovarianceMatrix(const Eigen::MatrixXd& P,
                               const std::vector<std::string>& labels,
                               const std::string& window_name = "Covariance Matrix P",
                               int cell_width = 70,
                               int cell_height = 35)
{
    static bool is_first_call = true;

    if (P.rows() == 0 || P.rows() != P.cols()) {
        std::cerr << "Error: Matrix is empty or not square." << std::endl;
        return;
    }

    int n = P.rows();
    bool has_labels = !labels.empty();
    if (has_labels && labels.size() != n) {
        std::cerr << "Error: Number of labels does not match matrix dimension." << std::endl;
        return;
    }

    // --- 修改点 1: 动态计算标题边距 ---
    int left_header_space = 0;
    int top_header_space = 0;
    const int padding = 15; // 文字和网格之间的间距
    double label_font_scale = 0.4;
    int label_thickness = 1;

    if (has_labels) {
        int max_label_width = 0;
        int max_label_height = 0;
        for (const auto& label : labels) {
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, label_font_scale, label_thickness, 0);
            max_label_width = std::max(max_label_width, text_size.width);
            max_label_height = std::max(max_label_height, text_size.height);
        }
        left_header_space = max_label_width + padding;
        top_header_space = max_label_height + padding;
    }

    int grid_width = n * cell_width;
    int grid_height = n * cell_height;
    int image_width = grid_width + left_header_space;
    int image_height = grid_height + top_header_space;

    cv::Mat image = cv::Mat::zeros(image_height, image_width, CV_8UC3);

    // 绘制矩阵部分
    double min_val = P.minCoeff();
    double max_val = P.maxCoeff();
    double range = max_val - min_val;
    if (range < 1e-9) range = 1.0;

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            cv::Rect cell_rect(j * cell_width + left_header_space, i * cell_height + top_header_space, cell_width, cell_height);

            double value = P(i, j);
            double normalized_value = (value - min_val) / range;
            uchar blue = 0, green = 0, red = 0;
            if (normalized_value < 0.5) {
                blue = static_cast<uchar>(255 * (1 - normalized_value * 2));
                green = static_cast<uchar>(255 * (normalized_value * 2));
            } else {
                green = static_cast<uchar>(255 * (1 - (normalized_value - 0.5) * 2));
                red = static_cast<uchar>(255 * ((normalized_value - 0.5) * 2));
            }

            cv::rectangle(image, cell_rect, cv::Scalar(blue, green, red), -1);
            cv::rectangle(image, cell_rect, cv::Scalar(100, 100, 100), 1);

            std::stringstream ss;
            ss << std::setprecision(2) << value;
            std::string text = ss.str();

            double font_scale = 0.4;
            int thickness = 1;
            cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, 0);
            while (text_size.width > cell_width * 0.9 && font_scale > 0.2) {
                font_scale *= 0.9;
                text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, 0);
            }

            cv::Scalar text_color = (normalized_value > 0.4 && normalized_value < 0.8) ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);
            cv::Point text_origin(cell_rect.x + (cell_width - text_size.width) / 2, cell_rect.y + (cell_height + text_size.height) / 2);
            cv::putText(image, text, text_origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness, cv::LINE_AA);
        }
    }

    // 绘制标题
    if (has_labels) {
        cv::Scalar label_color(220, 220, 220);

        // 绘制行标题 (左侧)
        for (int i = 0; i < n; ++i) {
            cv::Size text_size = cv::getTextSize(labels[i], cv::FONT_HERSHEY_SIMPLEX, label_font_scale, label_thickness, 0);
            // 文字右对齐
            cv::Point text_origin(left_header_space - text_size.width - 5, i * cell_height + top_header_space + (cell_height + text_size.height) / 2);
            cv::putText(image, labels[i], text_origin, cv::FONT_HERSHEY_SIMPLEX, label_font_scale, label_color, label_thickness, cv::LINE_AA);
        }

        // --- 修改点 2: 绘制水平的、不旋转的列标题 ---
        for (int j = 0; j < n; ++j) {
            cv::Size text_size = cv::getTextSize(labels[j], cv::FONT_HERSHEY_SIMPLEX, label_font_scale, label_thickness, 0);
            // 文字在单元格上方居中
             cv::Point text_origin(
                j * cell_width + left_header_space + (cell_width - text_size.width) / 2,
                top_header_space - (padding / 2) // 放在顶部区域的下半部分
            );
            cv::putText(image, labels[j], text_origin, cv::FONT_HERSHEY_SIMPLEX, label_font_scale, label_color, label_thickness, cv::LINE_AA);
        }
    }

    if (is_first_call) {
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
        is_first_call = false;
    }

    cv::imshow(window_name, image);
    cv::waitKey(1);
}


/**
 * @brief 利用lidar数据进行更新，寻找最近邻点、匹配对应点、迭代 kalman 更新当前状态 state_，迭代过程存储在s_next_v中
 * @param Dedistort_clouds_lidar 去畸变点云，lidar坐标系
 */
void EskfEstimator::UpdateByLidarIESKF(PointCloudXYZI &Dedistort_clouds_lidar) {
    const auto update_begin_tp = SteadyClock::now();
    state_vector_post_.clear();
    State s_next = state_;
    if (CovMatrix_vis) visualizeCovarianceMatrix(state_.P, createStateLabels());
    const int iter_num = std::max(1, lidar_ieskf_max_iterations);
    int point_num = Dedistort_clouds_lidar.points.size();
    if (point_num < 10) {
        state_vector_post_.push_back(s_next);
        cout << "too few points" << endl;
        return;
    }
    if(s_next.in_elevator) {
        cout << "in_elevator : True " << endl;
    }
    prepareFrameCache(Dedistort_clouds_lidar);
    bool converge;
    int i;
    Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> J_k_inv =
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
    bool kd_research_en = true;
    const Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> identity =
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
    double calculate_total_ms = 0.0;
    double solve_total_ms = 0.0;
    double last_avg_abs_residual = 0.0;
    double last_max_abs_residual = 0.0;
    double last_pose_rot_update = 0.0;
    double last_pose_pos_update = 0.0;
    bool early_stop = false;
    int rematch_count = 0;
    Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> final_P_k = s_next.P;
    if (usePose6LidarUpdate()) {
        PoseJacobianMatrix H_pose;
        Eigen::VectorXd Z_pose;
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> final_K_x =
                Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Zero();
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> final_iter_P =
                Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> final_update_x =
                Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1>::Zero();
        bool rematch_next_iter = false;
        int converge_count = 0;

        for (i = 0; i < iter_num; i++) {
            const Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> error_s = getErrorState(s_next, state_);
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> dx_new = error_s;
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> P_iter = s_next.P;
            applyPoseIterationCorrection(error_s, dx_new, P_iter);
            final_P_k = P_iter;

            const auto calc_begin_tp = SteadyClock::now();
            const bool search_this_iter = (i == 0) || rematch_next_iter;
            if (search_this_iter) {
                ++rematch_count;
            }
            if (!calculatePose(s_next, Dedistort_clouds_lidar, Z_pose, H_pose, search_this_iter)) {
                const int executed_iters = std::min(i + 1, iter_num);
                state_ = s_next;
                state_vector_post_.push_back(s_next);
                state_.P = P_iter;
                recordAndMaybeReportLidarPerf({
                    std::chrono::duration<double, std::milli>(SteadyClock::now() - update_begin_tp).count(),
                    calculate_total_ms + std::chrono::duration<double, std::milli>(SteadyClock::now() - calc_begin_tp).count(),
                    solve_total_ms,
                    point_num,
                    vaild_points_num_,
                    executed_iters,
                    true,
                    last_avg_abs_residual,
                    last_max_abs_residual,
                    last_pose_rot_update,
                    last_pose_pos_update,
                    early_stop,
                    executed_iters >= iter_num,
                    rematch_count
                });
                return;
            }
            calculate_total_ms += std::chrono::duration<double, std::milli>(SteadyClock::now() - calc_begin_tp).count();

            Eigen::Matrix<double, kLidarPoseDim, kLidarPoseDim> HTH =
                    Eigen::Matrix<double, kLidarPoseDim, kLidarPoseDim>::Zero();
            Eigen::Matrix<double, kLidarPoseDim, 1> Ht_h =
                    Eigen::Matrix<double, kLidarPoseDim, 1>::Zero();
            for (int r = 0; r < H_pose.rows(); ++r) {
                const double w = effect_meas_inv_var_.size() == static_cast<size_t>(H_pose.rows())
                                 ? effect_meas_inv_var_[static_cast<size_t>(r)] : 1.0;
                const Eigen::Matrix<double, 1, kLidarPoseDim> h_row = H_pose.row(r);
                const double residual = -Z_pose(r);
                HTH.noalias() += w * (h_row.transpose() * h_row);
                Ht_h.noalias() += w * h_row.transpose() * residual;
            }

            const auto solve_begin_tp = SteadyClock::now();
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> P_temp =
                    (P_iter / kLidarPriorScale).inverse();
            P_temp.block<kLidarPoseDim, kLidarPoseDim>(StateIndex::R, StateIndex::R) += HTH;
            const Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> P_post_like = P_temp.inverse();
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> K_x =
                    Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Zero();
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> K_h =
                    Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1>::Zero();
            K_h = P_post_like.block<StateIndex::STATE_TOTAL, kLidarPoseDim>(0, StateIndex::R) * Ht_h;
            K_x.block<StateIndex::STATE_TOTAL, kLidarPoseDim>(0, StateIndex::R) =
                    P_post_like.block<StateIndex::STATE_TOTAL, kLidarPoseDim>(0, StateIndex::R) * HTH;
            Eigen::Matrix<double, StateIndex::STATE_TOTAL, 1> update_x =
                    K_h + (K_x - identity) * dx_new;
            solve_total_ms += std::chrono::duration<double, std::milli>(SteadyClock::now() - solve_begin_tp).count();

            last_avg_abs_residual = Z_pose.size() > 0 ? Z_pose.cwiseAbs().mean() : 0.0;
            last_max_abs_residual = Z_pose.size() > 0 ? Z_pose.cwiseAbs().maxCoeff() : 0.0;
            last_pose_rot_update = update_x.segment<3>(StateIndex::R).cwiseAbs().maxCoeff();
            last_pose_pos_update = update_x.segment<3>(StateIndex::P).cwiseAbs().maxCoeff();
            converge = poseConverged(update_x);

            s_next.q = s_next.q.toRotationMatrix() * so3Exp(update_x.block<3,1>(StateIndex::R,0));
            s_next.q.normalize();
            s_next.p  = s_next.p  + update_x.block<3,1>(StateIndex::P,0);
            s_next.v  = s_next.v  + update_x.block<3,1>(StateIndex::V,0);
            s_next.bw = s_next.bw + update_x.block<3,1>(StateIndex::BW,0);
            s_next.ba = s_next.ba + update_x.block<3,1>(StateIndex::BA,0);
            if (online_gravity_estimation_enable) {
                s_next.g = s_next.g + update_x.block<3,1>(StateIndex::G,0);
            }
            s_next.z  = s_next.z  + update_x(StateIndex::Z,0);
            s_next.vz = s_next.vz + update_x(StateIndex::VZ,0);
            s_next.az = s_next.az + update_x(StateIndex::AZ,0);

            final_update_x = update_x;
            final_K_x = K_x;
            final_iter_P = P_iter;

            if (converge) {
                ++converge_count;
            } else {
                converge_count = 0;
            }
            rematch_next_iter = converge || (converge_count == 0 && i == iter_num - 2);

            if (converge_count > 1 || i == iter_num - 1) {
                early_stop = (i + 1) < iter_num;
                break;
            }
        }

        state_ = s_next;
        state_vector_post_.push_back(s_next);
        const int executed_iters = std::min(i + 1, iter_num);
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> L = final_iter_P;
        Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> P_base = final_iter_P;
        applyPoseFinalCovarianceCorrection(final_update_x, L, P_base, final_K_x);
        state_.P = L - final_K_x * P_base;
        state_.P = 0.5 * (state_.P + state_.P.transpose());
        recordAndMaybeReportLidarPerf({
            std::chrono::duration<double, std::milli>(SteadyClock::now() - update_begin_tp).count(),
            calculate_total_ms,
            solve_total_ms,
            point_num,
            vaild_points_num_,
            executed_iters,
            true,
            last_avg_abs_residual,
            last_max_abs_residual,
            last_pose_rot_update,
            last_pose_pos_update,
            early_stop,
            executed_iters >= iter_num,
            rematch_count
        });
        return;
    }

    Eigen::MatrixXd K;
    Eigen::MatrixXd H_k;
    Eigen::MatrixXd Z_k;
    for (i = 0; i< iter_num; i++) {
        const Eigen::Matrix<double,static_cast<int>(StateIndex::STATE_TOTAL),1> error_s = getErrorState(s_next,state_);
        J_k_inv.setIdentity();
        J_k_inv.block<3,3>(StateIndex::R,StateIndex::R) = A_T(error_s.block<3,1>(StateIndex::R,0));
        const Eigen::MatrixXd P_k = J_k_inv * s_next.P * J_k_inv.transpose();
        final_P_k = P_k;
        const Eigen::MatrixXd P_k_inv = P_k.inverse();
        const auto calc_begin_tp = SteadyClock::now();
        if (kd_research_en) {
            ++rematch_count;
        }
        if (!calculate(s_next,Dedistort_clouds_lidar,Z_k,H_k,kd_research_en)) {
            const int executed_iters = std::min(i + 1, iter_num);
            state_ = s_next;
            state_vector_post_.push_back(s_next);
            state_.P = P_k;
            recordAndMaybeReportLidarPerf({
                std::chrono::duration<double, std::milli>(SteadyClock::now() - update_begin_tp).count(),
                calculate_total_ms + std::chrono::duration<double, std::milli>(SteadyClock::now() - calc_begin_tp).count(),
                solve_total_ms,
                point_num,
                vaild_points_num_,
                executed_iters,
                false,
                last_avg_abs_residual,
                last_max_abs_residual,
                last_pose_rot_update,
                last_pose_pos_update,
                early_stop,
                executed_iters >= iter_num,
                rematch_count
            });
            return;
        }
        calculate_total_ms += std::chrono::duration<double, std::milli>(SteadyClock::now() - calc_begin_tp).count();
        Eigen::MatrixXd H_kt = H_k.transpose();
        const auto solve_begin_tp = SteadyClock::now();
        K = (H_kt * H_k + P_k_inv * kLidarPriorScale ).inverse() * H_kt;
        Eigen::MatrixXd left = -1 * K * Z_k;
        Eigen::MatrixXd right = -1 * (identity - K * H_k) * J_k_inv * error_s;
        Eigen::MatrixXd update_x = left + right;
        solve_total_ms += std::chrono::duration<double, std::milli>(SteadyClock::now() - solve_begin_tp).count();

        last_avg_abs_residual = Z_k.size() > 0 ? Z_k.cwiseAbs().mean() : 0.0;
        last_max_abs_residual = Z_k.size() > 0 ? Z_k.cwiseAbs().maxCoeff() : 0.0;
        last_pose_rot_update = update_x.block<3,1>(StateIndex::R,0).cwiseAbs().maxCoeff();
        last_pose_pos_update = update_x.block<3,1>(StateIndex::P,0).cwiseAbs().maxCoeff();

        converge = poseConverged(update_x);
        kd_research_en = false;
        if (shouldRematchPose(update_x, i, iter_num)) {
            kd_research_en = true;
        }

        s_next.q = s_next.q.toRotationMatrix() * so3Exp(update_x.block<3,1>(StateIndex::R,0));
        s_next.q.normalize();
        s_next.p  = s_next.p  + update_x.block<3,1>(StateIndex::P,0);
        s_next.v  = s_next.v  + update_x.block<3,1>(StateIndex::V,0);
        s_next.bw = s_next.bw + update_x.block<3,1>(StateIndex::BW,0);
        s_next.ba = s_next.ba + update_x.block<3,1>(StateIndex::BA,0);
        if (online_gravity_estimation_enable) {
            s_next.g = s_next.g + update_x.block<3,1>(StateIndex::G,0);
        }
        s_next.z  = s_next.z  + update_x(StateIndex::Z,0);
        s_next.vz = s_next.vz + update_x(StateIndex::VZ,0);
        s_next.az  = s_next.az  + update_x(StateIndex::AZ,0);
        if(converge){
            early_stop = (i + 1) < iter_num;
            break;
        }
    }
    // 输出迭代次数
    // cout << "iter_num: " << i << endl;
    //注释雷达更新
    state_ = s_next;
    state_vector_post_.push_back(s_next);
    const int executed_iters = std::min(i + 1, iter_num);
    //注释雷达更新
    state_.P = ( Eigen::Matrix<double,StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL>::Identity() - K * H_k ) * final_P_k;
    recordAndMaybeReportLidarPerf({
        std::chrono::duration<double, std::milli>(SteadyClock::now() - update_begin_tp).count(),
        calculate_total_ms,
        solve_total_ms,
        point_num,
        vaild_points_num_,
        executed_iters,
        false,
        last_avg_abs_residual,
        last_max_abs_residual,
        last_pose_rot_update,
        last_pose_pos_update,
        early_stop,
        executed_iters >= iter_num,
        rematch_count
    });
}

void EskfEstimator::UpdateByWheel(std::deque<WheelData> buf) {
    const double L1 = 0.25;
    const double L2 = 0.35;
    // 找到 LIO 输出时刻 last_lidar_end_time_ 对应的轮速计的值
    // cout << "buf.size(): " << buf.size() << endl;
    int i = 0;
    //while (buf[i].timestamp < last_lidar_end_time_ && i < buf.size()) {
    while (buf[i].timestamp < last_lidar_end_time_ && static_cast<size_t>(i) < buf.size()) {
        i++;
    }
    // cout << "i: "<< i << endl;
    // 使用最近邻数据
    WheelData wheel_data_1 = buf[i-1];
    WheelData wheel_data_2 = buf[i];
    double dt_1 = abs(wheel_data_1.timestamp - last_lidar_end_time_);
    double dt_2 = abs(wheel_data_2.timestamp - last_lidar_end_time_);
    WheelData wheel_data = dt_1 < dt_2 ? wheel_data_1:wheel_data_2;
    // 计算观测速度
    double vx_1 = wheel_data.v_L * cos(wheel_data.angle_L) + L2/L1 * wheel_data.v_L * sin(wheel_data.angle_L);
    double vx_2 = wheel_data.v_R * cos(wheel_data.angle_R) - L2/L1 * wheel_data.v_R * sin(wheel_data.angle_R);
    double vx = (vx_1+vx_2)/2;
    // 计算角速度
    double dtheta_1 = -2 * wheel_data.v_L * sin(wheel_data.angle_L) / L2;
    double dtheta_2 = -2 * wheel_data.v_R * sin(wheel_data.angle_R) / L2;
    double dtheta  = (dtheta_1+dtheta_2)/2;
    double L = 0.16;
    double vy = -dtheta * L;

    // 从本体坐标系转到Lidar系
    Eigen::Quaterniond q_B_L (0.70711,0,0,0.70711); // 绕 z 轴顺时针旋转90度

    // cout << "vx " << vx << "vy " << vy << "dtheta " << dtheta << endl;

    // 使用 wheel_data 计算更新状态
    Eigen::MatrixXd mat_C(3,static_cast<int>(StateIndex::STATE_TOTAL));
    mat_C.setZero();
    // 创建临时使用的矩阵和向量
    Eigen::Matrix3d submat;
    Eigen::Vector3d temp_vec;
    Eigen::Quaterniond q_L_W = state_.q;
    Eigen::Quaterniond q_B_W = q_B_L * q_L_W;
    // 第一子块
    temp_vec = q_B_W.conjugate() * state_.v ;
    submat = skew3d(temp_vec);
    mat_C.block<3,3>(0,0) = submat;
    // 第三子块
    submat = q_B_W.conjugate().toRotationMatrix();
    mat_C.block<3,3>(0,6) = submat;
    /******这里是W矩阵******/
    Eigen::MatrixXd mat_W = Eigen::MatrixXd::Identity(3, 3);
    // 计算增益
    Eigen::MatrixXd K(static_cast<int>(StateIndex::STATE_TOTAL),3);
    K.setZero();
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * 1E-5;
    K = state_.P * mat_C.transpose() * (mat_C * state_.P * mat_C.transpose() +
        mat_W * R * mat_W.transpose()).inverse();
    Eigen::VectorXd delta_dstate (static_cast<int>(StateIndex::STATE_TOTAL));
    // 计算残差
    Eigen::Vector3d res_wheel =  Eigen::Vector3d(vx,vy,0) - q_B_W.conjugate() * state_.v;
    delta_dstate = K * res_wheel;

    // cout << "K" << K << endl;
    // cout << "res_wheel" << res_wheel << endl;

    Eigen::Quaterniond dq (1,
        0.5*delta_dstate(0),
        0.5*delta_dstate(1),
        0.5*delta_dstate(2));
    state_.q = (state_.q * dq).normalized();
    state_.p += delta_dstate.segment<3>(3);
    state_.v += delta_dstate.segment<3>(6);
    state_.bw += delta_dstate.segment<3>(9);
    state_.ba += delta_dstate.segment<3>(12);
    state_.g += delta_dstate.segment<3>(15);
    // 更新协方差
    state_.P = state_.P - K * mat_C * state_.P;

    // 将z保存到文件中
    if (res_wheel_log_enable & log_save_enable) {
        static uint64_t wheel_res_seq = 0;
        std::ostringstream oss;
        oss << res_wheel(0) << "," << res_wheel(1) << "," << res_wheel(2);
        runtimeLogger().writeLine(
                LogFileChannel::WheelResidual,
                "res_x,res_y,res_z",
                oss.str(),
                ++wheel_res_seq);
    }
    for (int j =0 ;j < i ; j++) {
        buf.pop_front();
    }
}


Eigen::Matrix<double,StateIndex::STATE_TOTAL,1> EskfEstimator::getErrorState(const State &s1, const  State &s2){
    Eigen::Matrix<double,StateIndex::STATE_TOTAL,1> es;
    es.setZero();
    // 主状态18维
    es.segment<3>(StateIndex::R)  = SO3Log((s2.q.conjugate() * s1.q).toRotationMatrix()); // δθ
    es.segment<3>(StateIndex::P)  = s1.p  - s2.p;
    es.segment<3>(StateIndex::V)  = s1.v  - s2.v;
    es.segment<3>(StateIndex::BW) = s1.bw - s2.bw;
    es.segment<3>(StateIndex::BA) = s1.ba - s2.ba;
    es.segment<3>(StateIndex::G)  = s1.g  - s2.g;
    // 电梯1D（标量差即可；我们定义的是沿世界重力轴的标量）
    es(StateIndex::Z)  = s1.z  - s2.z;
    es(StateIndex::VZ) = s1.vz - s2.vz;
    es(StateIndex::AZ) = s1.az - s2.az;
    return es;
}


/**
 * @brief 利用点进行平面拟合，输出为ax+by+cz+d=0，d=-1/norm，注意法向量的方向
 * @param points 用于平面拟合的点
 * @param pabcd pabcd(0) = a（法向量的 x 分量）pabcd(1) = b（法向量的 y 分量）pabcd(2) = c（法向量的 z 分量），均归一化\n
 *              pabcd(3) = -1/normal，其中 normal 是法向量的模长，用以将法向量单位化。
 * @param threhold 平面拟合的阈值
 * @return bool量，是否是平面
 */
static bool planarFittingAndCheck(const PointVector & points, Eigen::Vector4d &pabcd, float threhold) {
    Eigen::Vector3d normal_vector;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    int point_num = points.size();
    A.resize(point_num,3);
    b.resize(point_num);
    b.setOnes();
    for (int i = 0; i < point_num; i++) {
        A(i,0) = points[i].x;
        A(i,1) = points[i].y;
        A(i,2) = points[i].z;
    }
    /**
     * 解超定方程 Ax = b 【其中A的每一行是一个点，x是待拟合平面的单位法向量，b是全为1的列向量】
     * 注意，此时的 normal_vector 并不是单位向量，还需要归一化
     * 此时解的方程相当于 ax + by + cz = 1
     */
    normal_vector = A.colPivHouseholderQr().solve(b);

    for (int j = 0; j < point_num; j++) {
        // 点到面的距离不能大于阈值，如果大于阈值，则不是平面
        if (fabs(normal_vector(0) * points[j].x + normal_vector(1) * points[j].y + normal_vector(2) * points[j].z - 1.0f) > threhold)
        {
            return false;
        }
    }
    double normal = normal_vector.norm();
    normal_vector.normalize();
    pabcd(0) = normal_vector(0);
    pabcd(1) = normal_vector(1);
    pabcd(2) = normal_vector(2);
    pabcd(3) = -1/normal;

    return true;
}

static bool planarFittingAndCheckFastest(const PointVector & points, Eigen::Vector4d &pabcd, double threshold_rmse) {
    int n = points.size();
    if (n < 5) return false;

    // 1. 栈内存映射
    Eigen::Matrix<double, 3, 5> pts_mat;
    for(int i = 0; i < 5; ++i) {
        pts_mat.col(i) << (double)points[i].x, (double)points[i].y, (double)points[i].z;
    }

    // 2. 质心与协方差计算
    Eigen::Vector3d center = pts_mat.rowwise().mean();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for(int i = 0; i < 5; ++i) {
        Eigen::Vector3d d = pts_mat.col(i) - center;
        cov += d * d.transpose();
    }

    // 3. 特征值分解 (Eigen 硬编码加速)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);
    Eigen::Vector3d evals = saes.eigenvalues();

    // 4. 厚度检查 (一票否决)
    double rmse = std::sqrt(evals[0] / 5.0);
    if (rmse > threshold_rmse) return false;

    // 5. 退化检查 (剔除线状特征)
    if (evals[1] < 2.0 * evals[0]) return false;

    // 6. 参数装填
    Eigen::Vector3d normal = saes.eigenvectors().col(0);
    if (normal.dot(center) > 0) normal = -normal;

    pabcd.block<3,1>(0,0) = normal;
    pabcd(3) = -normal.dot(center);

    return true;
}

/**
 * @brief 修正版：基于物理厚度(RMSE)的平面拟合
 * @param points 输入点
 * @param pabcd 输出参数
 * @param threshold_rmse 允许的最大均方根误差 (建议设为 0.03 ~ 0.04)
 */
static bool planarFittingAndCheck1(const PointVector & points, Eigen::Vector4d &pabcd, double threshold_rmse) {
    int n = points.size();
    if (n < 4) return false; // 边界修复：允许最少 4 个点，保证 5 减 1 后仍可拟合

    // 1. 零分配架构 (Zero Allocation)
    // 使用 Eigen 静态分配在栈内存上，彻底消除 OpenMP 里的堆内存锁竞争
    const int MAX_PTS = 5;
    int actual_n = std::min(n, MAX_PTS);
    Eigen::Matrix<double, 3, MAX_PTS> pts_mat;

    // 直接将点云数据映射到连续的栈内存矩阵中
    for(int i = 0; i < actual_n; ++i) {
        pts_mat.col(i) << (double)points[i].x, (double)points[i].y, (double)points[i].z;
    }

    Eigen::Vector3d normal, evals, center;
    double rmse = 0.0;

    // 闭包函数：通过传入 skip_idx 实现“虚拟剔除”，完全避免 erase() 带来的内存移动
    auto runPCA = [&](int skip_idx) {
        center.setZero();
        int valid_count = 0;
        for(int i = 0; i < actual_n; ++i) {
            if (i == skip_idx) continue; // 虚拟跳过离群点
            center += pts_mat.col(i);
            valid_count++;
        }
        center /= valid_count;

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for(int i = 0; i < actual_n; ++i) {
            if (i == skip_idx) continue;
            Eigen::Vector3d d = pts_mat.col(i) - center;
            cov += d * d.transpose();
        }

        // Eigen 针对 3x3 矩阵的特征值分解做了极致的硬编码优化，速度极快
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);
        evals = saes.eigenvalues();
        normal = saes.eigenvectors().col(0);
        rmse = std::sqrt(evals[0] / valid_count);
    };

    // --- 第一轮完整拟合 ---
    runPCA(-1); // -1 表示不跳过任何点

    // --- 第二轮：正交距离剔除与挽救机制 ---
    if (rmse > threshold_rmse) {
        if (actual_n <= 4) return false; // 点数不足 5 个，放弃挽救

        int max_idx = -1;
        double max_dist2 = -1.0;
        for(int i = 0; i < actual_n; ++i) {
            // [逻辑修复]: 计算点到平面的真实正交投影距离，而非到质心的距离
            double d_ortho = normal.dot(pts_mat.col(i) - center);
            double d2 = d_ortho * d_ortho;
            if(d2 > max_dist2) {
                max_dist2 = d2;
                max_idx = i;
            }
        }

        // 踢掉偏离平面最远的噪点，进行二次拟合
        runPCA(max_idx);

        // 如果挽救后依然不符合厚度要求，则判定为非平面
        if (rmse > threshold_rmse) return false;
    }

    // --- 第三轮：几何退化检查 ---
    // 利用 PCA 特性，精准拦截电线杆、边缘线等一维退化结构
    if (evals[1] < 2.0 * evals[0]) return false;

    // 输出参数装填
    if (normal.dot(center) > 0) normal = -normal;
    double d = -normal.dot(center);

    pabcd(0) = normal(0);
    pabcd(1) = normal(1);
    pabcd(2) = normal(2);
    pabcd(3) = d;

    return true;
}

EskfEstimator::ZIcpResult EskfEstimator::matchZByIcpAgainstIkdTree(
        const PointCloudXYZI &clouds_lidar, const State &initial_state) const {
    ZIcpResult result;
    if (ikdtree.Root_Node == nullptr) {
        result.reason = "ikd-tree map is empty";
        return result;
    }
    if (clouds_lidar.size() < static_cast<size_t>(elevator_exit_icp_z_min_effective_points)) {
        result.reason = "input cloud has too few points";
        return result;
    }

    IkdNearestQuery map_query;
    const int knn = std::max(4, ikdtree_match_knn);
    const Eigen::Matrix3d R_wb = initial_state.q.toRotationMatrix();
    const double max_neighbor_dist2 = elevator_exit_icp_z_max_neighbor_distance_m *
                                      elevator_exit_icp_z_max_neighbor_distance_m;
    double correction_z = 0.0;

    for (int iter = 0; iter < elevator_exit_icp_z_max_iterations; ++iter) {
        double z_information = 0.0;
        double solve_hessian_z = 0.0;
        double gradient_z = 0.0;
        double squared_residual_sum = 0.0;
        int geometric_points = 0;
        int effective_points = 0;

        for (const auto &point_lidar : clouds_lidar.points) {
            const Eigen::Vector3d p_lidar(point_lidar.x, point_lidar.y, point_lidar.z);
            const Eigen::Vector3d p_imu = imu_R_lidar * p_lidar + imu_t_lidar;
            Eigen::Vector3d p_world = R_wb * p_imu + initial_state.p;
            p_world.z() += correction_z;

            PointType query = point_lidar;
            query.x = static_cast<float>(p_world.x());
            query.y = static_cast<float>(p_world.y());
            query.z = static_cast<float>(p_world.z());

            PointVector nearest_points;
            std::vector<float> distance_sq;
            if (!map_query.searchKnn(query, knn, nearest_points, distance_sq) ||
                distance_sq[static_cast<size_t>(knn - 1)] > max_neighbor_dist2) {
                continue;
            }

            Eigen::Vector4d plane;
            if (!planarFittingAndCheck1(nearest_points, plane, ikdtree_plane_fit_rmse_threshold)) {
                continue;
            }
            const double residual = plane.head<3>().dot(p_world) + plane(3);
            if (!std::isfinite(residual) || std::abs(residual) > elevator_exit_icp_z_max_residual_m) {
                continue;
            }

            const double jacobian_z = plane(2);
            z_information += jacobian_z * jacobian_z;
            ++geometric_points;

            // 竖直面不能稳定约束 z；另外，每个对应点本身推导出的 z 修正也必须
            // 位于最终 0.3 m 接受门限内，避免顶板/底板跨层关联进入最小二乘。
            if (std::abs(jacobian_z) < elevator_exit_icp_z_min_abs_normal_z) {
                continue;
            }
            const double correspondence_delta_z = -residual / jacobian_z;
            if (!std::isfinite(correspondence_delta_z) ||
                std::abs(correspondence_delta_z) >= elevator_exit_icp_z_max_correction_m) {
                continue;
            }

            solve_hessian_z += jacobian_z * jacobian_z;
            gradient_z += jacobian_z * residual;
            squared_residual_sum += residual * residual;
            ++effective_points;
        }

        result.iterations = iter + 1;
        result.effective_points = effective_points;
        if (geometric_points < elevator_exit_icp_z_min_effective_points) {
            result.reason = "too few geometrically valid point-plane correspondences";
            return result;
        }

        result.information_ratio = z_information / static_cast<double>(geometric_points);
        if (!std::isfinite(result.information_ratio) ||
            result.information_ratio < elevator_exit_icp_z_min_information_ratio) {
            result.degenerate = true;
            result.reason = "z direction is geometrically degenerate";
            return result;
        }
        if (effective_points < elevator_exit_icp_z_min_effective_points) {
            result.reason = "too few safe z point-plane correspondences";
            return result;
        }
        result.rmse = std::sqrt(squared_residual_sum / static_cast<double>(effective_points));

        const double delta_z = -gradient_z / solve_hessian_z;
        if (!std::isfinite(delta_z)) {
            result.reason = "non-finite z increment";
            return result;
        }
        const double proposed_correction = correction_z + delta_z;
        if (std::abs(proposed_correction) >= elevator_exit_icp_z_max_correction_m) {
            result.correction_z = proposed_correction;
            result.reason = "z correction exceeds acceptance threshold";
            return result;
        }

        correction_z = proposed_correction;
        result.correction_z = correction_z;
        if (std::abs(delta_z) < elevator_exit_icp_z_converge_m) {
            result.converged = true;
            result.accepted = true;
            result.reason = "accepted";
            return result;
        }
    }

    result.reason = "ICP did not converge";
    return result;
}

void EskfEstimator::prepareFrameCache(const PointCloudXYZI &clouds_lidar) {
    const size_t point_num = clouds_lidar.size();
    cached_point_num_ = point_num;
    point_lidar_vec_.resize(point_num);
    point_imu_vec_.resize(point_num);
    point_lidar_range_.resize(point_num);
    point_world_cache_.resize(point_num);
    norm_vector_.resize(point_num);
    is_effect_point_.resize(point_num, 0);
    Nearest_Points.resize(point_num);
    knn_distance_sq_.resize(point_num);
    for (size_t i = 0; i < point_num; ++i) {
        knn_distance_sq_[i].assign(static_cast<size_t>(std::max(1, ikdtree_match_knn)), 0.0f);
    }

    for (size_t i = 0; i < point_num; ++i) {
        const Eigen::Vector3d point_lidar = clouds_lidar.points[i].getVector3fMap().cast<double>();
        point_lidar_vec_[i] = point_lidar;
        point_imu_vec_[i] = imu_R_lidar * point_lidar + imu_t_lidar;
        point_lidar_range_[i] = point_lidar.norm();
    }
}

void EskfEstimator::evaluatePointMatches(const State &state, PointCloudXYZI &clouds_lidar, bool kd_research_en) {
    static IkdNearestQuery map_query;

    const size_t point_num = clouds_lidar.size();
    const int near_points_num = std::max(1, ikdtree_match_knn);
    const Eigen::Matrix3d R_wb = state.q.toRotationMatrix();

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for schedule(guided, 16)
#endif
    for (size_t i = 0; i < point_num; ++i) {
        point_world_cache_[i] = R_wb * point_imu_vec_[i] + state.p;
    }

    if (!kd_research_en) {
        return;
    }

    std::fill(is_effect_point_.begin(), is_effect_point_.end(), static_cast<uint8_t>(0));
    effective_indices_.clear();
    reject_indices_.clear();

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for schedule(guided, 16)
#endif
    for (size_t i = 0; i < point_num; ++i) {
        PointType point_world = clouds_lidar.points[i];
        point_world.x = static_cast<float>(point_world_cache_[i].x());
        point_world.y = static_cast<float>(point_world_cache_[i].y());
        point_world.z = static_cast<float>(point_world_cache_[i].z());

        auto &points_near = Nearest_Points[i];
        auto &distance_sq = knn_distance_sq_[i];
        const bool enough_neighbors = map_query.searchKnn(point_world, near_points_num, points_near, distance_sq);
        if (!enough_neighbors || distance_sq[near_points_num - 1] > ikdtree_neighbor_max_dist2) {
            continue;
        }

        Eigen::Vector4d pabcd;
        if (!planarFittingAndCheck1(points_near, pabcd, ikdtree_plane_fit_rmse_threshold)) {
            continue;
        }

        const double pd = point_world.x * pabcd(0) + point_world.y * pabcd(1) + point_world.z * pabcd(2) + pabcd(3);
        if (std::isnan(pd)) {
            continue;
        }

        bool accept = false;
        if (useFastLioScoreGate()) {
            const double score = 1.0 - 0.9 * std::fabs(pd) / std::sqrt(std::max(point_lidar_range_[i], 1e-6));
            accept = score > ikdtree_score_min;
        } else {
            const double threshold = ikdtree_match_thresh_base + ikdtree_match_thresh_scale * point_lidar_range_[i];
            accept = std::fabs(pd) < threshold;
        }
        if (accept) {
            norm_vector_[i].norm_vec = pabcd.block<3,1>(0,0);
            norm_vector_[i].point_imu_xyz = point_imu_vec_[i];
            norm_vector_[i].d = pd;
            is_effect_point_[i] = 1;
        }
    }

    effective_indices_.reserve(point_num);
    if (down_reject_cloud_lidar) {
        reject_indices_.reserve(point_num);
    }
    for (size_t i = 0; i < point_num; ++i) {
        if (is_effect_point_[i]) {
            effective_indices_.push_back(static_cast<int>(i));
        } else if (down_reject_cloud_lidar) {
            reject_indices_.push_back(static_cast<int>(i));
        }
    }
}

void EskfEstimator::packMeasurementMatrices(const State &state, PointCloudXYZI &clouds_lidar,
                                            Eigen::MatrixXd &Z, Eigen::MatrixXd &H) {
    const Eigen::Matrix3d R_wb = state.q.toRotationMatrix();
    const int valid_points_num = static_cast<int>(effective_indices_.size());

    H = Eigen::MatrixXd::Zero(valid_points_num, StateIndex::STATE_TOTAL);
    Z.resize(valid_points_num, 1);
    effect_meas_inv_var_.clear();
    effect_meas_inv_var_.reserve(static_cast<size_t>(valid_points_num));

    down_effect_cloud_lidar->clear();
    down_effect_cloud_lidar->points.reserve(static_cast<size_t>(valid_points_num));

    for (int effect_i = 0; effect_i < valid_points_num; ++effect_i) {
        const size_t vi = static_cast<size_t>(effective_indices_[static_cast<size_t>(effect_i)]);
        down_effect_cloud_lidar->points.push_back(clouds_lidar.points[vi]);

        const Eigen::Vector3d dr = -norm_vector_[vi].norm_vec.transpose() * R_wb * skew3d(norm_vector_[vi].point_imu_xyz);
        H.block<1,3>(effect_i, 0) = dr.transpose();
        H.block<1,3>(effect_i, 3) = norm_vector_[vi].norm_vec.transpose();
        H.block<1,6>(effect_i, 6).setZero();
        Z(effect_i, 0) = norm_vector_[vi].d;
        effect_meas_inv_var_.push_back(1.0);
    }
    down_effect_cloud_lidar->width = down_effect_cloud_lidar->points.size();
    down_effect_cloud_lidar->height = 1;
    down_effect_cloud_lidar->is_dense = true;
}

void EskfEstimator::packPoseMeasurementMatrices(const State &state, PointCloudXYZI &clouds_lidar,
                                                Eigen::VectorXd &Z, PoseJacobianMatrix &H_pose) {
    const Eigen::Matrix3d R_wb = state.q.toRotationMatrix();
    const int valid_points_num = static_cast<int>(effective_indices_.size());

    H_pose.resize(valid_points_num, kLidarPoseDim);
    Z.resize(valid_points_num);
    effect_meas_inv_var_.clear();
    effect_meas_inv_var_.reserve(static_cast<size_t>(valid_points_num));

    down_effect_cloud_lidar->clear();
    down_effect_cloud_lidar->points.reserve(static_cast<size_t>(valid_points_num));

    for (int effect_i = 0; effect_i < valid_points_num; ++effect_i) {
        const size_t vi = static_cast<size_t>(effective_indices_[static_cast<size_t>(effect_i)]);
        down_effect_cloud_lidar->points.push_back(clouds_lidar.points[vi]);

        const Eigen::Vector3d dr = -norm_vector_[vi].norm_vec.transpose() * R_wb * skew3d(norm_vector_[vi].point_imu_xyz);
        H_pose.block<1,3>(effect_i, 0) = dr.transpose();
        H_pose.block<1,3>(effect_i, 3) = norm_vector_[vi].norm_vec.transpose();
        Z(effect_i) = norm_vector_[vi].d;
        effect_meas_inv_var_.push_back(1.0);
    }
    down_effect_cloud_lidar->width = down_effect_cloud_lidar->points.size();
    down_effect_cloud_lidar->height = 1;
    down_effect_cloud_lidar->is_dense = true;
}

/**
 * @brief 将点从Lidar坐标系转化到世界坐标系
 * @param point Lidar坐标系中的点
 * @return point_world 世界坐标系中的点
 */
PointType EskfEstimator::transLidar2World(const PointType &point) {
    Eigen::Quaterniond q = state_.q;
    Eigen::Vector3d p = state_.p;
    // Lidar -> IMU -> World
    PointType point_world = transformPoint(transformPoint(point,imu_R_lidar,imu_t_lidar),q,p);
    return point_world;
}


/**
 * @brief 将点从Lidar坐标系转化到电梯坐标系 【 copy 并修改自 transLidar2World 】
 * @param points_lidar Lidar坐标系下的点云
 * @return points_elevator 电梯坐标系下的点云
 */
PointCloudXYZIPtr EskfEstimator::transLidar2Elevator(const PointCloudXYZI &points_lidar) {
    auto start = std::chrono::high_resolution_clock::now();
    /********** ！注意是先平移后旋转！ **********/
    // 先应用从 Lidar 系到 IMU 系的变换   imu_T_lidar
    Eigen::Matrix4d imu_T_lidar = Eigen::Matrix4d::Identity();
    imu_T_lidar.block<3, 3>(0, 0) = imu_R_lidar;
    imu_T_lidar.block<3, 1>(0, 3) = imu_t_lidar;
    // 再应用从 IMU 系到 World 系的变换  world_T_imu * imu_T_lidar
    Eigen::Matrix4d world_T_imu = Eigen::Matrix4d::Identity();
    world_T_imu.block<3, 3>(0, 0) = state_.q.toRotationMatrix();
    world_T_imu.block<3, 1>(0, 3) = state_.p;
    // 应用总的变换矩阵
    Eigen::Matrix4d world_T_lidar = world_T_imu * imu_T_lidar;
    PointCloudXYZIPtr points_world (new PointCloudXYZI);
    // 使用 Eigen 库 + 并行化 比使用 pcl::transformPointCloud 快一些
    points_world->points.resize(points_lidar.size());
    for (size_t i = 0; i < points_lidar.size(); ++i) {
        const auto &pt = points_lidar.points[i];
        Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d pw = world_T_lidar * p;
        auto &out_pt = points_world->points[i];
        out_pt.x = pw.x();
        out_pt.y = pw.y();
        out_pt.z = pw.z();
        out_pt.curvature = pt.curvature;
        out_pt.intensity = pt.intensity;
    }
    // 手动设置元信息
    points_world->width = points_world->points.size();  // 设置 width
    points_world->height = 1;                            // 点云是无组织的（非图像结构）
    points_world->is_dense = true;                       // 没有 NaN 或无效点
    // 结束时间
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> diff_ms = end - start;
    // 将时间（单位 ms）和 points_lidar 的点云数量写入文件中
    // if (log_save_enable) {
    //     ofstream outFile;
    //     outFile.open(string(PACKAGE_ROOT_DIR) + "/trans_time_log.txt", std::ios::app);
    //     outFile << diff_ms.count() << " " << points_lidar.size() << endl;
    //     outFile.close();
    // }
    return points_world;
}


/**
 * @brief 将点云从Lidar坐标系转化到世界坐标系 【 注意，这里是手动实现，只附带了 curvature 和 intensity 信息】
 * @param points_lidar Lidar坐标系下的点云
 * @return points_world 世界坐标系下的点云
 */
PointCloudXYZIPtr EskfEstimator::transLidar2World(const PointCloudXYZI &points_lidar) {
    auto start = std::chrono::high_resolution_clock::now();
    /********** ！注意是先平移后旋转！ **********/
    // 先应用从 Lidar 系到 IMU 系的变换   imu_T_lidar
    Eigen::Matrix4d imu_T_lidar = Eigen::Matrix4d::Identity();
    imu_T_lidar.block<3, 3>(0, 0) = imu_R_lidar;
    imu_T_lidar.block<3, 1>(0, 3) = imu_t_lidar;
    // 再应用从 IMU 系到 World 系的变换  world_T_imu * imu_T_lidar
    Eigen::Matrix4d world_T_imu = Eigen::Matrix4d::Identity();
    world_T_imu.block<3, 3>(0, 0) = state_.q.toRotationMatrix();
    world_T_imu.block<3, 1>(0, 3) = state_.p;
    // 应用总的变换矩阵
    Eigen::Matrix4d world_T_lidar = world_T_imu * imu_T_lidar;
    PointCloudXYZIPtr points_world (new PointCloudXYZI);
    // 使用 Eigen 库 + 并行化 比使用 pcl::transformPointCloud 快一些
    points_world->points.resize(points_lidar.size());
    for (size_t i = 0; i < points_lidar.size(); ++i) {
        const auto &pt = points_lidar.points[i];
        Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d pw = world_T_lidar * p;
        auto &out_pt = points_world->points[i];
        out_pt.x = pw.x();
        out_pt.y = pw.y();
        out_pt.z = pw.z();
        out_pt.curvature = pt.curvature;
        out_pt.intensity = pt.intensity;
    }
    // 手动设置元信息
    points_world->width = points_world->points.size();  // 设置 width
    points_world->height = 1;                            // 点云是无组织的（非图像结构）
    points_world->is_dense = true;                       // 没有 NaN 或无效点
    // 结束时间
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> diff_ms = end - start;
    // 将时间（单位 ms）和 points_lidar 的点云数量写入文件中
    // if (log_save_enable) {
    //     ofstream outFile;
    //     outFile.open(string(PACKAGE_ROOT_DIR) + "/trans_time_log.txt", std::ios::app);
    //     outFile << diff_ms.count() << " " << points_lidar.size() << endl;
    //     outFile.close();
    // }
    return points_world;
}
//
// PointCloudXYZIPtr EskfEstimator::transLidar2World(const PointCloudXYZI& points_lidar)
// {
//     /* —— 1. 组合齐次变换 —— */
//     Eigen::Matrix4d world_T_lidar = Eigen::Matrix4d::Identity();
//
//     // imu_T_lidar
//     world_T_lidar.block<3,3>(0,0) = state_.q.toRotationMatrix() * imu_R_lidar;
//     world_T_lidar.block<3,1>(0,3) = state_.q.toRotationMatrix() * imu_t_lidar + state_.p;
//
//     /* —— 2. 逐点变换 —— */
//     auto points_world = std::make_shared<PointCloudXYZI>();
//     points_world->points.resize(points_lidar.size());
//
//     for (size_t i = 0; i < points_lidar.size(); ++i)
//     {
//         const auto&  src = points_lidar.points[i];
//         auto&        dst = points_world->points[i];
//
//         Eigen::Vector4d p(src.x, src.y, src.z, 1.0);
//         Eigen::Vector4d pw = world_T_lidar * p;
//
//         dst.x = pw.x();
//         dst.y = pw.y();
//         dst.z = pw.z();
//         dst.intensity = src.intensity;
//         dst.curvature = src.curvature;
//     }
//
//     /* —— 3. 填充元信息 —— */
//     points_world->width     = static_cast<uint32_t>(points_world->points.size());
//     points_world->height    = 1;
//     points_world->is_dense  = true;
//
//     return points_world;
// }


/**
 * @brief 根据当前状态计算点云与最近点构成平面间残差（忽略异常点），并给出雅可比矩阵用于迭代kalman滤波（类似梯度下降）
 * @param state 当前迭代状态
 * @param clouds_lidar Lidar系下当前帧点云
 * @param Z 残差
 * @param H 雅可比矩阵
 * @return bool量，表示是否有有效点
 */
bool EskfEstimator::calculate(const State&state, PointCloudXYZI &clouds_lidar, Eigen::MatrixXd & Z,Eigen::MatrixXd & H, bool kd_research_en) {
    if (cached_point_num_ != clouds_lidar.size()) {
        prepareFrameCache(clouds_lidar);
    }
    evaluatePointMatches(state, clouds_lidar, kd_research_en);

    vaild_points_num_ = static_cast<int>(effective_indices_.size());

    if (kd_research_en && down_reject_cloud_lidar) {
        down_reject_cloud_lidar->clear();
        down_reject_cloud_lidar->points.reserve(reject_indices_.size());
        for (int idx : reject_indices_) {
            down_reject_cloud_lidar->points.push_back(clouds_lidar.points[static_cast<size_t>(idx)]);
        }
        down_reject_cloud_lidar->width = down_reject_cloud_lidar->points.size();
        down_reject_cloud_lidar->height = 1;
        down_reject_cloud_lidar->is_dense = true;
    }

    // 输出有效点和实际点的数量
    ROS_DEBUG_STREAM_THROTTLE(1.0, "[calculate] Effective Points: "
                                   << vaild_points_num_ << " / " << clouds_lidar.size());

    // 如果没有有效点，返回false
    if (vaild_points_num_ < 1)
    {
        // std::cerr << "[calculate] Effective Points: " << vaild_points_num << " / "<< point_num << std::endl;
        std ::cerr << "[calculate] effect point num is less than 1" << std::endl;
        PointVector points;
        ikdtree.flatten(ikdtree.Root_Node, points, NOT_RECORD);
        if (!points.empty()) {
            // 将提取的点赋值给点云对象
            pcl::PointCloud<PointType>::Ptr cloud_ikdtree(new pcl::PointCloud<PointType>);
            cloud_ikdtree->points = points;
            cloud_ikdtree->width = points.size();
            cloud_ikdtree->height = 1;
            cloud_ikdtree->is_dense = true;
            // 定义保存路径（建议用时间戳命名，避免覆盖）
            std::time_t now = std::time(nullptr);
            std::string timestamp = std::to_string(now);       // 时间戳字符串
            const std::string pcd_filename = runtimeLogger().makePrefixedPath(
                    LogFileChannel::IkdTreeSnapshot, "_" + timestamp + ".pcd");
            if (!pcd_filename.empty() && runtimeLogger().shouldLog(LogFileChannel::IkdTreeSnapshot)) {
                if (pcl::io::savePCDFileBinary(pcd_filename, *cloud_ikdtree) == 0) {
                    LOG_WARN(Map, "saved ikdtree snapshot to " << pcd_filename << ", size=" << cloud_ikdtree->size());
                } else {
                    LOG_ERROR(Map, "failed to save ikdtree snapshot to " << pcd_filename);
                }
            }
        }
        return false;
    }

    packMeasurementMatrices(state, clouds_lidar, Z, H);
    return true;
}

bool EskfEstimator::calculatePose(const State &state, PointCloudXYZI &clouds_lidar,
                                  Eigen::VectorXd &Z, PoseJacobianMatrix &H_pose, bool kd_research_en) {
    if (cached_point_num_ != clouds_lidar.size()) {
        prepareFrameCache(clouds_lidar);
    }
    evaluatePointMatches(state, clouds_lidar, kd_research_en);

    vaild_points_num_ = static_cast<int>(effective_indices_.size());

    if (kd_research_en && down_reject_cloud_lidar) {
        down_reject_cloud_lidar->clear();
        down_reject_cloud_lidar->points.reserve(reject_indices_.size());
        for (int idx : reject_indices_) {
            down_reject_cloud_lidar->points.push_back(clouds_lidar.points[static_cast<size_t>(idx)]);
        }
        down_reject_cloud_lidar->width = down_reject_cloud_lidar->points.size();
        down_reject_cloud_lidar->height = 1;
        down_reject_cloud_lidar->is_dense = true;
    }

    ROS_DEBUG_STREAM_THROTTLE(1.0, "[calculatePose] Effective Points: "
                                   << vaild_points_num_ << " / " << clouds_lidar.size());

    if (vaild_points_num_ < 1) {
        std::cerr << "[calculatePose] effect point num is less than 1" << std::endl;
        PointVector points;
        ikdtree.flatten(ikdtree.Root_Node, points, NOT_RECORD);
        if (!points.empty()) {
            pcl::PointCloud<PointType>::Ptr cloud_ikdtree(new pcl::PointCloud<PointType>);
            cloud_ikdtree->points = points;
            cloud_ikdtree->width = points.size();
            cloud_ikdtree->height = 1;
            cloud_ikdtree->is_dense = true;
            std::time_t now = std::time(nullptr);
            std::string timestamp = std::to_string(now);
            const std::string pcd_filename = runtimeLogger().makePrefixedPath(
                    LogFileChannel::IkdTreeSnapshot, "_" + timestamp + ".pcd");
            if (!pcd_filename.empty() && runtimeLogger().shouldLog(LogFileChannel::IkdTreeSnapshot)) {
                if (pcl::io::savePCDFileBinary(pcd_filename, *cloud_ikdtree) == 0) {
                    LOG_WARN(Map, "saved ikdtree snapshot to " << pcd_filename << ", size=" << cloud_ikdtree->size());
                } else {
                    LOG_ERROR(Map, "failed to save ikdtree snapshot to " << pcd_filename);
                }
            }
        }
        return false;
    }

    packPoseMeasurementMatrices(state, clouds_lidar, Z, H_pose);
    return true;
}



// 返回后验状态向量组
// 返回后验状态向量组
std::vector<State, Eigen::aligned_allocator<State>> EskfEstimator::get_post_states() {
    return state_vector_post_;
}

// 返回先验状态向量组
std::vector<State, Eigen::aligned_allocator<State>> EskfEstimator::get_prior_states() {
    return state_vector_prior_;
}

// 获取当前状态（在雷达更新前使用 get_prior_states 会报错，这里使用 get_cur_state）
State EskfEstimator::get_cur_state() {
    return state_;
}

// 更新当前状态
void EskfEstimator::set_cur_state(State state_now) {
    state_ = state_now;
}
