/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Shared aliases, configuration, logging helpers, and global declarations.
 */

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

// 启用并行计算
#define MP_EN
#define MP_PROC_NUM 3

#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>  // 替代 geometry_msgs/msg/detail/pose__struct.hpp
#include <sensor_msgs/Imu.h>     // 替代 sensor_msgs/msg/imu.h
#include <ros/ros.h>            // 替代 rclcpp/rclcpp.hpp
#include <pcl/common/transforms.h>
#include "ikd_tree/ikd_Tree.h"
#include "ikd_tree/IkdMap.hpp"
#include "support/YamlReader.h"
#include "support/type.h"
#include <fstream>

// 这里是轮速计消息定义 ROS2
// #include "yhs_can_interfaces/msg/chassis_info_fb.hpp"

// 这里是轮速计消息定义 ROS1
// #include "yhs_can_msgs/wheel_info.h"
#include "lio/wheel_info.h"

#include <chrono>      // std::chrono::milliseconds
#include <algorithm>
#include <cctype>
#include <unordered_map>
#include <unordered_set>
#include <iomanip>     // std::setw
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <filesystem>
#include <limits>

// 定义 ANSI escape codes 用于颜色控制
#define ANSI_COLOR_RESET   "\x1b[0m"
#define ANSI_COLOR_BOLD    "\x1b[1m"
#define ANSI_COLOR_DIM     "\x1b[2m"
#define ANSI_COLOR_ITALIC  "\x1b[3m"
#define ANSI_COLOR_UNDERLINE "\x1b[4m"

#define ANSI_COLOR_BLACK   "\x1b[30m"
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_WHITE   "\x1b[37m"

#define ANSI_COLOR_BG_BLACK   "\x1b[40m"
#define ANSI_COLOR_BG_RED     "\x1b[41m"
#define ANSI_COLOR_BG_GREEN   "\x1b[42m"
#define ANSI_COLOR_BG_YELLOW  "\x1b[43m"
#define ANSI_COLOR_BG_BLUE    "\x1b[44m"
#define ANSI_COLOR_BG_MAGENTA "\x1b[45m"
#define ANSI_COLOR_BG_CYAN    "\x1b[46m"
#define ANSI_COLOR_BG_WHITE   "\x1b[47m"

// 漂亮的配置打印工具
namespace ConfigPrinter {
    inline void printHeader(const std::string& title) {
        std::cout << std::endl;
        std::cout << ANSI_COLOR_BOLD << ANSI_COLOR_CYAN;
        std::cout << "╔════════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║ " << std::left << std::setw(60) << title << " ║" << std::endl;
        std::cout << "╠════════════════════════════════════════════════════════════╣" << ANSI_COLOR_RESET << std::endl;
    }

    inline void printSubHeader(const std::string& title) {
        std::cout << ANSI_COLOR_BOLD << ANSI_COLOR_YELLOW;
        std::cout << "║ " << ANSI_COLOR_MAGENTA << "▸ " << title << ANSI_COLOR_RESET << std::endl;
    }

    inline void printSectionStart() {
    }

    inline void printSectionEnd() {
    }

    inline void printFooter() {
        std::cout << ANSI_COLOR_BOLD << ANSI_COLOR_CYAN;
        std::cout << "╚════════════════════════════════════════════════════════════╝" << ANSI_COLOR_RESET << std::endl;
        std::cout << std::endl;
    }

    template<typename T>
    inline void printItem(const std::string& key, const T& value, bool isDefault = false, int indent = 2) {
        std::cout << ANSI_COLOR_CYAN << "║" << std::string(indent, ' ');
        std::cout << ANSI_COLOR_WHITE << std::left << std::setw(32 - indent) << key << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_DIM << " : " << ANSI_COLOR_RESET;
        if (isDefault) {
            std::cout << ANSI_COLOR_DIM << ANSI_COLOR_YELLOW << value << " (default)" << ANSI_COLOR_RESET;
        } else {
            std::cout << ANSI_COLOR_GREEN << ANSI_COLOR_BOLD << value << ANSI_COLOR_RESET;
        }
        std::cout << std::endl;
    }

    inline void printItemBool(const std::string& key, bool value, bool isDefault = false, int indent = 2) {
        std::cout << ANSI_COLOR_CYAN << "║" << std::string(indent, ' ');
        std::cout << ANSI_COLOR_WHITE << std::left << std::setw(32 - indent) << key << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_DIM << " : " << ANSI_COLOR_RESET;
        if (isDefault) {
            std::cout << ANSI_COLOR_DIM;
        }
        if (value) {
            std::cout << ANSI_COLOR_GREEN << ANSI_COLOR_BOLD << "✓ ON" << ANSI_COLOR_RESET;
        } else {
            std::cout << ANSI_COLOR_RED << ANSI_COLOR_BOLD << "✗ OFF" << ANSI_COLOR_RESET;
        }
        if (isDefault) {
            std::cout << ANSI_COLOR_DIM << " (default)" << ANSI_COLOR_RESET;
        }
        std::cout << std::endl;
    }

    inline void printError(const std::string& key) {
        std::cout << ANSI_COLOR_CYAN << "║ " << ANSI_COLOR_RED << ANSI_COLOR_BOLD << "✗ " << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_RED << std::left << std::setw(30) << key << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_DIM << " : " << ANSI_COLOR_RED << "FAILED" << ANSI_COLOR_RESET;
        std::cout << std::endl;
    }

    inline void printVector(const std::string& key, const Eigen::Vector3d& vec, int indent = 2) {
        std::cout << ANSI_COLOR_CYAN << "║" << std::string(indent, ' ');
        std::cout << ANSI_COLOR_WHITE << std::left << std::setw(32 - indent) << key << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_DIM << " : " << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_GREEN << "[" << vec(0) << ", " << vec(1) << ", " << vec(2) << "]" << ANSI_COLOR_RESET;
        std::cout << std::endl;
    }

    inline void printVector(const std::string& key, const Eigen::Vector2d& vec, int indent = 2) {
        std::cout << ANSI_COLOR_CYAN << "║" << std::string(indent, ' ');
        std::cout << ANSI_COLOR_WHITE << std::left << std::setw(32 - indent) << key << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_DIM << " : " << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_GREEN << "[" << vec(0) << ", " << vec(1) << "]" << ANSI_COLOR_RESET;
        std::cout << std::endl;
    }

    inline void printMatrix(const std::string& key, const Eigen::Matrix3d& mat, int indent = 2) {
        std::cout << ANSI_COLOR_CYAN << "║" << std::string(indent, ' ');
        std::cout << ANSI_COLOR_WHITE << std::left << std::setw(32 - indent) << key << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_DIM << " : " << ANSI_COLOR_RESET << std::endl;
        for (int i = 0; i < 3; i++) {
            std::cout << ANSI_COLOR_CYAN << "║" << std::string(indent + 34, ' ');
            std::cout << ANSI_COLOR_GREEN << "[" << mat(i, 0) << ", " << mat(i, 1) << ", " << mat(i, 2) << "]" << ANSI_COLOR_RESET;
            std::cout << std::endl;
        }
    }

    inline void printIntVector(const std::string& key, const std::vector<int>& vec, bool isDefault = false, int indent = 2) {
        std::cout << ANSI_COLOR_CYAN << "║" << std::string(indent, ' ');
        std::cout << ANSI_COLOR_WHITE << std::left << std::setw(32 - indent) << key << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_DIM << " : " << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_GREEN << "[";
        for (size_t i = 0; i < vec.size(); i++) {
            if (i > 0) std::cout << ", ";
            std::cout << vec[i];
        }
        std::cout << "]" << ANSI_COLOR_RESET;
        if (isDefault) {
            std::cout << ANSI_COLOR_DIM << " (default)" << ANSI_COLOR_RESET;
        }
        std::cout << std::endl;
    }


    inline void printLidarType(int type, int indent = 2) {
        std::cout << ANSI_COLOR_CYAN << "║" << std::string(indent, ' ');
        std::cout << ANSI_COLOR_WHITE << std::left << std::setw(32 - indent) << "lidar_type" << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_DIM << " : " << ANSI_COLOR_RESET;
        std::cout << ANSI_COLOR_GREEN << ANSI_COLOR_BOLD;
        if (type == 1) {
            std::cout << "Livox";
        } else if (type == 2) {
            std::cout << "Ouster";
        } else if (type == 3) {
            std::cout << "Velodyne";
        } else if (type == 4) {
            std::cout << "XT32";
        } else {
            std::cout << type;
        }
        std::cout << ANSI_COLOR_RESET << std::endl;
    }
}

// 全局文件流管理器 - 用于避免频繁的文件 open/close
namespace FileManager {
    inline std::unordered_map<std::string, std::shared_ptr<std::ofstream>>& getLogFileMap() {
        static std::unordered_map<std::string, std::shared_ptr<std::ofstream>> file_map;
        return file_map;
    }
}

enum class LogSeverity {
    Debug,
    Info,
    Warn,
    Error,
};

enum class LogConsoleCategory {
    Core,
    FSM,
    Elevator,
    Perf,
    Map,
    Sensor,
};

enum class LogFileChannel {
    TrajectoryPost,
    TrajectoryPrior,
    RawImu,
    AdaptiveDownsample,
    WheelData,
    WheelResidual,
    ElevatorDebug,
    ElevatorZupt,
    PublishTime,
    IkdTreeSnapshot,
};

struct LogConsoleConfig {
    bool enabled = true;
    bool info = true;
    bool warn = true;
    bool error = true;
    bool debug = false;
    bool fsm = false;
    bool elevator = false;
    bool perf = false;
};

struct LogFileChannelConfig {
    bool enabled = false;
    int every_n = 1;
    double period_s = 0.0;
    std::string file;
    std::string file_prefix;
};

struct LoggerConfig {
    bool enabled = true;
    std::string session_dir = "temp";
    int flush_interval_ms = 1000;
    LogConsoleConfig console;
    struct {
        bool enabled = true;
        LogFileChannelConfig trajectory_post {true, 1, 0.0, "state/post_state.csv", ""};
        LogFileChannelConfig trajectory_prior {false, 10, 0.0, "state/pre_state.csv", ""};
        LogFileChannelConfig raw_imu {false, 20, 0.0, "sensor/raw_imu.csv", ""};
        LogFileChannelConfig adaptive_downsample {true, 1, 0.0, "map/adaptive_downsample.csv", ""};
        LogFileChannelConfig wheel_data {false, 10, 0.0, "sensor/wheel_data.csv", ""};
        LogFileChannelConfig wheel_residual {false, 10, 0.0, "debug/wheel_residual.csv", ""};
        LogFileChannelConfig elevator_debug {false, 5, 0.0, "debug/elevator_debug.csv", ""};
        LogFileChannelConfig elevator_zupt {false, 1, 0.0, "debug/elevator_zupt.txt", ""};
        LogFileChannelConfig publish_time {false, 20, 0.0, "perf/publish_time.csv", ""};
        LogFileChannelConfig ikdtree_snapshot {false, 1, 0.0, "", "debug/ikdtree_cloud"};
    } file;
};

struct LoggerRuntimeStats {
    uint64_t lines_written = 0;
    uint64_t flush_count = 0;
};

class RuntimeLogger {
public:
    void configure(const LoggerConfig &config);
    std::string initializeSession(const std::string &package_root);

    const LoggerConfig &config() const { return config_; }
    const std::string &sessionDir() const { return session_dir_abs_; }
    const LoggerRuntimeStats &stats() const { return stats_; }

    bool shouldLog(LogFileChannel channel, uint64_t seq = 0,
                   double stamp_sec = std::numeric_limits<double>::quiet_NaN());
    void writeLine(LogFileChannel channel, const std::string &header,
                   const std::string &line, uint64_t seq = 0,
                   double stamp_sec = std::numeric_limits<double>::quiet_NaN());
    std::string makePrefixedPath(LogFileChannel channel, const std::string &suffix) const;
    std::string resolvePath(LogFileChannel channel) const;
    void flushIfDue(bool force = false);

    bool shouldLogConsole(LogConsoleCategory category, LogSeverity severity) const;
    void logConsole(LogConsoleCategory category, LogSeverity severity, const std::string &message) const;

private:
    const LogFileChannelConfig &channelConfig(LogFileChannel channel) const;
    std::ofstream &openFileLocked(LogFileChannel channel, const std::string &header);
    static const char *severityName(LogSeverity severity);

    LoggerConfig config_;
    std::string session_dir_abs_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<std::ofstream>> files_;
    std::unordered_set<std::string> header_written_;
    std::unordered_map<int, uint64_t> counters_;
    std::unordered_map<int, double> last_stamp_sec_;
    std::chrono::steady_clock::time_point last_flush_tp_ = std::chrono::steady_clock::now();
    LoggerRuntimeStats stats_;
};

RuntimeLogger &runtimeLogger();

inline const char *logConsoleCategoryName(LogConsoleCategory category) {
    switch (category) {
        case LogConsoleCategory::Core: return "support";
        case LogConsoleCategory::FSM: return "fsm";
        case LogConsoleCategory::Elevator: return "elevator";
        case LogConsoleCategory::Perf: return "perf";
        case LogConsoleCategory::Map: return "map";
        case LogConsoleCategory::Sensor: return "sensor";
    }
    return "unknown";
}

#define LOG_STREAM(severity, category, expr) do { \
    if (runtimeLogger().shouldLogConsole(LogConsoleCategory::category, LogSeverity::severity)) { \
        std::ostringstream __log_stream_oss; \
        __log_stream_oss << expr; \
        runtimeLogger().logConsole(LogConsoleCategory::category, LogSeverity::severity, __log_stream_oss.str()); \
    } \
} while (0)

#define LOG_INFO(category, expr) LOG_STREAM(Info, category, expr)
#define LOG_WARN(category, expr) LOG_STREAM(Warn, category, expr)
#define LOG_ERROR(category, expr) LOG_STREAM(Error, category, expr)
#define LOG_DEBUG(category, expr) LOG_STREAM(Debug, category, expr)

inline std::unordered_map<std::string,
        std::chrono::steady_clock::time_point>& __timer_pool__()
{
    static std::unordered_map<std::string,
        std::chrono::steady_clock::time_point> pool;
    return pool;
}

#define TIC(tag)  (__timer_pool__()[tag] = std::chrono::steady_clock::now())
#define TOC(tag)  do {                                                               \
auto __t = std::chrono::steady_clock::now() - __timer_pool__()[tag];             \
double ms = std::chrono::duration_cast<std::chrono::microseconds>(__t).count() * 1e-3; \
std::cout << ANSI_COLOR_GREEN << "[Time] " << tag << " : " << ms << " ms"         \
<< ANSI_COLOR_RESET << std::endl;                                      \
} while(0)


struct State;

extern KD_TREE<PointType> ikdtree; // 在main.cpp中定义
extern std::shared_ptr<IkdMap<PointType>> ikd_map; // 在main.cpp中定义
extern PointCloudXYZI::Ptr down_effect_cloud_lidar; // 在main.cpp中定义
extern PointCloudXYZI::Ptr down_reject_cloud_lidar; // 在main.cpp中定义
extern PointCloudXYZI::Ptr map_world; // 在main.cpp中定义
extern vector<string> incremental_paths; // 在main.cpp中定义
extern vector<BoxPointType> cub_needrm; // 在main.cpp中定义
extern double lidar_end_time; // 在main.cpp中定义

extern double min_filter_size;

extern bool G_SCALE_UP;

extern bool EXIT_FROM_ELEVATOR;

extern bool ELEVATOR_TRIGGER;

extern string current_tempdir_log;


/************ variability load from yaml ************/
extern Eigen::Vector3d imu_t_lidar; // lidar 系相对 IMU 系的外参   imu_t_lidar           T 4x4  (R t) -> T
extern Eigen::Matrix3d imu_R_lidar; // lidar 系相对 IMU 系的外参   imu_R_lidar
extern Eigen::Vector3d lidar_t_body; // body 系相对 lidar 系的外参  lidar_t_body
extern Eigen::Matrix3d lidar_R_body; // body 系相对 lidar 系的外参  lidar_R_body
extern int point_filter_num; // 点云订阅时每隔 point_filter_num 个点才读入一次

extern double blind; // 距离原点过近的点会被忽略，半径为 blind
extern bool use_box_blind; // true: 删除 LiDAR 坐标系中指定长方体内的点；false: 使用 blind 球形盲区
extern Eigen::Vector2d box_corner_x; // 长方体 x 轴范围 [min, max]
extern Eigen::Vector2d box_corner_y; // 长方体 y 轴范围 [min, max]
extern Eigen::Vector2d box_corner_z; // 长方体 z 轴范围 [min, max]
extern float filter_size; // 点云降采样大小（ikdtree大小）
extern bool downsample_adaptive_enabled; // 是否启用变体素大小降采样
extern double downsample_target_points; // Adaptive voxel target points per second
extern double downsample_alpha;         // Adaptive voxel alpha
extern double downsample_min_voxel;     // Adaptive voxel min
extern double downsample_max_voxel;     // Adaptive voxel max
extern double local_map_cube_len;       // Local ikd-tree map cube edge length in metres

// 根据当前盲区配置判断 LiDAR 原始点是否保留。
// box 模式会删除长方体内（含边界）的自遮挡点；其他模式删除 blind 半径内的点。
inline bool keepLidarPoint(float x, float y, float z) {
    if (use_box_blind) {
        const double min_x = std::min(box_corner_x[0], box_corner_x[1]);
        const double max_x = std::max(box_corner_x[0], box_corner_x[1]);
        const double min_y = std::min(box_corner_y[0], box_corner_y[1]);
        const double max_y = std::max(box_corner_y[0], box_corner_y[1]);
        const double min_z = std::min(box_corner_z[0], box_corner_z[1]);
        const double max_z = std::max(box_corner_z[0], box_corner_z[1]);
        const bool inside_box =
            x >= min_x && x <= max_x &&
            y >= min_y && y <= max_y &&
            z >= min_z && z <= max_z;
        return !inside_box;
    }
    return x * x + y * y + z * z > blind * blind;
}

extern bool global_map_save_enable; // 是否启用点云保存
extern int every_n_frame_save; // 每隔多少帧保存一次单帧地图，-1表示不保存
extern bool timestamp_prefix; // 是否启用时间戳前缀
extern string pcd_save_name; // 点云保存的文件名
extern bool global_map_pub_enable; // 是否发布全局地图
extern bool ikdtree_pub_enable; // 是否发布全局地图
extern int clouds_lidar_pub_every_n; // lidar系当前帧点云每隔多少帧发布一次，1表示每帧都发布
extern bool high_frequency_odom; // 是否在两帧 LiDAR 之间按 IMU 预积分状态高频发布里程计
extern double lidar_frequency; // 高频里程计用于估计下一帧 LiDAR 到达时间的 LiDAR 频率 [Hz]

extern string lidar_topic_name;
extern string imu_topic_name;
extern string wheel_topic_name;
extern string image_topic_name;
extern string elevator_flag_topic_name;

extern string clouds_lidar_topic_name; // lidar系当前帧点云发布的话题名
extern string clouds_lidar_effect_topic_name; // lidar系当前帧有效点云发布的话题名
extern string clouds_lidar_reject_topic_name; // lidar系被滤除点云发布的话题名
extern string global_map_topic_name; // world系全局地图发布的话题名
extern string ikdtree_topic_name; // world系 ikdtree 发布的话题名
extern string odom_imu_topic_name; // imu位姿的话题名
extern string odom_vehicle_topic_name; // body位姿的话题名
extern string lidar_update_mode; // lidar更新模式: pose6 / full21
extern bool online_gravity_estimation_enable; // 是否在线估计重力加速度状态
extern int lidar_ieskf_max_iterations;
extern double lidar_ieskf_converge_rot_rad;
extern double lidar_ieskf_converge_pos_m;
extern double lidar_ieskf_rematch_rot_rad;
extern double lidar_ieskf_rematch_pos_m;
extern int ikdtree_match_knn;
extern double ikdtree_plane_fit_rmse_threshold;
extern double ikdtree_neighbor_max_dist2;
extern string ikdtree_gate_mode; // fastlio_score / linear_range
extern double ikdtree_score_min;
extern double ikdtree_match_thresh_base;
extern double ikdtree_match_thresh_scale;
extern bool relocation_enable; // 是否启用重定位
extern string pcd_load_name; // 重定位时，加载的点云文件名

extern bool elevator_enable;
extern bool self_exit_detector;
extern bool elevator_door_detector_enable;
extern double elevator_door_dist_threshold;
extern double elevator_door_time_threshold;
extern double elevator_door_filter_percent;
extern double elevator_door_cooldown_time;
extern bool elevator_strong_prior_enable;
extern double elevator_strong_prior_acc_var;
extern bool elevator_zupt_enable;
extern bool elevator_waiting_zupt_enable;
extern double elevator_waiting_zupt_period_s;
extern double elevator_waiting_zupt_vz_abs_thresh;
extern double elevator_waiting_zupt_az_abs_thresh;
extern bool elevator_exit_icp_z_enable;
extern double elevator_exit_icp_z_max_correction_m;
extern int elevator_exit_icp_z_max_iterations;
extern int elevator_exit_icp_z_min_effective_points;
extern double elevator_exit_icp_z_min_information_ratio;
extern double elevator_exit_icp_z_max_neighbor_distance_m;
extern double elevator_exit_icp_z_min_abs_normal_z;
extern double elevator_exit_icp_z_max_residual_m;
extern double elevator_exit_icp_z_converge_m;
extern bool elevator_rebuild_map_on_exit_enable;
extern int elevator_rebuild_map_on_exit_frames;
extern int elevator_rebuild_map_on_exit_min_points;

extern bool wheel_enable; // 是否启用wheel轮速计更新
extern bool log_save_enable; // 是否记录日志，全局设置
extern bool wheel_data_log_enable; // 是否记录轮速计数据
extern bool trajectory_log_enable; // 是否记录位姿数据
extern bool time_log_enable; // 是否记录程序耗时
extern bool pub_time_log_enable; // 是否记录topic发送耗时
extern bool res_wheel_log_enable; // 是否记录轮速计残差，在 UpdateByWheel() 中使用

extern bool CovMatrix_vis;
extern bool EleState_vis;

extern YamlReader reader;

extern int lidar_type; // Livox = 1, Ouster = 2, Velodyne = 3, XT32 = 4

// 返回叉乘反对称阵
inline Eigen::Matrix3d skew3d(const Eigen::Vector3d &a) {
    Eigen::Matrix3d A;
    A << 0, -a(2), a(1),
         a(2), 0, -a(0),
         -a(1), a(0), 0;
    return A;
}

inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d &so3 ){
    Eigen::Matrix3d  SO3;
    double so3_norm = so3.norm();
    if (so3_norm<=0.0000001)
    {
        SO3.setIdentity();
        return SO3;
    }

    Eigen::Matrix3d so3_skew_sym = skew3d(so3);
    SO3 = Eigen::Matrix3d::Identity()+(so3_skew_sym/so3_norm)*sin(so3_norm)+(so3_skew_sym*so3_skew_sym/(so3_norm*so3_norm))*(1-cos(so3_norm));
    return SO3;
}

void load_lio_yaml(const std::vector<string>& file_names);


extern string sensor_config_yaml;
extern string runtime_config_yaml;
extern string logging_config_yaml;
extern string camera_yaml;

void load_root_yaml(string file_name);



// 时间戳转换为秒
inline double get_time_sec(const ros::Time &time)
{
    return time.toSec();
}

// 时间戳转换为ros时间
inline ros::Time get_ros_time(double timestamp)
{
    int32_t sec = static_cast<int32_t>(std::floor(timestamp));
    uint32_t nanosec = static_cast<uint32_t>((timestamp - std::floor(timestamp)) * 1e9);
    return ros::Time(sec, nanosec);
}


#endif //COMMON_LIB_H
