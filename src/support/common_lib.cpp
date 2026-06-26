//
// Created by xiaofan on 25-4-3.
//

#include "support/common_lib.h"
#include <cmath>

namespace {
template <typename T>
void readYamlOrDefault(YamlReader &reader, const std::string &key, T &value) {
    T parsed = value;
    if (reader.readOptional(key, parsed)) {
        value = parsed;
    }
}

void readFileChannelConfig(YamlReader &reader, const std::string &base_key,
                           LogFileChannelConfig &cfg) {
    readYamlOrDefault(reader, base_key + ".enabled", cfg.enabled);
    readYamlOrDefault(reader, base_key + ".every_n", cfg.every_n);
    readYamlOrDefault(reader, base_key + ".period_s", cfg.period_s);
    if (!cfg.file.empty()) {
        readYamlOrDefault(reader, base_key + ".file", cfg.file);
    }
    if (!cfg.file_prefix.empty()) {
        readYamlOrDefault(reader, base_key + ".file_prefix", cfg.file_prefix);
    }
    if (cfg.every_n < 1) cfg.every_n = 1;
    if (cfg.period_s < 0.0) cfg.period_s = 0.0;
}
} // namespace

RuntimeLogger &runtimeLogger() {
    static RuntimeLogger logger;
    return logger;
}

void RuntimeLogger::configure(const LoggerConfig &config) {
    std::lock_guard<std::mutex> lk(mutex_);
    config_ = config;
}

std::string RuntimeLogger::initializeSession(const std::string &package_root) {
    std::lock_guard<std::mutex> lk(mutex_);
    files_.clear();
    header_written_.clear();
    counters_.clear();
    last_stamp_sec_.clear();
    stats_ = {};
    last_flush_tp_ = std::chrono::steady_clock::now();

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");
    session_dir_abs_ = package_root + "/" + config_.session_dir + "/" + ss.str();
    std::filesystem::create_directories(session_dir_abs_);
    return session_dir_abs_;
}

const LogFileChannelConfig &RuntimeLogger::channelConfig(LogFileChannel channel) const {
    switch (channel) {
        case LogFileChannel::TrajectoryPost: return config_.file.trajectory_post;
        case LogFileChannel::TrajectoryPrior: return config_.file.trajectory_prior;
        case LogFileChannel::RawImu: return config_.file.raw_imu;
        case LogFileChannel::AdaptiveDownsample: return config_.file.adaptive_downsample;
        case LogFileChannel::WheelData: return config_.file.wheel_data;
        case LogFileChannel::WheelResidual: return config_.file.wheel_residual;
        case LogFileChannel::ElevatorDebug: return config_.file.elevator_debug;
        case LogFileChannel::ElevatorZupt: return config_.file.elevator_zupt;
        case LogFileChannel::PublishTime: return config_.file.publish_time;
        case LogFileChannel::IkdTreeSnapshot: return config_.file.ikdtree_snapshot;
    }
    return config_.file.trajectory_post;
}

std::string RuntimeLogger::resolvePath(LogFileChannel channel) const {
    const auto &cfg = channelConfig(channel);
    if (session_dir_abs_.empty() || cfg.file.empty()) return "";
    return session_dir_abs_ + "/" + cfg.file;
}

std::string RuntimeLogger::makePrefixedPath(LogFileChannel channel, const std::string &suffix) const {
    const auto &cfg = channelConfig(channel);
    if (session_dir_abs_.empty() || cfg.file_prefix.empty()) return "";
    return session_dir_abs_ + "/" + cfg.file_prefix + suffix;
}

bool RuntimeLogger::shouldLog(LogFileChannel channel, uint64_t seq, double stamp_sec) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!config_.enabled || !config_.file.enabled) return false;

    const auto &cfg = channelConfig(channel);
    if (!cfg.enabled) return false;

    const int key = static_cast<int>(channel);
    const int every_n = cfg.every_n;
    if (seq > 0 && (seq % static_cast<uint64_t>(every_n) != 0)) return false;

    if (!std::isnan(stamp_sec) && cfg.period_s > 0.0) {
        auto it = last_stamp_sec_.find(key);
        if (it != last_stamp_sec_.end() && (stamp_sec - it->second) < cfg.period_s) {
            return false;
        }
        last_stamp_sec_[key] = stamp_sec;
    }

    ++counters_[key];
    return true;
}

std::ofstream &RuntimeLogger::openFileLocked(LogFileChannel channel, const std::string &header) {
    const std::string path = resolvePath(channel);
    auto it = files_.find(path);
    if (it == files_.end()) {
        std::filesystem::create_directories(std::filesystem::path(path).parent_path());
        auto file_ptr = std::make_shared<std::ofstream>(path, std::ios::out | std::ios::trunc);
        it = files_.emplace(path, file_ptr).first;
    }

    if (!header.empty() && header_written_.find(path) == header_written_.end() &&
        it->second && it->second->is_open()) {
        *it->second << header << '\n';
        header_written_.insert(path);
    }
    return *it->second;
}

void RuntimeLogger::writeLine(LogFileChannel channel, const std::string &header,
                              const std::string &line, uint64_t seq, double stamp_sec) {
    if (!shouldLog(channel, seq, stamp_sec)) return;

    std::lock_guard<std::mutex> lk(mutex_);
    auto &stream = openFileLocked(channel, header);
    if (!stream.is_open()) return;
    stream << line << '\n';
    ++stats_.lines_written;
    flushIfDue(false);
}

void RuntimeLogger::flushIfDue(bool force) {
    const auto now = std::chrono::steady_clock::now();
    if (!force && config_.flush_interval_ms > 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_flush_tp_).count() < config_.flush_interval_ms) {
        return;
    }
    for (auto &entry : files_) {
        if (entry.second && entry.second->is_open()) {
            entry.second->flush();
        }
    }
    last_flush_tp_ = now;
    ++stats_.flush_count;
}

bool RuntimeLogger::shouldLogConsole(LogConsoleCategory category, LogSeverity severity) const {
    if (!config_.enabled || !config_.console.enabled) return false;
    switch (severity) {
        case LogSeverity::Debug:
            if (!config_.console.debug) return false;
            break;
        case LogSeverity::Info:
            if (!config_.console.info) return false;
            break;
        case LogSeverity::Warn:
            if (!config_.console.warn) return false;
            break;
        case LogSeverity::Error:
            if (!config_.console.error) return false;
            break;
    }
    switch (category) {
        case LogConsoleCategory::FSM: return config_.console.fsm;
        case LogConsoleCategory::Elevator: return config_.console.elevator;
        case LogConsoleCategory::Perf: return config_.console.perf;
        default: return true;
    }
}

const char *RuntimeLogger::severityName(LogSeverity severity) {
    switch (severity) {
        case LogSeverity::Debug: return "DEBUG";
        case LogSeverity::Info: return "INFO";
        case LogSeverity::Warn: return "WARN";
        case LogSeverity::Error: return "ERROR";
    }
    return "INFO";
}

void RuntimeLogger::logConsole(LogConsoleCategory category, LogSeverity severity, const std::string &message) const {
    std::ostream &out = (severity == LogSeverity::Warn || severity == LogSeverity::Error) ? std::cerr : std::cout;
    out << "[" << severityName(severity) << "][" << logConsoleCategoryName(category) << "] " << message << '\n';
}

/************ variability load from yaml ************/
Eigen::Vector3d imu_t_lidar; // lidar 系相对 IMU 系的外参   imu_t_lidar           T 4x4  (R t) -> T
Eigen::Matrix3d imu_R_lidar; // lidar 系相对 IMU 系的外参   imu_R_lidar
Eigen::Vector3d lidar_t_body; // body 系相对 lidar 系的外参  lidar_t_body
Eigen::Matrix3d lidar_R_body; // body 系相对 lidar 系的外参  lidar_R_body
int point_filter_num; // 点云订阅时每隔 point_filter_num 个点才读入一次

double blind; // 距离原点过近的点会被忽略，半径为 blind
bool use_box_blind = false;
Eigen::Vector2d box_corner_x(-0.7, 0.1);
Eigen::Vector2d box_corner_y(-0.3, 0.3);
Eigen::Vector2d box_corner_z(-0.4, 0.4);
float filter_size = 0.5; // 点云降采样大小（ikdtree大小），默认值0.5
bool downsample_adaptive_enabled = false; // 是否启用变体素大小降采样
double downsample_target_points = 2500.0; // 自适应体素的目标每秒点数
double downsample_alpha = 1.5;
double downsample_min_voxel = 0.02;
double downsample_max_voxel = 0.8;
double local_map_cube_len = 1500.0;

bool global_map_save_enable; // 是否启用点云保存
int every_n_frame_save; // 每隔多少帧保存一次单帧地图，-1表示不保存
bool timestamp_prefix; // 是否启用时间戳前缀
string pcd_save_name; // 点云保存的文件名
bool global_map_pub_enable; // 是否发布全局地图
bool ikdtree_pub_enable; // 是否发布全局地图
int clouds_lidar_pub_every_n = 1;
bool high_frequency_odom = false;
double lidar_frequency = 10.0;

string lidar_topic_name;
string imu_topic_name;
string wheel_topic_name;
string image_topic_name;
string elevator_flag_topic_name;

string clouds_lidar_topic_name; // lidar系当前帧点云发布的话题名
string clouds_lidar_effect_topic_name; // lidar系当前帧有效点云发布的话题名
string clouds_lidar_reject_topic_name; // lidar系被滤除点云发布的话题名
string global_map_topic_name; // world系全局地图发布的话题名
string ikdtree_topic_name; // world系 ikdtree 发布的话题名
string odom_imu_topic_name; // imu位姿的话题名
string odom_vehicle_topic_name; // body位姿的话题名
string lidar_update_mode = "pose6";
bool online_gravity_estimation_enable = false;
int lidar_ieskf_max_iterations = 4;
double lidar_ieskf_converge_rot_rad = 1e-3;
double lidar_ieskf_converge_pos_m = 1e-3;
double lidar_ieskf_rematch_rot_rad = 2e-3;
double lidar_ieskf_rematch_pos_m = 2e-3;
int ikdtree_match_knn = 5;
double ikdtree_plane_fit_rmse_threshold = 0.05;
double ikdtree_neighbor_max_dist2 = 5.0;
string ikdtree_gate_mode = "linear_range";
double ikdtree_score_min = 0.95;
double ikdtree_match_thresh_base = 0.03;
double ikdtree_match_thresh_scale = 0.01;
bool relocation_enable; // 是否启用重定位
string pcd_load_name; // 重定位时，加载的点云文件名

bool elevator_enable = true;
bool self_exit_detector;
bool elevator_door_detector_enable = false;
double elevator_door_dist_threshold = 3.0;
double elevator_door_time_threshold = 2.0;
double elevator_door_filter_percent = 0.03;
double elevator_door_cooldown_time = 5.0;
bool elevator_strong_prior_enable = false;
double elevator_strong_prior_acc_var = 1e-3;
bool elevator_zupt_enable = true;
bool elevator_waiting_zupt_enable = true;
double elevator_waiting_zupt_period_s = 3.0;
double elevator_waiting_zupt_vz_abs_thresh = 0.1;
double elevator_waiting_zupt_az_abs_thresh = 0.1;
bool elevator_exit_icp_z_enable = true;
double elevator_exit_icp_z_max_correction_m = 0.3;
int elevator_exit_icp_z_max_iterations = 5;
int elevator_exit_icp_z_min_effective_points = 20;
double elevator_exit_icp_z_min_information_ratio = 0.1;
double elevator_exit_icp_z_max_neighbor_distance_m = 0.5;
double elevator_exit_icp_z_min_abs_normal_z = 0.7;
double elevator_exit_icp_z_max_residual_m = 0.3;
double elevator_exit_icp_z_converge_m = 1e-3;
bool elevator_rebuild_map_on_exit_enable = false;
int elevator_rebuild_map_on_exit_frames = 5;
int elevator_rebuild_map_on_exit_min_points = 2000;

bool wheel_enable; // 是否启用wheel轮速计更新


bool log_save_enable; // 是否记录日志，全局设置
bool wheel_data_log_enable; // 是否记录轮速计数据
bool trajectory_log_enable; // 是否记录位姿数据
bool time_log_enable; // 是否记录程序耗时
bool pub_time_log_enable; // 是否记录topic发送耗时
bool res_wheel_log_enable; // 是否记录轮速计残差，在 UpdateByWheel() 中使用
LoggerConfig logger_config;

bool CovMatrix_vis = false;
bool EleState_vis = false;

int lidar_type = 1; // Livox = 1, Ouster = 2, Velodyne = 3, XT32 = 4


void load_lio_yaml(const std::vector<string>& file_names) {
    std::vector<std::string> config_paths;
    config_paths.reserve(file_names.size());
    for (const auto& file_name : file_names) {
        config_paths.push_back(std::string(PACKAGE_ROOT_DIR) + "/yaml/" + file_name);
    }
    YamlReader reader(std::move(config_paths));
    if (reader.load()) {
        ConfigPrinter::printHeader("🚀 LIO Configuration");

        /******** lidar_type ********/
        ConfigPrinter::printSubHeader("Lidar Configuration");
        if (reader.read("lidar_type", lidar_type)) {
            ConfigPrinter::printLidarType(lidar_type);
        } else {
            ConfigPrinter::printError("lidar_type");
        }

        /******** offset ********/
        ConfigPrinter::printSubHeader("Extrinsic Parameters");
        if (reader.readEigen("offset.imu_t_lidar", imu_t_lidar)) {
            ConfigPrinter::printVector("imu_t_lidar", imu_t_lidar);
        } else {
            ConfigPrinter::printError("imu_t_lidar");
        }
        if (reader.readEigen("offset.imu_R_lidar", imu_R_lidar)) {
            ConfigPrinter::printMatrix("imu_R_lidar", imu_R_lidar);
        } else {
            ConfigPrinter::printError("imu_R_lidar");
        }
        if (reader.readEigen("offset.lidar_t_body", lidar_t_body)) {
            ConfigPrinter::printVector("lidar_t_body", lidar_t_body);
        } else {
            ConfigPrinter::printError("lidar_t_body");
        }
        if (reader.readEigen("offset.lidar_R_body", lidar_R_body)) {
            ConfigPrinter::printMatrix("lidar_R_body", lidar_R_body);
        } else {
            ConfigPrinter::printError("lidar_R_body");
        }

        /******** downsample ********/
        ConfigPrinter::printSubHeader("Downsample Parameters");
        if (reader.read("downsample.point_filter_num", point_filter_num)) {
            ConfigPrinter::printItem("point_filter_num", point_filter_num);
        } else {
            ConfigPrinter::printError("point_filter_num");
        }
        if (reader.read("downsample.blind", blind)) {
            ConfigPrinter::printItem("blind", blind);
        } else {
            ConfigPrinter::printError("blind");
        }
        if (reader.read("downsample.use_box_blind", use_box_blind)) {
            ConfigPrinter::printItemBool("use_box_blind", use_box_blind);
        } else {
            ConfigPrinter::printItemBool("use_box_blind", use_box_blind, true);
        }
        if (reader.readEigen("downsample.box_corner.box_corner_x", box_corner_x)) {
            ConfigPrinter::printVector("box_corner_x", box_corner_x);
        } else if (use_box_blind) {
            ConfigPrinter::printError("box_corner.box_corner_x");
        }
        if (reader.readEigen("downsample.box_corner.box_corner_y", box_corner_y)) {
            ConfigPrinter::printVector("box_corner_y", box_corner_y);
        } else if (use_box_blind) {
            ConfigPrinter::printError("box_corner.box_corner_y");
        }
        if (reader.readEigen("downsample.box_corner.box_corner_z", box_corner_z)) {
            ConfigPrinter::printVector("box_corner_z", box_corner_z);
        } else if (use_box_blind) {
            ConfigPrinter::printError("box_corner.box_corner_z");
        }
        if (reader.read("downsample.filter_size", filter_size)) {
            ConfigPrinter::printItem("filter_size", filter_size);
        } else {
            ConfigPrinter::printError("filter_size");
        }
        if (reader.read("downsample.adaptive.target_points", downsample_target_points)) {
            ConfigPrinter::printItem("adaptive.target_points", downsample_target_points);
        } else {
            ConfigPrinter::printError("adaptive.target_points");
        }
        if (reader.read("downsample.adaptive.alpha", downsample_alpha)) {
            ConfigPrinter::printItem("adaptive.alpha", downsample_alpha);
        } else {
            ConfigPrinter::printError("adaptive.alpha");
        }
        if (reader.read("downsample.adaptive.min_voxel", downsample_min_voxel)) {
            ConfigPrinter::printItem("adaptive.min_voxel", downsample_min_voxel);
        } else {
            ConfigPrinter::printError("adaptive.min_voxel");
        }
        if (reader.read("downsample.adaptive.max_voxel", downsample_max_voxel)) {
            ConfigPrinter::printItem("adaptive.max_voxel", downsample_max_voxel);
        } else {
            ConfigPrinter::printError("adaptive.max_voxel");
        }
        if (reader.read("downsample.adaptive.enable", downsample_adaptive_enabled)) {
            ConfigPrinter::printItemBool("adaptive.enable", downsample_adaptive_enabled);
        } else {
            ConfigPrinter::printItemBool("adaptive.enable", downsample_adaptive_enabled, true);
        }

        /******** pcd_save ********/
        ConfigPrinter::printSubHeader("PCD Save Settings");
        if (reader.read("pcd_save.global_map_save_enable", global_map_save_enable)) {
            ConfigPrinter::printItemBool("global_map_save_enable", global_map_save_enable);
        } else {
            ConfigPrinter::printError("global_map_save_enable");
        }
        if (reader.read("pcd_save.every_n_frame_save", every_n_frame_save)) {
            ConfigPrinter::printItem("every_n_frame_save", every_n_frame_save);
        } else {
            ConfigPrinter::printError("every_n_frame_save");
        }
        if (reader.read("pcd_save.timestamp_prefix", timestamp_prefix)) {
            ConfigPrinter::printItemBool("timestamp_prefix", timestamp_prefix);
        } else {
            ConfigPrinter::printError("timestamp_prefix");
        }
        if (reader.read("pcd_save.pcd_save_name", pcd_save_name)) {
            ConfigPrinter::printItem("pcd_save_name", pcd_save_name);
        } else {
            ConfigPrinter::printError("pcd_save_name");
        }

        /******** pub ********/
        ConfigPrinter::printSubHeader("Publish Settings");
        if (reader.read("pub.global_map_pub_enable", global_map_pub_enable)) {
            ConfigPrinter::printItemBool("global_map_pub_enable", global_map_pub_enable);
        } else {
            ConfigPrinter::printError("global_map_pub_enable");
        }
        if (reader.read("pub.ikdtree_pub_enable", ikdtree_pub_enable)) {
            ConfigPrinter::printItemBool("ikdtree_pub_enable", ikdtree_pub_enable);
        } else {
            ConfigPrinter::printError("ikdtree_pub_enable");
        }
        if (reader.read("pub.clouds_lidar_pub_every_n", clouds_lidar_pub_every_n)) {
            clouds_lidar_pub_every_n = std::max(1, clouds_lidar_pub_every_n);
            ConfigPrinter::printItem("clouds_lidar_pub_every_n", clouds_lidar_pub_every_n);
        } else {
            clouds_lidar_pub_every_n = 1;
            ConfigPrinter::printItem("clouds_lidar_pub_every_n", clouds_lidar_pub_every_n, true);
        }
        bool high_frequency_odom_configured = reader.read("pub.high_frequency_odom", high_frequency_odom);
        if (!high_frequency_odom_configured) {
            high_frequency_odom_configured = reader.read("high_frequency_odom", high_frequency_odom);
        }
        ConfigPrinter::printItemBool("high_frequency_odom", high_frequency_odom,
                                     !high_frequency_odom_configured);

        bool lidar_frequency_configured = reader.read("pub.lidar_frequency", lidar_frequency);
        if (!lidar_frequency_configured) {
            lidar_frequency_configured = reader.read("lidar_frequency", lidar_frequency);
        }
        if (lidar_frequency_configured) {
            if (lidar_frequency <= 0.0) {
                LOG_WARN(Core, "pub.lidar_frequency must be positive; fallback to 10 Hz");
                lidar_frequency = 10.0;
            }
        }
        ConfigPrinter::printItem("lidar_frequency", lidar_frequency, !lidar_frequency_configured);

        /******** topic_sub ********/
        ConfigPrinter::printSubHeader("Subscribe Topics");
        if (reader.read("topic_sub.lidar_topic_name", lidar_topic_name)) {
            ConfigPrinter::printItem("lidar_topic_name", lidar_topic_name);
        } else {
            ConfigPrinter::printError("lidar_topic_name");
        }
        if (reader.read("topic_sub.imu_topic_name", imu_topic_name)) {
            ConfigPrinter::printItem("imu_topic_name", imu_topic_name);
        } else {
            ConfigPrinter::printError("imu_topic_name");
        }
        if (reader.read("topic_sub.wheel_topic_name", wheel_topic_name)) {
            ConfigPrinter::printItem("wheel_topic_name", wheel_topic_name);
        } else {
            ConfigPrinter::printError("wheel_topic_name");
        }
        if (reader.read("topic_sub.elevator_flag_topic_name", elevator_flag_topic_name)) {
            ConfigPrinter::printItem("elevator_flag_topic_name", elevator_flag_topic_name);
        } else {
            ConfigPrinter::printError("elevator_flag_topic_name");
        }

        /******** topic_pub ********/
        ConfigPrinter::printSubHeader("Publish Topics");
        if (reader.read("topic_pub.clouds_lidar_topic_name", clouds_lidar_topic_name)) {
            ConfigPrinter::printItem("clouds_lidar_topic_name", clouds_lidar_topic_name);
        } else {
            ConfigPrinter::printError("clouds_lidar_topic_name");
        }
        if (reader.read("topic_pub.clouds_lidar_effect_topic_name", clouds_lidar_effect_topic_name)) {
            ConfigPrinter::printItem("clouds_lidar_effect_topic_name", clouds_lidar_effect_topic_name);
        } else {
            clouds_lidar_effect_topic_name = clouds_lidar_topic_name + "/effect";
            ConfigPrinter::printItem("clouds_lidar_effect_topic_name", clouds_lidar_effect_topic_name, true);
        }
        if (reader.read("topic_pub.clouds_lidar_reject_topic_name", clouds_lidar_reject_topic_name)) {
            ConfigPrinter::printItem("clouds_lidar_reject_topic_name", clouds_lidar_reject_topic_name);
        } else {
            clouds_lidar_reject_topic_name = clouds_lidar_topic_name + "/reject";
            ConfigPrinter::printItem("clouds_lidar_reject_topic_name", clouds_lidar_reject_topic_name, true);
        }
        if (reader.read("topic_pub.global_map_topic_name", global_map_topic_name)) {
            ConfigPrinter::printItem("global_map_topic_name", global_map_topic_name);
        } else {
            ConfigPrinter::printError("global_map_topic_name");
        }
        if (reader.read("topic_pub.ikdtree_topic_name", ikdtree_topic_name)) {
            ConfigPrinter::printItem("ikdtree_topic_name", ikdtree_topic_name);
        } else {
            ConfigPrinter::printError("ikdtree_topic_name");
        }
        if (reader.read("topic_pub.odom_imu_topic_name", odom_imu_topic_name)) {
            ConfigPrinter::printItem("odom_imu_topic_name", odom_imu_topic_name);
        } else {
            ConfigPrinter::printError("odom_imu_topic_name");
        }
        if (reader.read("topic_pub.odom_vehicle_topic_name", odom_vehicle_topic_name)) {
            ConfigPrinter::printItem("odom_vehicle_topic_name", odom_vehicle_topic_name);
        } else {
            ConfigPrinter::printError("odom_vehicle_topic_name");
        }

        /******** map ********/
        ConfigPrinter::printSubHeader("Map");
        ConfigPrinter::printItem("map_type", "ikdtree");
        readYamlOrDefault(reader, "map.local_map_cube_len", local_map_cube_len);
        ConfigPrinter::printItem("local_map_cube_len", local_map_cube_len);
        const bool lidar_update_mode_configured =
                reader.read("estimator.lidar_update_mode", lidar_update_mode);
        std::transform(lidar_update_mode.begin(), lidar_update_mode.end(), lidar_update_mode.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lidar_update_mode == "pose_6") lidar_update_mode = "pose6";
        if (lidar_update_mode == "full_21") lidar_update_mode = "full21";
        if (lidar_update_mode != "pose6" && lidar_update_mode != "full21") {
            LOG_WARN(Core, "unsupported estimator.lidar_update_mode='" << lidar_update_mode
                           << "', fallback to pose6");
            lidar_update_mode = "pose6";
        }
        ConfigPrinter::printItem("lidar_update_mode", lidar_update_mode,
                                 !lidar_update_mode_configured);
        readYamlOrDefault(reader, "estimator.online_gravity_estimation_enable", online_gravity_estimation_enable);
        readYamlOrDefault(reader, "estimator.lidar_ieskf.max_iterations", lidar_ieskf_max_iterations);
        readYamlOrDefault(reader, "estimator.lidar_ieskf.converge_rot_rad", lidar_ieskf_converge_rot_rad);
        readYamlOrDefault(reader, "estimator.lidar_ieskf.converge_pos_m", lidar_ieskf_converge_pos_m);
        readYamlOrDefault(reader, "estimator.lidar_ieskf.rematch_rot_rad", lidar_ieskf_rematch_rot_rad);
        readYamlOrDefault(reader, "estimator.lidar_ieskf.rematch_pos_m", lidar_ieskf_rematch_pos_m);
        readYamlOrDefault(reader, "estimator.ikdtree_match.knn", ikdtree_match_knn);
        readYamlOrDefault(reader, "estimator.ikdtree_match.plane_fit_rmse_threshold", ikdtree_plane_fit_rmse_threshold);
        readYamlOrDefault(reader, "estimator.ikdtree_match.neighbor_max_dist2", ikdtree_neighbor_max_dist2);
        readYamlOrDefault(reader, "estimator.ikdtree_match.gate_mode", ikdtree_gate_mode);
        readYamlOrDefault(reader, "estimator.ikdtree_match.score_min", ikdtree_score_min);
        readYamlOrDefault(reader, "estimator.ikdtree_match.match_thresh_base", ikdtree_match_thresh_base);
        readYamlOrDefault(reader, "estimator.ikdtree_match.match_thresh_scale", ikdtree_match_thresh_scale);
        ConfigPrinter::printItem("lidar_ieskf.max_iterations", lidar_ieskf_max_iterations);
        ConfigPrinter::printItem("lidar_ieskf.converge_rot_rad", lidar_ieskf_converge_rot_rad);
        ConfigPrinter::printItem("lidar_ieskf.converge_pos_m", lidar_ieskf_converge_pos_m);
        ConfigPrinter::printItem("lidar_ieskf.rematch_rot_rad", lidar_ieskf_rematch_rot_rad);
        ConfigPrinter::printItem("lidar_ieskf.rematch_pos_m", lidar_ieskf_rematch_pos_m);
        ConfigPrinter::printItemBool("online_gravity_estimation_enable", online_gravity_estimation_enable);
        ConfigPrinter::printItem("ikdtree_match.knn", ikdtree_match_knn);
        ConfigPrinter::printItem("ikdtree_match.plane_fit_rmse_threshold", ikdtree_plane_fit_rmse_threshold);
        ConfigPrinter::printItem("ikdtree_match.neighbor_max_dist2", ikdtree_neighbor_max_dist2);
        ConfigPrinter::printItem("ikdtree_match.gate_mode", ikdtree_gate_mode);
        ConfigPrinter::printItem("ikdtree_match.score_min", ikdtree_score_min);
        ConfigPrinter::printItem("ikdtree_match.match_thresh_base", ikdtree_match_thresh_base);
        ConfigPrinter::printItem("ikdtree_match.match_thresh_scale", ikdtree_match_thresh_scale);

        /***** relocation ******/
        ConfigPrinter::printSubHeader("Relocation");
        if (reader.read("relocation.relocation_enable", relocation_enable)) {
            ConfigPrinter::printItemBool("relocation_enable", relocation_enable);
        } else {
            ConfigPrinter::printError("relocation_enable");
        }
        if (reader.read("relocation.pcd_load_name", pcd_load_name)) {
            ConfigPrinter::printItem("pcd_load_name", pcd_load_name);
        } else {
            ConfigPrinter::printError("pcd_load_name");
        }

        /***** elevator *****/
        ConfigPrinter::printSubHeader("Elevator");
        readYamlOrDefault(reader, "elevator.enable", elevator_enable);
        if (reader.read("elevator.self_exit_detector", self_exit_detector)) {
            ConfigPrinter::printItemBool("self_exit_detector", self_exit_detector);
        } else {
            ConfigPrinter::printError("self_exit_detector");
        }
        readYamlOrDefault(reader, "elevator.door_detector.enable", elevator_door_detector_enable);
        readYamlOrDefault(reader, "elevator.door_detector.dist_threshold", elevator_door_dist_threshold);
        readYamlOrDefault(reader, "elevator.door_detector.time_threshold", elevator_door_time_threshold);
        readYamlOrDefault(reader, "elevator.door_detector.filter_percent", elevator_door_filter_percent);
        readYamlOrDefault(reader, "elevator.door_detector.cooldown_time", elevator_door_cooldown_time);
        readYamlOrDefault(reader, "elevator.strong_prior_enable", elevator_strong_prior_enable);
        readYamlOrDefault(reader, "elevator.strong_prior_acc_var", elevator_strong_prior_acc_var);
        readYamlOrDefault(reader, "elevator.zupt.enable", elevator_zupt_enable);
        readYamlOrDefault(reader, "elevator.zupt.waiting.enable", elevator_waiting_zupt_enable);
        readYamlOrDefault(reader, "elevator.zupt.waiting.period_s", elevator_waiting_zupt_period_s);
        readYamlOrDefault(reader, "elevator.zupt.waiting.vz_abs_thresh", elevator_waiting_zupt_vz_abs_thresh);
        readYamlOrDefault(reader, "elevator.zupt.waiting.az_abs_thresh", elevator_waiting_zupt_az_abs_thresh);
        readYamlOrDefault(reader, "elevator.exit_icp_z.enable", elevator_exit_icp_z_enable);
        readYamlOrDefault(reader, "elevator.exit_icp_z.max_correction_m", elevator_exit_icp_z_max_correction_m);
        readYamlOrDefault(reader, "elevator.exit_icp_z.max_iterations", elevator_exit_icp_z_max_iterations);
        readYamlOrDefault(reader, "elevator.exit_icp_z.min_effective_points", elevator_exit_icp_z_min_effective_points);
        readYamlOrDefault(reader, "elevator.exit_icp_z.min_information_ratio", elevator_exit_icp_z_min_information_ratio);
        readYamlOrDefault(reader, "elevator.exit_icp_z.max_neighbor_distance_m", elevator_exit_icp_z_max_neighbor_distance_m);
        readYamlOrDefault(reader, "elevator.exit_icp_z.min_abs_normal_z", elevator_exit_icp_z_min_abs_normal_z);
        readYamlOrDefault(reader, "elevator.exit_icp_z.max_residual_m", elevator_exit_icp_z_max_residual_m);
        readYamlOrDefault(reader, "elevator.exit_icp_z.converge_m", elevator_exit_icp_z_converge_m);
        readYamlOrDefault(reader, "elevator.rebuild_map_on_exit.enable", elevator_rebuild_map_on_exit_enable);
        readYamlOrDefault(reader, "elevator.rebuild_map_on_exit.frames", elevator_rebuild_map_on_exit_frames);
        readYamlOrDefault(reader, "elevator.rebuild_map_on_exit.min_points", elevator_rebuild_map_on_exit_min_points);
        if (elevator_door_filter_percent < 0.0) elevator_door_filter_percent = 0.0;
        if (elevator_door_filter_percent > 0.5) elevator_door_filter_percent = 0.5;
        if (elevator_door_cooldown_time < 0.0) elevator_door_cooldown_time = 0.0;
        if (elevator_strong_prior_acc_var <= 0.0) elevator_strong_prior_acc_var = 1e-3;
        if (elevator_waiting_zupt_period_s < 0.0) elevator_waiting_zupt_period_s = 0.0;
        if (elevator_waiting_zupt_vz_abs_thresh < 0.0) elevator_waiting_zupt_vz_abs_thresh = 0.0;
        if (elevator_waiting_zupt_az_abs_thresh < 0.0) elevator_waiting_zupt_az_abs_thresh = 0.0;
        if (elevator_exit_icp_z_max_correction_m <= 0.0) elevator_exit_icp_z_max_correction_m = 0.3;
        if (elevator_exit_icp_z_max_iterations < 1) elevator_exit_icp_z_max_iterations = 1;
        if (elevator_exit_icp_z_min_effective_points < 1) elevator_exit_icp_z_min_effective_points = 1;
        elevator_exit_icp_z_min_information_ratio = std::clamp(elevator_exit_icp_z_min_information_ratio, 0.0, 1.0);
        if (elevator_exit_icp_z_max_neighbor_distance_m <= 0.0) elevator_exit_icp_z_max_neighbor_distance_m = 0.5;
        elevator_exit_icp_z_min_abs_normal_z = std::clamp(elevator_exit_icp_z_min_abs_normal_z, 0.0, 1.0);
        if (elevator_exit_icp_z_max_residual_m <= 0.0) elevator_exit_icp_z_max_residual_m = 0.3;
        if (elevator_exit_icp_z_converge_m <= 0.0) elevator_exit_icp_z_converge_m = 1e-3;
        if (elevator_rebuild_map_on_exit_frames < 1) elevator_rebuild_map_on_exit_frames = 1;
        if (elevator_rebuild_map_on_exit_min_points < 1) elevator_rebuild_map_on_exit_min_points = 1;
        ConfigPrinter::printItemBool("enable", elevator_enable);
        ConfigPrinter::printItemBool("door_detector.enable", elevator_door_detector_enable);
        ConfigPrinter::printItem("door_detector.dist_threshold", elevator_door_dist_threshold);
        ConfigPrinter::printItem("door_detector.time_threshold", elevator_door_time_threshold);
        ConfigPrinter::printItem("door_detector.filter_percent", elevator_door_filter_percent);
        ConfigPrinter::printItem("door_detector.cooldown_time", elevator_door_cooldown_time);
        ConfigPrinter::printItemBool("strong_prior_enable", elevator_strong_prior_enable);
        ConfigPrinter::printItem("strong_prior_acc_var", elevator_strong_prior_acc_var);
        ConfigPrinter::printItemBool("zupt.enable", elevator_zupt_enable);
        ConfigPrinter::printItemBool("zupt.waiting.enable", elevator_waiting_zupt_enable);
        ConfigPrinter::printItem("zupt.waiting.period_s", elevator_waiting_zupt_period_s);
        ConfigPrinter::printItem("zupt.waiting.vz_abs_thresh", elevator_waiting_zupt_vz_abs_thresh);
        ConfigPrinter::printItem("zupt.waiting.az_abs_thresh", elevator_waiting_zupt_az_abs_thresh);
        ConfigPrinter::printItemBool("exit_icp_z.enable", elevator_exit_icp_z_enable);
        ConfigPrinter::printItem("exit_icp_z.max_correction_m", elevator_exit_icp_z_max_correction_m);
        ConfigPrinter::printItem("exit_icp_z.max_iterations", elevator_exit_icp_z_max_iterations);
        ConfigPrinter::printItem("exit_icp_z.min_effective_points", elevator_exit_icp_z_min_effective_points);
        ConfigPrinter::printItem("exit_icp_z.min_information_ratio", elevator_exit_icp_z_min_information_ratio);
        ConfigPrinter::printItem("exit_icp_z.max_neighbor_distance_m", elevator_exit_icp_z_max_neighbor_distance_m);
        ConfigPrinter::printItem("exit_icp_z.min_abs_normal_z", elevator_exit_icp_z_min_abs_normal_z);
        ConfigPrinter::printItem("exit_icp_z.max_residual_m", elevator_exit_icp_z_max_residual_m);
        ConfigPrinter::printItem("exit_icp_z.converge_m", elevator_exit_icp_z_converge_m);
        ConfigPrinter::printItemBool("rebuild_map_on_exit.enable", elevator_rebuild_map_on_exit_enable);
        ConfigPrinter::printItem("rebuild_map_on_exit.frames", elevator_rebuild_map_on_exit_frames);
        ConfigPrinter::printItem("rebuild_map_on_exit.min_points", elevator_rebuild_map_on_exit_min_points);

        /***** wheel ******/
        ConfigPrinter::printSubHeader("Wheel Odometry");
        if (reader.read("wheel.wheel_enable", wheel_enable)) {
            ConfigPrinter::printItemBool("wheel_enable", wheel_enable);
        } else {
            ConfigPrinter::printError("wheel_enable");
        }

        /***** log ******/
        ConfigPrinter::printSubHeader("Log Settings");
        readYamlOrDefault(reader, "log.enabled", logger_config.enabled);
        readYamlOrDefault(reader, "log.session_dir", logger_config.session_dir);
        readYamlOrDefault(reader, "log.flush_interval_ms", logger_config.flush_interval_ms);
        readYamlOrDefault(reader, "log.console.enabled", logger_config.console.enabled);
        readYamlOrDefault(reader, "log.console.info", logger_config.console.info);
        readYamlOrDefault(reader, "log.console.warn", logger_config.console.warn);
        readYamlOrDefault(reader, "log.console.error", logger_config.console.error);
        readYamlOrDefault(reader, "log.console.debug", logger_config.console.debug);
        readYamlOrDefault(reader, "log.console.fsm", logger_config.console.fsm);
        readYamlOrDefault(reader, "log.console.elevator", logger_config.console.elevator);
        readYamlOrDefault(reader, "log.console.perf", logger_config.console.perf);
        readYamlOrDefault(reader, "log.file.enabled", logger_config.file.enabled);
        readFileChannelConfig(reader, "log.file.trajectory", logger_config.file.trajectory_post);
        readFileChannelConfig(reader, "log.file.trajectory_prior", logger_config.file.trajectory_prior);
        readFileChannelConfig(reader, "log.file.raw_imu", logger_config.file.raw_imu);
        readFileChannelConfig(reader, "log.file.adaptive_downsample", logger_config.file.adaptive_downsample);
        readFileChannelConfig(reader, "log.file.wheel_data", logger_config.file.wheel_data);
        readFileChannelConfig(reader, "log.file.wheel_residual", logger_config.file.wheel_residual);
        readFileChannelConfig(reader, "log.file.elevator_debug", logger_config.file.elevator_debug);
        readFileChannelConfig(reader, "log.file.elevator_zupt", logger_config.file.elevator_zupt);
        readFileChannelConfig(reader, "log.file.publish_time", logger_config.file.publish_time);
        readFileChannelConfig(reader, "log.file.ikdtree_snapshot", logger_config.file.ikdtree_snapshot);

        runtimeLogger().configure(logger_config);

        log_save_enable = logger_config.enabled && logger_config.file.enabled;
        wheel_data_log_enable = logger_config.file.wheel_data.enabled;
        trajectory_log_enable = logger_config.file.trajectory_post.enabled;
        time_log_enable = logger_config.console.perf;
        pub_time_log_enable = logger_config.file.publish_time.enabled;
        res_wheel_log_enable = logger_config.file.wheel_residual.enabled;

        ConfigPrinter::printItemBool("log.enabled", logger_config.enabled);
        ConfigPrinter::printItem("log.session_dir", logger_config.session_dir);
        ConfigPrinter::printItem("log.flush_interval_ms", logger_config.flush_interval_ms);
        ConfigPrinter::printItemBool("log.console.fsm", logger_config.console.fsm);
        ConfigPrinter::printItemBool("log.console.elevator", logger_config.console.elevator);
        ConfigPrinter::printItemBool("log.console.perf", logger_config.console.perf);
        ConfigPrinter::printItemBool("log.file.trajectory", logger_config.file.trajectory_post.enabled);
        ConfigPrinter::printItemBool("log.file.trajectory_prior", logger_config.file.trajectory_prior.enabled);
        ConfigPrinter::printItemBool("log.file.raw_imu", logger_config.file.raw_imu.enabled);
        ConfigPrinter::printItemBool("log.file.adaptive_downsample", logger_config.file.adaptive_downsample.enabled);
        ConfigPrinter::printItemBool("log.file.wheel_data", logger_config.file.wheel_data.enabled);
        ConfigPrinter::printItemBool("log.file.wheel_residual", logger_config.file.wheel_residual.enabled);
        ConfigPrinter::printItemBool("log.file.elevator_debug", logger_config.file.elevator_debug.enabled);
        ConfigPrinter::printItemBool("log.file.elevator_zupt", logger_config.file.elevator_zupt.enabled);
        ConfigPrinter::printItemBool("log.file.publish_time", logger_config.file.publish_time.enabled);
        ConfigPrinter::printItemBool("log.file.ikdtree_snapshot", logger_config.file.ikdtree_snapshot.enabled);

        /***** visualize ******/
        ConfigPrinter::printSubHeader("Visualization");
        if (reader.read("visualize.CovMatrix_vis", CovMatrix_vis)) {
            ConfigPrinter::printItemBool("CovMatrix_vis", CovMatrix_vis);
        } else {
            ConfigPrinter::printError("CovMatrix_vis");
        }
        if (reader.read("visualize.EleState_vis", EleState_vis)) {
            ConfigPrinter::printItemBool("EleState_vis", EleState_vis);
        } else {
            ConfigPrinter::printError("EleState_vis");
        }

        ConfigPrinter::printFooter();
    }
    else {
        std::cerr << ANSI_COLOR_RED << ANSI_COLOR_BOLD << "✗ Failed to load YAML file" << ANSI_COLOR_RESET << std::endl;
    }
}

string sensor_config_yaml;
string runtime_config_yaml;
string logging_config_yaml;

void load_root_yaml( string file_name ) {
    YamlReader reader(std::string(PACKAGE_ROOT_DIR) +  "/yaml/" + file_name);
    if (reader.load()) {
        const bool has_split_config =
            reader.readOptional("sensor_config", sensor_config_yaml) &&
            reader.readOptional("runtime_config", runtime_config_yaml) &&
            reader.readOptional("logging_config", logging_config_yaml);

        if (!has_split_config || sensor_config_yaml.empty() ||
            runtime_config_yaml.empty() || logging_config_yaml.empty()) {
            std::cerr << ANSI_COLOR_RED
                      << "✗ root config requires sensor_config, runtime_config and logging_config"
                      << ANSI_COLOR_RESET << std::endl;
            return;
        }

        const std::vector<std::string> config_files = {
            sensor_config_yaml,
            runtime_config_yaml,
            logging_config_yaml,
        };
        for (const auto& config_file : config_files) {
            std::cout << ANSI_COLOR_CYAN << "📦 Loading config from: "
                      << ANSI_COLOR_GREEN << ANSI_COLOR_BOLD << config_file
                      << ANSI_COLOR_RESET << std::endl;
        }
        load_lio_yaml(config_files);
    }
    else {
        std::cerr << ANSI_COLOR_RED << ANSI_COLOR_BOLD << "✗ Failed to load ROOT YAML file" << ANSI_COLOR_RESET << std::endl;
    }
}
