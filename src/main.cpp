//
// Created by xiaofan on 24-11-25.
//

#include "support/LIONode.h"
#include "support/common_lib.h"
#include "node/LidarPipeline.h"
#include "elevator/ElevatorSelfExit.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>

/******************************** [M0] File guide ********************************
 * [M1] FSM definition
 * [M2] Global runtime state
 * [M3] LIONode construction
 * [M4] Timer callbacks
 *      [M4.1] Low-rate publish timers
 *      [M4.2] LiDAR pipeline timer (delegates to LidarPipeline)
 * [M5] IMU pipeline timer
 * [M6] Signal handler
 * [M7] Program entry and shutdown persistence
 *
 * Other modules (src/node/):
 *   publish.cpp  — ROS publishers (odometry, clouds, TF, map)
 *   map.cpp      — ikd-tree management, map init, trajectory conversion
 *   save.cpp     — trajectory and point-cloud persistence
 *   callbacks.cpp— ROS subscription callbacks
 *   LidarPipeline.cpp — LiDAR processing pipeline
 *******************************************************************************/

/******************************** [M1] FSM definition ********************************/
namespace FSM{
    // 用 pair<State,Event> 做键，建立转换表
    const std::map<Key, Transition> kTransitionTable = {
        // 请按此格式，为每一条“(当前状态, 事件) -> (下一个状态, 动作回调)”添加一行
        {{State::Waitting, Event::IMUTriger}, { State::Initializing, [](){  } }},
        {{State::Initializing, Event::InitEnd}, { State::IMUProcess, [](){  } }},
        {{State::IMUProcess, Event::LidarTriger}, { State::LidarProcess, [](){  } }},
        {{State::LidarProcess, Event::LidarEnd}, { State::IMUProcess, [](){  } }},
        {{State::IMUProcess, Event::ImgTriger}, { State::ImgProcess, [](){  } }},
        {{State::ImgProcess, Event::ImgEnd}, { State::IMUProcess, [](){  } }},
    };
    // Dispatch 函数
    State current_state = State::Waitting;  // 初始状态
    void dispatch(Event e) {
        auto it = kTransitionTable.find({ current_state, e });
        if (it != kTransitionTable.end()) {
            State next = it->second.next_state;
            LOG_INFO(FSM, to_string(current_state) << " + " << to_string(e) << " -> " << to_string(next));
            // 执行动作
            it->second.action();
            // 切换状态
            current_state = it->second.next_state;
        } else {
            LOG_ERROR(FSM, "illegal transition: state=" << static_cast<int>(current_state)
                                                         << " event=" << static_cast<int>(e));
        }
    }
}


/******************************** [M2] Global runtime state ********************************/
auto shared_bufs = std::make_shared<SharedBuffers>();
std::shared_ptr<TopicProcess> p_topic_process(new TopicProcess(shared_bufs));
std::shared_ptr<EskfEstimator> p_eskf_estimator(new EskfEstimator());
KD_TREE<PointType> ikdtree;
std::shared_ptr<IkdMap<PointType>> ikd_map;
vector<BoxPointType> cub_needrm;
PointCloudXYZI::Ptr down_effect_cloud_lidar(new PointCloudXYZI);
PointCloudXYZI::Ptr down_reject_cloud_lidar(new PointCloudXYZI);
PointCloudXYZI::Ptr map_world(new PointCloudXYZI);
vector<string> incremental_paths;

double lidar_end_time = 0;
double lastest_imu_time = 0;
double min_filter_size;

Eigen::Vector3d mean_acc;
Eigen::Vector3d mean_gyr;
bool G_SCALE_UP = false;

bool EXIT_FROM_ELEVATOR = false;

bool ELEVATOR_TRIGGER = false;

string current_tempdir_log;
string package_share_path;

#include "estimator/IMUProcess.h"
IMUProcess imu_process;

ElevatorSelfExit ele_self_exit(imu_process.elev_process, p_eskf_estimator);

/******************************** [M3] LIONode construction ********************************/

LIONode::LIONode(std::shared_ptr<lio_ros::Node> node)
    : node_(std::move(node)),
      lidar_pipeline_(*node_, shared_bufs, p_eskf_estimator, imu_process, ele_self_exit, ikd_map) {
    LOG_INFO(Core, "LIONode is running!");
    /****** Topic Publish ******/
    clouds_lidar_pub_ = node_->advertise<lio_ros::PointCloud2>(clouds_lidar_topic_name, 10);
    clouds_lidar_effect_pub_ = node_->advertise<lio_ros::PointCloud2>(clouds_lidar_effect_topic_name, 10);
    clouds_lidar_reject_pub_ = node_->advertise<lio_ros::PointCloud2>(clouds_lidar_reject_topic_name, 10);
    global_map_pub_ = node_->advertise<lio_ros::PointCloud2>(global_map_topic_name, 10);
    ikdtree_pub_ = node_->advertise<lio_ros::PointCloud2>(ikdtree_topic_name, 10);
    /****** Transform Publish ******/
#if LIO_ROS_VERSION == 1
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
#else
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_->raw());
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*node_->raw());
#endif
    /****** Odometry Publish ******/
    odom_imu_pub_ = node_->advertise<lio_ros::Odometry>(odom_imu_topic_name, 10);
    odom_body_pub_ = node_->advertise<lio_ros::Odometry>(odom_vehicle_topic_name, 10);
    odom_path_pub_ = node_->advertise<lio_ros::Path>("path_topic_name", 50);
    /****** Topic Subscriber ******/
    pointcloud_sub_ = lidar_type == LidarType::Livox ?
        node_->subscribe<lio_ros::CustomMsg>(lidar_topic_name, 5000, pcl_cbk_custom, true):
        node_->subscribe<lio_ros::PointCloud2>(lidar_topic_name, 5000, pcl_cbk_pc2, true);
    imu_sub_ = node_->subscribe<lio_ros::Imu>(imu_topic_name, 100000, imu_cbk, true);
    wheel_sub_ = node_->subscribe<lio_ros::WheelInfo>(wheel_topic_name, 5000, wheel_cbk, true);
    /****** Timer ******/
    timer_2000HZ_ = node_->create_timer(2000.0, [this]() { timer_2000HZ_callback(); });
    timer_500HZ_ = node_->create_timer(500.0, [this]() { timer_500HZ_callback(); });
    timer_10HZ_ = node_->create_timer(10.0, [this]() { timer_10HZ_callback(); });
    timer_1HZ_ = node_->create_timer(1.0, [this]() { timer_1HZ_callback(); });
    /****** Topic Subscriber (替代 Service) ******/
    elevator_flag_sub_ = node_->subscribe<lio_ros::Bool>(
        elevator_flag_topic_name, 1,
        [this](const lio_ros::BoolConstPtr& msg) { elevatorFlagCallback(msg); });
    /****** Elevator Publish ******/
    ele_state_pub_ = node_->advertise<lio_ros::Bool>("/LIO/in_elevator", 1, true);
}



/******************************** [M4] Timer callbacks ********************************/

/******** [M4.1] Low-rate publish timers ********/
void LIONode::timer_1HZ_callback() {
    /**** 发布静态tf ****/
    publishStaticTransform();
    /*** 发布全局地图 ***/
    publishGlobalMap();
    /*** 发布 ikdtree ***/
    publishIKDTree();
}

void LIONode::timer_10HZ_callback() {
    /*** 留空备用 ***/
}


/******** [M4.2] LiDAR pipeline timer ********/
void LIONode::timer_500HZ_callback() {
    auto result = lidar_pipeline_.process();

    if (result.processed) {
        const auto& post_state = result.post_state;
        publish_imu_odometry(post_state, post_state.time);
        publish_body_odometry(post_state, post_state.time);
        if (trajectory_log_enable & log_save_enable) {
            save_odometry(post_state, post_state.time);
        }
        publish_Dedistort_clouds_lidar(result.dedistorted_cloud);
        publish_Effect_clouds_lidar(down_effect_cloud_lidar);
        publish_Rejected_clouds_lidar(down_reject_cloud_lidar);
        save_singel_clouds_world(result.dedistorted_cloud);
    }

    /******** Wheel update ********/
    if (wheel_enable) {
        std::deque<WheelData> wheel_deq = p_topic_process -> get_wheel_buffer();
        if (!wheel_deq.empty()) {
            p_eskf_estimator -> UpdateByWheel(wheel_deq);
        }else {
            std::cout << "[Warning] [timer_500HZ_callback] wheel_deq is empty" << std::endl;
        }
    }
}

/******************************** [M5] IMU pipeline timer ********************************/
void LIONode::timer_2000HZ_callback()
{
    switch(FSM::current_state) {
    case FSM::State::Initializing: {
            if (shared_bufs->isEmptySafe(shared_bufs->imu_buf)) break;
            State state;
            ImuMsgConst msg = shared_bufs->getFrontSafe(shared_bufs->imu_buf);
            bool converge = imu_process.init(msg, 200 , mean_acc , mean_gyr, state);
            shared_bufs -> popFrontSafe(shared_bufs->imu_buf);
            if(converge) {
                cout << "mean_acc: \n" << mean_acc << endl;
                p_eskf_estimator -> mean_acc_ = mean_acc;
                p_eskf_estimator -> mean_gyr_ = mean_gyr;
                p_eskf_estimator -> set_cur_state(state);
                imu_process.set_init_state(state, get_time_sec(msg->header.stamp));
                FSM::dispatch(FSM::Event::InitEnd);
            }
            break;
    }
    case FSM::State::IMUProcess: {
            if (! shared_bufs->isEmptySafe(shared_bufs->imu_buf)) {
                ImuMsgConst msg = shared_bufs->getFrontSafe(shared_bufs->imu_buf);
                imu_process.integration(msg, mean_acc);
                lastest_imu_time = get_time_sec(msg->header.stamp);
                {
                    if (log_save_enable) {
                        static uint64_t raw_imu_seq = 0;
                        std::ostringstream imu_oss;
                        imu_oss << std::fixed << std::setprecision(9)
                                << lastest_imu_time << ","
                                << msg->linear_acceleration.x << ","
                                << msg->linear_acceleration.y << ","
                                << msg->linear_acceleration.z << ","
                                << msg->angular_velocity.x << ","
                                << msg->angular_velocity.y << ","
                                << msg->angular_velocity.z;
                        runtimeLogger().writeLine(
                                LogFileChannel::RawImu,
                                "time,ax,ay,az,gyrox,gyroy,gyroz",
                                imu_oss.str(),
                                ++raw_imu_seq,
                                lastest_imu_time);
                    }
                }
                State state;
                imu_process.get_state_at_t(state, lastest_imu_time);
                IMUProcess::ProcessedImuSample processed_imu;
                const bool have_processed_imu = imu_process.getLatestProcessedSample(processed_imu);
                Eigen::Vector3d raw_acc = {msg->linear_acceleration.x , msg->linear_acceleration.y , msg->linear_acceleration.z};
                Eigen::Vector3d acc_b = have_processed_imu
                                      ? processed_imu.acc
                                      : (G_SCALE_UP ? raw_acc * G_m_s2 : raw_acc);
                Eigen::Vector3d acc_w = state.q * (acc_b - state.ba) + state.g;

                ElevatorSelfExit::ImuFrame frame;
                frame.timestamp = lastest_imu_time;
                frame.acc_z = acc_w(2);
                frame.vel_z = state.vz;
                frame.in_elevator = state.in_elevator;

                ElevatorSelfExit::ProcessResult ele_result;
                // 有限的超量程值仍是当前实验的有效量测；只有 NaN/Inf 轴
                // （mask 高三位）不参与停靠状态机。
                constexpr std::uint8_t kInvalidAxisMask = 0x38u;
                const bool acceleration_is_trusted = !have_processed_imu ||
                        (processed_imu.acc_saturation_mask & kInvalidAxisMask) == 0;
                if (acceleration_is_trusted) {
                    ele_result = ele_self_exit.process(frame, state);
                }

                if (ele_result.apply_waiting_zupt) {
                    imu_process.elev_process.applyZeroVelocityUpdateZ(state);
                    imu_process.set_state_at_t(state, lastest_imu_time);
                    p_eskf_estimator->set_cur_state(state);
                    LOG_INFO(Elevator, "apply waiting ZUPT"
                                       << " vz=" << state.vz
                                       << " az=" << state.az);
                }

                if (log_save_enable) {
                    const auto& dbg = ele_self_exit.debugInfo();
                    static uint64_t elevator_debug_seq = 0;
                    std::ostringstream elevator_oss;
                    elevator_oss << std::fixed << std::setprecision(6)
                                 << lastest_imu_time << ","
                                 << dbg.acc_z << ","
                                 << dbg.vel_z << ","
                                 << dbg.std_dev << ","
                                 << dbg.state << ","
                                 << (ELEVATOR_TRIGGER ? 1 : 0) << ","
                                 << (state.in_elevator ? 1 : 0) << ","
                                 << imu_process.elev_process.doorLastFilteredMaxDistance() << ","
                                 << (imu_process.elev_process.doorLowDistanceActive() ? 1 : 0) << ","
                                 << imu_process.elev_process.doorCooldownRemaining(lastest_imu_time);

                    const std::string log_header =
                        "timestamp,acc_z,vel_z,std_dev,state,trigger,in_ele,door_dist,door_active,door_cooldown";

                    runtimeLogger().writeLine(
                            LogFileChannel::ElevatorDebug,
                            log_header,
                            elevator_oss.str(),
                            ++elevator_debug_seq,
                            lastest_imu_time);
                }
                // 高频里程计不能无条件把最新 IMU 预积分位姿直接发布出去：
                // LiDAR 更新可能发生在当前最新 IMU 时间之前，更新后需要把 IMU 状态回退到
                // LiDAR 修正时刻并重新积分到后续 IMU 时刻。如果提前发布了这些后续 IMU 位姿，
                // 后续 LiDAR 帧发布的校正位姿时间戳可能早于已发布时间戳，造成里程计时间回退。
                // 因此这里只在预计下一帧 LiDAR 到达前的安全窗口内发布 IMU 先验位姿。
                const double committed_lidar_time = lidar_pipeline_.lastCommittedTime();
                if (high_frequency_odom && lio_tracking_valid.load() && lidar_frequency > 0.0 && committed_lidar_time > 0.0) {
                    const double next_lidar_time = committed_lidar_time + 1.0 / lidar_frequency;
                    constexpr double kLidarArrivalGuardSec = 0.005;
                    double publish_until = next_lidar_time - kLidarArrivalGuardSec;
                    const auto pending_lidar = shared_bufs->getFrontLidarFrameSafe();
                    if (pending_lidar) {
                        publish_until = std::min(publish_until, pending_lidar->end_time);
                    }
                    if (lastest_imu_time > committed_lidar_time && lastest_imu_time < publish_until) {
                        publish_imu_odometry(state, lastest_imu_time);
                        publish_body_odometry(state, lastest_imu_time);
                        if (trajectory_log_enable & log_save_enable) {
                            save_odometry(state, lastest_imu_time);
                        }
                    }
                }
                {
                    // 在这里保存IMU积分先验位姿
                    save_odometry(state, lastest_imu_time, runtimeLogger().resolvePath(LogFileChannel::TrajectoryPrior));
                }
                shared_bufs -> popFrontSafe(shared_bufs->imu_buf);
            }
            const auto frame_opt = shared_bufs->getFrontLidarFrameSafe();
            if (frame_opt.has_value()) {
                lidar_end_time = frame_opt->end_time;
                if (lastest_imu_time >= lidar_end_time) {
                    FSM::dispatch(FSM::Event::LidarTriger);
                }
            }
            break;
    }
    }
}

/******************************** [M6] Signal handler ********************************/

// 按下ctrl c后的处理函数
namespace {
bool isSafeRelativeConfigPath(const std::filesystem::path &path) {
    if (path.empty() || path.is_absolute()) return false;
    for (const auto &component : path) {
        if (component == "..") return false;
    }
    return true;
}

void archiveActiveYamlConfigs(const std::string &package_share,
                              const std::string &root_config_file,
                              const std::string &session_dir) {
    const std::filesystem::path yaml_root = std::filesystem::path(package_share) / "yaml";
    const std::filesystem::path archive_root = std::filesystem::path(session_dir) / "yaml";
    const std::vector<std::string> active_configs = {
        root_config_file,
        sensor_config_yaml,
        runtime_config_yaml,
        logging_config_yaml,
    };

    std::error_code ec;
    for (const auto &config_file : active_configs) {
        const std::filesystem::path relative_path(config_file);
        if (!isSafeRelativeConfigPath(relative_path)) {
            LOG_WARN(Core, "skip unsafe YAML archive path: " << config_file);
            continue;
        }

        const std::filesystem::path source = yaml_root / relative_path;
        const std::filesystem::path destination = archive_root / relative_path;
        ec.clear();
        std::filesystem::create_directories(destination.parent_path(), ec);
        if (ec) {
            LOG_WARN(Core, "failed to create YAML archive directory: "
                           << destination.parent_path().string() << " error=" << ec.message());
            continue;
        }

        ec.clear();
        std::filesystem::copy_file(source, destination,
                                   std::filesystem::copy_options::overwrite_existing, ec);
        if (ec) {
            LOG_WARN(Core, "failed to archive YAML: " << source.string()
                           << " -> " << destination.string() << " error=" << ec.message());
        } else {
            LOG_INFO(Core, "archived YAML: " << destination.string());
        }
    }
}
}

#if LIO_ROS_VERSION == 1
void SigHandle(int sig)
{
    (void)sig;
    lio_ros::shutdown();
}
#endif

/******************************** [M7] Program entry and shutdown persistence ********************************/

void init_ikd_map();
void build_kdtree_from_file();
void shutdown_save_maps();

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
#if LIO_ROS_VERSION == 1
    // ROS1 延续自定义处理器：回调只触发 ros::shutdown()，spin 返回后统一落盘。
    lio_ros::init(argc, argv, "lio_node", false);
    signal(SIGINT, SigHandle);
    signal(SIGTERM, SigHandle);
#else
    // ROS2 的 shutdown 涉及锁，不能从异步 POSIX signal handler 中直接调用。
    // 使用 rclcpp 自带的信号处理线程，spin 返回后仍会执行下方统一落盘逻辑。
    lio_ros::init(argc, argv, "lio_node", true);
#endif

    // 初始化
    auto ros_node = std::make_shared<lio_ros::Node>("lio_node");
    ROS_INFO("Current NodeHandler namespace: %s", ros_node->get_namespace().c_str());

#if LIO_ROS_VERSION == 1
    package_share_path = PACKAGE_ROOT_DIR;
#else
    package_share_path = lio_ros::package_share_directory("lio");
#endif

    //读取配置文件路径
    std::string config_path;
    ros_node->param("config_path", config_path, string("root_config.yaml"));
    ROS_INFO("Config path: %s", config_path.c_str());
    load_root_yaml(config_path);

    // 设置地图的最小分辨率
    if (downsample_adaptive_enabled) {
        // 如果启用变体素降采样，使用最小体素作为地图分辨率
        min_filter_size = std::min(static_cast<double>(filter_size), downsample_min_voxel);
    }
    else {
        // 固定体素降采样，直接使用配置的filter_size
        min_filter_size = filter_size;
    }

    init_ikd_map();

    // 重定位模式下从文件中构建 kdtree
    build_kdtree_from_file();

    // 用来保存一些临时文件
    string tempdir_pcd = string(PACKAGE_ROOT_DIR) + "/PCD/Temp";
    std::filesystem::create_directories(tempdir_pcd);
    current_tempdir_log = runtimeLogger().initializeSession(string(PACKAGE_ROOT_DIR));
    LOG_INFO(Core, "log session dir: " << current_tempdir_log);
    archiveActiveYamlConfigs(package_share_path, config_path, current_tempdir_log);

    // 创建 LIONode 类型的节点
    //构造函数主要负责初始化 ROS 节点的各种发布器（Publisher）、订阅器（Subscriber）以及定时器（Timer）
    LIONode node(ros_node);

    // 启动并处理回调
    //ros::spin() 是 ROS 的主循环函数。  它会一直运行，直到节点被关闭（比如按下 Ctrl+C）。
    lio_ros::spin(*ros_node);

    /******** 下面的部分在按下 ctrl + c 后执行 *******/
    convert_traj_csv_to_tum();
    shutdown_save_maps();
    return 0;
}
