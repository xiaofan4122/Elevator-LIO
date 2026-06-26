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

#include "estimator/IMUProcess.h"
IMUProcess imu_process;

ElevatorSelfExit ele_self_exit(imu_process.elev_process, p_eskf_estimator);

/******************************** [M3] LIONode construction ********************************/

LIONode::LIONode() : nh_("~"), lidar_pipeline_(nh_, shared_bufs, p_eskf_estimator, imu_process, ele_self_exit, ikd_map) {
    LOG_INFO(Core, "LIONode is running!");
    /****** Topic Publish ******/
    clouds_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(clouds_lidar_topic_name, 10);
    clouds_lidar_effect_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(clouds_lidar_effect_topic_name, 10);
    clouds_lidar_reject_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(clouds_lidar_reject_topic_name, 10);
    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(global_map_topic_name, 10);
    ikdtree_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ikdtree_topic_name, 10);
    /****** Transform Publish ******/
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
    /****** Odometry Publish ******/
    odom_imu_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_imu_topic_name, 10);
    odom_body_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_vehicle_topic_name, 10);
    odom_path_pub_ = nh_.advertise<nav_msgs::Path>("path_topic_name", 50);
    /****** Topic Subscriber ******/
    pointcloud_sub_ = lidar_type == LidarType::Livox ?
        nh_.subscribe(lidar_topic_name, 5000, pcl_cbk_custom):
        nh_.subscribe(lidar_topic_name, 5000, pcl_cbk_pc2);
    imu_sub_ = nh_.subscribe(imu_topic_name, 100000, imu_cbk);
    wheel_sub_ = nh_.subscribe(wheel_topic_name, 5000, wheel_cbk);
    /****** Timer ******/
    timer_2000HZ_ = nh_.createTimer(ros::Rate(2000), &LIONode::timer_2000HZ_callback, this);
    timer_500HZ_ = nh_.createTimer(ros::Rate(500), &LIONode::timer_500HZ_callback, this);
    timer_10HZ_ = nh_.createTimer(ros::Rate(10), &LIONode::timer_10HZ_callback, this);
    timer_1HZ_ = nh_.createTimer(ros::Rate(1), &LIONode::timer_1HZ_callback, this);
    /****** Topic Subscriber (替代 Service) ******/
    elevator_flag_sub_ = nh_.subscribe(elevator_flag_topic_name, 1, &LIONode::elevatorFlagCallback, this);
    /****** Elevator Publish ******/
    ele_state_pub_ = nh_.advertise<std_msgs::Bool>("/LIO/in_elevator", 1, true);
}



/******************************** [M4] Timer callbacks ********************************/

/******** [M4.1] Low-rate publish timers ********/
void LIONode::timer_1HZ_callback(const ros::TimerEvent& ) {
    /**** 发布静态tf ****/
    publishStaticTransform();
    /*** 发布全局地图 ***/
    publishGlobalMap();
    /*** 发布 ikdtree ***/
    publishIKDTree();
}

void LIONode::timer_10HZ_callback(const ros::TimerEvent& ) {
    /*** 留空备用 ***/
}


/******** [M4.2] LiDAR pipeline timer ********/
void LIONode::timer_500HZ_callback(const ros::TimerEvent& ) {
    auto result = lidar_pipeline_.process();

    if (result.processed) {
        const auto& post_state = result.post_state;
        publish_imu_odometry(post_state);
        publish_body_odometry(post_state);
        if (trajectory_log_enable & log_save_enable) {
            save_odometry(post_state, lidar_end_time);
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
void LIONode::timer_2000HZ_callback(const ros::TimerEvent& )
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
                imu_process.set_init_state(state,  msg->header.stamp.toSec());
                FSM::dispatch(FSM::Event::InitEnd);
            }
            break;
    }
    case FSM::State::IMUProcess: {
            if (! shared_bufs->isEmptySafe(shared_bufs->imu_buf)) {
                ImuMsgConst msg = shared_bufs->getFrontSafe(shared_bufs->imu_buf);
                imu_process.integration(msg, mean_acc);
                lastest_imu_time = msg->header.stamp.toSec();
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
                Eigen::Vector3d raw_acc = {msg->linear_acceleration.x , msg->linear_acceleration.y , msg->linear_acceleration.z};
                Eigen::Vector3d acc_b = G_SCALE_UP ? raw_acc * G_m_s2 : raw_acc;
                Eigen::Vector3d acc_w = state.q * (acc_b - state.ba) + state.g;

                ElevatorSelfExit::ImuFrame frame;
                frame.timestamp = lastest_imu_time;
                frame.acc_z = acc_w(2);
                frame.vel_z = state.vz;
                frame.in_elevator = state.in_elevator;

                auto ele_result = ele_self_exit.process(frame, state);

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
                if (high_frequency_odom && lidar_frequency > 0.0 && lidar_end_time > 0.0) {
                    const double next_lidar_time = lidar_end_time + 1.0 / lidar_frequency;
                    constexpr double kLidarArrivalGuardSec = 0.005;
                    if (lastest_imu_time < next_lidar_time - kLidarArrivalGuardSec) {
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
volatile std::sig_atomic_t g_shutdown_signal = 0;

bool isSafeRelativeConfigPath(const std::filesystem::path &path) {
    if (path.empty() || path.is_absolute()) return false;
    for (const auto &component : path) {
        if (component == "..") return false;
    }
    return true;
}

void archiveActiveYamlConfigs(const std::string &package_root,
                              const std::string &root_config_file,
                              const std::string &session_dir) {
    const std::filesystem::path yaml_root = std::filesystem::path(package_root) / "yaml";
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

void SigHandle(int sig)
{
    g_shutdown_signal = sig;
    ros::shutdown();
}

/******************************** [M7] Program entry and shutdown persistence ********************************/

void init_ikd_map();
void build_kdtree_from_file();
void shutdown_save_maps();

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "lio_node");
    signal(SIGINT, SigHandle); // 关联SIGINT信号（通常是ctrl c）与自定义函数句柄 SigHandle
    signal(SIGTERM, SigHandle); // 关联SIGTERM信号（通常是ctrl c）与自定义函数句柄 SigHandle

    // 初始化
    ros::NodeHandle nh("~");
    ROS_INFO("Current NodeHandler namespace: %s", nh.getNamespace().c_str());

    //读取配置文件路径
    std::string config_path;
    nh.param("config_path", config_path, string("root_config.yaml"));
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
    archiveActiveYamlConfigs(string(PACKAGE_ROOT_DIR), config_path, current_tempdir_log);

    // 创建 LIONode 类型的节点
    //构造函数主要负责初始化 ROS 节点的各种发布器（Publisher）、订阅器（Subscriber）以及定时器（Timer）
    LIONode node;

    // 启动并处理回调
    //ros::spin() 是 ROS 的主循环函数。  它会一直运行，直到节点被关闭（比如按下 Ctrl+C）。
    ros::spin();

    /******** 下面的部分在按下 ctrl + c 后执行 *******/
    convert_traj_csv_to_tum();
    shutdown_save_maps();
    return 0;
}
