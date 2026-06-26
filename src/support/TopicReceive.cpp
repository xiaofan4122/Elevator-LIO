//
// Created by xiaofan on 25-1-1.
//

/** @file TopicProcess.cpp
 * @brief 包含一个TopicProcess类，用于处理接受到的lidar和imu话题，并完成打包
 * receive_points()函数：将点云数据存入点云缓冲队列(cloud_buffer)
 * receive_imu()函数：将IMU数据存入IMU缓冲队列(imu_buffer)
 * sync_packages()函数：将点云数据和IMU数据从缓存队列中弹出，打包成一帧，存入meas中
 * << 注意meas中只能存放一帧数据，每次成功调用sync_packages()都会覆盖之前的数据 >>
 * @date 2025/1/7
 */

#include "support/TopicProcess.h"
#include <sensor_msgs/PointCloud2.h>  // 添加头文件
#include <sensor_msgs/PointField.h>
#include <pcl_conversions/pcl_conversions.h>  // 用于转换 PointCloud2 <-> PCL 点云
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <algorithm>

#include "support/LIONode.h"
#include "support/type.h"
// std::mutex mtx_buffer;
// std::condition_variable sig_buffer; //条件变量

const double THRE_DIFF_IMU = 10; // 异常值处理阈值

namespace {
constexpr double kOusterTimeNanosecondToMillisecond = 1e-6;
constexpr double kSecondToMillisecond = 1e3;
constexpr double kMicrosecondToMillisecond = 1e-3;

void sortByPointOffsetTime(PointCloudXYZI &cloud) {
    std::sort(cloud.points.begin(), cloud.points.end(),
              [](const PointType &a, const PointType &b) {
                  return a.curvature < b.curvature;
              });
}

double inferPointTimeToMillisecond(double raw_time) {
    if (raw_time <= 0.0) return 0.0;
    if (raw_time < 1.0) return raw_time * kSecondToMillisecond;
    if (raw_time < 1000.0) return raw_time;
    if (raw_time < 1.0e6) return raw_time * kMicrosecondToMillisecond;
    return raw_time * kOusterTimeNanosecondToMillisecond;
}
}

/**
 * @brief:
 * 将点云数据存入点云缓冲队列(cloud_buffer)
 * 在时间戳出现回溯时，清除缓存（播包时可能出现）
 * 在每次接收到 lidar topic 后执行
 * @param msg: 接收到的点云数据
 */
void TopicProcess::receive_points_ntu(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (FSM::current_state == FSM::State::Initializing) return;
    cloud_topic_count_++;
    double timestamp = get_time_sec(msg->header.stamp);

    struct FieldMeta {
        int offset = -1;
        uint8_t datatype = sensor_msgs::PointField::FLOAT32;
    };
    auto find_field = [&](const std::string &name) {
        FieldMeta meta;
        for (const auto &f : msg->fields) {
            if (f.name == name) {
                meta.offset = static_cast<int>(f.offset);
                meta.datatype = f.datatype;
                break;
            }
        }
        return meta;
    };
    auto read_scalar = [](const uint8_t *ptr, uint8_t datatype) -> double {
        switch (datatype) {
            case sensor_msgs::PointField::INT8:   return static_cast<double>(*reinterpret_cast<const int8_t *>(ptr));
            case sensor_msgs::PointField::UINT8:  return static_cast<double>(*reinterpret_cast<const uint8_t *>(ptr));
            case sensor_msgs::PointField::INT16:  return static_cast<double>(*reinterpret_cast<const int16_t *>(ptr));
            case sensor_msgs::PointField::UINT16: return static_cast<double>(*reinterpret_cast<const uint16_t *>(ptr));
            case sensor_msgs::PointField::INT32:  return static_cast<double>(*reinterpret_cast<const int32_t *>(ptr));
            case sensor_msgs::PointField::UINT32: return static_cast<double>(*reinterpret_cast<const uint32_t *>(ptr));
            case sensor_msgs::PointField::FLOAT64:return *reinterpret_cast<const double *>(ptr);
            case sensor_msgs::PointField::FLOAT32:
            default:
                return static_cast<double>(*reinterpret_cast<const float *>(ptr));
        }
    };

    const FieldMeta fx = find_field("x");
    const FieldMeta fy = find_field("y");
    const FieldMeta fz = find_field("z");
    const FieldMeta fint = find_field("intensity");
    const FieldMeta frefl = find_field("reflectivity");
    const FieldMeta ft = find_field("t");
    const FieldMeta ftime = find_field("time");
    const FieldMeta ftimestamp = find_field("timestamp");

    if (fx.offset < 0 || fy.offset < 0 || fz.offset < 0) {
        ROS_WARN_THROTTLE(1.0, "[receive_points_ntu] missing xyz fields, skip frame.");
        return;
    }

    const int plsize = static_cast<int>(msg->width * msg->height);

    if (plsize <= 1) {
        std::cerr << "[receive_points] too few points\n";
        return;
    }

    static double sum_time = 0;
    static int cnt = 0;
    static bool first = true;

    if (first) {
        last_timestamp_lidar_ = timestamp;
        first = false;
    } else {
        double time_diff = timestamp - last_timestamp_lidar_;
        if (sum_time < 5) {
            cnt++;
            sum_time += time_diff;
        } else {
            LOG_DEBUG(Sensor, "[receive_points] lidar frequency=" << cnt / sum_time << " HZ");
            cnt = 0;
            sum_time = 0;
        }
        last_timestamp_lidar_ = timestamp;
    }

    PointCloudXYZI cloud_xyzi;  // 假设 PointCloudXYZI 是你自定义的点类型
    cloud_xyzi.reserve(plsize);

    uint valid_num = 0;
    double max_offset_ms = 0.0;

    const uint8_t *data_ptr = msg->data.data();
    const int point_step = static_cast<int>(msg->point_step);
    for (int i = 0; i < plsize; ++i)  // 注意索引从 0 开始
    {
        const uint8_t *base = data_ptr + i * point_step;
        const float x = static_cast<float>(read_scalar(base + fx.offset, fx.datatype));
        const float y = static_cast<float>(read_scalar(base + fy.offset, fy.datatype));
        const float z = static_cast<float>(read_scalar(base + fz.offset, fz.datatype));

        float intensity = 1.0f;
        if (fint.offset >= 0) {
            intensity = static_cast<float>(read_scalar(base + fint.offset, fint.datatype));
        } else if (frefl.offset >= 0) {
            intensity = static_cast<float>(read_scalar(base + frefl.offset, frefl.datatype));
        }

        // FAST-LIO convention: curvature stores point offset time in milliseconds.
        // Ouster's t field is nanoseconds in the standard driver.
        double offset_ms = 0.0;
        if (ft.offset >= 0) {
            offset_ms = read_scalar(base + ft.offset, ft.datatype) * kOusterTimeNanosecondToMillisecond;
        } else if (ftime.offset >= 0) {
            offset_ms = inferPointTimeToMillisecond(read_scalar(base + ftime.offset, ftime.datatype));
        } else if (ftimestamp.offset >= 0) {
            offset_ms = inferPointTimeToMillisecond(read_scalar(base + ftimestamp.offset, ftimestamp.datatype));
        }

        if ((valid_num % point_filter_num == 0))
        {
            pcl::PointXYZINormal pt_normal;
            pt_normal.x = x;
            pt_normal.y = y;
            pt_normal.z = z;
            pt_normal.normal_x = 0;
            pt_normal.normal_y = 0;
            pt_normal.normal_z = 0;
            pt_normal.intensity = intensity;
            pt_normal.curvature = static_cast<float>(offset_ms);
            if (keepLidarPoint(x, y, z)) {
                cloud_xyzi.emplace_back(pt_normal);
                if (offset_ms > max_offset_ms) {
                    max_offset_ms = offset_ms;
                }
            }
        }
        ++valid_num;
    }

    ROS_DEBUG_STREAM_THROTTLE(1.0, "[receive_points_ntu] valid points: " << cloud_xyzi.size());

    if (!cloud_xyzi.empty()) {
        sortByPointOffsetTime(cloud_xyzi);
        SharedBuffers::LidarFrame frame;
        frame.cloud.reset(new PointCloudXYZI(std::move(cloud_xyzi)));
        frame.base_time = timestamp;
        frame.end_time = timestamp + max_offset_ms / kSecondToMillisecond;
        buffers_->pushLidarFrame(std::move(frame));
    }
}

void TopicProcess::receive_points_velodyne(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // === 1. Initial Setup and Frequency Calculation ===
    cloud_topic_count_++;
    double timestamp = get_time_sec(msg->header.stamp);

    // Frequency calculation logic from your original function
    static double sum_time = 0;
    static int cnt = 0;
    static bool first = true;
    constexpr int MAX_LINE_NUM = 128;

    if (first) {
        last_timestamp_lidar_ = timestamp;
        first = false;
    } else {
        double time_diff = timestamp - last_timestamp_lidar_;
        if (sum_time < 5.0) {
            cnt++;
            sum_time += time_diff;
        } else {
            LOG_DEBUG(Sensor, "[velodyne_callback] lidar frequency=" << cnt / sum_time << " HZ");
            cnt = 0;
            sum_time = 0;
        }
        last_timestamp_lidar_ = timestamp;
    }

    // === 2. Convert ROS Msg to PCL PointCloud ===
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize <= 1) {
        std::cerr << "[velodyne_callback] too few points\n";
        return;
    }

    bool is_first[MAX_LINE_NUM];
    double yaw_fp[MAX_LINE_NUM] = {0};     // yaw of first scan point
    double omega_l = 3.61;                 // scan angular velocity, deg/ms for 10 Hz spinning LiDAR
    float time_last[MAX_LINE_NUM] = {0.0}; // last offset time
    const bool given_offset_time = (pl_orig.points.back().time > 0.0f);

    memset(is_first, true, sizeof(is_first));

    PointCloudXYZI cloud_xyzi; // Local variable for processed points
    cloud_xyzi.reserve(plsize);

    // === 4. Process Each Point ===
    for (int i = 0; i < plsize; ++i) {
        pcl::PointXYZINormal pt;

        int layer = pl_orig.points[i].ring;
        if (layer >= MAX_LINE_NUM) continue;

        pt.x = pl_orig.points[i].x;
        pt.y = pl_orig.points[i].y;
        pt.z = pl_orig.points[i].z;
        pt.intensity = pl_orig.points[i].intensity;

        // FAST-LIO Velodyne config uses timestamp_unit=2, so PointCloud2 time is microseconds.
        if (given_offset_time) {
            pt.curvature = static_cast<float>(pl_orig.points[i].time * kMicrosecondToMillisecond);
        } else {
            double yaw_angle = atan2(pt.y, pt.x) * 180.0 / M_PI; // degrees

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                pt.curvature = 0.0f;
                time_last[layer] = pt.curvature;
                continue;
            }

            if (yaw_angle <= yaw_fp[layer]) {
                pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (pt.curvature < time_last[layer]) {
                pt.curvature += 360.0 / omega_l;
            }

            time_last[layer] = pt.curvature;
        }

        // === 6. Filtering (from your original function) ===
        if (i % point_filter_num == 0) { // point_filter_num is a class member
            if (keepLidarPoint(pt.x, pt.y, pt.z)) {
                cloud_xyzi.emplace_back(pt);
            }
        }
    }

    // === 7. Buffering (from your original function) ===
    if (!cloud_xyzi.empty()) {
        sortByPointOffsetTime(cloud_xyzi);
        const double lidar_end_time = timestamp + cloud_xyzi.back().curvature / kSecondToMillisecond;

        SharedBuffers::LidarFrame frame;
        frame.cloud.reset(new PointCloudXYZI(std::move(cloud_xyzi)));
        frame.base_time = timestamp;
        frame.end_time = lidar_end_time;
        buffers_->pushLidarFrame(std::move(frame));
    }
}

void TopicProcess::receive_points_xt32(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (FSM::current_state == FSM::State::Initializing) return;
    cloud_topic_count_++;
    double timestamp = get_time_sec(msg->header.stamp);

    static double sum_time = 0;
    static int cnt = 0;
    static bool first = true;
    constexpr int MAX_LINE_NUM = 64;
    constexpr int N_SCANS = 32;

    if (first) {
        last_timestamp_lidar_ = timestamp;
        first = false;
    } else {
        double time_diff = timestamp - last_timestamp_lidar_;
        if (sum_time < 5.0) {
            cnt++;
            sum_time += time_diff;
        } else {
            LOG_DEBUG(Sensor, "[xt32_callback] lidar frequency=" << cnt / sum_time << " HZ");
            cnt = 0;
            sum_time = 0;
        }
        last_timestamp_lidar_ = timestamp;
    }

    pcl::PointCloud<xt32_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = static_cast<int>(pl_orig.points.size());
    if (plsize <= 1) {
        std::cerr << "[xt32_callback] too few points\n";
        return;
    }

    bool is_first[MAX_LINE_NUM];
    std::memset(is_first, true, sizeof(is_first));
    double yaw_fp[MAX_LINE_NUM] = {0};
    double omega_l = 3.61;
    float yaw_last[MAX_LINE_NUM] = {0.0};
    float time_last[MAX_LINE_NUM] = {0.0};

    bool given_offset_time = (pl_orig.points[plsize - 1].timestamp > 0);

    if (!given_offset_time) {
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
        (void)yaw_end;
    }

    double time_head = pl_orig.points[0].timestamp;

    PointCloudXYZI cloud_xyzi;
    cloud_xyzi.reserve(plsize);

    for (int i = 0; i < plsize; ++i) {
        pcl::PointXYZINormal added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;

        int layer = pl_orig.points[i].ring;
        if (layer >= N_SCANS) continue;

        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;

        if (given_offset_time) {
            added_pt.curvature = static_cast<float>((pl_orig.points[i].timestamp - time_head) * 1000.0);
        } else {
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0f;
                yaw_last[layer] = static_cast<float>(yaw_angle);
                time_last[layer] = added_pt.curvature;
                continue;
            }

            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.curvature = static_cast<float>((yaw_fp[layer] - yaw_angle) / omega_l);
            } else {
                added_pt.curvature = static_cast<float>((yaw_fp[layer] - yaw_angle + 360.0) / omega_l);
            }

            if (added_pt.curvature < time_last[layer]) {
                added_pt.curvature += static_cast<float>(360.0 / omega_l);
            }

            yaw_last[layer] = static_cast<float>(yaw_angle);
            time_last[layer] = added_pt.curvature;
        }

        if (i % point_filter_num == 0) {
            if (keepLidarPoint(added_pt.x, added_pt.y, added_pt.z)) {
                cloud_xyzi.emplace_back(added_pt);
            }
        }
    }

    if (!cloud_xyzi.empty()) {
        double lidar_end_time = timestamp + cloud_xyzi.back().curvature / 1e3;
        SharedBuffers::LidarFrame frame;
        frame.cloud.reset(new PointCloudXYZI(std::move(cloud_xyzi)));
        frame.base_time = timestamp;
        frame.end_time = lidar_end_time;
        buffers_->pushLidarFrame(std::move(frame));
    }
}

void TopicProcess::receive_points(const lio::CustomMsgConstPtr& msg)
{

    cloud_topic_count_++;
    double timestamp = get_time_sec(msg->header.stamp);
    int plsize = msg->point_num;
    if (plsize <= 1) {
        std::cerr << "[receive_points] too few points\n";
        return;
    }

    static constexpr int N_SCANS = 6;
    static double sum_time = 0;
    static int cnt = 0;
    static bool first = true;

    if (first) {
        last_timestamp_lidar_ = timestamp;
        first = false;
    } else {
        double time_diff = timestamp - last_timestamp_lidar_;
        if (sum_time < 5) {
            cnt++;
            sum_time += time_diff;
        } else {
            LOG_DEBUG(Sensor, "[receive_points] lidar frequency=" << cnt / sum_time << " HZ");
            cnt = 0;
            sum_time = 0;
        }
        last_timestamp_lidar_ = timestamp;
    }

    PointCloudXYZI cloud_xyzi;  // 局部变量，线程安全
    cloud_xyzi.reserve(plsize);

    uint valid_num = 0;
    for (int i = 1; i < plsize; ++i)
    {
        if ((msg->points[i].line < N_SCANS) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            valid_num++;
            if (valid_num % point_filter_num == 0)
            {
                pcl::PointXYZINormal pt;
                pt.x = msg->points[i].x;
                pt.y = msg->points[i].y;
                pt.z = msg->points[i].z;
                pt.intensity = msg->points[i].reflectivity;
                pt.curvature = msg->points[i].offset_time / 1e6f; // ms

                if (keepLidarPoint(pt.x, pt.y, pt.z)) {
                    cloud_xyzi.emplace_back(pt);
                }
            }
        }
    }

    double lidar_end_time = get_time_sec(msg->header.stamp) + msg->points.back().offset_time / 1E9;

    if (!cloud_xyzi.empty()) {
        SharedBuffers::LidarFrame frame;
        frame.cloud.reset(new PointCloudXYZI(std::move(cloud_xyzi)));
        frame.base_time = timestamp;
        frame.end_time = lidar_end_time;
        buffers_->pushLidarFrame(std::move(frame));
    }
}




/**
 * @brief:
 * 将 IMU 数据存入 IMU 缓冲队列(imu_buffer)
 * 在时间戳出现回溯时，清除缓存（播包时可能出现）
 * 在每次接收到 IMU topic后执行
 * @param msg: 接收到的IMU数据
 */
void TopicProcess::receive_imu(const sensor_msgs::ImuConstPtr& msg)
{
    imu_topic_count_++;
    double timestamp = get_time_sec(msg->header.stamp);

    auto now = std::chrono::high_resolution_clock::now();
    double time_sys = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() * 1e-6;


    static sensor_msgs::ImuConstPtr last_imu_msg = msg;
    Eigen::Vector3d last_ang(last_imu_msg->angular_velocity.x,
                             last_imu_msg->angular_velocity.y,
                             last_imu_msg->angular_velocity.z);
    Eigen::Vector3d curr_ang(msg->angular_velocity.x,
                             msg->angular_velocity.y,
                             msg->angular_velocity.z);

    if ((curr_ang - last_ang).norm() > THRE_DIFF_IMU) {
        std::cerr << "[receive_imu] imu_acc outliers detected!\n";
        sensor_msgs::ImuPtr corrected_msg(new sensor_msgs::Imu(*last_imu_msg));
        corrected_msg->header.stamp = msg->header.stamp;
        buffers_->pushBackSafe(buffers_->imu_buf, corrected_msg);
    } else {
        last_imu_msg = msg;
        buffers_->pushBackSafe(buffers_->imu_buf, msg);
    }

    /************** frequency calculate ****************/
    static bool first = true;
    if (first) {
        last_timestamp_imu_ = timestamp;
        first = false;
    } else {
        double time_diff = timestamp - last_timestamp_imu_;
        static double sum_time = 0;
        static int i = 0;
        if (sum_time < 5) {
            i++;
            sum_time += time_diff;
        } else {
            std::cout << "[receive_imu] IMU frequency: " << i / sum_time << " HZ" << std::endl;
            i = 0;
            sum_time = 0;
        }
        last_timestamp_imu_ = timestamp;
    }

    buffers_->notifyAll();
}

/**
 * @brief:
 * 将轮速计数据存入轮速缓冲队列
 * @param msg: 接收到的轮速计消息
 */
void TopicProcess::receive_wheel(const lio::wheel_infoConstPtr& msg)
{
    wheel_topic_count_++;
    /**
     * 按照象限划分，取出四个轮子角度 angle 和速度 v
     * 以车中心为原点，向前为x轴，向左为y轴
     */
    double angle_L = (msg->front_angle_fb_l - msg->rear_angle_fb_l) / 2.0;
    double angle_R = (msg->front_angle_fb_r - msg->rear_angle_fb_r) / 2.0;
    double v_L = (msg->lf_wheel_fb_velocity + msg->lr_wheel_fb_velocity) / 2.0;
    double v_R = (msg->rf_wheel_fb_velocity + msg->rr_wheel_fb_velocity) / 2.0;

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

    WheelData wheel_data{};
    wheel_data.timestamp = timestamp;
    wheel_data.angle_L = angle_L * M_PI / 180.0;
    wheel_data.angle_R = angle_R * M_PI / 180.0;
    wheel_data.v_L = v_L;
    wheel_data.v_R = v_R;

    buffers_->pushBackSafe(buffers_->wheel_buf, wheel_data);

    buffers_->notifyAll();
}






std::deque<WheelData> TopicProcess::get_wheel_buffer() const {
    std::scoped_lock lk(buffers_->mtx);
    return buffers_->wheel_buf;  // ✅ 返回拷贝
}
