//
// Created by yczhang on 25-5-20.
//

#include "estimator/IMUProcess.h"
#include "support/common_lib.h"
#include <iostream>
#include "estimator/ESEKF.h"
#include <algorithm>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>

using namespace std;

#include <opencv2/opencv.hpp>
#include <deque>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>

class StatePlotter {
public:
    // 配置参数
    struct Config {
        int width = 800;
        int height = 400;
        size_t max_history = 500; // <--- 修改点1: 改为 size_t 消除警告
        std::vector<cv::Scalar> colors = {
            cv::Scalar(0, 255, 255),   // 黄色
            cv::Scalar(0, 255, 0),     // 绿色
            cv::Scalar(255, 100, 100)  // 蓝色
        };

        // <--- 修改点2: 显式添加默认构造函数，解决编译报错
        Config() {}
    };

    StatePlotter(std::string name, std::vector<std::string> labels, Config conf = Config())
        : window_name(name), channel_labels(labels), config(conf) {

        data_queues.resize(labels.size());
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    }

    void update(const std::vector<double>& values, bool in_elevator) {
        if (values.size() != data_queues.size()) return;

        for (size_t i = 0; i < values.size(); ++i) {
            data_queues[i].push_back(values[i]);
            // size() 返回的是 size_t，现在和 config.max_history 类型一致了
            if (data_queues[i].size() > config.max_history) {
                data_queues[i].pop_front();
            }
        }
        elevator_status.push_back(in_elevator);
        if (elevator_status.size() > config.max_history) elevator_status.pop_front();

        draw();
    }

private:
    std::string window_name;
    std::vector<std::string> channel_labels;
    Config config;
    std::vector<std::deque<double>> data_queues;
    std::deque<bool> elevator_status;

    void draw() {
        cv::Mat img = cv::Mat::zeros(config.height, config.width, CV_8UC3);

        if (data_queues[0].empty()) return;

        // 1. Auto-Scale 计算
        double min_val = 1e9, max_val = -1e9;
        for (const auto& q : data_queues) {
            for (double v : q) {
                if (v < min_val) min_val = v;
                if (v > max_val) max_val = v;
            }
        }
        if (std::abs(max_val - min_val) < 1e-6) {
            max_val += 1.0;
            min_val -= 1.0;
        }
        double range = max_val - min_val;
        double scale_y = (config.height - 40) / range;

        // 2. 绘制背景 (int cast 防止警告)
        int step_x = config.width / (int)config.max_history;
        if (step_x < 1) step_x = 1;

        for (size_t t = 0; t < elevator_status.size(); ++t) {
            if (elevator_status[t]) {
                // 注意这里要做类型转换防止溢出或警告
                int x1 = static_cast<int>(t) * config.width / static_cast<int>(config.max_history);
                int x2 = static_cast<int>(t + 1) * config.width / static_cast<int>(config.max_history);
                cv::rectangle(img, cv::Rect(x1, 0, x2 - x1 + 1, config.height), cv::Scalar(40, 0, 40), -1);
            }
        }

        // 3. 绘制参考线
        if (min_val < 0 && max_val > 0) {
            int y0 = config.height - 20 - (0 - min_val) * scale_y;
            cv::line(img, cv::Point(0, y0), cv::Point(config.width, y0), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
        }

        // 4. 绘制曲线
        for (size_t ch = 0; ch < data_queues.size(); ++ch) {
            const auto& q = data_queues[ch];
            cv::Scalar color = config.colors[ch % config.colors.size()];

            std::vector<cv::Point> points;
            for (size_t t = 0; t < q.size(); ++t) {
                int x = static_cast<int>(t) * config.width / static_cast<int>(config.max_history);
                int y = config.height - 20 - (q[t] - min_val) * scale_y;
                points.push_back(cv::Point(x, y));
            }
            cv::polylines(img, points, false, color, 2, cv::LINE_AA);

            std::stringstream ss;
            ss << channel_labels[ch] << ": " << std::fixed << std::setprecision(3) << q.back();
            cv::putText(img, ss.str(), cv::Point(10, 20 + ch * 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
        }

        std::stringstream ss_range;
        ss_range << "Max: " << std::fixed << std::setprecision(2) << max_val
                 << " Min: " << min_val;
        cv::putText(img, ss_range.str(), cv::Point(config.width - 200, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);

        cv::imshow(window_name, img);
        cv::waitKey(1);
    }
};


/**
 * @brief 积分一个 IMU 帧，并更新环形缓冲区
 * @param IMU_acc
 * @param IMU_gyr
 * @param time_stamp
 */
//TODO 初始化问题
void IMUProcess::integration(Vector3d IMU_acc, Vector3d IMU_gyr, double time_stamp) {

    std::uint8_t acc_saturation_mask = 0;
    std::uint8_t gyr_saturation_mask = 0;
    sanitizeImuInput(IMU_acc, IMU_gyr, acc_saturation_mask, gyr_saturation_mask);

    /* ---------- 1. 写入环形缓冲 ---------- */
    imu_acc_buffer[head_]  = std::move(IMU_acc);
    imu_gyr_buffer[head_]  = std::move(IMU_gyr);
    imu_acc_saturation_buffer[head_] = acc_saturation_mask;
    imu_gyr_saturation_buffer[head_] = gyr_saturation_mask;
    time_buffer[head_] = time_stamp;

    /* ---------- 2. 第一帧：初始化 ---------- */
    if (count_ == 0) {
        // 已经在外部调用 set_init_state 初始化了
        if (!initialized)initialize(time_stamp);
        // 第一帧不计算，仅写入状态
    }
    else {
        /* 2.1 上一帧索引与 Δt */
        int    prev = (head_ - 1 + MAX_LEN) % MAX_LEN;
        double dt   = time_stamp - time_buffer[prev];
        if (dt > imu_gap_threshold_s) {
            latest_gap_end = time_stamp;
            latest_gap_duration = dt;
            if (sensor_gap_recovery_enable) lio_tracking_valid.store(false);
            LOG_WARN(Sensor, "[imu_gap] start=" << std::setprecision(16)
                     << time_buffer[prev] << " end=" << time_stamp << " dt=" << dt);
        }

        /* 2.2 梯形法求平均量 */
        Eigen::Vector3d acc_avg = 0.5 * (imu_acc_buffer[prev] + imu_acc_buffer[head_]);
        Eigen::Vector3d gyr_avg = 0.5 * (imu_gyr_buffer[prev] + imu_gyr_buffer[head_]);
       // Eigen::Vector3d acc_avg = 0.5 * (imu_acc_buffer[prev] + IMU_acc);
       // Eigen::Vector3d gyr_avg = 0.5 * (imu_gyr_buffer[prev] + IMU_gyr);


        /* 2.3 预测下一状态 */
        const std::uint8_t interval_acc_mask =
                imu_acc_saturation_buffer[prev] | imu_acc_saturation_buffer[head_];
        const std::uint8_t interval_gyr_mask =
                imu_gyr_saturation_buffer[prev] | imu_gyr_saturation_buffer[head_];
        const bool previous_mode = states_imu[prev].in_elevator;
        State state_next = predictOnce(states_imu[prev], acc_avg, gyr_avg, dt, time_stamp,
                                       interval_acc_mask, interval_gyr_mask);

        if (!elevator_enable) {
            ELEVATOR_TRIGGER = false;
            EXIT_FROM_ELEVATOR = false;
        }
        state_next.in_elevator = elevator_enable && ELEVATOR_TRIGGER;

        if (!previous_mode && state_next.in_elevator) {
            elev_process.ElevatorModeEnter(state_next, acc_avg);
        }

        if (previous_mode && !state_next.in_elevator) {
            EXIT_FROM_ELEVATOR = true;
        }

        // 状态可视化
        if (EleState_vis) {
            static StatePlotter plotter("Elevator State Monitor", {"Vz (Vel)", "Az (Acc)", "Z (Pos)"});
            // 2. 准备数据并更新
            std::vector<double> current_vals;
            current_vals.push_back(state_next.vz);        // 观察速度是否归零
            current_vals.push_back(state_next.az);        // 观察加速度偏置
            current_vals.push_back(state_next.z);         // 观察高度
            // 3. 传入数据和电梯状态标志
            // in_elevator 为 true 时，背景会变红，方便你看是否是模式切换瞬间导致的跳变
            plotter.update(current_vals, state_next.in_elevator);
        }

        states_imu[head_] = state_next;

        // static bool drift_ = true;
        // if (drift_ && time_stamp > 1761836126) {
        //     drift_ = false;
        //     IMUProcess::correctZDrift(head_ - 12 * 200, head_);
        // }

        /* 2.4 调试日志  */
        // logFrame();
    }

    /* ---------- 3. 更新环形指针 ---------- */
    head_  = (head_ + 1) % MAX_LEN;
    count_ = std::min(count_ + 1, MAX_LEN);
}

void IMUProcess::set_init_state(State state_init ,double time_stamp) {
    /* ---------- 填充环形缓冲 ---------- */
    std::fill(states_imu.begin(), states_imu.end(), state_init);
    std::fill(time_buffer.begin(), time_buffer.end(), time_stamp);
    std::fill(imu_acc_buffer.begin(),  imu_acc_buffer.end(),  Eigen::Vector3d::Zero());
    std::fill(imu_gyr_buffer.begin(),  imu_gyr_buffer.end(),  Eigen::Vector3d::Zero());
    std::fill(imu_acc_saturation_buffer.begin(), imu_acc_saturation_buffer.end(), 0);
    std::fill(imu_gyr_saturation_buffer.begin(), imu_gyr_saturation_buffer.end(), 0);

    /* ---------- 过程噪声 Q (12×12) ---------- */
    ACC_NOISE_VAR = imu_input_acc_noise_variance;
    GYRO_NOISE_VAR = imu_input_gyro_noise_variance;
    Q_.setZero(StateNoiseIndex::NOISE_TOTAL,StateNoiseIndex::NOISE_TOTAL);
    Q_.block<3,3>(StateNoiseIndex::ACC_NOISE,        StateNoiseIndex::ACC_NOISE)         = ACC_NOISE_VAR        * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_NOISE,       StateNoiseIndex::GYRO_NOISE)        = GYRO_NOISE_VAR       * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::ACC_RANDOM_WALK,  StateNoiseIndex::ACC_RANDOM_WALK)   = ACC_RANDOM_WALK_VAR  * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK)  = GYRO_RANDOM_WALK_VAR * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GRAVITY_NOISE, StateNoiseIndex::GRAVITY_NOISE)  = GRAVITY_NOISE_VAR * Eigen::Matrix3d::Identity();
    Q_(StateNoiseIndex::ELEVATOR_ACC_NOISE, StateNoiseIndex::ELEVATOR_ACC_NOISE)  = ELEVATOR_ACC_NOISE_VAR;


    initialized = true;
}

void IMUProcess::integration(ImuMsgConst imu_msg, Eigen::Vector3d mean_acc) {
    Eigen::Vector3d cur_gyr (imu_msg->angular_velocity.x,
                             imu_msg->angular_velocity.y,
                             imu_msg->angular_velocity.z);
    Eigen::Vector3d cur_acc (imu_msg->linear_acceleration.x,
                             imu_msg->linear_acceleration.y,
                             imu_msg->linear_acceleration.z);
    if (G_SCALE_UP) {
        cur_acc *= G_m_s2;
    }
    double time_stamp = get_time_sec(imu_msg->header.stamp);
    integration(cur_acc, cur_gyr, time_stamp);
}

bool IMUProcess::getLatestProcessedSample(ProcessedImuSample &sample) const {
    if (count_ <= 0) return false;

    const int latest = (head_ - 1 + MAX_LEN) % MAX_LEN;
    sample.time = time_buffer[latest];
    sample.acc = imu_acc_buffer[latest];
    sample.gyr = imu_gyr_buffer[latest];
    sample.acc_saturation_mask = imu_acc_saturation_buffer[latest];
    sample.gyr_saturation_mask = imu_gyr_saturation_buffer[latest];
    return true;
}

bool IMUProcess::coversTimeRange(double start_time, double end_time) const {
    if (count_ < 2 || end_time < start_time) return false;
    const double oldest_time = time_buffer[wrapIndex(head_ - count_)];
    const double newest_time = time_buffer[wrapIndex(head_ - 1)];
    constexpr double kTimeEpsilon = 1e-6;
    return start_time >= oldest_time - kTimeEpsilon &&
           end_time <= newest_time + kTimeEpsilon;
}

bool IMUProcess::hasSaturationBetween(double start_time, double end_time,
                                      int *sample_count) const {
    if (sample_count) *sample_count = 0;
    if (!imu_update_enable || count_ <= 0 || end_time < start_time) {
        return false;
    }

    constexpr double kTimeEpsilon = 1e-6;
    int count = 0;
    for (int i = 0; i < count_; ++i) {
        const int index = wrapIndex(head_ - count_ + i);
        const double stamp = time_buffer[index];
        if (stamp < start_time - kTimeEpsilon) continue;
        if (stamp > end_time + kTimeEpsilon) break;
        if (imu_acc_saturation_buffer[index] != 0 ||
            imu_gyr_saturation_buffer[index] != 0) {
            ++count;
        }
    }
    if (sample_count) *sample_count = count;
    return count > 0;
}

void IMUProcess::sanitizeImuInput(Eigen::Vector3d &acc, Eigen::Vector3d &gyr,
                                  std::uint8_t &acc_mask, std::uint8_t &gyr_mask) {
    acc_mask = 0;
    gyr_mask = 0;
    const bool detect_saturation = imu_update_enable &&
            !(elevator_enable && (ELEVATOR_TRIGGER ||
              (count_ > 0 && states_imu[wrapIndex(head_ - 1)].in_elevator)));
    const double acc_threshold = imu_saturation_accel_limit_g * G_m_s2;
    const double gyr_threshold = imu_saturation_gyro_limit_rad_s;

    for (int axis = 0; axis < 3; ++axis) {
        const std::uint8_t bit = static_cast<std::uint8_t>(1u << axis);
        // Low three bits mean "over the configured range" and are used to
        // trigger the temporal LiDAR schedule.  High three bits distinguish
        // genuinely unusable NaN/Inf samples from finite over-range samples.
        const std::uint8_t invalid_bit = static_cast<std::uint8_t>(1u << (axis + 3));
        const bool acc_invalid = !std::isfinite(acc(axis));
        const bool gyr_invalid = !std::isfinite(gyr(axis));
        const bool acc_saturated = detect_saturation &&
                                   !acc_invalid && std::abs(acc(axis)) >= acc_threshold;
        const bool gyr_saturated = detect_saturation &&
                                   !gyr_invalid && std::abs(gyr(axis)) >= gyr_threshold;

        if (acc_invalid) {
            acc_mask |= static_cast<std::uint8_t>(bit | invalid_bit);
            if (have_trusted_acc_[static_cast<std::size_t>(axis)]) {
                acc(axis) = last_trusted_acc_(axis);
            } else {
                acc(axis) = 0.0;
            }
        } else if (acc_saturated) {
            acc_mask |= bit;
            // A finite over-range value remains usable. The mask selects the
            // local output-state replay and inflates the ordinary input-model
            // process noise; it must not replace the measurement itself.
        } else {
            last_trusted_acc_(axis) = acc(axis);
            have_trusted_acc_[static_cast<std::size_t>(axis)] = true;
        }

        if (gyr_invalid) {
            gyr_mask |= static_cast<std::uint8_t>(bit | invalid_bit);
            if (have_trusted_gyr_[static_cast<std::size_t>(axis)]) {
                gyr(axis) = last_trusted_gyr_(axis);
            } else {
                gyr(axis) = 0.0;
            }
        } else if (gyr_saturated) {
            gyr_mask |= bit;
            // Keep the finite measurement for both the live input propagation
            // and the on-demand output-state replay.
        } else {
            last_trusted_gyr_(axis) = gyr(axis);
            have_trusted_gyr_[static_cast<std::size_t>(axis)] = true;
        }
    }

    if (acc_mask != 0 || gyr_mask != 0) {
        ++saturation_event_count_;
        if (saturation_event_count_ == 1 || saturation_event_count_ % 200 == 0) {
            LOG_WARN(Sensor, "IMU possible saturation/non-finite input: acc_mask=0x"
                     << std::hex << static_cast<int>(acc_mask)
                     << " gyro_mask=0x" << static_cast<int>(gyr_mask) << std::dec
                     << " events=" << saturation_event_count_);
        }
    }
}

bool IMUProcess::beginReplay(double start_time, double end_time, ReplayCursor &cursor,
                             bool output_covariance_at_imu_rate) {
    cursor = ReplayCursor{};
    cursor.output_covariance_at_imu_rate = output_covariance_at_imu_rate;
    if (count_ < 2 || end_time < start_time) return false;

    cursor.samples.reserve(static_cast<std::size_t>(count_));
    for (int i = 0; i < count_; ++i) {
        const int index = wrapIndex(head_ - count_ + i);
        ReplaySample sample;
        sample.time = time_buffer[index];
        sample.acc = imu_acc_buffer[index];
        sample.gyr = imu_gyr_buffer[index];
        sample.acc_saturation_mask = imu_acc_saturation_buffer[index];
        sample.gyr_saturation_mask = imu_gyr_saturation_buffer[index];
        sample.in_elevator = states_imu[index].in_elevator;
        if (!cursor.samples.empty() &&
            sample.time <= cursor.samples.back().time) {
            LOG_ERROR(Sensor, "non-monotonic raw IMU timestamps in replay buffer: prev="
                      << std::fixed << std::setprecision(9) << cursor.samples.back().time
                      << " current=" << sample.time);
            cursor.samples.clear();
            return false;
        }
        cursor.samples.push_back(sample);
    }

    constexpr double kTimeEpsilon = 1e-6;
    if (start_time < cursor.samples.front().time - kTimeEpsilon ||
        end_time > cursor.samples.back().time + kTimeEpsilon) {
        cursor.samples.clear();
        return false;
    }

    std::size_t left = 0;
    while (left + 1 < cursor.samples.size() &&
           cursor.samples[left + 1].time <= start_time + kTimeEpsilon) {
        ++left;
    }
    if (left + 1 >= cursor.samples.size() && end_time > start_time + kTimeEpsilon) {
        cursor.samples.clear();
        return false;
    }

    const ReplaySample &left_sample = cursor.samples[left];
    // The live ring buffer always uses the original input-IMU model. The
    // caller may replace this state with the last committed LiDAR posterior,
    // then explicitly activate the local output-state replay.
    get_state_at_t(cursor.state, start_time);
    cursor.state.time = start_time;
    if (std::abs(left_sample.time - start_time) <= kTimeEpsilon ||
        left + 1 >= cursor.samples.size()) {
        cursor.current_acc = left_sample.acc;
        cursor.current_gyr = left_sample.gyr;
        cursor.current_acc_saturation_mask = left_sample.acc_saturation_mask;
        cursor.current_gyr_saturation_mask = left_sample.gyr_saturation_mask;
        cursor.next_sample = left + 1;
    } else {
        const ReplaySample &right_sample = cursor.samples[left + 1];
        const double span = right_sample.time - left_sample.time;
        if (span <= 0.0) return false;
        const double ratio = std::clamp((start_time - left_sample.time) / span, 0.0, 1.0);
        cursor.current_acc = (1.0 - ratio) * left_sample.acc + ratio * right_sample.acc;
        cursor.current_gyr = (1.0 - ratio) * left_sample.gyr + ratio * right_sample.gyr;
        cursor.current_acc_saturation_mask =
                left_sample.acc_saturation_mask | right_sample.acc_saturation_mask;
        cursor.current_gyr_saturation_mask =
                left_sample.gyr_saturation_mask | right_sample.gyr_saturation_mask;
        cursor.next_sample = left + 1;
    }
    cursor.covariance_time = start_time;
    cursor.valid = true;
    return true;
}

void IMUProcess::resetOutputSubstateCovariance(State &state) const {
    if (state.P.rows() != StateIndex::STATE_TOTAL ||
        state.P.cols() != StateIndex::STATE_TOTAL) {
        return;
    }
    constexpr int kOutputDim = 6;
    state.P.block(StateIndex::OMG, 0, kOutputDim, StateIndex::STATE_TOTAL).setZero();
    state.P.block(0, StateIndex::OMG, StateIndex::STATE_TOTAL, kOutputDim).setZero();
    state.P.block<3, 3>(StateIndex::OMG, StateIndex::OMG) =
            Eigen::Matrix3d::Identity() * 1e-2;
    state.P.block<3, 3>(StateIndex::ACC, StateIndex::ACC) =
            Eigen::Matrix3d::Identity() * 1e-2;
}

bool IMUProcess::activateOutputReplay(ReplayCursor &cursor) const {
    if (!cursor.valid || !imu_update_enable || cursor.state.in_elevator) return false;

    // Transform the input-model anchor into an output-model anchor without
    // changing pose, velocity, biases, gravity, or their covariance marginal.
    // The current raw IMU sample is exactly represented as omg+bw / acc+ba,
    // so entering the local model cannot introduce a state jump.
    cursor.state.omg = cursor.current_gyr - cursor.state.bw;
    cursor.state.acc = cursor.current_acc - cursor.state.ba;
    resetOutputSubstateCovariance(cursor.state);
    cursor.output_model_active = true;
    cursor.covariance_time = cursor.state.time;
    return true;
}

void IMUProcess::deactivateOutputReplay(State &state) const {
    // omg/acc are dormant in the live input model. Remove their replay-created
    // cross covariance so later ordinary frame updates cannot feed the local
    // output states back into pose, velocity, or IMU biases.
    resetOutputSubstateCovariance(state);
}

bool IMUProcess::propagateReplay(ReplayCursor &cursor, double target_time) {
    constexpr double kTimeEpsilon = 1e-9;
    if (!cursor.valid || target_time < cursor.state.time - kTimeEpsilon ||
        cursor.samples.empty() || target_time > cursor.samples.back().time + 1e-6) {
        return false;
    }

    while (cursor.next_sample < cursor.samples.size() &&
           cursor.samples[cursor.next_sample].time <= target_time + kTimeEpsilon) {
        const ReplaySample &sample = cursor.samples[cursor.next_sample];
        const double dt = sample.time - cursor.state.time;
        const bool previous_mode = cursor.state.in_elevator;
        const bool output_model_before = cursor.output_model_active;
        const bool target_mode = elevator_enable && sample.in_elevator;
        if (output_model_before && target_mode) return false;
        Eigen::Vector3d acc_avg = sample.acc;
        if (dt > kTimeEpsilon) {
            acc_avg = 0.5 * (cursor.current_acc + sample.acc);
            const Eigen::Vector3d gyr_avg = 0.5 * (cursor.current_gyr + sample.gyr);
            cursor.state = predictOnce(
                    cursor.state, acc_avg, gyr_avg, dt, sample.time,
                    cursor.current_acc_saturation_mask | sample.acc_saturation_mask,
                    cursor.current_gyr_saturation_mask | sample.gyr_saturation_mask,
                    !output_model_before, output_model_before);
            if (output_model_before && !cursor.output_covariance_at_imu_rate) {
                const ReplaySample &interval_left = cursor.samples[cursor.next_sample - 1];
                propagateOutputCovariance(cursor.state, dt,
                                          sample.time - interval_left.time);
            }
        }
        if (output_model_before) {
            if (cursor.output_covariance_at_imu_rate) {
                const double covariance_dt = sample.time - cursor.covariance_time;
                if (covariance_dt > kTimeEpsilon) {
                    // Point-LIO with prop_at_freq_of_imu=true propagates the
                    // covariance once per IMU sample, after the mean has reached
                    // that IMU time and after any intervening point updates.
                    propagateOutputCovariance(cursor.state, covariance_dt,
                                              covariance_dt);
                    cursor.covariance_time = sample.time;
                }
            }
        } else {
            // Do not let a later switch back to the output model integrate one
            // large covariance interval accumulated while inside the elevator.
            cursor.covariance_time = sample.time;
        }
        cursor.state.in_elevator = target_mode;
        if (!previous_mode && target_mode) {
            elev_process.ElevatorModeEnter(cursor.state, acc_avg);
        }
        if (cursor.output_model_active) {
            updateOutputImuMeasurement(cursor.state, sample.acc, sample.gyr,
                                       sample.acc_saturation_mask,
                                       sample.gyr_saturation_mask);
        }
        cursor.current_acc = sample.acc;
        cursor.current_gyr = sample.gyr;
        cursor.current_acc_saturation_mask = sample.acc_saturation_mask;
        cursor.current_gyr_saturation_mask = sample.gyr_saturation_mask;
        ++cursor.next_sample;
    }

    const double remaining = target_time - cursor.state.time;
    if (remaining > kTimeEpsilon) {
        if (cursor.next_sample >= cursor.samples.size() || cursor.next_sample == 0) return false;
        const ReplaySample &left = cursor.samples[cursor.next_sample - 1];
        const ReplaySample &right = cursor.samples[cursor.next_sample];
        const double span = right.time - left.time;
        if (span <= 0.0) return false;
        const double ratio = std::clamp((target_time - left.time) / span, 0.0, 1.0);
        const Eigen::Vector3d target_acc = (1.0 - ratio) * left.acc + ratio * right.acc;
        const Eigen::Vector3d target_gyr = (1.0 - ratio) * left.gyr + ratio * right.gyr;
        const std::uint8_t target_acc_mask =
                left.acc_saturation_mask | right.acc_saturation_mask;
        const std::uint8_t target_gyr_mask =
                left.gyr_saturation_mask | right.gyr_saturation_mask;
        const bool previous_mode = cursor.state.in_elevator;
        const bool output_model_before = cursor.output_model_active;
        const bool target_mode = elevator_enable &&
                ((ratio < 0.5) ? left.in_elevator : right.in_elevator);
        if (output_model_before && target_mode) return false;
        const Eigen::Vector3d acc_avg = 0.5 * (cursor.current_acc + target_acc);
        cursor.state = predictOnce(
                cursor.state, acc_avg,
                0.5 * (cursor.current_gyr + target_gyr), remaining, target_time,
                cursor.current_acc_saturation_mask | target_acc_mask,
                cursor.current_gyr_saturation_mask | target_gyr_mask,
                !output_model_before, output_model_before);
        if (output_model_before && !cursor.output_covariance_at_imu_rate) {
            propagateOutputCovariance(cursor.state, remaining, span);
        } else if (!output_model_before) {
            cursor.covariance_time = target_time;
        }
        cursor.state.in_elevator = target_mode;
        if (!previous_mode && target_mode) {
            elev_process.ElevatorModeEnter(cursor.state, acc_avg);
        }
        cursor.current_acc = target_acc;
        cursor.current_gyr = target_gyr;
        cursor.current_acc_saturation_mask = target_acc_mask;
        cursor.current_gyr_saturation_mask = target_gyr_mask;
    } else {
        cursor.state.time = target_time;
    }
    return true;
}

/**
 * @brief  根据上一状态 + 平均 IMU（body 系） + dt 预测下一状态并传播协方差
 * @param  prev     上一时刻状态（均值 & 协方差）
 * @param  acc_b    本区间平均线加速度  [m/s²]  (body frame, 未去 bias)
 * @param  gyr_b    本区间平均角速度    [rad/s] (body frame, 未去 bias)
 * @param  dt       区间长度           [s]
 * @param  stamp    输出状态的时间戳    [s]
 * @return          预测后的 State
 */
State IMUProcess::predictOnce(const State            &prev,
                              const Eigen::Vector3d  &acc_b,
                              const Eigen::Vector3d  &gyr_b,
                              double                  dt,
                              double                  stamp,
                              std::uint8_t             acc_saturation_mask,
                              std::uint8_t             gyr_saturation_mask,
                              bool                     propagate_covariance,
                              bool                     output_model) const
{
    /* === 0. 复制旧状态，准备写新量 === */
    State nxt = prev;      // bias、g 等直接拷贝
    nxt.time = stamp;      // 最后统一写入

    if (output_model) {
        // Point-LIO output-state model: IMU readings are measurements of the
        // explicit angular-rate/specific-force states, not propagation inputs.
        nxt.q = (prev.q * Eigen::Quaterniond(so3Exp(prev.omg * dt))).normalized();
        Eigen::Vector3d acc_w = nxt.q * prev.acc + prev.g;
        double elevator_acc = prev.az;

        if (prev.in_elevator) {
            const Eigen::Vector3d ezu = ElevatorProcess::ezu_from_g(prev.g);
            if (elevator_strong_prior_enable) {
                Eigen::Matrix<double, 1, StateIndex::STATE_TOTAL> H;
                H.setZero();
                H(0, StateIndex::AZ) = 1.0;
                Eigen::Matrix<double, 1, 1> residual;
                residual(0) = ezu.dot(acc_w) - nxt.az;
                Eigen::Matrix<double, 1, 1> measurement_covariance;
                measurement_covariance(0, 0) = elevator_strong_prior_acc_var;
                ekfUpdate<1>(nxt, H, residual, measurement_covariance);
                acc_w = nxt.q * nxt.acc + nxt.g;
                elevator_acc = nxt.az;
            }
            acc_w -= elevator_acc * ezu;
        }

        nxt.p = prev.p + prev.v * dt + 0.5 * acc_w * dt * dt;
        nxt.v = prev.v + acc_w * dt;
        if (prev.in_elevator) {
            nxt.z = prev.z + prev.vz * dt + 0.5 * elevator_acc * dt * dt;
            nxt.vz = prev.vz + elevator_acc * dt;
        }

        if (propagate_covariance) propagateOutputCovariance(nxt, dt);
        return nxt;
    }

    /* === 1. 姿态更新（四元数左乘） === */
    Eigen::Vector3d dtheta = (gyr_b - prev.bw) * dt;     // 去 gyro-bias
    double th = dtheta.norm();
    Eigen::Quaterniond dq(1.0, 0.0, 0.0, 0.0);
    if (th > 1e-12) {
        Eigen::Vector3d axis = dtheta / th;
        double half = 0.5 * th;
        dq.w()   = std::cos(half);
        dq.vec() = axis * std::sin(half);
    }
    nxt.q = (prev.q * dq).normalized();

    /* === 2. 世界系线加速度 === */
    Eigen::Vector3d acc_w = nxt.q * (acc_b - prev.ba) + prev.g;
    double elevator_acc = prev.az;

    if (prev.in_elevator) {
        const Eigen::Vector3d ezu = ElevatorProcess::ezu_from_g(prev.g);
        if (elevator_strong_prior_enable && acc_saturation_mask == 0) {
            Eigen::Matrix<double, 1, StateIndex::STATE_TOTAL> H;
            H.setZero();
            H(0, StateIndex::AZ) = 1.0;

            Eigen::Matrix<double, 1, 1> r;
            r(0) = ezu.dot(acc_w) - nxt.az;

            Eigen::Matrix<double, 1, 1> R;
            R(0, 0) = elevator_strong_prior_acc_var;
            ekfUpdate<1>(nxt, H, r, R);

            acc_w = nxt.q * (acc_b - nxt.ba) + nxt.g;
            elevator_acc = nxt.az;
        }
        acc_w -= elevator_acc * ezu;
    }

    /* === 3. 位置 & 速度 === */
    nxt.p = prev.p + prev.v * dt + 0.5 * acc_w * dt * dt;
    nxt.v = prev.v + acc_w * dt;

    if (prev.in_elevator) {
        nxt.z  = prev.z + prev.vz * dt + 0.5 * elevator_acc * dt * dt;
        nxt.vz = prev.vz + elevator_acc * dt;
    }

    /* ----------------------------------------------------------------
     *                       协  方  差  传  播
     * ---------------------------------------------------------------- */
    /* === 4. 构建连续时间雅可比 A 和 噪声耦合 U === */
    Eigen::Matrix<double, STATE_TOTAL, STATE_TOTAL> A;
    A.setZero();

    // A(δθ,δθ)
    A.block<3,3>(R, R) = -skew3d(gyr_b - prev.bw);
    // A(δθ,δbw)
    A.block<3,3>(R, BW) = -Eigen::Matrix3d::Identity();

    // A(δp,δv)
    A.block<3,3>(P, V) = Eigen::Matrix3d::Identity();

    // A(δv,δθ)
    A.block<3,3>(V, R) = nxt.q.toRotationMatrix() * -skew3d(acc_b - prev.ba);
    // A(δv,δba)
    A.block<3,3>(V, BA) = -nxt.q.toRotationMatrix();
    // A(δv,δg)
    if (online_gravity_estimation_enable) {
        A.block<3,3>(V, G) = Eigen::Matrix3d::Identity();
    }

    /* --- U (27×16) --- */
    Eigen::Matrix<double, STATE_TOTAL, NOISE_TOTAL> U;
    U.setZero();
    U.block<3,3>(R, GYRO_NOISE)   = -Eigen::Matrix3d::Identity();           // gyro-noise
    U.block<3,3>(V, ACC_NOISE)   = -nxt.q.toRotationMatrix();              // acc-noise
    U.block<3,3>(BW, GYRO_RANDOM_WALK) =  Eigen::Matrix3d::Identity();           // gyro-bias random walk
    U.block<3,3>(BA, ACC_RANDOM_WALK) =  Eigen::Matrix3d::Identity();           // acc-bias random walk
    if (online_gravity_estimation_enable) {
        U.block<3,3>(G, GRAVITY_NOISE) = Eigen::Matrix3d::Identity();
    }

    /* === 5. 离散化：F ≈ I + A·dt === */
    Eigen::Matrix<double, STATE_TOTAL, STATE_TOTAL> F =
        Eigen::Matrix<double, STATE_TOTAL, STATE_TOTAL>::Identity() + dt * A;

    /* === 6. 根据是否在电梯中填充 F 和 U 矩阵 === */
    if (prev.in_elevator) {
        const double dt2 = dt * dt;
        F.block<3,1>(P, AZ) = -Eigen::Vector3d(0,0,0.5 * dt2);
        F.block<3,1>(V, AZ) = -Eigen::Vector3d(0,0,dt);
        F(Z, VZ) = dt;
        F(Z, AZ) = 0.5 * dt2;
        F(VZ, AZ) = dt;
        U(AZ, ELEVATOR_ACC_NOISE) = 1.0;
    }

    /* === 7. 协方差传播 === */
    Eigen::MatrixXd Q_step = Q_;
    // Finite over-range events must not alter the elevator propagation model.
    // Non-finite axes remain invalid and retain their uncertainty treatment.
    if (prev.in_elevator) {
        Q_step.block<3,3>(StateNoiseIndex::ACC_NOISE, StateNoiseIndex::ACC_NOISE) =
                imu_elevator_acc_noise_variance * Eigen::Matrix3d::Identity();
        Q_step.block<3,3>(StateNoiseIndex::GYRO_NOISE, StateNoiseIndex::GYRO_NOISE) =
                imu_elevator_gyro_noise_variance * Eigen::Matrix3d::Identity();
        acc_saturation_mask = (acc_saturation_mask >> 3) & 0x07;
        gyr_saturation_mask = (gyr_saturation_mask >> 3) & 0x07;
    }
    for (int axis = 0; axis < 3; ++axis) {
        const std::uint8_t bit = static_cast<std::uint8_t>(1u << axis);
        if ((gyr_saturation_mask & bit) != 0) {
            Q_step(StateNoiseIndex::GYRO_NOISE + axis,
                   StateNoiseIndex::GYRO_NOISE + axis) *= imu_saturation_process_noise_scale;
        }
        if ((acc_saturation_mask & bit) != 0) {
            Q_step(StateNoiseIndex::ACC_NOISE + axis,
                   StateNoiseIndex::ACC_NOISE + axis) *= imu_saturation_process_noise_scale;
        }
    }
    nxt.P = F * prev.P * F.transpose() + U * Q_step * U.transpose() * dt;

    return nxt;
}

void IMUProcess::propagateOutputCovariance(State &state, double dt,
                                           double noise_reference_dt) const {
    if (dt <= 0.0) return;
    using StateMatrix = Eigen::Matrix<double, StateIndex::STATE_TOTAL,
                                      StateIndex::STATE_TOTAL>;
    StateMatrix A = StateMatrix::Zero();
    A.block<3, 3>(StateIndex::R, StateIndex::R) = -skew3d(state.omg);
    A.block<3, 3>(StateIndex::R, StateIndex::OMG) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(StateIndex::P, StateIndex::V) = Eigen::Matrix3d::Identity();
    A.block<3, 3>(StateIndex::V, StateIndex::R) =
            state.q.toRotationMatrix() * -skew3d(state.acc);
    A.block<3, 3>(StateIndex::V, StateIndex::ACC) = state.q.toRotationMatrix();
    if (online_gravity_estimation_enable) {
        A.block<3, 3>(StateIndex::V, StateIndex::G) = Eigen::Matrix3d::Identity();
    }

    StateMatrix F = StateMatrix::Identity() + A * dt;
    if (state.in_elevator) {
        const double dt2 = dt * dt;
        const Eigen::Vector3d ezu = ElevatorProcess::ezu_from_g(state.g);
        F.block<3, 1>(StateIndex::P, StateIndex::AZ) = -0.5 * dt2 * ezu;
        F.block<3, 1>(StateIndex::V, StateIndex::AZ) = -dt * ezu;
        F(StateIndex::Z, StateIndex::VZ) = dt;
        F(StateIndex::Z, StateIndex::AZ) = 0.5 * dt2;
        F(StateIndex::VZ, StateIndex::AZ) = dt;
    }

    StateMatrix Q = StateMatrix::Zero();
    Q.block<3, 3>(StateIndex::V, StateIndex::V) =
            output_imu_velocity_process_variance * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(StateIndex::OMG, StateIndex::OMG) =
            output_imu_angular_rate_process_variance * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(StateIndex::ACC, StateIndex::ACC) =
            output_imu_acceleration_process_variance * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(StateIndex::BW, StateIndex::BW) =
            output_imu_gyro_bias_process_variance * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(StateIndex::BA, StateIndex::BA) =
            output_imu_accel_bias_process_variance * Eigen::Matrix3d::Identity();
    if (online_gravity_estimation_enable) {
        Q.block<3, 3>(StateIndex::G, StateIndex::G) =
                GRAVITY_NOISE_VAR * Eigen::Matrix3d::Identity();
    }
    // 避免将同一个 IMU 周期切成大量点间隔后导致过程噪声随点数变化：
    // 每个子段注入 dt_i*T，整个周期之和仍为 Point-LIO 的 T^2。
    const double noise_dt = noise_reference_dt > 0.0 ? noise_reference_dt : dt;
    state.P = F * state.P * F.transpose() + Q * (dt * noise_dt);
    if (state.in_elevator) {
        state.P(StateIndex::AZ, StateIndex::AZ) += ELEVATOR_ACC_NOISE_VAR * dt;
    }
    state.P = 0.5 * (state.P + state.P.transpose());
}

void IMUProcess::updateOutputImuMeasurement(
        State &state,
        const Eigen::Vector3d &acc,
        const Eigen::Vector3d &gyr,
        std::uint8_t acc_saturation_mask,
        std::uint8_t gyr_saturation_mask) const {
    int measurement_count = 0;
    for (int axis = 0; axis < 3; ++axis) {
        const std::uint8_t invalid_bit = static_cast<std::uint8_t>(1u << (axis + 3));
        if ((gyr_saturation_mask & invalid_bit) == 0) ++measurement_count;
        if ((acc_saturation_mask & invalid_bit) == 0) ++measurement_count;
    }
    if (measurement_count == 0) return;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(measurement_count, StateIndex::STATE_TOTAL);
    Eigen::VectorXd residual(measurement_count);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(measurement_count, measurement_count);
    int row = 0;
    for (int axis = 0; axis < 3; ++axis) {
        const std::uint8_t invalid_bit = static_cast<std::uint8_t>(1u << (axis + 3));
        if ((gyr_saturation_mask & invalid_bit) != 0) continue;
        H(row, StateIndex::OMG + axis) = 1.0;
        H(row, StateIndex::BW + axis) = 1.0;
        residual(row) = gyr(axis) - state.omg(axis) - state.bw(axis);
        R(row, row) = output_imu_gyro_measurement_variance;
        ++row;
    }
    for (int axis = 0; axis < 3; ++axis) {
        const std::uint8_t invalid_bit = static_cast<std::uint8_t>(1u << (axis + 3));
        if ((acc_saturation_mask & invalid_bit) != 0) continue;
        H(row, StateIndex::ACC + axis) = 1.0;
        H(row, StateIndex::BA + axis) = 1.0;
        residual(row) = acc(axis) - state.acc(axis) - state.ba(axis);
        R(row, row) = output_imu_accel_measurement_variance;
        ++row;
    }

    const Eigen::MatrixXd prior_covariance = state.P;
    const Eigen::MatrixXd PHt = prior_covariance * H.transpose();
    Eigen::MatrixXd innovation = H * PHt + R;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(innovation);
    if (ldlt.info() != Eigen::Success) {
        innovation.diagonal().array() += 1e-9;
        ldlt.compute(innovation);
    }
    if (ldlt.info() != Eigen::Success) return;
    Eigen::MatrixXd K = PHt * ldlt.solve(Eigen::MatrixXd::Identity(
            measurement_count, measurement_count));
    if (!online_gravity_estimation_enable) {
        K.block(StateIndex::G, 0, 3, measurement_count).setZero();
    }
    const Eigen::VectorXd dx = K * residual;
    if (!dx.allFinite()) return;

    const Eigen::Vector3d dtheta = dx.segment<3>(StateIndex::R);
    state.q = (state.q * Eigen::Quaterniond(so3Exp(dtheta))).normalized();
    state.p += dx.segment<3>(StateIndex::P);
    state.v += dx.segment<3>(StateIndex::V);
    state.omg += dx.segment<3>(StateIndex::OMG);
    state.acc += dx.segment<3>(StateIndex::ACC);
    state.bw += dx.segment<3>(StateIndex::BW);
    state.ba += dx.segment<3>(StateIndex::BA);
    if (online_gravity_estimation_enable) state.g += dx.segment<3>(StateIndex::G);
    state.z += dx(StateIndex::Z);
    state.vz += dx(StateIndex::VZ);
    state.az += dx(StateIndex::AZ);

    const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(
            StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL);
    const Eigen::MatrixXd I_KH = identity - K * H;
    state.P = I_KH * prior_covariance * I_KH.transpose() + K * R * K.transpose();
    Eigen::MatrixXd reset = identity;
    reset.block<3, 3>(StateIndex::R, StateIndex::R) -= 0.5 * skew3d(dtheta);
    state.P = reset * state.P * reset.transpose();
    state.P = 0.5 * (state.P + state.P.transpose());
}

/* --------------------------------------------------------------------
 *  IMUProcess::initialize
 *  初始化环形缓冲、初始状态、协方差、过程噪声
 * ------------------------------------------------------------------*/
void IMUProcess::initialize(double t0 )
{
    initialized = true;
    /* ---------- 0. 初始 State ---------- */
    State init;
    init.q  = Eigen::Quaterniond::Identity();
    init.p  = Eigen::Vector3d::Zero();
    init.v  = Eigen::Vector3d::Zero();
    init.omg = Eigen::Vector3d::Zero();
    init.acc = Eigen::Vector3d(0, 0, G_m_s2);
    init.bw = Eigen::Vector3d::Zero();
    init.ba = Eigen::Vector3d::Zero();
    init.g  = Eigen::Vector3d(0, 0, -G_m_s2);
    init.z = 0.0;
    init.vz = 0.0;
    init.az = 0.0;
    init.time = t0;

    /* 协方差 P (27×27) */
    init.P.setZero(StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL);
    init.P.block<3,3>(StateIndex::R,  StateIndex::R)  = Eigen::Matrix3d::Identity() * 1e-4;
    init.P.block<3,3>(StateIndex::P,  StateIndex::P)  = Eigen::Matrix3d::Identity() * 1e-2;
    init.P.block<3,3>(StateIndex::V,  StateIndex::V)  = Eigen::Matrix3d::Identity() * 1e-2;
    init.P.block<3,3>(StateIndex::OMG, StateIndex::OMG) = Eigen::Matrix3d::Identity() * 1e-2;
    init.P.block<3,3>(StateIndex::ACC, StateIndex::ACC) = Eigen::Matrix3d::Identity() * 1e-2;
    init.P.block<3,3>(StateIndex::BW, StateIndex::BW) = Eigen::Matrix3d::Identity() * 1e-6;
    init.P.block<3,3>(StateIndex::BA, StateIndex::BA) = Eigen::Matrix3d::Identity() * 1e-6;
    init.P.block<3,3>(StateIndex::G,  StateIndex::G)  = Eigen::Matrix3d::Identity() * 1e-4;
    init.P(StateIndex::Z, StateIndex::Z) = 1e-3;
    init.P(StateIndex::VZ, StateIndex::VZ) = 1e-2;
    init.P(StateIndex::AZ, StateIndex::AZ) = 1e-1;

    /* ---------- 1. 填充环形缓冲 ---------- */
    std::fill(states_imu.begin(), states_imu.end(), init);
    std::fill(time_buffer.begin(), time_buffer.end(), t0);
    std::fill(imu_acc_buffer.begin(),  imu_acc_buffer.end(),  Eigen::Vector3d::Zero());
    std::fill(imu_gyr_buffer.begin(),  imu_gyr_buffer.end(),  Eigen::Vector3d::Zero());
    std::fill(imu_acc_saturation_buffer.begin(), imu_acc_saturation_buffer.end(), 0);
    std::fill(imu_gyr_saturation_buffer.begin(), imu_gyr_saturation_buffer.end(), 0);

    /* ---------- 2. 过程噪声 Q (12×12) ---------- */
    ACC_NOISE_VAR = imu_input_acc_noise_variance;
    GYRO_NOISE_VAR = imu_input_gyro_noise_variance;
    Q_.setZero(StateNoiseIndex::NOISE_TOTAL,StateNoiseIndex::NOISE_TOTAL);
    Q_.block<3,3>(StateNoiseIndex::ACC_NOISE,        StateNoiseIndex::ACC_NOISE)         = ACC_NOISE_VAR        * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_NOISE,       StateNoiseIndex::GYRO_NOISE)        = GYRO_NOISE_VAR       * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::ACC_RANDOM_WALK,  StateNoiseIndex::ACC_RANDOM_WALK)   = ACC_RANDOM_WALK_VAR  * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK)  = GYRO_RANDOM_WALK_VAR * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GRAVITY_NOISE, StateNoiseIndex::GRAVITY_NOISE)  = GRAVITY_NOISE_VAR * Eigen::Matrix3d::Identity();
    Q_(StateNoiseIndex::ELEVATOR_ACC_NOISE, StateNoiseIndex::ELEVATOR_ACC_NOISE) = ELEVATOR_ACC_NOISE_VAR;

    LOG_INFO(Core, "IMUProcess initialized at t = " << t0 << " s");
}

/* ========= 静态 ofstream 定义 ========= */

// 环形缓冲索引映射：保证返回值在 [0, MAX_LEN)
inline int IMUProcess::wrapIndex(int idx) const {
    idx %= MAX_LEN;
    return idx < 0 ? idx + MAX_LEN : idx;
}

void IMUProcess::set_state_at_t(State &state_in, double time_stamp,
                                double covariance_time) {
    (void)covariance_time;
    constexpr double kTimeEpsilon = 1e-6;
    if (count_ <= 0) {
        cerr << "[IMUProcess] Error: cannot commit state into an empty IMU buffer." << endl;
        return;
    }

    const int oldest_index = wrapIndex(head_ - count_);
    const int latest_index = wrapIndex(head_ - 1);
    const double oldest_time = time_buffer[oldest_index];
    const double latest_time = time_buffer[latest_index];
    if (time_stamp < oldest_time - kTimeEpsilon ||
        time_stamp > latest_time + kTimeEpsilon) {
        cerr << "[IMUProcess] Error: committed state timestamp is outside the IMU buffer: "
             << std::fixed << std::setprecision(9) << time_stamp
             << " not in [" << oldest_time << ", " << latest_time << "]" << endl;
        return;
    }

    // The old implementation moved the timestamp of the preceding raw IMU
    // slot to time_stamp while retaining that slot's old measurement.  That
    // silently paired a measurement with the wrong time during the next scan.
    // Keep all raw IMU slots immutable and only rebuild states after the LiDAR
    // posterior.  If the posterior lies between IMU samples, interpolate the
    // measurement at that instant for the first integration segment.
    int exact_position = -1;
    int lower_position = -1;
    for (int position = 0; position < count_; ++position) {
        const int index = wrapIndex(head_ - count_ + position);
        const double sample_time = time_buffer[index];
        if (std::abs(sample_time - time_stamp) <= kTimeEpsilon) {
            exact_position = position;
            break;
        }
        if (sample_time < time_stamp) {
            lower_position = position;
        } else {
            break;
        }
    }

    State propagated = state_in;
    propagated.time = time_stamp;
    Eigen::Vector3d current_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_gyr = Eigen::Vector3d::Zero();
    std::uint8_t current_acc_mask = 0;
    std::uint8_t current_gyr_mask = 0;
    int next_position = -1;

    if (exact_position >= 0) {
        const int exact_index = wrapIndex(head_ - count_ + exact_position);
        states_imu[exact_index] = propagated;
        current_acc = imu_acc_buffer[exact_index];
        current_gyr = imu_gyr_buffer[exact_index];
        current_acc_mask = imu_acc_saturation_buffer[exact_index];
        current_gyr_mask = imu_gyr_saturation_buffer[exact_index];
        next_position = exact_position + 1;
    } else {
        if (lower_position < 0 || lower_position + 1 >= count_) {
            cerr << "[IMUProcess] Error: cannot bracket committed state timestamp." << endl;
            return;
        }
        const int lower_index = wrapIndex(head_ - count_ + lower_position);
        const int upper_index = wrapIndex(head_ - count_ + lower_position + 1);
        const double span = time_buffer[upper_index] - time_buffer[lower_index];
        if (span <= 0.0) {
            cerr << "[IMUProcess] Error: non-monotonic raw IMU timestamps in buffer." << endl;
            return;
        }
        const double ratio = std::clamp(
                (time_stamp - time_buffer[lower_index]) / span, 0.0, 1.0);
        current_acc = (1.0 - ratio) * imu_acc_buffer[lower_index] +
                      ratio * imu_acc_buffer[upper_index];
        current_gyr = (1.0 - ratio) * imu_gyr_buffer[lower_index] +
                      ratio * imu_gyr_buffer[upper_index];
        current_acc_mask = imu_acc_saturation_buffer[lower_index] |
                           imu_acc_saturation_buffer[upper_index];
        current_gyr_mask = imu_gyr_saturation_buffer[lower_index] |
                           imu_gyr_saturation_buffer[upper_index];
        next_position = lower_position + 1;
    }

    for (int position = next_position; position < count_; ++position) {
        const int index = wrapIndex(head_ - count_ + position);
        const bool target_mode = elevator_enable && states_imu[index].in_elevator;
        const double target_time = time_buffer[index];
        const double dt = target_time - propagated.time;
        if (dt < -kTimeEpsilon) {
            cerr << "[IMUProcess] Error: non-monotonic raw IMU timestamps during repropagation." << endl;
            return;
        }
        if (dt <= kTimeEpsilon) continue;

        const Eigen::Vector3d acc_avg = 0.5 * (current_acc + imu_acc_buffer[index]);
        const Eigen::Vector3d gyr_avg = 0.5 * (current_gyr + imu_gyr_buffer[index]);
        const std::uint8_t interval_acc_mask =
                current_acc_mask | imu_acc_saturation_buffer[index];
        const std::uint8_t interval_gyr_mask =
                current_gyr_mask | imu_gyr_saturation_buffer[index];

        const bool previous_mode = propagated.in_elevator;
        propagated = predictOnce(propagated, acc_avg, gyr_avg, dt, target_time,
                                 interval_acc_mask, interval_gyr_mask, true);

        propagated.in_elevator = target_mode;
        if (!previous_mode && target_mode) {
            elev_process.ElevatorModeEnter(propagated, acc_avg);
        }
        states_imu[index] = propagated;
        current_acc = imu_acc_buffer[index];
        current_gyr = imu_gyr_buffer[index];
        current_acc_mask = imu_acc_saturation_buffer[index];
        current_gyr_mask = imu_gyr_saturation_buffer[index];
    }
}

void IMUProcess::get_state_at_t(State &state_out, double time_stamp) {
    // 默认返回最新状态，确保任何异常路径都不会留下未初始化输出
    if (count_ <= 0) {
        cerr << "[IMUProcess] Error: state buffer is empty." << endl;
        return;
    }
    state_out = states_imu[wrapIndex(head_ - 1)];

    if (count_ < 2) {
        // 缓冲区状态太少，无法做插值，直接返回最新状态
        return;
    }

    const double latest_t = time_buffer[wrapIndex(head_ - 1)];
    const double oldest_t = time_buffer[wrapIndex(head_ - count_)];

    // 边界点直接返回，避免后续 lower_index = -1 的越界问题
    if (std::abs(time_stamp - latest_t) < 1E-6) {
        state_out = states_imu[wrapIndex(head_ - 1)];
        return;
    }
    if (std::abs(time_stamp - oldest_t) < 1E-6) {
        state_out = states_imu[wrapIndex(head_ - count_)];
        return;
    }

    // 1. 时间范围检查
    if (time_stamp >= latest_t + 1E-6 ||
        time_stamp <= oldest_t - 1E-6) {
        cerr << "[IMUProcess] Error: time_stamp is out of range." << endl;
        return;
    }

    // 2. 逆序查找第一个时间戳小于目标时间戳的索引位置
    int lower_index = -1;
    for (int i = 0; i < count_; ++i) {
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp - 1E-6 ) {
            lower_index = time_index;
            break;
        }
    }

    if (lower_index < 0) {
        // 没找到严格小于目标时，回退到最老状态，避免后续访问 -1 索引
        state_out = states_imu[wrapIndex(head_ - count_)];
        return;
    }

    // 3. 检查是否有完全重合的时间戳
    int next_index = wrapIndex(lower_index + 1);
    bool replace = std::abs(time_buffer[next_index] - time_stamp) < 1E-6;

     if (replace) {
        state_out = states_imu[next_index];
    }else {
        state_out = states_imu[next_index]; // 补丁，不特别关注的量直接复制
        int left_index = lower_index;
        int right_index = next_index;
        // 4. 计算插值比例
        double t_left = time_buffer[left_index];
        double t_right = time_buffer[right_index];
        // 5. 进行状态插值
        const State& left_state = states_imu[left_index];
        const State& right_state = states_imu[right_index];
        if (std::abs(t_right - t_left) < 1E-9) {
            state_out = right_state;
            return;
        }
        double ratio = (time_stamp - t_left) / (t_right - t_left);
        const bool target_mode = (ratio < 0.5)
                ? left_state.in_elevator : right_state.in_elevator;
        // 位置、速度、bias和重力使用线性插值
        state_out.p = (1.0 - ratio) * left_state.p + ratio * right_state.p;
        state_out.v = (1.0 - ratio) * left_state.v + ratio * right_state.v;
        state_out.omg = (1.0 - ratio) * left_state.omg + ratio * right_state.omg;
        state_out.acc = (1.0 - ratio) * left_state.acc + ratio * right_state.acc;
        state_out.ba = (1.0 - ratio) * left_state.ba + ratio * right_state.ba;
        state_out.bw = (1.0 - ratio) * left_state.bw + ratio * right_state.bw;
        state_out.g = (1.0 - ratio) * left_state.g + ratio * right_state.g;
        // 姿态使用四元数球面线性插值
        state_out.q = left_state.q.slerp(ratio, right_state.q);
        // 电梯子状态也做线性插值，避免查询状态与 p/v/q 脱节。
        state_out.z = (1.0 - ratio) * left_state.z + ratio * right_state.z;
        state_out.vz = (1.0 - ratio) * left_state.vz + ratio * right_state.vz;
        state_out.az = (1.0 - ratio) * left_state.az + ratio * right_state.az;
        // 模式标志取更接近目标时刻的一侧，避免插值过程中 mode 抖动。
        state_out.in_elevator = target_mode;
        // 协方差矩阵取较近的一侧
        state_out.P = (ratio < 0.5) ? left_state.P : right_state.P;
        state_out.time = time_stamp;
    }
}

inline Pose IMUProcess::get_pose(int index , double base_time) {
    Pose IMU_Pose;
    State state_ = states_imu[index];
    IMU_Pose.q = state_.q;
    IMU_Pose.p = state_.p;
    IMU_Pose.v = state_.v;
    Eigen::Vector3d acc_world =
            state_.q * (imu_acc_buffer[index] - state_.ba) + state_.g;
    Eigen::Vector3d gyr = imu_gyr_buffer[index] - state_.bw;
    IMU_Pose.acc = acc_world ;
    IMU_Pose.gyr = gyr;
    IMU_Pose.offset_time = time_buffer[index] - base_time;
    return IMU_Pose;
}

void IMUProcess::get_imu_pose_from_t1_to_t2(std::vector<Pose> &IMU_Pose_vec, double time_stamp_1, double time_stamp_2) {
    // 1. 清空输出容器
    IMU_Pose_vec.clear();

    // 2. 检查时间范围有效性
    if (time_stamp_1 > time_stamp_2) {
        cerr << "[IMUProcess] Warnning: time_stamp_1 is larger than time_stamp_2. Already been reversed." << endl;
        double time_temp = time_stamp_1;
        time_stamp_1 = time_stamp_2;
        time_stamp_2 = time_temp;
    }

    // 3. 检查请求时间是否在缓冲区范围内
    double min_time = time_buffer[wrapIndex(head_ - count_)];
    double max_time = time_buffer[wrapIndex(head_ - 1)];
    if (time_stamp_1 < min_time - 1E-6 || time_stamp_2 > max_time + 1E-6) {
         cerr << "[IMUProcess] Error: Requested time range is out of range." << endl;
        return;
    }

    // 4. 逆序查找时间戳小于目标时间戳的索引位置
    int left_index = -1,  right_index = -1 , i ;
    
    for (i = 0; i < count_; ++i) { // 第一次查找小于 time_stamp_2 的索引
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp_2 - 1E-6 ) {
            right_index = time_index;
            break;
        }
    }
    
    for (; i < count_; ++i) { // 第二次查找小于 time_stamp_1 的索引
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp_1 - 1E-6 ) {
            left_index = time_index;
            break;
        }
    }

    // 5. 未找到索引
    if (left_index == -1 || right_index == -1) {
        cerr << "[IMUProcess] Error: IMU deque size may be too small or time span is too large. "
        << "IMU deque size = " << MAX_LEN << " time span = " << time_stamp_2 - time_stamp_1 << endl;
        return;
    }

    // 检查左侧是否存在重合时间戳
    State state_begin;
    int next_index = wrapIndex(left_index + 1);
    bool replace = std::abs(time_buffer[next_index] - time_stamp_1) < 1E-6;

    if (replace) {
        // 添加范围内的完整状态【注意这里已经把 right_index 对应的插入了】
        for (int j = wrapIndex(left_index + 1); j != wrapIndex(right_index + 1); j = wrapIndex(j+1)) {
            IMU_Pose_vec.push_back(get_pose(j,time_stamp_1));
        }
    }else {
        // 多插值一个起始状态
        double ratio = (time_stamp_1 - time_buffer[left_index]) /
                       (time_buffer[next_index] - time_buffer[left_index]);
        const State& prev_state = states_imu[left_index];
        const State& next_state = states_imu[next_index];

        state_begin.p = (1.0 - ratio) * prev_state.p + ratio * next_state.p;
        state_begin.v = (1.0 - ratio) * prev_state.v + ratio * next_state.v;
        state_begin.omg = (1.0 - ratio) * prev_state.omg + ratio * next_state.omg;
        state_begin.acc = (1.0 - ratio) * prev_state.acc + ratio * next_state.acc;
        state_begin.ba = (1.0 - ratio) * prev_state.ba + ratio * next_state.ba;
        state_begin.bw = (1.0 - ratio) * prev_state.bw + ratio * next_state.bw;
        state_begin.g = (1.0 - ratio) * prev_state.g + ratio * next_state.g;
        state_begin.q = prev_state.q.slerp(ratio, next_state.q);
        state_begin.P = (ratio < 0.5) ? prev_state.P : next_state.P;
        state_begin.time = time_stamp_1;
        state_begin.z = (1.0 - ratio) * prev_state.z + ratio * next_state.z;
        state_begin.vz = (1.0 - ratio) * prev_state.vz + ratio * next_state.vz;
        state_begin.az = (1.0 - ratio) * prev_state.az + ratio * next_state.az;
        state_begin.in_elevator = (ratio < 0.5)
                ? prev_state.in_elevator : next_state.in_elevator;

        Eigen::Vector3d imu_acc_begin = (1.0 - ratio) * imu_acc_buffer[left_index] + ratio * imu_acc_buffer[next_index];
        Eigen::Vector3d imu_gyr_begin = (1.0 - ratio) * imu_gyr_buffer[left_index] + ratio * imu_gyr_buffer[next_index];

        Pose IMU_Pose;
        IMU_Pose.q = state_begin.q;
        IMU_Pose.p = state_begin.p;
        IMU_Pose.v = state_begin.v;
        Eigen::Vector3d acc_world =
                state_begin.q * (imu_acc_begin - state_begin.ba) + state_begin.g;
        Eigen::Vector3d gyr = imu_gyr_begin - state_begin.bw;
        IMU_Pose.acc = acc_world ;
        IMU_Pose.gyr = gyr;
        IMU_Pose.offset_time = 0;

        IMU_Pose_vec.push_back(IMU_Pose);
        for (int j = wrapIndex(left_index + 1); j != wrapIndex(right_index + 1); j = wrapIndex(j+1)) {
            IMU_Pose_vec.push_back(get_pose(j,time_stamp_1));
        }
    }

    // 检查右侧是否存在重合时间戳
    State state_end;
    next_index = wrapIndex(right_index + 1);
    replace = std::abs(time_buffer[next_index] - time_stamp_2) < 1E-6;

    if (replace) {
        IMU_Pose_vec.push_back(get_pose(next_index,time_stamp_1));
    }else {
        // 多插值一个结束状态
        double ratio = (time_stamp_2 - time_buffer[right_index]) /
                      (time_buffer[next_index] - time_buffer[right_index]);
        const State& prev_state = states_imu[right_index];
        const State& next_state = states_imu[next_index];

        state_end.p = (1.0 - ratio) * prev_state.p + ratio * next_state.p;
        state_end.v = (1.0 - ratio) * prev_state.v + ratio * next_state.v;
        state_end.omg = (1.0 - ratio) * prev_state.omg + ratio * next_state.omg;
        state_end.acc = (1.0 - ratio) * prev_state.acc + ratio * next_state.acc;
        state_end.ba = (1.0 - ratio) * prev_state.ba + ratio * next_state.ba;
        state_end.bw = (1.0 - ratio) * prev_state.bw + ratio * next_state.bw;
        state_end.g = (1.0 - ratio) * prev_state.g + ratio * next_state.g;
        state_end.q = prev_state.q.slerp(ratio, next_state.q);
        state_end.P = (ratio < 0.5) ? prev_state.P : next_state.P;
        state_end.z = (1.0 - ratio) * prev_state.z + ratio * next_state.z;
        state_end.vz = (1.0 - ratio) * prev_state.vz + ratio * next_state.vz;
        state_end.az = (1.0 - ratio) * prev_state.az + ratio * next_state.az;
        state_end.time = time_stamp_2;
        state_end.in_elevator = (ratio < 0.5)
                ? prev_state.in_elevator : next_state.in_elevator;

        Eigen::Vector3d imu_acc_end = (1.0 - ratio) * imu_acc_buffer[right_index] + ratio * imu_acc_buffer[next_index];
        Eigen::Vector3d imu_gyr_end = (1.0 - ratio) * imu_gyr_buffer[right_index] + ratio * imu_gyr_buffer[next_index];

        Pose IMU_Pose;
        IMU_Pose.q = state_end.q;
        IMU_Pose.p = state_end.p;
        IMU_Pose.v = state_end.v;
        Eigen::Vector3d acc_world =
                state_end.q * (imu_acc_end - state_end.ba) + state_end.g;
        Eigen::Vector3d gyr = imu_gyr_end - state_end.bw;
        IMU_Pose.acc = acc_world ;
        IMU_Pose.gyr = gyr;
        IMU_Pose.offset_time = time_stamp_2 - time_stamp_1;

        IMU_Pose_vec.push_back(IMU_Pose);
    }
}

bool IMUProcess::init(ImuMsgConst &msg, int init_imu_num,
                      Eigen::Vector3d &mean_acc_, Eigen::Vector3d &mean_gyr_,
                      State &state) {
    Eigen::Vector3d cur_acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d cur_gyr(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    init_acc_buf_.push_back(cur_acc);
    init_gyr_buf_.push_back(cur_gyr);

    if (init_acc_buf_.size() < init_imu_num) {
        return false;
    }

    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyr = Eigen::Vector3d::Zero();
    for (const auto &acc : init_acc_buf_) mean_acc += acc;
    for (const auto &gyr : init_gyr_buf_) mean_gyr += gyr;
    mean_acc /= init_acc_buf_.size();
    mean_gyr /= init_gyr_buf_.size();

    const double raw_acc_norm = mean_acc.norm();
    G_SCALE_UP = false;
    if (raw_acc_norm < 1.2) {
        G_SCALE_UP = true;
        mean_acc *= G_m_s2;
    }

    mean_acc_ = mean_acc;
    mean_gyr_ = mean_gyr;

    state.bw = mean_gyr_;
    double gravity_norm = mean_acc.norm();
    state.g = Eigen::Vector3d(0, 0, -gravity_norm);
    state.q = Eigen::Quaterniond::FromTwoVectors(-mean_acc, state.g);
    state.ba = Eigen::Vector3d::Zero();
    state.omg = Eigen::Vector3d::Zero();
    state.acc = mean_acc;
    state.p = Eigen::Vector3d::Zero();
    state.v = Eigen::Vector3d::Zero();
    state.z = 0.0;
    state.vz = 0.0;
    state.az = 0.0;
    state.in_elevator = false;

    state.P.setZero(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL);
    state.P.block<3, 3>(StateIndex::R, StateIndex::R) = Eigen::Matrix3d::Identity() * 1e-3;
    state.P.block<3, 3>(StateIndex::P, StateIndex::P) = Eigen::Matrix3d::Identity() * 1e-2;
    state.P.block<3, 3>(StateIndex::V, StateIndex::V) = Eigen::Matrix3d::Identity() * 1e-2;
    state.P.block<3, 3>(StateIndex::OMG, StateIndex::OMG) = Eigen::Matrix3d::Identity() * 1e-2;
    state.P.block<3, 3>(StateIndex::ACC, StateIndex::ACC) = Eigen::Matrix3d::Identity() * 1e-2;
    state.P.block<3, 3>(StateIndex::BW, StateIndex::BW) = Eigen::Matrix3d::Identity() * 1e-4;
    state.P.block<3, 3>(StateIndex::BA, StateIndex::BA) = Eigen::Matrix3d::Identity() * 1e-2;
    state.P.block<3, 3>(StateIndex::G, StateIndex::G) = Eigen::Matrix3d::Identity() * 1e-3;
    state.P(StateIndex::Z, StateIndex::Z) = 1e-3;
    state.P(StateIndex::VZ, StateIndex::VZ) = 1e-2;
    state.P(StateIndex::AZ, StateIndex::AZ) = 1e-1;

    init_acc_buf_.clear();
    init_gyr_buf_.clear();

    return true;
}
