//
// Created by yczhang on 25-5-20.
//

#include "estimator/IMUProcess.h"
#include "support/common_lib.h"
#include <iostream>
#include "estimator/ESEKF.h"
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

    /* ---------- 1. 写入环形缓冲 ---------- */
    imu_acc_buffer[head_]  = std::move(IMU_acc);
    imu_gyr_buffer[head_]  = std::move(IMU_gyr);
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

        /* 2.2 梯形法求平均量 */
        Eigen::Vector3d acc_avg = 0.5 * (imu_acc_buffer[prev] + imu_acc_buffer[head_]);
        Eigen::Vector3d gyr_avg = 0.5 * (imu_gyr_buffer[prev] + imu_gyr_buffer[head_]);
       // Eigen::Vector3d acc_avg = 0.5 * (imu_acc_buffer[prev] + IMU_acc);
       // Eigen::Vector3d gyr_avg = 0.5 * (imu_gyr_buffer[prev] + IMU_gyr);


        /* 2.3 预测下一状态 */
        State state_next = predictOnce(states_imu[prev] , acc_avg , gyr_avg, dt, time_stamp);

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

        if (!elevator_enable) {
            ELEVATOR_TRIGGER = false;
            EXIT_FROM_ELEVATOR = false;
        }
        state_next.in_elevator = elevator_enable && ELEVATOR_TRIGGER;

        if (!states_imu[prev].in_elevator & state_next.in_elevator) {
            elev_process.ElevatorModeEnter(state_next, acc_avg);
        }

        if (states_imu[prev].in_elevator & !state_next.in_elevator) {
            // elev_process.applyZeroVelocityUpdateZ(state_next);
            // cout << "========= applyZeroVelocityUpdateZ =========" << endl;
            EXIT_FROM_ELEVATOR = true;
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

    /* ---------- 过程噪声 Q (12×12) ---------- */
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
    if (G_SCALE_UP) cur_acc = cur_acc * G_m_s2;
    double time_stamp = imu_msg->header.stamp.toSec();
    integration(cur_acc, cur_gyr, time_stamp);
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
                              double                  stamp) const
{
    /* === 0. 复制旧状态，准备写新量 === */
    State nxt = prev;      // bias、g 等直接拷贝
    nxt.time = stamp;      // 最后统一写入

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
        if (elevator_strong_prior_enable) {
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

    /* --- U (18×15) --- */
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
    nxt.P = F * prev.P * F.transpose() + U * Q_ * U.transpose() * dt;

    return nxt;
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
    init.bw = Eigen::Vector3d::Zero();
    init.ba = Eigen::Vector3d::Zero();
    init.g  = Eigen::Vector3d(0, 0, -G_m_s2);
    init.z = 0.0;
    init.vz = 0.0;
    init.az = 0.0;
    init.time = t0;

    /* 协方差 P (18×18) */
    init.P.setZero(StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL);
    init.P.block<3,3>(StateIndex::R,  StateIndex::R)  = Eigen::Matrix3d::Identity() * 1e-4;
    init.P.block<3,3>(StateIndex::P,  StateIndex::P)  = Eigen::Matrix3d::Identity() * 1e-2;
    init.P.block<3,3>(StateIndex::V,  StateIndex::V)  = Eigen::Matrix3d::Identity() * 1e-2;
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

    /* ---------- 2. 过程噪声 Q (12×12) ---------- */
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

void IMUProcess::set_state_at_t(State &state_in, double time_stamp) {
    if (time_stamp > time_buffer[wrapIndex(head_ - 1)])
        cerr  << "[IMUProcess] Error: time_stamp is larger than the latest time stamp in the buffer." << endl;

    // 1. 查找插入位置和插入模式
    int index = -1;
    // 逆序查找第一个时间戳小于目标时间戳的索引位置
    int lower_index = -1;
    for (int i = 0; i < count_; ++i) {
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp - 1E-6 ) {
            lower_index = time_index;
            break;
        }
    }

    // 如果所有时间戳都大于目标时间戳，则插入在第一个位置，同时警告
    if (lower_index == -1) {
        index = wrapIndex(head_ - count_ );
        cerr << "[IMUProcess] Warning: timestamp was pushed to the front of the queue, which means the previous"
                "imu may have be aborted. assert_time_stamp : "
             << std::fixed << std::setprecision(6) << time_stamp
             << " The earliest time : "
             << std::fixed << std::setprecision(6) << time_buffer[index] << endl;
    }

    // 3. 检查是否有完全重合的时间戳
    int next_index = wrapIndex(lower_index + 1);
    bool replace = std::abs(time_buffer[next_index] - time_stamp) < 1E-6;

    if (replace) {
        index = next_index;
    }else {
        index = lower_index;
    }

    // 2. 更新目标位置的状态和时间戳
    states_imu[index] = state_in;
    time_buffer[index] = time_stamp;

    // 3. 从目标位置开始重新积分后续状态
    for (int i = 1; wrapIndex(index + i) != head_; ++i) {
        int index_ = wrapIndex(index + i);
        int pre_index_ = wrapIndex(index + i - 1);
        double dt = time_buffer[index_] - time_buffer[pre_index_];


        /* 2.2 梯形法求平均量 */
        Eigen::Vector3d acc_avg = 0.5 * (imu_acc_buffer[pre_index_] + imu_acc_buffer[index_]);
        Eigen::Vector3d gyr_avg = 0.5 * (imu_gyr_buffer[pre_index_] + imu_gyr_buffer[index_]);

        // 使用预测函数计算下一个状态
        states_imu[index_] = predictOnce(states_imu[pre_index_], acc_avg, gyr_avg, dt, time_buffer[index_]);
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
        // 位置、速度、bias和重力使用线性插值
        state_out.p = (1.0 - ratio) * left_state.p + ratio * right_state.p;
        state_out.v = (1.0 - ratio) * left_state.v + ratio * right_state.v;
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
        state_out.in_elevator = (ratio < 0.5) ? left_state.in_elevator : right_state.in_elevator;
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
    Eigen::Vector3d acc_world = state_.q * (imu_acc_buffer[index] - state_.ba) + state_.g;
    Eigen::Vector3d gyr = imu_gyr_buffer[index];
    IMU_Pose.acc = acc_world ;
    IMU_Pose.gyr = gyr - state_.bw;
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
        state_begin.ba = (1.0 - ratio) * prev_state.ba + ratio * next_state.ba;
        state_begin.bw = (1.0 - ratio) * prev_state.bw + ratio * next_state.bw;
        state_begin.g = (1.0 - ratio) * prev_state.g + ratio * next_state.g;
        state_begin.q = prev_state.q.slerp(ratio, next_state.q);
        state_begin.P = (ratio < 0.5) ? prev_state.P : next_state.P;
        state_begin.time = time_stamp_1;
        state_begin.z = (1.0 - ratio) * prev_state.z + ratio * next_state.z;
        state_begin.vz = (1.0 - ratio) * prev_state.vz + ratio * next_state.vz;
        state_begin.az = (1.0 - ratio) * prev_state.az + ratio * next_state.az;

        Eigen::Vector3d imu_acc_begin = (1.0 - ratio) * imu_acc_buffer[left_index] + ratio * imu_acc_buffer[next_index];
        Eigen::Vector3d imu_gyr_begin = (1.0 - ratio) * imu_gyr_buffer[left_index] + ratio * imu_gyr_buffer[next_index];

        Pose IMU_Pose;
        IMU_Pose.q = state_begin.q;
        IMU_Pose.p = state_begin.p;
        IMU_Pose.v = state_begin.v;
        Eigen::Vector3d acc_world = state_begin.q * (imu_acc_begin - state_begin.ba) + state_begin.g;
        Eigen::Vector3d gyr = imu_gyr_begin;
        IMU_Pose.acc = acc_world ;
        IMU_Pose.gyr = gyr - state_begin.bw;
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
        IMU_Pose_vec.push_back(get_pose(next_index,time_stamp_2));
    }else {
        // 多插值一个结束状态
        double ratio = (time_stamp_2 - time_buffer[right_index]) /
                      (time_buffer[next_index] - time_buffer[right_index]);
        const State& prev_state = states_imu[right_index];
        const State& next_state = states_imu[next_index];

        state_end.p = (1.0 - ratio) * prev_state.p + ratio * next_state.p;
        state_end.v = (1.0 - ratio) * prev_state.v + ratio * next_state.v;
        state_end.ba = (1.0 - ratio) * prev_state.ba + ratio * next_state.ba;
        state_end.bw = (1.0 - ratio) * prev_state.bw + ratio * next_state.bw;
        state_end.g = (1.0 - ratio) * prev_state.g + ratio * next_state.g;
        state_end.q = prev_state.q.slerp(ratio, next_state.q);
        state_end.P = (ratio < 0.5) ? prev_state.P : next_state.P;
        state_end.z = (1.0 - ratio) * prev_state.z + ratio * next_state.z;
        state_end.vz = (1.0 - ratio) * prev_state.vz + ratio * next_state.vz;
        state_end.az = (1.0 - ratio) * prev_state.az + ratio * next_state.az;
        state_end.time = time_stamp_2;

        Eigen::Vector3d imu_acc_end = (1.0 - ratio) * imu_acc_buffer[right_index] + ratio * imu_acc_buffer[next_index];
        Eigen::Vector3d imu_gyr_end = (1.0 - ratio) * imu_gyr_buffer[right_index] + ratio * imu_gyr_buffer[next_index];

        Pose IMU_Pose;
        IMU_Pose.q = state_end.q;
        IMU_Pose.p = state_end.p;
        IMU_Pose.v = state_end.v;
        Eigen::Vector3d acc_world = state_end.q * (imu_acc_end - state_end.ba) + state_end.g;
        Eigen::Vector3d gyr = imu_gyr_end;
        IMU_Pose.acc = acc_world ;
        IMU_Pose.gyr = gyr - state_end.bw;
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

    G_SCALE_UP = false;
    if (mean_acc.norm() < 1.2) {
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
