/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Elevator self-exit state machine: IDLE → ACCELERATING → CONSTANT_SPEED
 * → DECELERATING → STOP_CHECK, plus waiting-ZUPT and debug logging.
 * Extracted from timer_2000HZ_callback in main.cpp.
 */

#ifndef ELEVATOR_SELF_EXIT_H
#define ELEVATOR_SELF_EXIT_H

#include "support/type.h"
#include "support/common_lib.h"
#include <limits>
#include <deque>

class ElevatorProcess;
class EskfEstimator;

struct WindowStdDev {
    size_t kWinSize;
    double sum = 0.0;
    double sq_sum = 0.0;
    std::deque<double> hist;

    explicit WindowStdDev(size_t window_size) : kWinSize(window_size) {}
    double update(double value);
    void clear();
};

class ElevatorSelfExit {
public:
    struct ImuFrame {
        double timestamp = 0.0;
        double acc_z = 0.0;
        double vel_z = 0.0;
        bool in_elevator = false;
    };

    struct DebugInfo {
        double acc_z = 0.0;
        double vel_z = 0.0;
        double std_dev = 0.0;
        int state = 0;
    };

    struct ProcessResult {
        bool apply_waiting_zupt = false;
        bool stop_confirmed = false;
    };

    ElevatorSelfExit(ElevatorProcess& elev_process, std::shared_ptr<EskfEstimator> estimator);

    ProcessResult process(const ImuFrame& frame, State& state);

    void reset();

    int eleState() const { return static_cast<int>(ele_state_); }
    bool stopCheckConfirmed() const { return stop_check_confirmed_; }
    int stopCheckLidarZuptCount() const { return stop_check_lidar_zupt_count_; }
    void setStopCheckLidarZuptCount(int c) { stop_check_lidar_zupt_count_ = c; }
    const DebugInfo& debugInfo() const { return debug_; }

private:
    enum Phase { IDLE, ACCELERATING, CONSTANT_SPEED, DECELERATING, STOP_CHECK };

    ElevatorProcess& elev_process_;
    std::shared_ptr<EskfEstimator> estimator_;

    Phase ele_state_ = IDLE;
    int stop_check_lidar_zupt_count_ = 0;
    bool stop_check_confirmed_ = false;

    int stable_tick_ = 0;
    double low_var_hold_s_ = 0.0;
    double last_exit_eval_time_ = -1.0;
    double last_waiting_zupt_time_ = -std::numeric_limits<double>::infinity();

    WindowStdDev rolling_std_{300};
    DebugInfo debug_;
};

#endif // ELEVATOR_SELF_EXIT_H
