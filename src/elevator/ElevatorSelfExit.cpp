/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 *
 * Elevator self-exit state machine implementation.
 */

#include "elevator/ElevatorSelfExit.h"
#include "elevator/ElevatorProcess.h"
#include "estimator/ESEKF.h"
#include "estimator/IMUProcess.h"

// ===================== WindowStdDev =====================

double WindowStdDev::update(double value) {
    hist.push_back(value);
    sum += value;
    sq_sum += value * value;
    if (hist.size() > kWinSize) {
        double oldest = hist.front();
        sum -= oldest;
        sq_sum -= oldest * oldest;
        hist.pop_front();
    }
    if (hist.size() == kWinSize) {
        double mean = sum / kWinSize;
        double variance = (sq_sum / kWinSize) - (mean * mean);
        return std::max(0.0, variance);
    }
    return 0.0;
}

void WindowStdDev::clear() {
    sum = 0.0;
    sq_sum = 0.0;
    hist.clear();
}

// ===================== ElevatorSelfExit =====================

ElevatorSelfExit::ElevatorSelfExit(ElevatorProcess& elev_process,
                                   std::shared_ptr<EskfEstimator> estimator)
    : elev_process_(elev_process), estimator_(std::move(estimator)) {}

ElevatorSelfExit::ProcessResult ElevatorSelfExit::process(const ImuFrame& frame, State& state) {
    ProcessResult result;
    if (!elevator_enable) {
        reset();
        return result;
    }

    const double eval_dt = (last_exit_eval_time_ > 0.0)
        ? std::max(0.0, frame.timestamp - last_exit_eval_time_)
        : 0.0;
    last_exit_eval_time_ = frame.timestamp;

    debug_.acc_z = frame.acc_z;
    debug_.vel_z = frame.vel_z;
    debug_.std_dev = rolling_std_.update(frame.vel_z);

    if (!frame.in_elevator) {
        ele_state_ = IDLE;
        stable_tick_ = 0;
        low_var_hold_s_ = 0.0;
        stop_check_lidar_zupt_count_ = 0;
        stop_check_confirmed_ = false;
        last_waiting_zupt_time_ = -std::numeric_limits<double>::infinity();
        return result;
    }

    // --- Waiting ZUPT ---
    const double kWaitingVzAbsThresh = elevator_waiting_zupt_vz_abs_thresh;
    const double kWaitingAzAbsThresh = elevator_waiting_zupt_az_abs_thresh;
    const double kWaitingZuptPeriodS = elevator_waiting_zupt_period_s;

    const bool waiting_like =
        std::abs(frame.vel_z) < kWaitingVzAbsThresh &&
        std::abs(state.az) < kWaitingAzAbsThresh;

    if (elevator_zupt_enable &&
        elevator_waiting_zupt_enable &&
        ele_state_ == IDLE &&
        waiting_like &&
        (frame.timestamp - last_waiting_zupt_time_) >= kWaitingZuptPeriodS) {
        result.apply_waiting_zupt = true;
        last_waiting_zupt_time_ = frame.timestamp;
    }

    // --- Self-exit state machine ---
    if (!self_exit_detector) return result;

    const double kVelThresh = 0.2;
    const double kStopVelProtect = 0.5;
    const double kStdHigh = 0.01;
    const double kStdLow = 0.01;
    const double kConfirmHoldS = 1.5;

    const double abs_vel = std::abs(frame.vel_z);
    const double current_stdDev = debug_.std_dev;

    switch (ele_state_) {
    case IDLE:
        if (abs_vel > kVelThresh && current_stdDev > kStdHigh) {
            ele_state_ = ACCELERATING;
            LOG_INFO(Elevator, "state: IDLE -> ACCELERATING (vel=" << abs_vel
                                 << ", var=" << current_stdDev << ")");
        }
        break;

    case ACCELERATING:
        if (abs_vel > kVelThresh && current_stdDev < kStdLow) {
            ele_state_ = CONSTANT_SPEED;
            LOG_INFO(Elevator, "state: ACCEL -> CONSTANT_SPEED");
        }
        break;

    case CONSTANT_SPEED:
        if (current_stdDev > kStdHigh) {
            ele_state_ = DECELERATING;
            LOG_INFO(Elevator, "state: CONSTANT -> DECELERATING (var spike=" << current_stdDev << ")");
        }
        break;

    case DECELERATING:
        if (current_stdDev < kStdLow && abs_vel < kStopVelProtect) {
            ele_state_ = STOP_CHECK;
            stable_tick_ = 0;
            low_var_hold_s_ = 0.0;
            stop_check_lidar_zupt_count_ = 0;
            stop_check_confirmed_ = false;
            LOG_INFO(Elevator, "state: DECEL -> STOP_CHECK");
        }
        break;

    case STOP_CHECK:
        if (current_stdDev < kStdLow && abs_vel < kStopVelProtect) {
            stable_tick_++;
            low_var_hold_s_ += eval_dt;
            if (low_var_hold_s_ >= kConfirmHoldS && !stop_check_confirmed_) {
                stop_check_confirmed_ = true;
                stop_check_lidar_zupt_count_ = 0;
                result.stop_confirmed = true;
                LOG_INFO(Elevator, "full stop confirmed, start 3x lidar ZUPT");
            }
        } else {
            stable_tick_ = 0;
            low_var_hold_s_ = 0.0;
            stop_check_confirmed_ = false;
            stop_check_lidar_zupt_count_ = 0;
            if (current_stdDev > kStdHigh) {
                ele_state_ = DECELERATING;
                LOG_INFO(Elevator, "state: re-entered DECEL logic");
            }
        }
        break;
    }

    return result;
}

void ElevatorSelfExit::reset() {
    ele_state_ = IDLE;
    stable_tick_ = 0;
    low_var_hold_s_ = 0.0;
    stop_check_lidar_zupt_count_ = 0;
    stop_check_confirmed_ = false;
    last_waiting_zupt_time_ = -std::numeric_limits<double>::infinity();
    rolling_std_.clear();
}
