/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 */

/**
 * @file ElevatorTrajectory.hpp
 * @brief Elevator kinematic trajectory generator (analytical, header-only)
 * @details
 * - Generates ground-truth states along Z axis: position/velocity/acceleration (no gravity).
 * - MOVE uses 3-phase profile: quintic S-curve ramp + cruise + mirrored decel,
 *   matching the planning logic in your ElevatorIMUGenerator.
 * - Optional CSV export for intermediate elevator truth:
 *   If config["elevator"]["output_csv"] exists (string), export truth CSV.
 *
 * JSON contract (relevant parts):
 * {
 *   "simulation": {
 *     "frequency": 200,
 *     "duration_buffer": 2.0,
 *     "output_file": "imu.csv"   // reserved for final IMU, elevator won't use it
 *   },
 *   "elevator": {
 *     "max_velocity": 2.0,
 *     "max_acceleration": 1.0,
 *     "output_csv": "elevator_truth.csv",   // optional: if present => export
 *     "sequence": [
 *        { "type": "WAIT", "duration": 5.0 },
 *        { "type": "MOVE", "delta_height": 12.0 }
 *     ]
 *   }
 * }
 */

#ifndef ELEVATOR_TRAJECTORY_HPP
#define ELEVATOR_TRAJECTORY_HPP

#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <string>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

// -------------------------
// State (Z-axis truth)
// -------------------------
struct ElevatorState {
    double t;   // timestamp (s)
    double p;   // position z (m)
    double v;   // velocity z (m/s)
    double a;   // acceleration z (m/s^2), pure motion (no gravity)
};

// -------------------------
// Segment types
// -------------------------
enum SegmentType { STATIC, MOVE };

// MOVE profile parameters
struct MoveProfile {
    int dir = +1;           // +1 up, -1 down
    double dist = 0.0;      // |delta_height|
    double v_cruise = 0.0;  // >= 0
    double t_ramp = 0.0;
    double t_cruise = 0.0;
    double d_ramp = 0.0;    // 0.5 * v_cruise * t_ramp
    double d_cruise = 0.0;  // v_cruise * t_cruise
    double total_time() const { return 2.0 * t_ramp + t_cruise; }
};

struct TrajectorySegment {
    SegmentType type = STATIC;
    double t_start = 0.0;
    double duration = 0.0;

    double p_start = 0.0;
    double height_change = 0.0; // only valid for MOVE

    MoveProfile move;           // only valid for MOVE

    double t_end() const { return t_start + duration; }
};

// -------------------------
// ElevatorTrajectory
// -------------------------
class ElevatorTrajectory {
private:
    std::vector<TrajectorySegment> segments_;
    double total_duration_ = 0.0;

    // physical limits
    double max_vel_ = 2.0;
    double max_acc_ = 1.0;

    static inline int signum(double x) { return (x >= 0.0) ? +1 : -1; }

    // Quintic minimum-jerk S-curve (same shape as your IMUGenerator):
    // S(tau)  = 10τ^3 - 15τ^4 + 6τ^5
    // dS/dtau = 30τ^2 - 60τ^3 + 30τ^4
    // I(tau)  = ∫0^τ S(u)du = 2.5τ^4 - 3τ^5 + τ^6
    static inline void s_curve(double tau, double& S, double& dS, double& I) {
        if (tau < 0.0) tau = 0.0;
        if (tau > 1.0) tau = 1.0;

        double t2 = tau * tau;
        double t3 = t2 * tau;
        double t4 = t3 * tau;
        double t5 = t4 * tau;
        double t6 = t5 * tau;

        S  = 10.0 * t3 - 15.0 * t4 +  6.0 * t5;
        dS = 30.0 * t2 - 60.0 * t3 + 30.0 * t4;
        I  =  2.5 * t4 -  3.0 * t5 +  1.0 * t6;
    }

    // Planning logic EXACTLY matches ElevatorIMUGenerator:
    // min_ramp_time = 1.875 * Vmax / Amax
    // dist_ramp_needed = 0.5 * Vmax * min_ramp_time
    // if short distance => triangle profile (no cruise):
    //   v_cruise = sqrt(dist * Amax / 1.875)
    //   t_ramp = 1.875 * v_cruise / Amax
    // else trapezoid in velocity with quintic ramps:
    //   t_cruise = (dist - 2*dist_ramp_needed) / v_cruise
    MoveProfile planMoveProfile(double delta_h) const {
        MoveProfile mp;
        mp.dir  = signum(delta_h);
        mp.dist = std::abs(delta_h);

        if (mp.dist <= 0.0) return mp;

        double min_ramp_time = 1.875 * max_vel_ / max_acc_;
        double dist_ramp_needed = 0.5 * max_vel_ * min_ramp_time;

        mp.v_cruise = max_vel_;
        mp.t_ramp   = min_ramp_time;
        mp.t_cruise = 0.0;

        if (2.0 * dist_ramp_needed > mp.dist) {
            // triangle (no cruise)
            mp.v_cruise = std::sqrt(mp.dist * max_acc_ / 1.875);
            mp.t_ramp   = 1.875 * mp.v_cruise / max_acc_;
            mp.t_cruise = 0.0;
        } else {
            double dist_cruise = mp.dist - 2.0 * dist_ramp_needed;
            mp.t_cruise = dist_cruise / mp.v_cruise;
        }

        mp.d_ramp   = 0.5 * mp.v_cruise * mp.t_ramp;
        mp.d_cruise = mp.v_cruise * mp.t_cruise;
        return mp;
    }

public:
    ElevatorTrajectory() = default;

    // Build timeline segments from JSON
    bool loadConfig(const json& config) {
        segments_.clear();
        total_duration_ = 0.0;

        if (!config.contains("elevator")) {
            std::cerr << "[Elevator] Missing 'elevator' in config.\n";
            return false;
        }

        const auto& e_conf = config["elevator"];
        max_vel_ = e_conf.value("max_velocity", 2.0);
        max_acc_ = e_conf.value("max_acceleration", 1.0);

        if (!e_conf.contains("sequence")) {
            std::cerr << "[Elevator] Missing 'elevator.sequence' in config.\n";
            return false;
        }

        double current_t = 0.0;
        double current_p = 0.0;

        for (const auto& item : e_conf["sequence"]) {
            std::string type = item.value("type", "WAIT");

            TrajectorySegment seg;
            seg.t_start = current_t;
            seg.p_start = current_p;

            if (type == "WAIT") {
                seg.type = STATIC;
                seg.duration = item.value("duration", 1.0);
                seg.height_change = 0.0;

                current_t += seg.duration;
            } else if (type == "MOVE") {
                seg.type = MOVE;
                seg.height_change = item.value("delta_height", 0.0);

                seg.move = planMoveProfile(seg.height_change);
                seg.duration = seg.move.total_time();

                current_t += seg.duration;
                current_p += seg.height_change;
            } else {
                std::cerr << "[Elevator] Unknown segment type: " << type << "\n";
                return false;
            }

            segments_.push_back(seg);
        }

        total_duration_ = current_t;
        std::cout << "[Elevator] Trajectory built. Total duration: " << total_duration_ << " s\n";
        return true;
    }

    // Analytical state query at time t
    ElevatorState getState(double t) const {
        if (segments_.empty()) return {t, 0.0, 0.0, 0.0};

        // clamp boundaries
        if (t <= 0.0) {
            return {0.0, segments_.front().p_start, 0.0, 0.0};
        }
        if (t >= total_duration_) {
            const auto& last = segments_.back();
            double final_pos = last.p_start + last.height_change;
            return {t, final_pos, 0.0, 0.0};
        }

        // find segment (linear scan; can be optimized later)
        const TrajectorySegment* segp = nullptr;
        for (const auto& seg : segments_) {
            if (t >= seg.t_start && t < seg.t_end()) { segp = &seg; break; }
        }
        if (!segp) return {t, 0.0, 0.0, 0.0};

        const auto& seg = *segp;
        double local_t = t - seg.t_start;

        if (seg.type == STATIC) {
            return {t, seg.p_start, 0.0, 0.0};
        }

        // MOVE: 3-phase analytical solution
        const auto& mp = seg.move;
        const double dir = static_cast<double>(mp.dir);

        const double t1 = mp.t_ramp;
        const double t2 = mp.t_ramp + mp.t_cruise;
        const double T  = mp.total_time();

        if (local_t < 0.0) local_t = 0.0;
        if (local_t > T)   local_t = T;

        double p = seg.p_start;
        double v = 0.0;
        double a = 0.0;

        if (local_t < t1) {
            // Phase 1: accel
            double tau = (mp.t_ramp > 0.0) ? (local_t / mp.t_ramp) : 0.0;
            double S, dS, I;
            s_curve(tau, S, dS, I);

            v = dir * mp.v_cruise * S;
            a = dir * mp.v_cruise * dS / mp.t_ramp;
            p = seg.p_start + dir * (mp.v_cruise * mp.t_ramp * I);
        } else if (local_t < t2) {
            // Phase 2: cruise
            double tc = local_t - mp.t_ramp;
            v = dir * mp.v_cruise;
            a = 0.0;
            p = seg.p_start + dir * (mp.d_ramp + mp.v_cruise * tc);
        } else {
            // Phase 3: decel (mirror)
            double td = local_t - (mp.t_ramp + mp.t_cruise);
            double tau = (mp.t_ramp > 0.0) ? (td / mp.t_ramp) : 1.0;
            double S, dS, I;
            s_curve(tau, S, dS, I);

            v = dir * mp.v_cruise * (1.0 - S);
            a = dir * (-mp.v_cruise * dS / mp.t_ramp);

            double decel_dp = mp.v_cruise * mp.t_ramp * (tau - I);
            p = seg.p_start + dir * (mp.d_ramp + mp.d_cruise + decel_dp);
        }

        return {t, p, v, a};
    }

    double getTotalDuration() const { return total_duration_; }

    // Uniform sampling
    std::vector<ElevatorState> sample(double freq, double duration_buffer = 0.0) const {
        std::vector<ElevatorState> out;
        if (freq <= 0.0) return out;

        double dt = 1.0 / freq;
        double T = total_duration_ + std::max(0.0, duration_buffer);
        long N = static_cast<long>(std::ceil(T / dt)) + 1;

        out.reserve(static_cast<size_t>(N));

        // end-state for buffer (hold still)
        ElevatorState end_state = getState(total_duration_);

        for (long i = 0; i < N; ++i) {
            double t = i * dt;
            if (t <= total_duration_) {
                out.push_back(getState(t));
            } else {
                ElevatorState s = end_state;
                s.t = t;
                s.v = 0.0;
                s.a = 0.0;
                out.push_back(s);
            }
        }
        return out;
    }

    // sampling parameters (freq/buffer) come from simulation block
    std::vector<ElevatorState> sampleFromConfig(const json& config) const {
        double freq = 200.0;
        double buffer = 0.0;

        if (config.contains("simulation")) {
            const auto& s = config["simulation"];
            freq = s.value("frequency", 200.0);
            buffer = s.value("duration_buffer", 0.0);
        }
        return sample(freq, buffer);
    }

    // CSV writer (always includes header)
    static bool saveToCSV(const std::vector<ElevatorState>& states,
                          const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[Elevator] Error: Could not open " << filename << " for writing.\n";
            return false;
        }

        file << std::fixed << std::setprecision(6);
        file << "timestamp,pos_z,vel_z,acc_z\n";

        for (const auto& s : states) {
            file << s.t << "," << s.p << "," << s.v << "," << s.a << "\n";
        }

        std::cout << "[Elevator] Saved " << states.size() << " rows to " << filename << "\n";
        return true;
    }

    // Export elevator truth CSV only if elevator.output_csv exists (string)
    bool exportElevatorCSVIfConfigured(const json& config) const {
        if (!config.contains("elevator")) {
            std::cerr << "[Elevator] Missing 'elevator' in config.\n";
            return false;
        }

        const auto& e = config["elevator"];
        if (!e.contains("output_csv")) {
            // not configured => do nothing
            std::cout << "[Elevator] elevator.output_csv not set, skip elevator CSV export.\n";
            return true;
        }

        if (!e["output_csv"].is_string()) {
            std::cerr << "[Elevator] elevator.output_csv must be a string path, e.g. \"elevator_truth.csv\".\n";
            return false;
        }

        std::string out = e["output_csv"].get<std::string>();
        auto states = sampleFromConfig(config);
        return saveToCSV(states, out);
    }
};

#endif // ELEVATOR_TRAJECTORY_HPP
