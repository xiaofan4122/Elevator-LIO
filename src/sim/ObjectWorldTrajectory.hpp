/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 */

/**
 * @file ObjectWorldTrajectory.hpp
 * @brief Compose world-frame object truth from elevator z-motion + 6DoF object relative trajectory.
 * @details
 * Frames:
 *  - World frame W: right-handed, +Z up.
 *  - Elevator car frame C: attached to the cabin. Without tilt, C aligned with W.
 *  - Body frame B: attached to the object (IMU body).
 *
 * Inputs:
 *  - ElevatorTrajectory -> ElevatorState: p(z), v(z), a(z) in world.
 *  - ObjectRelativeTrajectory -> ObjectRelState6D: p,v,a in car frame C; q_CB; omega_B (body); alpha_B (body).
 *  - Optional constant cabin tilt: elevator.tilt_pitch_deg about +Y of C. Define q_WC (constant).
 *
 * Composition:
 *  - p_W = [0,0,z_e] + R_WC * p_C
 *  - v_W = [0,0,v_e] + R_WC * v_C
 *  - a_W = [0,0,a_e] + R_WC * a_C
 *  - q_WB = q_WC ⊗ q_CB
 *
 * Angular:
 *  - Because q_WC is constant => omega_WC = 0, alpha_WC = 0.
 *  - Therefore the object's body angular velocity/acceleration are the same as relative trajectory output (in body).
 *    (If you later make tilt time-varying, you'll need to add coupling terms.)
 *
 * Optional CSV:
 *  - if object.world_output_csv is set => export sampled world truth CSV.
 */

#ifndef OBJECT_WORLD_TRAJECTORY_HPP
#define OBJECT_WORLD_TRAJECTORY_HPP

#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "ElevatorTrajectory.hpp"
#include "ObjectRelativeTrajectory.hpp" // upgraded 6DoF version

// -------------------------
// World-frame truth state
// -------------------------
struct ObjectWorldState6D {
    double t = 0.0;

    // world position (m)
    double x = 0.0, y = 0.0, z = 0.0;

    // world velocity (m/s)
    double vx = 0.0, vy = 0.0, vz = 0.0;

    // world acceleration (m/s^2) (pure motion; gravity not included)
    double ax = 0.0, ay = 0.0, az = 0.0;

    // orientation quaternion q_WB [w,x,y,z] (unit)
    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;

    // angular velocity (rad/s) in body frame
    double wx = 0.0, wy = 0.0, wz = 0.0;

    // angular acceleration (rad/s^2) in body frame
    double alphax = 0.0, alphay = 0.0, alphaz = 0.0;

    // optional debug columns
    double elev_z = 0.0, elev_vz = 0.0, elev_az = 0.0;
    double rel_x = 0.0, rel_y = 0.0, rel_z = 0.0;
};

// -------------------------
// Minimal quaternion + vector rotation helpers
// -------------------------
struct QuatW {
    double w=1,x=0,y=0,z=0;
};

static inline QuatW quatNormalizeW(const QuatW& q) {
    double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n <= 0.0) return {1,0,0,0};
    return {q.w/n, q.x/n, q.y/n, q.z/n};
}

static inline QuatW quatMulW(const QuatW& a, const QuatW& b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

static inline QuatW quatInvW(const QuatW& q) {
    return {q.w, -q.x, -q.y, -q.z}; // unit assumed
}

// rotate vector v by quaternion q: v' = R(q) * v
static inline void quatRotateVec(const QuatW& q_in,
                                double vx, double vy, double vz,
                                double& ox, double& oy, double& oz) {
    QuatW q = quatNormalizeW(q_in);

    // Using: v' = q * [0,v] * q^{-1}
    QuatW vq{0.0, vx, vy, vz};
    QuatW qi = quatInvW(q);
    QuatW tmp = quatMulW(q, vq);
    QuatW out = quatMulW(tmp, qi);
    ox = out.x; oy = out.y; oz = out.z;
}

static inline QuatW quatFromPitchY(double pitch_rad) {
    double half = 0.5 * pitch_rad;
    return quatNormalizeW({std::cos(half), 0.0, std::sin(half), 0.0}); // rotation about +Y
}

// -------------------------
// ObjectWorldTrajectory
// -------------------------
class ObjectWorldTrajectory {
public:
    struct Params {
        double tilt_pitch_deg = 0.0;  // constant cabin tilt about +Y
        std::string output_csv;       // object.world_output_csv (optional)
    };

    ObjectWorldTrajectory() = default;

    bool loadConfig(const json& config) {
        params_ = Params{};
        if (config.contains("perturbations") &&
            config["perturbations"].contains("cabin_inclination_error")) {
            const auto& c = config["perturbations"]["cabin_inclination_error"];
            params_.tilt_pitch_deg = c.value("pitch_deg", 0.0);
        }
        if (config.contains("elevator")) {
            const auto& e = config["elevator"];
            if (!(config.contains("perturbations") &&
                  config["perturbations"].contains("cabin_inclination_error"))) {
                params_.tilt_pitch_deg = e.value("tilt_pitch_deg", 0.0);
            }
        }
        if (config.contains("object")) {
            const auto& o = config["object"];
            if (o.contains("world_output_csv") && o["world_output_csv"].is_string()) {
                params_.output_csv = o["world_output_csv"].get<std::string>();
            }
        }

        // precompute constant q_WC
        double pitch_rad = params_.tilt_pitch_deg * kDeg2Rad;
        q_WC_ = quatFromPitchY(pitch_rad);

        return true;
    }

    ObjectWorldState6D getState(double t,
                                const ElevatorTrajectory& elev,
                                const ObjectRelativeTrajectory& objRel) const {
        ObjectWorldState6D s;
        s.t = t;

        // elevator truth (world z motion)
        ElevatorState e = elev.getState(t);
        s.elev_z  = e.p;
        s.elev_vz = e.v;
        s.elev_az = e.a;

        // object relative truth (car frame)
        ObjectRelState6D r = objRel.getState(t);
        s.rel_x = r.x; s.rel_y = r.y; s.rel_z = r.z;

        // rotate relative p/v/a from car frame to world via constant q_WC
        double prx, pry, prz;
        double vrx, vry, vrz;
        double arx, ary, arz;

        quatRotateVec(q_WC_, r.x,  r.y,  r.z,  prx, pry, prz);
        quatRotateVec(q_WC_, r.vx, r.vy, r.vz, vrx, vry, vrz);
        quatRotateVec(q_WC_, r.ax, r.ay, r.az, arx, ary, arz);

        // compose translation
        s.x = prx;
        s.y = pry;
        s.z = e.p + prz;

        s.vx = vrx;
        s.vy = vry;
        s.vz = e.v + vrz;

        s.ax = arx;
        s.ay = ary;
        s.az = e.a + arz;

        // compose orientation: q_WB = q_WC ⊗ q_CB
        QuatW q_CB{r.qw, r.qx, r.qy, r.qz};
        QuatW q_WB = quatNormalizeW(quatMulW(q_WC_, q_CB));

        s.qw = q_WB.w; s.qx = q_WB.x; s.qy = q_WB.y; s.qz = q_WB.z;

        // angular: because q_WC constant => omega/alpha in body unchanged
        s.wx = r.wx; s.wy = r.wy; s.wz = r.wz;
        s.alphax = r.alphax; s.alphay = r.alphay; s.alphaz = r.alphaz;

        return s;
    }

    std::vector<ObjectWorldState6D> sampleFromConfig(const json& config,
                                                     const ElevatorTrajectory& elev,
                                                     const ObjectRelativeTrajectory& objRel) const {
        double freq = 200.0;
        double buffer = 0.0;
        if (config.contains("simulation")) {
            const auto& sim = config["simulation"];
            freq = sim.value("frequency", 200.0);
            buffer = sim.value("duration_buffer", 0.0);
        }
        return sample(freq, buffer, elev, objRel);
    }

    std::vector<ObjectWorldState6D> sample(double freq, double duration_buffer,
                                           const ElevatorTrajectory& elev,
                                           const ObjectRelativeTrajectory& objRel) const {
        std::vector<ObjectWorldState6D> out;
        if (freq <= 0.0) return out;

        double T = std::max(elev.getTotalDuration(), objRel.getTotalDuration());
        double dt = 1.0 / freq;
        double Tout = T + std::max(0.0, duration_buffer);

        long N = static_cast<long>(std::ceil(Tout / dt)) + 1;
        out.reserve(static_cast<size_t>(N));

        ObjectWorldState6D end_state = getState(T, elev, objRel);
        // hold pose during buffer; zero rates
        end_state.vx=end_state.vy=end_state.vz=0.0;
        end_state.ax=end_state.ay=end_state.az=0.0;
        end_state.wx=end_state.wy=end_state.wz=0.0;
        end_state.alphax=end_state.alphay=end_state.alphaz=0.0;

        for (long i=0;i<N;++i) {
            double t = i * dt;
            if (t <= T) {
                out.push_back(getState(t, elev, objRel));
            } else {
                ObjectWorldState6D s = end_state;
                s.t = t;
                out.push_back(s);
            }
        }
        return out;
    }

    static bool saveToCSV(const std::vector<ObjectWorldState6D>& states, const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[ObjectWorld] Error: Could not open " << filename << " for writing.\n";
            return false;
        }

        file << std::fixed << std::setprecision(6);
        file << "timestamp,"
                "w_x,w_y,w_z,w_vx,w_vy,w_vz,w_ax,w_ay,w_az,"
                "qw,qx,qy,qz,wx,wy,wz,alphax,alphay,alphaz,"
                "elev_z,elev_vz,elev_az,rel_x,rel_y,rel_z\n";

        for (const auto& s : states) {
            file << s.t << ","
                 << s.x << "," << s.y << "," << s.z << ","
                 << s.vx << "," << s.vy << "," << s.vz << ","
                 << s.ax << "," << s.ay << "," << s.az << ","
                 << s.qw << "," << s.qx << "," << s.qy << "," << s.qz << ","
                 << s.wx << "," << s.wy << "," << s.wz << ","
                 << s.alphax << "," << s.alphay << "," << s.alphaz << ","
                 << s.elev_z << "," << s.elev_vz << "," << s.elev_az << ","
                 << s.rel_x << "," << s.rel_y << "," << s.rel_z
                 << "\n";
        }

        std::cout << "[ObjectWorld] Saved " << states.size() << " rows to " << filename << "\n";
        return true;
    }

    static std::string toTumPath(const std::string& csv_path) {
        const std::string suffix = ".csv";
        if (csv_path.size() >= suffix.size() &&
            csv_path.compare(csv_path.size() - suffix.size(), suffix.size(), suffix) == 0) {
            return csv_path.substr(0, csv_path.size() - suffix.size()) + ".tum";
        }
        return csv_path + ".tum";
    }

    static bool saveToTUM(const std::vector<ObjectWorldState6D>& states, const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[ObjectWorld] Error: Could not open " << filename << " for writing.\n";
            return false;
        }

        file << std::fixed << std::setprecision(6);
        for (const auto& s : states) {
            // TUM: t tx ty tz qx qy qz qw
            file << s.t << " "
                 << s.x << " " << s.y << " " << s.z << " "
                 << s.qx << " " << s.qy << " " << s.qz << " " << s.qw
                 << "\n";
        }

        std::cout << "[ObjectWorld] Saved " << states.size() << " rows to " << filename << "\n";
        return true;
    }

    bool exportWorldCSVIfConfigured(const json& config,
                                    const ElevatorTrajectory& elev,
                                    const ObjectRelativeTrajectory& objRel) const {
        if (params_.output_csv.empty()) {
            std::cout << "[ObjectWorld] object.world_output_csv not set, skip world CSV export.\n";
            return true;
        }
        auto states = sampleFromConfig(config, elev, objRel);
        bool ok_csv = saveToCSV(states, params_.output_csv);
        bool ok_tum = saveToTUM(states, toTumPath(params_.output_csv));
        return ok_csv && ok_tum;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kDeg2Rad = kPi / 180.0;

    Params params_;
    QuatW q_WC_{1,0,0,0}; // constant cabin rotation from C to W
};

#endif // OBJECT_WORLD_TRAJECTORY_HPP
