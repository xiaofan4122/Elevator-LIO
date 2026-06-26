/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 */

/**
 * @file ObjectRelativeTrajectory.hpp
 * @brief 6DoF object relative trajectory in elevator car frame (SO(3) + C2 quintic, header-only)
 * @details
 * - Position: piecewise quintic (C2) per axis.
 * - Orientation: per-segment SO(3) interpolation via slerp (q_i -> q_{i+1})
 * - Angular velocity (body frame): estimated by finite difference on slerp quaternions
 * - Angular acceleration: central difference on omega (stable & accurate for IMU sim).
 *
 * JSON waypoint orientation formats (priority):
 *  1) "quat": [w,x,y,z]  (unit quaternion; will be normalized)
 *  2) "rpy":  [roll,pitch,yaw] in degrees
 *  3) "yaw":  yaw in degrees (roll=pitch=0)
 *
 * Waypoint "stop":
 *  stop=true  => enforce linear v=a=0 at that waypoint
 *  stop=false => estimate node derivatives for smooth pass-through
 *
 * Optional CSV:
 *  if object.output_csv is set (string) => export sampled relative truth CSV.
 */

#ifndef OBJECT_RELATIVE_TRAJECTORY_HPP
#define OBJECT_RELATIVE_TRAJECTORY_HPP

#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

// -------------------------
// Output state (relative to elevator car frame)
// -------------------------
struct ObjectRelState6D {
    double t = 0.0;

    // position (m)
    double x = 0.0, y = 0.0, z = 0.0;

    // velocity (m/s)
    double vx = 0.0, vy = 0.0, vz = 0.0;

    // acceleration (m/s^2)
    double ax = 0.0, ay = 0.0, az = 0.0;

    // orientation quaternion [w,x,y,z] (unit)
    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;

    // angular velocity (rad/s) in body frame
    double wx = 0.0, wy = 0.0, wz = 0.0;

    // angular acceleration (rad/s^2) in body frame
    double alphax = 0.0, alphay = 0.0, alphaz = 0.0;
};

// -------------------------
// Internal quaternion helpers
// -------------------------
struct Quat {
    double w=1, x=0, y=0, z=0; // [w,x,y,z]
};

static inline double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

static inline Quat quatNormalize(const Quat& q) {
    double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n <= 0.0) return {1,0,0,0};
    return {q.w/n, q.x/n, q.y/n, q.z/n};
}

static inline double quatDot(const Quat& a, const Quat& b) {
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline Quat quatNeg(const Quat& q) {
    return {-q.w, -q.x, -q.y, -q.z};
}

static inline Quat quatMul(const Quat& a, const Quat& b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

static inline Quat quatInv(const Quat& q) {
    // assume unit
    return {q.w, -q.x, -q.y, -q.z};
}

static inline Quat quatSlerp(const Quat& a_in, const Quat& b_in, double u) {
    Quat a = quatNormalize(a_in);
    Quat b = quatNormalize(b_in);

    double dot = quatDot(a, b);
    if (dot < 0.0) {
        b = quatNeg(b);
        dot = -dot;
    }

    const double kEps = 1e-8;
    if (dot > 1.0 - kEps) {
        Quat q{
            a.w + u*(b.w - a.w),
            a.x + u*(b.x - a.x),
            a.y + u*(b.y - a.y),
            a.z + u*(b.z - a.z)
        };
        return quatNormalize(q);
    }

    double theta = std::acos(clampd(dot, -1.0, 1.0));
    double sin_theta = std::sin(theta);
    double w1 = std::sin((1.0 - u) * theta) / sin_theta;
    double w2 = std::sin(u * theta) / sin_theta;

    Quat q{
        w1*a.w + w2*b.w,
        w1*a.x + w2*b.x,
        w1*a.y + w2*b.y,
        w1*a.z + w2*b.z
    };
    return quatNormalize(q);
}

// Euler ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll)
static inline Quat quatFromRPY_ZYX(double roll_rad, double pitch_rad, double yaw_rad) {
    double cr = std::cos(roll_rad*0.5),  sr = std::sin(roll_rad*0.5);
    double cp = std::cos(pitch_rad*0.5), sp = std::sin(pitch_rad*0.5);
    double cy = std::cos(yaw_rad*0.5),   sy = std::sin(yaw_rad*0.5);

    Quat q;
    q.w = cy*cp*cr + sy*sp*sr;
    q.x = cy*cp*sr - sy*sp*cr;
    q.y = cy*sp*cr + sy*cp*sr;
    q.z = sy*cp*cr - cy*sp*sr;
    return quatNormalize(q);
}

// SO(3) Exp: rotation vector phi -> quaternion
static inline Quat so3Exp(const double phi[3]) {
    double th = std::sqrt(phi[0]*phi[0] + phi[1]*phi[1] + phi[2]*phi[2]);
    if (th < 1e-12) {
        // small-angle approx
        return quatNormalize({1.0, 0.5*phi[0], 0.5*phi[1], 0.5*phi[2]});
    }
    double half = 0.5 * th;
    double s = std::sin(half) / th;
    return quatNormalize({std::cos(half), s*phi[0], s*phi[1], s*phi[2]});
}

// SO(3) Log: quaternion -> rotation vector (principal, theta in [0,pi])
static inline void so3Log(const Quat& q_in, double phi[3]) {
    Quat q = quatNormalize(q_in);
    double w = clampd(q.w, -1.0, 1.0);
    double vnorm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z);

    if (vnorm < 1e-12) {
        phi[0]=phi[1]=phi[2]=0.0;
        return;
    }
    double th = 2.0 * std::atan2(vnorm, w);
    double k = th / vnorm;
    phi[0] = k*q.x;
    phi[1] = k*q.y;
    phi[2] = k*q.z;
}

static inline void so3Hat(const double a[3], double A[3][3]) {
    A[0][0]=0.0;    A[0][1]=-a[2];  A[0][2]= a[1];
    A[1][0]= a[2];  A[1][1]=0.0;    A[1][2]=-a[0];
    A[2][0]=-a[1];  A[2][1]= a[0];  A[2][2]=0.0;
}

static inline void matMul3(const double A[3][3], const double B[3][3], double C[3][3]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        C[i][j]=0.0;
        for (int k=0;k<3;k++) C[i][j]+=A[i][k]*B[k][j];
    }
}

static inline void matVec3(const double A[3][3], const double v[3], double out[3]) {
    for (int i=0;i<3;i++) {
        out[i] = A[i][0]*v[0] + A[i][1]*v[1] + A[i][2]*v[2];
    }
}

// Right Jacobian inverse Jr^{-1}(phi) (SO(3))
// omega_body = Jr^{-1}(phi) * phidot
static inline void so3RightJacInv(const double phi[3], double JrInv[3][3]) {
    // init identity
    JrInv[0][0]=JrInv[1][1]=JrInv[2][2]=1.0;
    JrInv[0][1]=JrInv[0][2]=JrInv[1][0]=JrInv[1][2]=JrInv[2][0]=JrInv[2][1]=0.0;

    double th = std::sqrt(phi[0]*phi[0] + phi[1]*phi[1] + phi[2]*phi[2]);
    double A[3][3], A2[3][3];
    so3Hat(phi, A);
    matMul3(A, A, A2);

    if (th < 1e-8) {
        // series: Jr^{-1} ≈ I + 0.5*A + 1/12*A^2
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
            JrInv[i][j] += 0.5*A[i][j] + (1.0/12.0)*A2[i][j];
        }
        return;
    }

    double th2 = th*th;
    double half = 0.5*th;
    double sin_half = std::sin(half);
    double cos_half = std::cos(half);
    double cot_half = cos_half / sin_half;

    // b = (1/th^2) * (1 - 0.5*th*cot(th/2))
    double b = (1.0/th2) * (1.0 - 0.5*th*cot_half);

    for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
        JrInv[i][j] += 0.5*A[i][j] + b*A2[i][j];
    }
}

// -------------------------
// Quintic segment (scalar)
// p(tau)=c0+c1*tau+...+c5*tau^5, tau in [0,T]
// -------------------------
struct QuinticCoeffs {
    double c0=0,c1=0,c2=0,c3=0,c4=0,c5=0;
    double T=0;
};

static inline QuinticCoeffs makeQuintic(double p0, double v0, double a0,
                                       double p1, double v1, double a1,
                                       double T) {
    QuinticCoeffs q;
    q.T = T;

    if (T <= 0.0) {
        q.c0 = p0;
        q.c1=q.c2=q.c3=q.c4=q.c5=0.0;
        return q;
    }

    double T2 = T*T;
    double T3 = T2*T;
    double T4 = T3*T;
    double T5 = T4*T;

    q.c0 = p0;
    q.c1 = v0;
    q.c2 = 0.5*a0;

    q.c3 = (20.0*(p1 - p0) - (8.0*v1 + 12.0*v0)*T - (3.0*a0 - a1)*T2) / (2.0*T3);
    q.c4 = (30.0*(p0 - p1) + (14.0*v1 + 16.0*v0)*T + (3.0*a0 - 2.0*a1)*T2) / (2.0*T4);
    q.c5 = (12.0*(p1 - p0) - (6.0*v1 + 6.0*v0)*T - (a0 - a1)*T2) / (2.0*T5);

    return q;
}

static inline void evalQuintic(const QuinticCoeffs& q, double tau,
                               double& p, double& v, double& a) {
    if (tau < 0.0) tau = 0.0;
    if (tau > q.T) tau = q.T;

    double t = tau;
    double t2 = t*t;
    double t3 = t2*t;
    double t4 = t3*t;
    double t5 = t4*t;

    p = q.c0 + q.c1*t + q.c2*t2 + q.c3*t3 + q.c4*t4 + q.c5*t5;
    v = q.c1 + 2.0*q.c2*t + 3.0*q.c3*t2 + 4.0*q.c4*t3 + 5.0*q.c5*t4;
    a = 2.0*q.c2 + 6.0*q.c3*t + 12.0*q.c4*t2 + 20.0*q.c5*t3;
}

// -------------------------
// Internal waypoint and segment
// -------------------------
struct ObjectWaypoint6D {
    double t=0.0;
    double x=0.0,y=0.0,z=0.0;
    Quat q;       // unit
    bool stop=false;
};

struct ObjectRelSegment6D {
    double t_start=0.0, t_end=0.0;

    QuinticCoeffs x,y,z;

    // orientation endpoints for slerp
    Quat q_start, q_end;

    bool contains(double t) const { return (t >= t_start && t < t_end); }
};

// -------------------------
// Trajectory class
// -------------------------
class ObjectRelativeTrajectory {
public:
    ObjectRelativeTrajectory() = default;

    bool loadConfig(const json& config) {
        wps_.clear();
        segs_.clear();
        total_duration_ = 0.0;
        output_csv_.clear();

        if (!config.contains("object")) {
            std::cerr << "[ObjectRel] Missing 'object' in config.\n";
            return false;
        }
        const auto& obj = config["object"];

        if (obj.contains("output_csv") && obj["output_csv"].is_string()) {
            output_csv_ = obj["output_csv"].get<std::string>();
        }

        if (!obj.contains("waypoints") || !obj["waypoints"].is_array() || obj["waypoints"].empty()) {
            std::cerr << "[ObjectRel] Missing or empty 'object.waypoints'.\n";
            return false;
        }

        // parse waypoints
        for (const auto& w : obj["waypoints"]) {
            ObjectWaypoint6D wp;
            wp.t = w.value("time", 0.0);
            wp.x = w.value("x", 0.0);
            wp.y = w.value("y", 0.0);
            wp.z = w.value("z", 0.0);
            wp.stop = w.value("stop", false);

            wp.q = parseOrientation(w);
            wps_.push_back(wp);
        }

        // sanity: strictly increasing time
        for (size_t i=1;i<wps_.size();++i) {
            if (!(wps_[i].t > wps_[i-1].t)) {
                std::cerr << "[ObjectRel] Waypoint times must be strictly increasing.\n";
                return false;
            }
        }

        // hemisphere fix to avoid q and -q flips
        for (size_t i=1;i<wps_.size();++i) {
            if (quatDot(wps_[i].q, wps_[i-1].q) < 0.0) {
                wps_[i].q = quatNeg(wps_[i].q);
            }
        }

        buildSegments();

        total_duration_ = wps_.back().t;
        std::cout << "[ObjectRel] 6DoF trajectory built. Total duration: " << total_duration_ << " s\n";
        return true;
    }

    double getTotalDuration() const { return total_duration_; }

    ObjectRelState6D getState(double t) const {
        ObjectRelState6D s;
        if (wps_.size() < 2 || segs_.empty()) {
            s.t = t;
            return s;
        }

        // clamp with hold behavior (before start / after end)
        if (t <= wps_.front().t) {
            s = evalAtTime(wps_.front().t);
            s.t = t;
            zeroRates(s);
            return s;
        }
        if (t >= wps_.back().t) {
            s = evalAtTime(wps_.back().t);
            s.t = t;
            zeroRates(s);
            return s;
        }

        s = evalAtTime(t);
        return s;
    }

    std::vector<ObjectRelState6D> sample(double freq, double duration_buffer = 0.0) const {
        std::vector<ObjectRelState6D> out;
        if (freq <= 0.0) return out;

        double dt = 1.0 / freq;
        double T = total_duration_ + std::max(0.0, duration_buffer);
        long N = static_cast<long>(std::ceil(T / dt)) + 1;
        out.reserve(static_cast<size_t>(N));

        ObjectRelState6D end_state = getState(total_duration_);
        zeroRates(end_state);

        for (long i=0;i<N;++i) {
            double t = i * dt;
            if (t <= total_duration_) {
                out.push_back(getState(t));
            } else {
                ObjectRelState6D s = end_state;
                s.t = t;
                out.push_back(s);
            }
        }
        return out;
    }

    std::vector<ObjectRelState6D> sampleFromConfig(const json& config) const {
        double freq = 200.0;
        double buffer = 0.0;
        if (config.contains("simulation")) {
            const auto& sim = config["simulation"];
            freq = sim.value("frequency", 200.0);
            buffer = sim.value("duration_buffer", 0.0);
        }
        return sample(freq, buffer);
    }

    // CSV writer (always includes header)
    static bool saveToCSV(const std::vector<ObjectRelState6D>& states, const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[ObjectRel] Error: Could not open " << filename << " for writing.\n";
            return false;
        }

        file << std::fixed << std::setprecision(6);
        file << "timestamp,rel_x,rel_y,rel_z,rel_vx,rel_vy,rel_vz,rel_ax,rel_ay,rel_az,"
                "qw,qx,qy,qz,wx,wy,wz,alphax,alphay,alphaz\n";

        for (const auto& s : states) {
            file << s.t << ","
                 << s.x << "," << s.y << "," << s.z << ","
                 << s.vx << "," << s.vy << "," << s.vz << ","
                 << s.ax << "," << s.ay << "," << s.az << ","
                 << s.qw << "," << s.qx << "," << s.qy << "," << s.qz << ","
                 << s.wx << "," << s.wy << "," << s.wz << ","
                 << s.alphax << "," << s.alphay << "," << s.alphaz
                 << "\n";
        }

        std::cout << "[ObjectRel] Saved " << states.size() << " rows to " << filename << "\n";
        return true;
    }

    bool exportObjectCSVIfConfigured(const json& config) const {
        if (output_csv_.empty()) {
            std::cout << "[ObjectRel] object.output_csv not set, skip object CSV export.\n";
            return true;
        }
        auto states = sampleFromConfig(config);
        return saveToCSV(states, output_csv_);
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kDeg2Rad = kPi / 180.0;

    std::vector<ObjectWaypoint6D> wps_;
    std::vector<ObjectRelSegment6D> segs_;
    double total_duration_ = 0.0;
    std::string output_csv_;

    static inline void zeroRates(ObjectRelState6D& s) {
        s.vx=s.vy=s.vz=0.0;
        s.ax=s.ay=s.az=0.0;
        s.wx=s.wy=s.wz=0.0;
        s.alphax=s.alphay=s.alphaz=0.0;
    }

    static inline Quat parseOrientation(const json& wpt) {
        // priority: quat > rpy > yaw
        if (wpt.contains("quat") && wpt["quat"].is_array() && wpt["quat"].size() == 4) {
            Quat q;
            q.w = wpt["quat"][0].get<double>();
            q.x = wpt["quat"][1].get<double>();
            q.y = wpt["quat"][2].get<double>();
            q.z = wpt["quat"][3].get<double>();
            return quatNormalize(q);
        }
        if (wpt.contains("rpy") && wpt["rpy"].is_array() && wpt["rpy"].size() == 3) {
            double roll_deg  = wpt["rpy"][0].get<double>();
            double pitch_deg = wpt["rpy"][1].get<double>();
            double yaw_deg   = wpt["rpy"][2].get<double>();
            return quatFromRPY_ZYX(roll_deg*kDeg2Rad, pitch_deg*kDeg2Rad, yaw_deg*kDeg2Rad);
        }
        double yaw_deg = wpt.value("yaw", 0.0);
        return quatFromRPY_ZYX(0.0, 0.0, yaw_deg*kDeg2Rad);
    }

    // estimate node derivatives for one scalar array p[i]
    static void estimateNodeDerivatives(const std::vector<double>& t,
                                        const std::vector<double>& p,
                                        const std::vector<bool>& stop,
                                        std::vector<double>& v_out,
                                        std::vector<double>& a_out) {
        const size_t N = p.size();
        v_out.assign(N, 0.0);
        a_out.assign(N, 0.0);
        if (N < 2) return;

        // velocity estimate
        for (size_t i=0;i<N;++i) {
            if (stop[i]) { v_out[i]=0.0; continue; }
            if (i==0) {
                double dt = t[1]-t[0];
                v_out[i] = (dt>0.0)? (p[1]-p[0])/dt : 0.0;
            } else if (i==N-1) {
                double dt = t[N-1]-t[N-2];
                v_out[i] = (dt>0.0)? (p[N-1]-p[N-2])/dt : 0.0;
            } else {
                double dt = t[i+1]-t[i-1];
                v_out[i] = (dt>0.0)? (p[i+1]-p[i-1])/dt : 0.0;
            }
        }

        // acceleration estimate
        for (size_t i=0;i<N;++i) {
            if (stop[i]) { a_out[i]=0.0; continue; }
            if (i==0 || i==N-1) {
                a_out[i]=0.0;
            } else {
                double dt_prev = t[i]-t[i-1];
                double dt_next = t[i+1]-t[i];
                if (dt_prev<=0.0 || dt_next<=0.0) { a_out[i]=0.0; continue; }
                double s_prev = (p[i]-p[i-1])/dt_prev;
                double s_next = (p[i+1]-p[i])/dt_next;
                a_out[i] = 2.0*(s_next - s_prev)/(dt_prev + dt_next);
            }
        }
    }

    void buildSegments() {
        const size_t N = wps_.size();
        if (N < 2) return;

        // times & stop flags
        std::vector<double> t(N);
        std::vector<bool> stop(N);
        std::vector<double> px(N), py(N), pz(N);

        for (size_t i=0;i<N;++i) {
            t[i]=wps_[i].t;
            stop[i]=wps_[i].stop;
            px[i]=wps_[i].x;
            py[i]=wps_[i].y;
            pz[i]=wps_[i].z;
        }

        // node derivatives (linear)
        std::vector<double> vx, ax, vy, ay, vz, az;
        estimateNodeDerivatives(t, px, stop, vx, ax);
        estimateNodeDerivatives(t, py, stop, vy, ay);
        estimateNodeDerivatives(t, pz, stop, vz, az);

        // build segments
        segs_.clear();
        segs_.reserve(N-1);

        for (size_t i=0;i+1<N;++i) {
            double T = t[i+1]-t[i];
            ObjectRelSegment6D seg;
            seg.t_start = t[i];
            seg.t_end   = t[i+1];

            seg.x = makeQuintic(px[i], vx[i], ax[i], px[i+1], vx[i+1], ax[i+1], T);
            seg.y = makeQuintic(py[i], vy[i], ay[i], py[i+1], vy[i+1], ay[i+1], T);
            seg.z = makeQuintic(pz[i], vz[i], az[i], pz[i+1], vz[i+1], az[i+1], T);

            seg.q_start = wps_[i].q;
            seg.q_end = wps_[i+1].q;

            segs_.push_back(seg);
        }
    }

    Quat evalQuatAtTime(double t) const {
        if (wps_.size() < 2 || segs_.empty()) {
            return {1,0,0,0};
        }
        if (t <= wps_.front().t) return wps_.front().q;
        if (t >= wps_.back().t) return wps_.back().q;

        const ObjectRelSegment6D* segp = nullptr;
        for (const auto& seg : segs_) {
            if (seg.contains(t)) { segp = &seg; break; }
        }
        if (!segp) segp = &segs_.back();

        double T = segp->t_end - segp->t_start;
        double u = (T > 0.0) ? (t - segp->t_start) / T : 0.0;
        u = clampd(u, 0.0, 1.0);
        return quatSlerp(segp->q_start, segp->q_end, u);
    }

    // Evaluate pos/vel/acc and orientation omega at time t (within [t0, tN])
    ObjectRelState6D evalAtTime(double t) const {
        // find segment
        const ObjectRelSegment6D* segp = nullptr;
        for (const auto& seg : segs_) {
            if (seg.contains(t)) { segp = &seg; break; }
        }
        if (!segp) {
            // edge: use last
            segp = &segs_.back();
        }
        const auto& seg = *segp;
        double tau = t - seg.t_start;

        ObjectRelState6D s;
        s.t = t;

        evalQuintic(seg.x, tau, s.x, s.vx, s.ax);
        evalQuintic(seg.y, tau, s.y, s.vy, s.ay);
        evalQuintic(seg.z, tau, s.z, s.vz, s.az);

        // quaternion: per-segment slerp
        Quat qt = evalQuatAtTime(t);
        s.qw = qt.w; s.qx = qt.x; s.qy = qt.y; s.qz = qt.z;

        // omega body: finite difference on slerp
        double omega[3];
        evalOmegaOnly(t, omega);
        s.wx = omega[0]; s.wy = omega[1]; s.wz = omega[2];

        // alpha (body): central difference on omega (stable)
        // Use a small eps, and clamp inside trajectory range.
        const double eps = 1e-4;
        double t0 = wps_.front().t;
        double tN = wps_.back().t;

        double tp = std::min(tN, t + eps);
        double tm = std::max(t0, t - eps);

        double op[3], om[3];
        evalOmegaOnly(tp, op);
        evalOmegaOnly(tm, om);

        double dt = tp - tm;
        if (dt > 0.0) {
            s.alphax = (op[0] - om[0]) / dt;
            s.alphay = (op[1] - om[1]) / dt;
            s.alphaz = (op[2] - om[2]) / dt;
        } else {
            s.alphax = s.alphay = s.alphaz = 0.0;
        }

        return s;
    }

    void evalOmegaOnly(double t, double omega_out[3]) const {
        if (wps_.size() < 2 || segs_.empty()) {
            omega_out[0]=omega_out[1]=omega_out[2]=0.0;
            return;
        }

        double t0 = wps_.front().t;
        double tN = wps_.back().t;
        double ta = clampd(t, t0, tN);
        double dt = 1e-4;
        double tb = (ta + dt <= tN) ? (ta + dt) : (ta - dt);
        if (tb < t0 || tb > tN || tb == ta) {
            omega_out[0]=omega_out[1]=omega_out[2]=0.0;
            return;
        }

        Quat qa = evalQuatAtTime(ta);
        Quat qb = evalQuatAtTime(tb);
        Quat dq = quatMul(quatInv(qa), qb);

        double phi[3];
        so3Log(dq, phi);
        double inv_dt = 1.0 / (tb - ta);
        omega_out[0] = phi[0] * inv_dt;
        omega_out[1] = phi[1] * inv_dt;
        omega_out[2] = phi[2] * inv_dt;
    }
};

#endif // OBJECT_RELATIVE_TRAJECTORY_HPP
