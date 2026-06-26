/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 *
 * Power-law controller for adaptive voxel size selection.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

// Lightweight adaptive voxel size controller based on a power-law model.
// v_{t+1} = v_t * (N_t / N_target)^(1/alpha), with clamping.
class AdaptiveVoxelPController {
public:
    struct Config {
        double target_points = 2500.0;  // N_target
        double alpha = 1.5;            // effective dimension, > 0
        double min_voxel = 0.02;        // lower bound
        double max_voxel = 0.8;         // upper bound
    };

    AdaptiveVoxelPController() = default;

    explicit AdaptiveVoxelPController(double initial_voxel)
        : cfg_(), voxel_(initial_voxel) {
        clamp_state_();
    }

    AdaptiveVoxelPController(double initial_voxel, const Config& cfg)
        : cfg_(cfg), voxel_(initial_voxel) {
        clamp_state_();
    }

    void setConfig(const Config& cfg) {
        cfg_ = cfg;
        clamp_state_();
    }

    const Config& config() const { return cfg_; }

    void setVoxel(double v) {
        voxel_ = v;
        clamp_state_();
    }

    double voxel() const { return voxel_; }

    // Update voxel size given current downsampled point count N_t.
    // Returns the next voxel size v_{t+1}.
    double update(std::size_t n_points) {
        if (cfg_.alpha <= 0.0 || cfg_.target_points <= 0.0) {
            return voxel_;
        }
        const double ratio = static_cast<double>(n_points) / cfg_.target_points;
        const double scale = std::pow(ratio, 1.0 / cfg_.alpha);
        voxel_ = voxel_ * scale;
        clamp_state_();
        return voxel_;
    }

private:
    void clamp_state_() {
        if (cfg_.alpha <= 0.0) {
            cfg_.alpha = 1.0;
        }
        if (cfg_.target_points <= 0.0) {
            cfg_.target_points = 1.0;
        }
        if (cfg_.min_voxel <= 0.0) {
            cfg_.min_voxel = 0.001;
        }
        if (cfg_.max_voxel < cfg_.min_voxel) {
            cfg_.max_voxel = cfg_.min_voxel;
        }
        voxel_ = std::clamp(voxel_, cfg_.min_voxel, cfg_.max_voxel);
    }

    Config cfg_;
    double voxel_ = 0.1;
};
