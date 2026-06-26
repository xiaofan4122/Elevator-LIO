/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 *
 * Adaptive voxel-grid filtering helper.
 */

#ifndef ADAPTIVE_VOXEL_FILTER_HPP
#define ADAPTIVE_VOXEL_FILTER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include "support/YamlReader.h"

template <typename PointT>
class AdaptiveVoxelFilter {
public:
    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

    AdaptiveVoxelFilter() = default;

    // 初始化 YAML 中的自适应滤波配置。
    void setConfiguration(const AdaptiveConfig& config, double default_leaf_size, double blind = 0.1) {
        config_ = config;
        default_leaf_size_ = default_leaf_size;
        blind_sq_ = blind * blind;

        filters_.resize(config_.regions.size() + 1);
    }

    void setInputCloud(const PointCloudPtr& input) {
        input_cloud_ = input;
    }

    void filter(pcl::PointCloud<PointT>& output) {
        if (!input_cloud_ || input_cloud_->empty()) {
            output.clear();
            return;
        }

        if (!config_.enabled || config_.regions.empty()) {
            pcl::VoxelGrid<PointT> temp_filter;
            temp_filter.setLeafSize(default_leaf_size_, default_leaf_size_, default_leaf_size_);
            temp_filter.setInputCloud(input_cloud_);
            temp_filter.filter(output);
            return;
        }

        // 区域数 + 1 个桶，最后一个桶收纳超出配置范围的点。
        size_t num_buckets = config_.regions.size() + 1;
        if (bucket_clouds_.size() != num_buckets) {
            bucket_clouds_.resize(num_buckets);
            for (auto& ptr : bucket_clouds_) {
                ptr.reset(new pcl::PointCloud<PointT>());
            }
        }

        for (auto& ptr : bucket_clouds_) ptr->clear();

        for (const auto& pt : input_cloud_->points) {
            float range2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;

            if (range2 < blind_sq_) continue;

            bool placed = false;
            for (size_t i = 0; i < config_.regions.size(); ++i) {
                float max_dist_sq = config_.regions[i].max_dist * config_.regions[i].max_dist;
                if (range2 < max_dist_sq) {
                    bucket_clouds_[i]->push_back(pt);
                    placed = true;
                    break;
                }
            }
            if (!placed) {
                bucket_clouds_.back()->push_back(pt);
            }
        }

        // C. 分别滤波并合并 (Filtering & Merging)
        output.clear(); // 清空输出

        // 预估输出大小，减少 resize 开销
        output.reserve(input_cloud_->size() / 2);

        pcl::PointCloud<PointT> temp_filtered;

        for (size_t i = 0; i < num_buckets; ++i) {
            if (bucket_clouds_[i]->empty()) continue;

            // 确定当前的 leaf size
            float leaf = default_leaf_size_;
            if (i < config_.regions.size()) {
                leaf = config_.regions[i].leaf_size;
            }

            // 执行滤波
            filters_[i].setLeafSize(leaf, leaf, leaf);
            filters_[i].setInputCloud(bucket_clouds_[i]);
            filters_[i].filter(temp_filtered);

            // 合并
            output += temp_filtered;
        }
    }

private:
    PointCloudPtr input_cloud_;
    AdaptiveConfig config_;
    double default_leaf_size_ = 0.5;
    double blind_sq_ = 0.01;

    // 缓存容器，避免内存抖动
    std::vector<PointCloudPtr> bucket_clouds_;
    std::vector<pcl::VoxelGrid<PointT>> filters_;
};

#endif // ADAPTIVE_VOXEL_FILTER_HPP
