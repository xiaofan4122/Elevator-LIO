/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Local map maintenance and point-cloud conversion helpers.
 */

#include "support/LIONode.h"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <cctype>
#include <iomanip>

extern std::shared_ptr<EskfEstimator> p_eskf_estimator;
extern vector<BoxPointType> cub_needrm;
extern std::shared_ptr<IkdMap<PointType>> ikd_map;
extern KD_TREE<PointType> ikdtree;
extern double min_filter_size;
extern bool relocation_enable;
extern string pcd_load_name;

namespace {
std::string trim_copy(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
        ++start;
    }
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
    }
    return s.substr(start, end - start);
}

std::vector<std::string> split_ws(const std::string& s) {
    std::istringstream iss(s);
    std::vector<std::string> out;
    std::string token;
    while (iss >> token) {
        out.push_back(token);
    }
    return out;
}

std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::string token;
    std::istringstream ss(s);
    while (std::getline(ss, token, ',')) {
        out.push_back(token);
    }
    return out;
}

bool parse_double(const std::string& s, double& v) {
    try {
        size_t idx = 0;
        v = std::stod(s, &idx);
        return idx == s.size();
    } catch (...) {
        return false;
    }
}

bool is_tum_line(const std::string& line) {
    auto parts = split_ws(line);
    if (parts.size() < 8) return false;
    for (size_t i = 0; i < 8; ++i) {
        double v = 0.0;
        if (!parse_double(parts[i], v)) return false;
    }
    return true;
}
} // namespace

/********************************************** Other Function **********************************************/

void convert_traj_csv_to_tum() {
    runtimeLogger().flushIfDue(true);

    std::string src_path = runtimeLogger().resolvePath(LogFileChannel::TrajectoryPost);
    if (src_path.empty()) {
        src_path = string(PACKAGE_ROOT_DIR) + "/trajectory.csv";
    }
    const std::string dst_path = string(PACKAGE_ROOT_DIR) + "/traj.tum";
    std::ifstream fin(src_path);
    if (!fin.is_open()) {
        std::cerr << "[traj_to_tum] Cannot open: " << src_path << std::endl;
        return;
    }

    std::string first_line;
    while (std::getline(fin, first_line)) {
        first_line = trim_copy(first_line);
        if (!first_line.empty()) break;
    }

    if (first_line.empty()) {
        std::cerr << "[traj_to_tum] Empty file: " << src_path << std::endl;
        return;
    }

    if (is_tum_line(first_line)) {
        fin.clear();
        fin.seekg(0, std::ios::beg);
        std::ofstream fout(dst_path);
        if (!fout.is_open()) {
            std::cerr << "[traj_to_tum] Cannot write: " << dst_path << std::endl;
            return;
        }
        std::string line;
        while (std::getline(fin, line)) {
            line = trim_copy(line);
            if (line.empty()) continue;
            fout << line << "\n";
        }
        std::cout << "[traj_to_tum] Wrote " << dst_path << std::endl;
        return;
    }

    std::vector<std::string> header = split_csv(first_line);
    std::unordered_map<std::string, size_t> idx;
    for (size_t i = 0; i < header.size(); ++i) {
        idx[trim_copy(header[i])] = i;
    }

    auto find_key = [&](const std::vector<std::string>& keys, std::string& out) -> bool {
        for (const auto& k : keys) {
            if (idx.count(k)) { out = k; return true; }
        }
        return false;
    };

    std::string t_key, x_key, y_key, z_key;
    if (!find_key({"timestamp", "t", "time"}, t_key)) {
        std::cerr << "[traj_to_tum] Missing time column.\n";
        return;
    }
    if (idx.count("x") && idx.count("y") && idx.count("z")) {
        x_key = "x"; y_key = "y"; z_key = "z";
    } else if (idx.count("p_x") && idx.count("p_y") && idx.count("p_z")) {
        x_key = "p_x"; y_key = "p_y"; z_key = "p_z";
    } else {
        std::cerr << "[traj_to_tum] Missing position columns.\n";
        return;
    }

    if (!(idx.count("qx") && idx.count("qy") && idx.count("qz") && idx.count("qw"))) {
        std::cerr << "[traj_to_tum] Missing quaternion columns.\n";
        return;
    }

    const bool has_ez = idx.count("Ez") > 0;
    std::ofstream fout(dst_path);
    if (!fout.is_open()) {
        std::cerr << "[traj_to_tum] Cannot write: " << dst_path << std::endl;
        return;
    }
    fout.setf(std::ios::fixed);
    fout << std::setprecision(6);

    std::string line;
    while (std::getline(fin, line)) {
        line = trim_copy(line);
        if (line.empty()) continue;
        std::vector<std::string> cols = split_csv(line);
        if (cols.size() < header.size()) continue;
        double t=0,x=0,y=0,z=0,qx=0,qy=0,qz=0,qw=0,ez=0;
        if (!parse_double(cols[idx[t_key]], t)) continue;
        if (!parse_double(cols[idx[x_key]], x)) continue;
        if (!parse_double(cols[idx[y_key]], y)) continue;
        if (!parse_double(cols[idx[z_key]], z)) continue;
        if (has_ez && parse_double(cols[idx["Ez"]], ez)) z += ez;
        if (!parse_double(cols[idx["qx"]], qx)) continue;
        if (!parse_double(cols[idx["qy"]], qy)) continue;
        if (!parse_double(cols[idx["qz"]], qz)) continue;
        if (!parse_double(cols[idx["qw"]], qw)) continue;
        fout << t << " " << x << " " << y << " " << z << " "
             << qx << " " << qy << " " << qz << " " << qw << "\n";
    }
    std::cout << "[traj_to_tum] Wrote " << dst_path << std::endl;
}

/**
 * @brief 根据当前位置动态移动局部地图，减少内存占用
 */
void LIONode::lasermap_fov_segment() {
    if (!ikd_map) return;
    if (p_eskf_estimator -> get_prior_states().empty()) return;
    auto states = p_eskf_estimator -> get_prior_states();
    const Eigen::Vector3d pos_lidar = states.back().p;  // world 系下lidar位置
    ikd_map->removeFar(pos_lidar);
}


void LIONode::map_incremental(PointCloudXYZI & lidar_clouds) {
    /************************** 重定位模式不更新地图 ******************************/
    if (lidar_clouds.empty()) return;
    if (relocation_enable) return ;
    if (!ikd_map) return;
    /******************************** 建图模式 *********************************/
    PointCloudXYZI::Ptr world_cloud = p_eskf_estimator->transLidar2World(lidar_clouds);
    ikd_map->add(world_cloud);
}

void init_ikd_map() {
    IkdMap<PointType>::Config map_config;
    map_config.local_map_cube_len = local_map_cube_len;
    ikd_map = std::make_shared<IkdMap<PointType>>(map_config, &ikdtree);
    ikd_map->init(min_filter_size > 0.0 ? min_filter_size : filter_size);
}

void build_kdtree_from_file() {
    if (relocation_enable) {
        PointCloudXYZI::Ptr cloud_init_world(new PointCloudXYZI);
        std::string pcd_path = string(PACKAGE_ROOT_DIR) + "/PCD/" + pcd_load_name;
        pcl::io::loadPCDFile(pcd_path, *cloud_init_world);
        PointCloudXYZI::Ptr cloud_init_world_downsampled(new PointCloudXYZI);
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
        downSizeFilter.setInputCloud(cloud_init_world);
        downSizeFilter.filter(*cloud_init_world_downsampled);
        ikd_map->add(cloud_init_world_downsampled);
        std::cout << "[build_kdtree_from_file] Relocation Map Model " << std::endl;
        std::cout << "[build_kdtree_from_file] Load Points Numble : " << cloud_init_world_downsampled->points.size() << std::endl;
    }
}
