/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 */

#include "support/YamlReader.h"
#include <utility>
#include <vector>

// explicit instantiations
template bool YamlReader::read<std::string>(const std::string &, std::string &);
template bool YamlReader::read<int>(const std::string &, int &);
template bool YamlReader::read<double>(const std::string &, double &);
template bool YamlReader::read<float>(const std::string &, float &);
template bool YamlReader::read<bool>(const std::string &, bool &);
template bool YamlReader::read<std::vector<int>>(const std::string &, std::vector<int> &);
template bool YamlReader::readOptional<std::string>(const std::string &, std::string &);
template bool YamlReader::readOptional<int>(const std::string &, int &);
template bool YamlReader::readOptional<double>(const std::string &, double &);
template bool YamlReader::readOptional<float>(const std::string &, float &);
template bool YamlReader::readOptional<bool>(const std::string &, bool &);
template bool YamlReader::readOptional<std::vector<int>>(const std::string &, std::vector<int> &);
template bool YamlReader::readEigen<Eigen::Vector2d>(const std::string &, Eigen::MatrixBase<Eigen::Vector2d> &);
template bool YamlReader::readEigen<Eigen::Vector3d>(const std::string &, Eigen::MatrixBase<Eigen::Vector3d> &);
template bool YamlReader::readEigen<Eigen::Vector4d>(const std::string &, Eigen::MatrixBase<Eigen::Vector4d> &);
template bool YamlReader::readEigen<Eigen::Matrix3d>(const std::string &, Eigen::MatrixBase<Eigen::Matrix3d> &);
template bool YamlReader::readEigen<Eigen::Matrix4d>(const std::string &, Eigen::MatrixBase<Eigen::Matrix4d> &);
// You can continue to add more explicit instantiations as needed.


namespace {
bool mergeDisjointYaml(YAML::Node target, const YAML::Node& source,
                       const std::string& source_file, const std::string& prefix = "") {
    if (!source.IsMap()) {
        std::cerr << "YAML config must contain a mapping at its root: " << source_file << std::endl;
        return false;
    }

    for (const auto& entry : source) {
        const std::string key = entry.first.as<std::string>();
        const std::string full_key = prefix.empty() ? key : prefix + "." + key;
        const YAML::Node incoming = entry.second;
        const YAML::Node existing = target[key];

        if (!existing) {
            target[key] = incoming;
            continue;
        }

        if (existing.IsMap() && incoming.IsMap()) {
            if (!mergeDisjointYaml(existing, incoming, source_file, full_key)) {
                return false;
            }
            continue;
        }

        std::cerr << "Duplicate YAML key is not allowed: " << full_key
                  << " | file: " << source_file << std::endl;
        return false;
    }
    return true;
}
}  // namespace

YamlReader::YamlReader(std::string filePath) : filePaths_{std::move(filePath)} {}

YamlReader::YamlReader(std::vector<std::string> filePaths)
    : filePaths_(std::move(filePaths)) {}

bool YamlReader::load() {
    rootNode_ = YAML::Node(YAML::NodeType::Map);
    try {
        for (const auto& file_path : filePaths_) {
            const YAML::Node file_node = YAML::LoadFile(file_path);
            if (!mergeDisjointYaml(rootNode_, file_node, file_path)) {
                fileLoaded_ = false;
                return false;
            }
        }
        fileLoaded_ = true;
        return true;
    } catch (const YAML::BadFile &e) {
        std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
        fileLoaded_ = false;
        return false;
    } catch (const YAML::Exception &e) {
        std::cerr << "Failed to parse YAML configuration: " << e.what() << std::endl;
        fileLoaded_ = false;
        return false;
    }
}

template <typename T>
bool YamlReader::read(const std::string &key, T &value) {
    YAML::Node node;
    if (!getValueByKey(key, node)) {
        std::cerr << "Failed to find key: " << key << std::endl;
        return false;
    }

    try {
        value = node.as<T>();
        return true;
    } catch (const YAML::BadConversion &e) {
        std::cerr << "Type conversion error for key '" << key << "': " << e.what() << std::endl;
        return false;
    }
}

template <typename T>
bool YamlReader::readOptional(const std::string &key, T &value) {
    YAML::Node node;
    if (!getValueByKey(key, node)) {
        return false;
    }

    try {
        value = node.as<T>();
        return true;
    } catch (const YAML::BadConversion &e) {
        std::cerr << "Type conversion error for key '" << key << "': " << e.what() << std::endl;
        return false;
    }
}

template <typename Derived>
bool YamlReader::readEigen(const std::string &key, Eigen::MatrixBase<Derived> &matrix) {
    YAML::Node node;
    if (!getValueByKey(key, node)) {
        std::cerr << "Failed to find key: " << key << std::endl;
        return false;
    }

    if constexpr (Derived::ColsAtCompileTime == 1 || Derived::RowsAtCompileTime == 1) {
        // process vector
        if (!node.IsSequence() || node.size() != matrix.size()) {
            std::cerr << "Node is not a sequence or size mismatch for key: " << key << std::endl;
            return false;
        }

        for (std::size_t i = 0; i < node.size(); ++i) {
            matrix(i) = node[i].as<typename Derived::Scalar>();
        }
    } else {
        // process matrix
        if (!node.IsSequence() || node.size() != matrix.rows()) {
            std::cerr << "Node is not a sequence or row size mismatch for key: " << key << std::endl;
            return false;
        }

        for (std::size_t i = 0; i < node.size(); ++i) {
            if (!node[i].IsSequence() || node[i].size() != matrix.cols()) {
                std::cerr << "Column size mismatch at row " << i << " for key: " << key << std::endl;
                return false;
            }

            for (std::size_t j = 0; j < node[i].size(); ++j) {
                matrix(i, j) = node[i][j].as<typename Derived::Scalar>();
            }
        }
    }

    return true;
}

bool YamlReader::getValueByKey(const std::string &key, YAML::Node &node_out) {
    if (!rootNode_.IsDefined()) return false;

    // 从根节点开始
    // 注意：Assigning a node acts like a shared_ptr, it's cheap.
    // 我们使用 reset() 来重置 currentNode，或者直接拷贝赋值
    YAML::Node currentNode = rootNode_;

    std::istringstream keyStream(key);
    std::string subKey;

    while (std::getline(keyStream, subKey, delimiter_)) {
        // 注意：const Node 的 operator[] 不会修改树，但如果没有 key 会返回 undefined node
        // 所以我们必须先检查 key 是否存在，不能直接 currentNode = currentNode[subKey]

        if (currentNode[subKey]) {
            currentNode.reset(currentNode[subKey]); // 进入下一层
        } else {
            return false;
        }
    }

    // 找到了，赋值给输出
    node_out = currentNode;
    return true;
}
