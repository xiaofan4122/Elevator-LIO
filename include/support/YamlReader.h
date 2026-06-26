/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 */

#ifndef YAML_READER_H
#define YAML_READER_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "support/type.h"
#include <iostream>




class YamlReader {
public:
    explicit YamlReader(std::string filePath);
    explicit YamlReader(std::vector<std::string> filePaths);

    bool load();

    template <typename T>
    bool read(const std::string &key, T &value);

    template <typename T>
    bool readOptional(const std::string &key, T &value);

    template <typename Derived>
    bool readEigen(const std::string &key, Eigen::MatrixBase<Derived> &matrix);


private:
    // 将 Node 改为 const 引用传递，避免修改原树
    bool getValueByKey(const std::string& key, YAML::Node& node_out);

    YAML::Node rootNode_; // 直接存 Node，不需要 shared_ptr
    std::vector<std::string> filePaths_;
    char delimiter_ = '.';  // use '.' to seperate the key in yaml
    bool fileLoaded_ = false;
};

#endif // YAML_READER_H
