/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 *
 * Shared sensor buffers exchanged between ROS callbacks and processing loops.
 */

#ifndef SHARED_BUFFERS_H
#define SHARED_BUFFERS_H

#include <deque>
#include <mutex>
#include <optional>
#include <condition_variable>
#include "support/common_lib.h"
#include "support/type.h"

struct SharedBuffers
{
    struct LidarFrame {
        PointCloudXYZI::Ptr cloud;
        double base_time = 0.0;
        double end_time = 0.0;
    };

    // ========= 生产者 / 消费者共用的数据 =========
    std::deque<ImuMsgConst>                imu_buf;
    std::deque<LidarFrame>                 lidar_frame_buf;
    std::deque<double>                     img_time_buf;
    std::deque<WheelData>                  wheel_buf;

    // ========= 互斥锁 & 条件变量 =========
    std::mutex                mtx;
    std::condition_variable   sig;

    // ========= 通用线程安全访问接口 =========

    void pushLidarFrame(LidarFrame frame) {
        {
            std::scoped_lock lk(mtx);
            lidar_frame_buf.push_back(std::move(frame));
        }
        notifyAll();
    }

    std::optional<LidarFrame> getFrontLidarFrameSafe() {
        std::scoped_lock lk(mtx);
        if (lidar_frame_buf.empty()) {
            return std::nullopt;
        }
        return lidar_frame_buf.front();
    }

    void popFrontLidarFrameSafe() {
        std::scoped_lock lk(mtx);
        if (!lidar_frame_buf.empty()) {
            lidar_frame_buf.pop_front();
        }
    }

    // 安全 pop_front
    template<typename BufferType>
    void popFrontSafe(BufferType& buffer) {
        std::scoped_lock lk(mtx);
        if (!buffer.empty()) {
            buffer.pop_front();
        }
    }

    // 安全 push_back
    template<typename BufferType, typename T>
    void pushBackSafe(BufferType& buffer, T&& value) {
        std::scoped_lock lk(mtx);
        buffer.push_back(std::forward<T>(value));
    }

    // 安全 empty 检查
    template<typename BufferType>
    bool isEmptySafe(const BufferType& buffer) {
        std::scoped_lock lk(mtx);
        return buffer.empty();
    }

    // 安全取 front 元素
    template<typename BufferType>
    typename BufferType::value_type getFrontSafe(const BufferType& buffer) {
        std::scoped_lock lk(mtx);
        return buffer.front();
    }

    // 安全取 back 元素
    template<typename BufferType>
    typename BufferType::value_type getBackSafe(const BufferType& buffer) {
        std::scoped_lock lk(mtx);
        return buffer.back();
    }

    // 安全获取 size
    template<typename BufferType>
    size_t getSizeSafe(const BufferType& buffer) {
        std::scoped_lock lk(mtx);
        return buffer.size();
    }

    // 安全 clear
    template<typename BufferType>
    void clearSafe(BufferType& buffer) {
        std::scoped_lock lk(mtx);
        buffer.clear();
    }

    // 批量取出所有元素并清空（move）
    template<typename BufferType>
    BufferType moveOutAllSafe(BufferType& buffer) {
        std::scoped_lock lk(mtx);
        BufferType tmp;
        std::swap(tmp, buffer);
        return tmp;
    }

    // 唤醒所有等待线程
    void notifyAll() {
        sig.notify_all();
    }
};



#endif //SHARED_BUFFERS_H
