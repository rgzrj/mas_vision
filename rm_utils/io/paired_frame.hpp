#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <cstdint>
#include <optional>

namespace tool {

// 图像、曝光时刻及插值姿态一起传递；时间单位为主机 steady_clock 纳秒。
struct RecordedFrame
{
    cv::Mat image;
    std::int64_t index = 0;
    // 曝光时刻和接收时刻共用主机 steady_clock 时间基，均为纳秒。
    std::int64_t timestamp_ns = 0;
    std::int64_t t_recv_ns    = 0;
    // 与曝光时刻配对的姿态；无值时 CSV 以 pose_valid=0 明确保留缺失信息。
    std::optional<Eigen::Quaterniond> pose;
    int mode = 0;
};

} // namespace tool
