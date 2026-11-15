#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <cstdint>
#include <optional>
#include <iomanip>
#include <ostream>

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

/**
 * @brief 写入固定 CSV 表头及四元数所需的浮点精度。
 */
inline void write_metadata_header(std::ostream &out)
{
    out << std::setprecision(17) << "frame_index,timestamp_ns,t_recv_ns,q_w,q_x,q_y,q_z,mode,pose_valid\n";
}

/**
 * @brief 按 w/x/y/z 顺序写入姿态；无姿态时写零占位及有效性标志。
 */
inline void write_frame_metadata(std::ostream &out, const RecordedFrame &frame)
{
    out << frame.index << ',' << frame.timestamp_ns << ',' << frame.t_recv_ns << ',';
    if (frame.pose)
    {
        const auto &q = *frame.pose;
        out << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z();
    }
    else
        out << "0,0,0,0";
    out << ',' << frame.mode << ',' << bool(frame.pose) << '\n';
}

} // namespace tool
