#pragma once

#include <Eigen/Geometry>
#include <algorithm>

namespace rm_utils::util {

// 沿用 mas_vision 的姿态插值：最短路径、夹紧到窗口、不做角速度外推。
inline Eigen::Quaterniond interpolate_orientation(Eigen::Quaterniond first,
                                                  Eigen::Quaterniond second,
                                                  double ratio) {
    if (first.dot(second) < 0)
        second.coeffs() *= -1;
    return first.slerp(std::clamp(ratio, 0.0, 1.0), second).normalized();
}

// mas_vision 串口四元数采用 IMU 本体基，世界基也按相同方式转换到云台参考基。
inline Eigen::Quaterniond gimbal_orientation(const Eigen::Quaterniond& imu_to_world,
                                             const Eigen::Quaterniond& gimbal_to_imu) {
    return (gimbal_to_imu.conjugate() * imu_to_world * gimbal_to_imu).normalized();
}

} // namespace rm_utils::util
