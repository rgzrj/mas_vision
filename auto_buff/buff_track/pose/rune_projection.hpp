#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <array>

namespace auto_buff {

using RuneStateVector      = Eigen::Matrix<double, 6, 1>; // [x,y,z,角速度,转角,符面偏航]，米、秒、弧度。
inline constexpr auto kX   = 0;
inline constexpr auto kY   = 1;
inline constexpr auto kZ   = 2;
inline constexpr auto kW   = 3;
inline constexpr auto kA   = 4;
inline constexpr auto kPsi = 5;

// 0 为 R，1～5 为符板中心；pose 和 track 使用完全相同的几何投影。
struct RuneObservable
{
    cv::Mat intrinsic, distortion;
    Eigen::Isometry3d camera_to_world = Eigen::Isometry3d::Identity();

    static constexpr int kFeatureIcon  = 0;
    static constexpr int kFeatureCount = 6;

    std::array<Eigen::Vector2d, kFeatureCount> projected; // 原图像素；不可见点置零。
    std::array<Eigen::Vector3d, kFeatureCount> in_camera; // RMCS 前/左/上相机系，单位米。
    std::array<bool, kFeatureCount> visible;              // 投影有效且光学深度大于 0.1 m。

    /**
     * @brief 从状态和相机变换刷新 R 与五个符板中心的投影。
     * @param state 当前六维状态；本函数只刷新投影缓存，不校正或预测状态。
     *                   [x,y,z,角速度,转角,符面偏航]，米、秒、弧度。
     */
    auto update(const RuneStateVector &state) noexcept -> void;
};

} // namespace auto_buff
