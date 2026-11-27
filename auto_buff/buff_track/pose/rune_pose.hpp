#pragma once

#include "auto_buff/buff_track/track_types.hpp"

#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <optional>

namespace auto_buff
{

/**
 * @brief 遍历 R × 待击板候选，用 PnP 和整体几何选择初始位姿。
 * @param config 初始化误差、符面倾角及中心关联门限。
 * @param camera_matrix 原图内参矩阵。
 * @param distort_coeffs 五项畸变系数。
 * @param T_camera2world 相机到世界的 Eigen 刚体变换。
 * @param icons 全部 R 候选，使用原图像素坐标。
 * @param bullseyes 符板观测；仅完整待击板可作 PnP 种子，端点须由上层验证。
 * @return 世界系符中心、符面偏航及转角；无满足约束的组合时为空。
 */
auto solve_rune_pose(const RuneConfig &config, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs,
                     const Eigen::Isometry3d &T_camera2world, std::span<const RuneIcon> icons,
                     std::span<const RuneBullseye> bullseyes) -> std::optional<RunePose>;

} // namespace auto_buff
