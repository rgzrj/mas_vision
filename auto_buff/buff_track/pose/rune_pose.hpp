#pragma once

#include "auto_buff/buff_track/buff_state.hpp"
#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <optional>

namespace auto_buff {

/**
 * @brief 遍历 R × 待击板候选，用 PnP 和整体几何选择初始位姿。
 * @param config 初始化误差、符面倾角及中心关联门限。
 * @param intrinsic 原图内参矩阵。
 * @param distortion 五项畸变系数。
 * @param camera_transform 相机到世界的 Eigen 刚体变换。
 * @param icons 全部 R 候选，使用原图像素坐标。
 * @param bullseyes 符板观测；仅完整待击板可作 PnP 种子，端点须由上层验证。
 * @return 世界系符中心、符面偏航及转角；无满足约束的组合时为空。
 * @note 不持有 EKF，不推进时间；种子端点会结合 R 位置重新排序。
 * 深大/自训练五点由 detect 拆为独立 R 和四端点；网络第 2 点是 R，不是符板中心。
 * 传统检测仍使用同一入口；此层不根据模型来源切换算法。
 * 沿用上游限制：初始化需要 1～2 块待击板，超过两块时本轮不建立模型。
 */
auto solve_rune_pose(const RuneConfig &config, const cv::Mat &intrinsic, const cv::Mat &distortion, const Eigen::Isometry3d &camera_transform, std::span<const auto_buff::RuneIcon> icons,
                     std::span<const auto_buff::RuneBullseye> bullseyes) -> std::optional<RunePose>;

} // namespace auto_buff
