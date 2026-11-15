#pragma once

#include "rune_pose.hpp"
#include "auto_buff/buff_detect/detection.hpp"

namespace auto_buff {

// 同帧完整待击板求符面；用深大灯臂模型匹配每根实测轮廓，独立求转角后投影板心。
// 不读取 EKF/历史预测，不伪造端点；失败时返回空，R 仍由原通道处理。
std::vector<Board> reconstruct_big_activated(std::span<const LightArm> arms,
                                            const cv::Mat &intrinsic, const cv::Mat &distortion, const Eigen::Isometry3d &camera_transform,
                                            const RuneConfig &config,
                                            std::span<const auto_buff::RuneIcon> icons,
                                            std::span<const auto_buff::RuneBullseye> bullseyes);

} // namespace auto_buff
