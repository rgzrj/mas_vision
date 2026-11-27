#pragma once

#include "auto_buff/buff_track/track_types.hpp"
#include "auto_buff/buff_types.hpp"

namespace auto_buff
{

/**
 * @brief 匹配大符激活灯臂轮廓，重建其对应靶心的原图像素位置。
 * @param arms 灯臂观测，使用 state=2 的关键点、颜色和实测轮廓。
 * @param camera_matrix 原图相机内参，不是网络输入缩放后的内参。
 * @param distort_coeffs 相机畸变系数。
 * @param T_camera2world 相机 FLU 到世界系的刚体变换，平移单位为米。
 * @param config 符面初始化及观测去重的几何门限。
 * @param icons 同帧 R 候选，用于求解符面位姿并校验灯臂所属转盘。
 * @param bullseyes 同帧符板观测；完整待击板用于求解符面，已有靶心用于去重。
 * @return 通过几何检查的靶心观测，附带匹配误差和支持比例；不生成四端点。
 * @note 不依赖 EKF 或历史预测；缺少同帧 R 或可用待击板、位姿求解失败时返回空。
 *       内部几何统一使用 FLU，仅在 OpenCV 投影、反投影边界转换光学坐标。
 */
std::vector<Board> reconstruct_big_activated(std::span<const LightArm> arms, const cv::Mat &camera_matrix,
                                             const cv::Mat &distort_coeffs, const Eigen::Isometry3d &T_camera2world,
                                             const RuneConfig &config, std::span<const RuneIcon> icons,
                                             std::span<const RuneBullseye> bullseyes);

} // namespace auto_buff
