#pragma once

#include "auto_aim/debug/armor_debug.hpp"
#include "auto_aim/armor_track/armor_pose.hpp"

namespace auto_aim
{
/**
 * @brief 位姿与重投影的调试绘制
 */
class PoseDebug : public ArmorDebug
{
public:
    explicit PoseDebug(bool enabled = false) : ArmorDebug(enabled, "HikCamera_track_pose") {}

    // 显示 IMU 和首块装甲板的位姿，对比原始角点与重投影；关闭 debug 时立即返回。
    void draw(const cv::Mat &bgr_img, const std::vector<Armor> &armors, const ArmorPose &pose);
};

} // namespace auto_aim
