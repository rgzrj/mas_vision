#pragma once

#include "auto_aim/debug/armor_debug.hpp"
#include "auto_aim/armor_track/armor_target.hpp"

#include <optional>

namespace auto_aim
{
class ArmorTrack;

/**
 * @brief 跟踪状态与预测模型的调试绘制
 */
class TrackDebug : public ArmorDebug
{
public:
    explicit TrackDebug(bool enabled = false) : ArmorDebug(enabled, "HikCamera_track") {}

    // 绘制跟踪状态、EKF 参数和模型投影；关闭 debug 时立即返回。
    void draw(const cv::Mat &bgr_img, const std::vector<Armor> &armors, const ArmorTrack &tracker);

    // 记录目标有效标志、11 维状态和协方差对角线；空目标写入无效标志和 NaN。
    static void plot(const std::optional<Target> &target);

    // 清理本模块的 RRD 曲线；调用方按 debug 开关决定是否调用。
    static void clear();
};

} // namespace auto_aim
