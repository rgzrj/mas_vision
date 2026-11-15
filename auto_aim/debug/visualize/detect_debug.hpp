#pragma once

#include "auto_aim/debug/armor_debug.hpp"
#include "auto_aim/armor_types.hpp"

namespace auto_aim
{
/**
 * @brief 检测灯条与装甲板的调试绘制
 */
class DetectDebug : public ArmorDebug
{
public:
    explicit DetectDebug(bool enabled = false) : ArmorDebug(enabled, "HikCamera") {}

    // 绘制灯条、四角点、类别分数和计数；关闭 debug 时立即返回。
    void draw(const cv::Mat &bgr_img, const std::vector<Armor> &armors, const std::vector<LightBar> &lights);

    // 将 RRD 中的装甲板、灯条计数归零；调用方按 debug 开关决定是否调用。
    static void clear();
};

} // namespace auto_aim
