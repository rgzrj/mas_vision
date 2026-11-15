#pragma once

#include "auto_aim/debug/armor_debug.hpp"
#include "auto_aim/armor_track/armor_target.hpp"
#include "hardware/serial/serial_types.hpp"

#include <optional>

namespace auto_aim
{
struct AimPoint;

/**
 * @brief 瞄准与火控信息的调试绘制
 */
class ShootDebug : public ArmorDebug
{
public:
    explicit ShootDebug(bool enabled = false) : ArmorDebug(enabled, "HikCamera_shoot") {}

    // 绘制云台角、目标角、角误差和瞄准信息；关闭 debug 时立即返回。
    void draw(const cv::Mat &bgr_img, const std::optional<Target> &target, const SendPacket &packet,
              const AimPoint &aim_point, const Eigen::Matrix3d &R_gimbal2world);
    // 记录视觉指令中的 yaw、pitch 和开火建议，不发送串口数据。
    static void plot(const SendPacket &packet);

    // 将本模块的 RRD 指令曲线归零；调用方按 debug 开关决定是否调用。
    static void clear();
};

} // namespace auto_aim
