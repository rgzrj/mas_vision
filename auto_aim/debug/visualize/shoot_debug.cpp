#include "shoot_debug.hpp"
#include "auto_aim/armor_shoot/armor_shoot.hpp"
#include "math_tools.hpp"

#include <cmath>
#include <spdlog/fmt/fmt.h>

namespace auto_aim
{
void ShootDebug::draw(const cv::Mat &bgr_img, const std::optional<Target> &target, const SendPacket &packet,
                      const AimPoint &aim_point, const Eigen::Matrix3d &R_gimbal2world)
{
    if (!enabled() || bgr_img.empty())
        return;

    plot(packet);
    const double fps = update_fps();
    if (!rm_utils::Display::getInstance().image_due())
        return;
    const auto gimbal         = rm_utils::eulers(R_gimbal2world, 2, 1, 0);
    const double gimbal_yaw   = gimbal[0];
    const double gimbal_pitch = gimbal[1];
    const float target_yaw    = packet.target_yaw;
    const float target_pitch  = packet.target_pitch;
    const auto fire           = packet.fire_advice;

    std::vector<rm_utils::DisplayText> texts;
    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine> lines;

    // 数值明确按小数位格式化，角度在显示边界统一换算为度。
    texts.push_back({fmt::format("FPS: {}", int(fps)), 10, 10, 22, {255, 255, 0, 255}});
    texts.push_back({fmt::format("Gimbal  yaw:{:.2f}  pitch:{:.2f} deg", rm_utils::rad2deg(gimbal_yaw),
                                rm_utils::rad2deg(gimbal_pitch)),
                     10, 36, 32, {0, 255, 255, 255}});
    if (!target)
    {
        texts.push_back({"No Target", 10, 70, 32, {200, 200, 200, 255}});
        show(bgr_img, texts, points, lines);
        return;
    }

    const Eigen::VectorXd &target_state = target->ekf().x;
    texts.push_back({fmt::format("Target  yaw:{:.2f}  pitch:{:.2f} deg", rm_utils::rad2deg(target_yaw),
                                rm_utils::rad2deg(target_pitch)),
                     10, 70, 32, {0, 255, 0, 255}});
    texts.push_back({fmt::format("Diff_yaw:{:.2f}Diff_pitch:{:.2f} deg",
                                rm_utils::rad2deg(target_yaw - gimbal_yaw),
                                rm_utils::rad2deg(target_pitch - gimbal_pitch)),
                     10, 104, 32, {255, 200, 100, 255}});

    // 瞄准策略与目标状态：模式来自火控计算，此处仅显示。
    std::string mode_name;
    switch (aim_point.mode)
    {
    case AimMode::TRACK:
        mode_name = "TRACK";
        break;
    case AimMode::CENTER:
        mode_name = "CENTER";
        break;
    case AimMode::COMING:
        mode_name = "COMING";
        break;
    }

    texts.push_back({fmt::format("Name:{}  armor_num:{}  omega:{:.3f}  fire:{}  {}", target->name,
                                target->armor_num, target_state[7], int(fire), mode_name),
                     10, 144, 32, {255, 150, 0, 255}});

    // 瞄准点的水平距离、有效标志和模型装甲板 ID。
    const double distance = std::sqrt(aim_point.xyza[0] * aim_point.xyza[0] + aim_point.xyza[1] * aim_point.xyza[1]);
    texts.push_back({fmt::format("Dist:{:.3f}m  Valid:{}  ArmorID:{}", distance,
                                aim_point.valid ? "YES" : "NO", aim_point.armor_id),
                     10, 170, 28, aim_point.valid ? rm_utils::DisplayColor{0, 255, 0, 255}
                                                : rm_utils::DisplayColor{0, 0, 255, 255}});

    show(bgr_img, texts, points, lines);
}

void ShootDebug::plot(const SendPacket &packet)
{
    auto &display = rm_utils::Display::getInstance();
    if (!display.values_enabled())
        return;
    display.scalar("armor/shoot/angles/yaw_rad", packet.target_yaw);
    display.scalar("armor/shoot/angles/pitch_rad", packet.target_pitch);
    display.scalar("armor/shoot/state/fire", packet.fire_advice);
}

void ShootDebug::clear()
{
    plot({});
}

} // namespace auto_aim
