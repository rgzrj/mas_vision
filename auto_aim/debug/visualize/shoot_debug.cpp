#include "shoot_debug.hpp"
#include "auto_aim/armor_shoot/armor_shoot.hpp"
#include "math_tools.hpp"

#include <cmath>

namespace auto_aim
{
void ShootDebug::draw(const cv::Mat &bgr_img, const std::optional<Target> &target, const SendPacket &packet,
                      const AimPoint &aim_point, const Eigen::Matrix3d &R_gimbal2world)
{
    if (!enabled() || bgr_img.empty())
        return;

    plot(packet);
    const double fps          = update_fps();
    const auto gimbal         = rm_utils::eulers(R_gimbal2world, 2, 1, 0);
    const double gimbal_yaw   = gimbal[0];
    const double gimbal_pitch = gimbal[1];
    const float target_yaw    = packet.target_yaw;
    const float target_pitch  = packet.target_pitch;
    const auto fire           = packet.fire_advice;

    std::vector<rm_utils::DisplayText> texts;
    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine> lines;

    // 左上角状态：本模块绘制帧率、云台角和目标角。
    rm_utils::DisplayText fps_text;
    fps_text.content = "FPS: " + std::to_string(static_cast<int>(fps));
    fps_text.x       = 10;
    fps_text.y       = 10;
    fps_text.size    = 22;
    fps_text.color   = {255, 255, 0, 255};
    texts.push_back(fps_text);

    // 云台当前角度
    rm_utils::DisplayText gimbal_text;
    gimbal_text.content = "Gimbal  yaw:" + std::to_string(gimbal_yaw * 57.3f).substr(0, 6) +
                          "  pitch:" + std::to_string(gimbal_pitch * 57.3f).substr(0, 6) + " deg";
    gimbal_text.x     = 10;
    gimbal_text.y     = 36;
    gimbal_text.size  = 32;
    gimbal_text.color = {0, 255, 255, 255};
    texts.push_back(gimbal_text);

    // 无目标情况
    if (!target.has_value())
    {
        rm_utils::DisplayText no_target_text;
        no_target_text.content = "No Target";
        no_target_text.x       = 10;
        no_target_text.y       = 36;
        no_target_text.size    = 32;
        no_target_text.color   = {200, 200, 200, 255};
        texts.push_back(no_target_text);
        show(bgr_img, texts, points, lines);
        return;
    }

    const Eigen::VectorXd &target_state = target->ekf().x;

    // 目标角度（解算结果）
    rm_utils::DisplayText target_text;
    target_text.content = "Target  yaw:" + std::to_string(target_yaw * 57.3f).substr(0, 6) +
                          "  pitch:" + std::to_string(target_pitch * 57.3f).substr(0, 6) + " deg";
    target_text.x     = 10;
    target_text.y     = 70;
    target_text.size  = 32;
    target_text.color = {0, 255, 0, 255};
    texts.push_back(target_text);

    // 指令角减去当前云台角，统一换算为度。
    rm_utils::DisplayText error_text;
    double yaw_error   = (target_yaw - gimbal_yaw) * 57.3;
    double pitch_error = (target_pitch - gimbal_pitch) * 57.3;
    error_text.content =
        "Diff_yaw:" + std::to_string(yaw_error).substr(0, std::to_string(yaw_error).find('.') + 3) +
        "Diff_pitch:" + std::to_string(pitch_error).substr(0, std::to_string(pitch_error).find('.') + 3) + " deg";
    error_text.x     = 10;
    error_text.y     = 104;
    error_text.size  = 32;
    error_text.color = {255, 200, 100, 255};
    texts.push_back(error_text);

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

    rm_utils::DisplayText aim_text;
    aim_text.content = "Name:" + target->name + "  armor_num:" + std::to_string(target->armor_num) +
                       "  omega:" + std::to_string(target_state[7]).substr(0, 5) + "  fire:" + std::to_string(fire) +
                       "  " + mode_name;
    aim_text.x     = 10;
    aim_text.y     = 144;
    aim_text.size  = 32;
    aim_text.color = {255, 150, 0, 255};
    texts.push_back(aim_text);

    // 瞄准点的水平距离、有效标志和模型装甲板 ID。
    double distance = std::sqrt(aim_point.xyza[0] * aim_point.xyza[0] + aim_point.xyza[1] * aim_point.xyza[1]);
    rm_utils::DisplayText distance_text;
    distance_text.content = "Dist:" + std::to_string(distance).substr(0, 5) + "m  " +
                            "Valid:" + std::string(aim_point.valid ? "YES" : "NO") + "  " +
                            "ArmorID:" + std::to_string(aim_point.armor_id);
    distance_text.x    = 10;
    distance_text.y    = 170;
    distance_text.size = 28;
    distance_text.color =
        aim_point.valid ? rm_utils::DisplayColor{0, 255, 0, 255} : rm_utils::DisplayColor{0, 0, 255, 255};
    texts.push_back(distance_text);

    show(bgr_img, texts, points, lines);
}

void ShootDebug::plot(const SendPacket &packet)
{
    auto &display = rm_utils::Display::getInstance();
    display.scalar("armor/shoot/angles/yaw_rad", packet.target_yaw);
    display.scalar("armor/shoot/angles/pitch_rad", packet.target_pitch);
    display.scalar("armor/shoot/state/fire", packet.fire_advice);
}

void ShootDebug::clear()
{
    plot({});
}

} // namespace auto_aim
