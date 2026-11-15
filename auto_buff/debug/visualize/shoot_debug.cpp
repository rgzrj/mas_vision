#include "shoot_debug.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "rm_utils/algorithm/math_tools.hpp"
#include "display.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

using namespace rm_utils;

namespace auto_buff
{

void ShootDebug::draw(const TrackResult &track, const std::optional<RuneShooter::Aimed> &aimed, const cv::Mat &image)
{
    if (!enabled() || image.empty())
        return;

    const double processing_fps = update_fps();
    auto &display               = rm_utils::Display::getInstance();
    const bool values           = display.values_enabled();
    const bool image_due        = display.image_due();
    if (!values && !image_due)
        return;

    // 姿态同时供Rerun曲线和图像文字使用，只计算一次。
    std::optional<std::pair<double, double>> gimbal_angles;
    if (track.gimbal_orientation)
    {
        const Eigen::Vector3d forward = *track.gimbal_orientation * Eigen::Vector3d::UnitX();
        gimbal_angles = std::pair{std::atan2(forward.y(), forward.x()),
                                  std::atan2(-forward.z(), std::hypot(forward.x(), forward.y()))};
    }
    if (values)
    {
        display.scalar("shoot/valid", bool(aimed));
        display.scalar("shoot/should_shoot", aimed && aimed->should_shoot);
        if (aimed)
        {
            display.scalar("shoot/yaw_rad", aimed->aim_yaw);
            display.scalar("shoot/pitch_rad", aimed->aim_pitch);
            display.scalar("shoot/flight_s", aimed->fly_time);
            display.scalar("shoot/prediction_s", aimed->prediction_time);
            display.point3d("world/attack", aimed->attack.x(), aimed->attack.y(), aimed->attack.z());
        }
        else
            clear();
        display.scalar("shoot/gimbal_yaw_rad", gimbal_angles ? gimbal_angles->first : NAN);
        display.scalar("shoot/gimbal_pitch_rad", gimbal_angles ? gimbal_angles->second : NAN);
    }
    if (!image_due)
        return;

    std::vector<rm_utils::DisplayText> texts;
    texts.push_back({rm_utils::processing_fps_text(processing_fps), 10, 8, 15, {255, 255, 0, 255}});
    const auto angle_text = [](const char *name, double yaw, double pitch)
    {
        std::ostringstream out;
        out << std::fixed << std::setprecision(2) << name << " yaw:" << rad2deg(yaw) << "  pitch:" << rad2deg(pitch)
            << " deg";
        return out.str();
    };

    // 云台姿态来自本帧配对 IMU，pitch 向下为正，与火控输出一致。
    // 普通离线视频没有这份观测，显示 N/A，不能拿固定外参或目标角冒充当前云台角。
    if (gimbal_angles)
    {
        texts.push_back(
            {angle_text("Gimbal(frame)", gimbal_angles->first, gimbal_angles->second), 10, 26, 20, {0, 255, 255, 255}});
    }
    else
    {
        texts.push_back({"Gimbal(frame): N/A", 10, 26, 20, {200, 200, 200, 255}});
    }

    if (!track.initialized)
        texts.push_back({"No Target", 10, 50, 20, {200, 200, 200, 255}});
    else if (!aimed)
        texts.push_back({"No ballistic solution", 10, 50, 20, {255, 100, 100, 255}});
    else
    {
        texts.push_back({angle_text("Target", aimed->aim_yaw, aimed->aim_pitch), 10, 50, 20, {0, 255, 0, 255}});
        if (gimbal_angles)
            texts.push_back({angle_text("Diff", rm_utils::limit_rad(aimed->aim_yaw - gimbal_angles->first),
                                        aimed->aim_pitch - gimbal_angles->second),
                             10,
                             74,
                             20,
                             {255, 200, 100, 255}});
        else
            texts.push_back({"Diff: N/A", 10, 74, 20, {200, 200, 200, 255}});
        std::ostringstream info, distance, timing;
        info << std::fixed << std::setprecision(3) << "RUNE  omega:" << track.state.rotation_speed
             << " rad/s  fire:" << int(aimed->should_shoot) << (aimed->idle ? "  IDLE" : "  SHOOT");
        distance << std::fixed << std::setprecision(2) << "Dist:" << aimed->attack.norm() << " m  Valid:YES";
        timing << std::fixed << std::setprecision(1) << "Flight:" << aimed->fly_time * 1000
               << " ms  Prediction:" << aimed->prediction_time * 1000 << " ms";
        texts.push_back({info.str(), 10, 100, 18, {255, 150, 0, 255}});
        texts.push_back({distance.str(), 10, 122, 18, {0, 255, 0, 255}});
        texts.push_back({timing.str(), 10, 144, 16});
    }
    for (std::size_t i = 0; i < texts.size(); ++i)
    {
        auto &text         = texts[i];
        text.image_overlay = true;
        text.x             = 16;
        text.y             = 12 + int(i) * 32;
        text.size          = 24;
    }
    show(image, texts, {}, {});
}

void ShootDebug::clear()
{
    auto &display = rm_utils::Display::getInstance();
    if (!display.values_enabled())
        return;
    display.scalar("shoot/valid", 0);
    display.scalar("shoot/should_shoot", 0);
    for (const char *name : {"yaw_rad", "pitch_rad", "flight_s", "prediction_s", "gimbal_yaw_rad", "gimbal_pitch_rad"})
        display.scalar(std::string("shoot/") + name, NAN);
    display.point3d("world/attack", NAN, NAN, NAN);
}

} // namespace auto_buff
