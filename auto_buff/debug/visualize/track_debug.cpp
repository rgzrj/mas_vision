#include "track_debug.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "display.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace auto_buff
{
std::string rune_motion_text(const TrackResult &track)
{
    if (!track.initialized)
        return {};

    const auto &state = track.state;
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << std::showpos;

    // 字段及回退顺序沿用 RMCS：正弦拟合 → 恒速拟合 → EKF 当前角度。
    if (state.sine_valid)
    {
        out << "spd_" << std::noshowpos << state.update_count << std::showpos << "(t)=" << state.sine_v << state.sine_a
            << "*sin(" << state.sine_phase << state.sine_omega << "t)";
    }
    else if (state.use_prediction_speed)
    {
        out << "spd_" << std::noshowpos << state.update_count << std::showpos << "(t)=" << state.rotation_speed;
    }
    else
    {
        out << "theta_ekf=" << state.rotation_angle;
        return out.str();
    }

    out << ", e=" << std::noshowpos << std::setprecision(3) << state.prediction_cost;
    return out.str();
}

void TrackDebug::draw(const TrackResult &result, const cv::Mat &image)
{
    if (!enabled() || image.empty())
        return;

    const double processing_fps = update_fps();
    auto &display               = rm_utils::Display::getInstance();
    if (display.values_enabled())
    {
        display.scalar("track/initialized", result.initialized);
        display.scalar("track/observed", result.observed);
        display.scalar("track/ready", result.ready);
        display.scalar("track/matched", double(result.matched.size()));
        if (result.initialized)
        {
            const auto &state = result.state;
            display.scalar("track/angle_rad", state.rotation_angle);
            display.scalar("track/speed_rad_s", state.rotation_speed);
            display.scalar("track/fit_error", state.prediction_cost);
            display.scalar("track/sine_valid", state.sine_valid);
            constexpr std::array names{"x_m", "y_m", "z_m", "omega_rad_s", "theta_rad", "face_yaw_rad"};
            for (std::size_t i = 0; i < names.size(); ++i)
            {
                display.scalar(std::string("track/ekf/") + names[i], result.ekf.state[i]);
                display.scalar(std::string("track/ekf/P_diag/") + names[i], result.ekf.variance[i]);
            }
            display.point3d("world/rotation_center", state.x, state.y, state.z);
        }
        else
            clear(); // 丢失后不沿用上一帧的空间点、状态或协方差曲线。
    }
    if (!display.image_due())
        return;
    std::vector<rm_utils::DisplayText> texts;
    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine> lines;
    texts.push_back({rm_utils::processing_fps_text(processing_fps), 16, 12, 24, {255, 255, 0, 255}});
    const rm_utils::DisplayColor status_color = !result.initialized ? rm_utils::DisplayColor{200, 200, 200, 255}
                                                : result.ready      ? rm_utils::DisplayColor{0, 255, 0, 255}
                                                                    : rm_utils::DisplayColor{255, 255, 0, 255};
    texts.push_back({"Track: " + result.status, 20, 10, 16, status_color});
    texts.push_back({"matched=" + std::to_string(result.matched.size()) + (result.ready ? " ready" : " warming up"),
                     20,
                     30,
                     14,
                     {0, 255, 255, 255}});
    if (result.initialized)
    {
        texts.push_back({rune_motion_text(result), 20, 50, 14, {255, 180, 80, 255}});
        // 展示原始六维后验，不能把用于外推的拟合角速度冒充 EKF 角速度。
        const auto &x = result.ekf.state;
        const auto &p = result.ekf.variance;
        std::ostringstream position, rotation, position_variance, rotation_variance;
        position << std::fixed << std::setprecision(3) << "EKF xyz: " << x[0] << ", " << x[1] << ", " << x[2] << " m";
        rotation << std::fixed << std::setprecision(3) << "EKF theta=" << x[4] << " rad  w=" << x[3]
                 << " rad/s  psi=" << x[5] << " rad";
        position_variance << std::scientific << std::setprecision(2) << "P xyz: " << p[0] << ", " << p[1] << ", "
                          << p[2] << " m^2";
        rotation_variance << std::scientific << std::setprecision(2) << "P w/theta/psi: " << p[3] << ", " << p[4]
                          << ", " << p[5];
        texts.push_back({position.str(), 20, 70, 16, {0, 255, 0, 255}});
        texts.push_back({rotation.str(), 20, 90, 16, {255, 180, 80, 255}});
        texts.push_back({position_variance.str(), 20, 110, 16, {230, 100, 255, 255}});
        texts.push_back({rotation_variance.str(), 20, 130, 16, {80, 180, 255, 255}});
        std::array<cv::Point2d, 6> projected{};
        std::array<bool, 6> seen{};
        bool valid_polygon = result.predicted.size() == 6;
        for (const auto &feature : result.predicted)
        {
            const auto &point = feature.point;
            const int id      = feature.feature_id;
            if (id < 0 || id > 5 || seen[id] || !std::isfinite(point.x) || !std::isfinite(point.y))
            {
                valid_polygon = false;
                continue;
            }
            seen[id]      = true;
            projected[id] = {point.x, point.y};
            points.push_back({cvRound(point.x), cvRound(point.y), {255, 255, 0, 255}, 4});
        }
        // 六特征为 R 与五个符板中心；五边形连线按特征 ID，不能按向量顺序。
        if (valid_polygon && std::all_of(seen.begin(), seen.end(), [](bool value) { return value; }))
        {
            cv::Point2d center{};
            for (int id = 1; id <= 5; ++id)
            {
                const auto &a = projected[id];
                const auto &b = projected[id % 5 + 1];
                lines.push_back({cvRound(a.x), cvRound(a.y), cvRound(b.x), cvRound(b.y), 1, {255, 255, 0, 255}});
                center += a / 5;
            }
            lines.push_back({cvRound(projected[0].x),
                             cvRound(projected[0].y),
                             cvRound(center.x),
                             cvRound(center.y),
                             1,
                             {255, 255, 0, 255}});
        }
        for (const auto &feature : result.matched)
            points.push_back({cvRound(feature.point.x), cvRound(feature.point.y), {0, 255, 0, 255}, 4});
        for (const auto &feature : result.future)
            points.push_back({cvRound(feature.point.x), cvRound(feature.point.y), {0, 255, 255, 255}, 5});
    }
    for (std::size_t i = 0; i < texts.size(); ++i)
    {
        auto &text         = texts[i];
        text.image_overlay = true;
        text.x             = 16;
        text.y             = 12 + int(i) * 32;
        text.size          = 24;
    }
    const int legend_y = 12 + int(texts.size()) * 32;
    texts.push_back({"Yellow: prior", 16, legend_y, 24, {255, 255, 0, 255}, 1, true});
    texts.push_back({"Green: matched", 300, legend_y, 24, {0, 255, 0, 255}, 1, true});
    texts.push_back({"Cyan: future", 640, legend_y, 24, {0, 255, 255, 255}, 1, true});
    show(image, texts, points, lines);
}

void TrackDebug::clear()
{
    auto &display = rm_utils::Display::getInstance();
    if (!display.values_enabled())
        return;
    for (const char *name : {"initialized", "observed", "ready", "matched"})
        display.scalar(std::string("track/") + name, 0);
    for (const char *name : {"angle_rad", "speed_rad_s", "fit_error"})
        display.scalar(std::string("track/") + name, NAN);
    display.scalar("track/sine_valid", 0);
    for (const char *name : {"x_m", "y_m", "z_m", "omega_rad_s", "theta_rad", "face_yaw_rad"})
    {
        display.scalar(std::string("track/ekf/") + name, NAN);
        display.scalar(std::string("track/ekf/P_diag/") + name, NAN);
    }
    display.point3d("world/rotation_center", NAN, NAN, NAN);
}

} // namespace auto_buff
