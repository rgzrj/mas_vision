#include "armor_debug.hpp"
#include "armor_blueprints.hpp"
#include "visualize/shoot_debug.hpp"
#include "visualize/track_debug.hpp"

#include <array>
#include <chrono>
#include <yaml-cpp/yaml.h>

namespace auto_aim
{
unsigned ArmorDebug::enabled_views(const YAML::Node &config)
{
    const auto aim      = config["auto_aim"];
    const auto track    = aim["armor_track"];
    const bool detect   = aim["armor_detector"]["debug"].as<bool>(false);
    const bool tracking = track["armor_track_debug"].as<bool>(track["debug"].as<bool>(false));
    const bool shoot    = aim["armor_shoot"]["debug"].as<bool>(false);
    const bool pose     = tracking && track["armor_pose_debug"].as<bool>(false);

    // 位顺序与 blueprints/generate.py 一致；pose 随 track 开关启用。
    return unsigned(detect) | (unsigned(tracking) << 1) | (unsigned(shoot) << 2) | (unsigned(pose) << 3);
}

void ArmorDebug::configure_layout(unsigned mask, bool realtime)
{
    // HUD 高度按各窗口的文字行数保留；实际排列顺序为 detect、track、pose、shoot。
    std::vector<rm_utils::DisplayView> views;
    if (mask & 1)
        views.push_back({"HikCamera", 128});
    if (mask & 2)
        views.push_back({"HikCamera_track", 320});
    if (mask & 8)
        views.push_back({"HikCamera_track_pose", 512});
    if (mask & 4)
        views.push_back({"HikCamera_shoot", 288});

    auto &display = rm_utils::Display::getInstance();
    if (realtime)
        display.register_layout("armor", views, kArmorBlueprints[mask]);
    else
        display.layout(views, kArmorBlueprints[mask]);
}

void ArmorDebug::unavailable(const cv::Mat &image, unsigned views, const std::string &status, bool include_detect)
{
    if (image.empty())
        return;

    auto &display = rm_utils::Display::getInstance();
    const std::array<const char *, 4> names{"HikCamera", "HikCamera_track", "HikCamera_shoot", "HikCamera_track_pose"};

    // 缺姿态时保留本帧检测图；模式停用时可连检测窗口一起清理。
    for (unsigned i = include_detect ? 0 : 1; i < names.size(); ++i)
        if (views & (1u << i))
            display.display_add(names[i], image, {{status, 20, 20, 24, {255, 220, 0, 255}, 1, true}});
}

ArmorDebug::ArmorDebug(const std::filesystem::path &config)
    : views_(enabled_views(YAML::LoadFile(config.string())))
{
    configure_layout(views_, true);
}

std::unique_lock<std::mutex> ArmorDebug::begin_frame(std::chrono::steady_clock::time_point timestamp) const
{
    if (!views_)
        return {};

    auto &display = rm_utils::Display::getInstance();
    std::unique_lock lock(display.frame_mutex());
    display.set_time(std::chrono::duration<double>(timestamp.time_since_epoch()).count());
    return lock;
}

void ArmorDebug::pose_unavailable(const cv::Mat &image) const
{
    unavailable(image, views_, "Pose unavailable");

    if (views_ & 2)
        TrackDebug::clear();
    if (views_ & 4)
        ShootDebug::clear();
}

void ArmorDebug::end_frame() const
{
    if (views_)
        rm_utils::Display::getInstance().end_frame();
}

ArmorDebugBase::ArmorDebugBase(bool enabled, const std::string &window_name)
    : enabled_(enabled), window_name_(window_name)
{
}

double ArmorDebugBase::update_fps()
{
    const auto now = std::chrono::steady_clock::now();
    if (fps_start_ == std::chrono::steady_clock::time_point{})
        fps_start_ = now;

    // 每个实例只累计真正进入绘制的帧，满一秒后更新显示值。
    const double seconds = std::chrono::duration<double>(now - fps_start_).count();
    ++fps_frames_;
    if (seconds >= 1)
    {
        fps_        = fps_frames_ / seconds;
        fps_frames_ = 0;
        fps_start_  = now;
    }
    return fps_;
}

void ArmorDebugBase::show(const cv::Mat &image, std::vector<rm_utils::DisplayText> &texts,
                          const std::vector<rm_utils::DisplayPoint> &points,
                          const std::vector<rm_utils::DisplayLine> &lines) const
{
    for (auto &text : texts)
        text.image_overlay = true;

    rm_utils::Display::getInstance().display_add(window_name_, image, texts, points, lines);
}

} // namespace auto_aim
