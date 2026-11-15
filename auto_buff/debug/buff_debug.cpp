#include "buff_debug.hpp"
#include "buff_blueprints.hpp"

#include <array>
#include <chrono>
#include <yaml-cpp/yaml.h>

namespace auto_buff
{
unsigned BuffDebug::enabled_views(const YAML::Node &config)
{
    const auto buff   = config["auto_buff"];
    const bool detect = buff["detect"]["debug"].as<bool>(false);
    const bool track  = buff["track"]["debug"].as<bool>(false);
    const bool shoot  = buff["shoot"]["debug"].as<bool>(false);
    // 位顺序与现有 Rerun 布局一致：detect、track、shoot。
    return unsigned(detect) | (unsigned(track) << 1) | (unsigned(shoot) << 2);
}

unsigned BuffDebug::configure_layout(const std::filesystem::path &config_path)
{
    const auto views = enabled_views(YAML::LoadFile(config_path.string()));
    configure_layout(views, true);
    return views;
}

void BuffDebug::configure_layout(unsigned mask, bool realtime)
{
    // 三个阶段各自保留原来的窗口名和 HUD 高度。
    std::vector<rm_utils::DisplayView> views;
    if (mask & 1)
        views.push_back({"detect", 128});
    if (mask & 2)
        views.push_back({"track", 320});
    if (mask & 4)
        views.push_back({"shoot", 288});

    auto &display = rm_utils::Display::getInstance();
    if (realtime)
        display.register_layout("buff", views, kBuffBlueprints[mask]);
    else
        display.layout(views, kBuffBlueprints[mask]);
}

BuffDebug::BuffDebug(bool enabled, const std::string &window_name) : enabled_(enabled), window_name_(window_name) {}

double BuffDebug::update_fps()
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

void BuffDebug::show(const cv::Mat &image, std::vector<rm_utils::DisplayText> &texts,
                     const std::vector<rm_utils::DisplayPoint> &points,
                     const std::vector<rm_utils::DisplayLine> &lines) const
{
    for (auto &text : texts)
        text.image_overlay = true;

    rm_utils::Display::getInstance().display_add(window_name_, image, texts, points, lines);
}

void BuffDebug::unavailable(const cv::Mat &image, unsigned views, const std::string &status, bool include_detect)
{
    if (image.empty())
        return;

    auto &display = rm_utils::Display::getInstance();
    const std::array<const char *, 3> names{"detect", "track", "shoot"};

    // 缺姿态时保留本帧检测图；模式停用时可连检测窗口一起清理。
    for (unsigned i = include_detect ? 0 : 1; i < names.size(); ++i)
        if (views & (1u << i))
            display.display_add(names[i], image, {{status, 20, 20, 24, {255, 220, 0, 255}, 1, true}});
}

} // namespace auto_buff
