#include "display.hpp"
#include "rerun/rerun_display.hpp"
#include "sdl/sdl_display.hpp"
#include "rm_utils/io/playback_control.hpp"

#include <cmath>
#include <format>
#include <mutex>
#include <stdexcept>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <unordered_map>

namespace rm_utils
{

std::string processing_fps_text(double fps)
{
    return std::isfinite(fps) && fps >= 0 ? std::format("Process FPS: {:.1f}", fps) : "Process FPS: --";
}

struct Display::Impl
{
    std::mutex mutex;
    std::unique_ptr<SdlDisplay> sdl;
    std::unique_ptr<RerunDisplay> rerun;
    double frame_time = NAN;
    std::optional<double> last_image_time;
    bool draw = false, playback_paused = false;
    YAML::Node deferred_settings;
    std::filesystem::path config_path, output;
    bool deferred_start = false, layout_changed = false;
    std::string selected_layout;
    std::unordered_map<std::string, std::pair<std::vector<DisplayView>, std::vector<unsigned char>>> layouts;

    bool image_due(double seconds, std::optional<double> last) const
    {
        const double fps = sdl ? Display::kSdlImageFps : Display::kImageFps;
        return (sdl || rerun) && std::isfinite(seconds) &&
               ((sdl && playback_paused) || !last || seconds < *last ||
                std::floor(seconds * fps) != std::floor(*last * fps));
    }
};

Display::Display() : impl_(std::make_unique<Impl>()) {}
Display::~Display()
{
    shutdown();
}

Display &Display::getInstance()
{
    static Display display;
    return display;
}

void Display::start(bool spawn, const std::string &file, const std::string &address,
                    const std::string &backend, const std::string &image_encoding,
                    const std::string &hud_encoding)
{
    if (backend != "rerun" && backend != "sdl")
        throw std::runtime_error("display.backend must be rerun or sdl");
    if (backend == "sdl" && !address.empty())
        throw std::runtime_error("SDL uses a local window; remove display.address");
    {
        std::lock_guard lock(impl_->mutex);
        if (impl_->sdl || impl_->rerun)
            throw std::runtime_error("Display session already started");
    }

    // 先在局部完成初始化；失败时不留下半开的窗口或会话。
    // start只由主线程调用；启动Viewer期间不占提交锁，算法线程可以继续处理。
    auto sdl = backend == "sdl" ? std::make_unique<SdlDisplay>() : nullptr;
    std::unique_ptr<RerunDisplay> rerun;
    if (!sdl || !file.empty())
        rerun = std::make_unique<RerunDisplay>(!sdl, spawn, file, address, image_encoding, hud_encoding);
    std::lock_guard lock(impl_->mutex);
    impl_->sdl = std::move(sdl);
    impl_->rerun = std::move(rerun);
    impl_->frame_time = NAN;
    impl_->last_image_time.reset();
    impl_->draw = false;
    impl_->playback_paused = false;
}

void Display::start(const std::filesystem::path &config_path, const std::filesystem::path &output)
{
    const auto settings = YAML::LoadFile(config_path.string())["display"];
    std::lock_guard lock(impl_->mutex);
    if (impl_->sdl || impl_->rerun || impl_->deferred_start)
        throw std::runtime_error("Display session already started");
    // main只准备后端配置；当前领域确有调试视图时，poll才在主线程创建会话。
    impl_->deferred_settings = settings;
    impl_->config_path       = config_path;
    impl_->output            = output;
    impl_->deferred_start    = true;
    impl_->layout_changed    = true;
}

void Display::start(const YAML::Node &settings, const std::filesystem::path &config_path,
                    const std::filesystem::path &output)
{
    const auto value = settings["recording"].as<std::string>("");
    auto file        = value.empty() ? (output.empty() ? std::filesystem::path{} : output / "debug.rrd")
                                     : config_path.parent_path() / value;
    if (!file.empty())
    {
        file = std::filesystem::absolute(file).lexically_normal();
        std::filesystem::create_directories(file.parent_path());
    }
    const auto backend = settings["backend"].as<std::string>("sdl");
    start(settings["spawn"].as<bool>(true), file.string(), settings["address"].as<std::string>(""), backend,
          settings["image_encoding"].as<std::string>("jpeg"), settings["hud_encoding"].as<std::string>("png"));
    std::cout << "Debug display: " << backend << '\n';
    if (!file.empty())
        std::cout << "Debug recording: " << file.string() << '\n';
}

void Display::shutdown() noexcept
{
    std::lock_guard lock(impl_->mutex);
    auto rerun = std::move(impl_->rerun);
    impl_->sdl.reset(); // 与初始化、poll 同属主线程；BuffDebug 须先退出。
    impl_->draw = false;
    impl_->deferred_start = false;
    impl_->layouts.clear();
    impl_->selected_layout.clear();
    impl_->layout_changed = false;
    if (rerun)
    {
        try
        {
            rerun->flush();
        }
        catch (const std::exception &error)
        {
            std::cerr << "Display shutdown: " << error.what() << '\n';
        }
    }
}

bool Display::image_due(double seconds, std::optional<double> last_image_time) const
{
    std::lock_guard lock(impl_->mutex);
    return impl_->image_due(seconds, last_image_time);
}

void Display::end_frame()
{
    std::lock_guard lock(impl_->mutex);
    if (impl_->sdl)
        impl_->sdl->end_frame();
}

void Display::poll(tool::PlaybackControl *playback, const std::function<void(int)> &on_key)
{
    YAML::Node settings;
    std::filesystem::path config, output;
    std::vector<DisplayView> views;
    std::vector<unsigned char> blueprint;
    bool launch = false, apply_layout = false;
    {
        std::lock_guard lock(impl_->mutex);
        const auto found = impl_->layouts.find(impl_->selected_layout);
        if (impl_->layout_changed && found != impl_->layouts.end())
        {
            views     = found->second.first;
            blueprint = found->second.second;
            if (impl_->deferred_start && !views.empty())
            {
                launch                = true;
                settings              = impl_->deferred_settings;
                config                = impl_->config_path;
                output                = impl_->output;
                impl_->deferred_start = false;
            }
            apply_layout          = launch || impl_->sdl || impl_->rerun;
            impl_->layout_changed = false;
        }
    }
    if (launch)
        start(settings, config, output);
    if (apply_layout)
        layout(views, blueprint);
    // 生命周期由主线程独占；SDL 的最新帧槽自行同步，不持 SDK 锁渲染。
    if (impl_->sdl)
        impl_->sdl->poll(playback, on_key);
    // 暂停后的单步必须显示刚处理的帧，不能因媒体时间间隔不足1/30秒被采样跳过。
    std::lock_guard lock(impl_->mutex);
    impl_->playback_paused = playback && playback->paused;
}

void Display::register_layout(const std::string &name, const std::vector<DisplayView> &views,
                              std::span<const unsigned char> blueprint)
{
    std::lock_guard lock(impl_->mutex);
    impl_->layouts[name] = {views, {blueprint.begin(), blueprint.end()}};
    if (impl_->selected_layout == name)
        impl_->layout_changed = true;
}

void Display::select_layout(const std::string &name)
{
    std::lock_guard lock(impl_->mutex);
    if (impl_->selected_layout != name)
    {
        impl_->selected_layout = name;
        impl_->layout_changed  = true;
    }
}

void Display::layout(const std::vector<DisplayView> &views, std::span<const unsigned char> blueprint)
{
    std::lock_guard lock(impl_->mutex);
    if (impl_->sdl)
        impl_->sdl->layout(views);
    else if (impl_->rerun)
        impl_->rerun->layout(views, blueprint);
}

void Display::set_time(double seconds)
{
    std::lock_guard lock(impl_->mutex);
    if ((!impl_->sdl && !impl_->rerun) || !std::isfinite(seconds))
        return;
    if (impl_->rerun)
        impl_->rerun->set_time(seconds);
    if (seconds != impl_->frame_time)
    {
        impl_->frame_time = seconds;
        // 图像提交前采样；数值仍逐帧记录，算法时间与FPS统计不受图像刷新率影响。
        impl_->draw = impl_->image_due(seconds, impl_->last_image_time);
        if (impl_->draw)
            impl_->last_image_time = seconds;
        if (impl_->sdl)
            impl_->sdl->begin_frame();
    }
}

void Display::display_add(const std::string &name, const cv::Mat &image,
                          const std::vector<DisplayText> &texts, const std::vector<DisplayPoint> &points,
                          const std::vector<DisplayLine> &lines, int width, int)
{
    std::lock_guard lock(impl_->mutex);
    if (!impl_->draw || image.empty() || name.empty())
        return;
    if (impl_->sdl)
        impl_->sdl->add(name, image, texts, points, lines, width);
    else if (impl_->rerun)
        impl_->rerun->add(name, image, texts, points, lines);
}

void Display::scalar(const std::string &path, double value)
{
    std::lock_guard lock(impl_->mutex);
    if (impl_->rerun)
        impl_->rerun->scalar(path, value);
}

void Display::points2d(const std::string &path, const std::vector<cv::Point2d> &points,
                       const std::vector<std::string> &labels)
{
    std::lock_guard lock(impl_->mutex);
    if (!impl_->draw)
        return;
    if (impl_->sdl)
        impl_->sdl->points(path, points, labels);
    else if (impl_->rerun)
        impl_->rerun->points(path, points, labels);
}

void Display::point3d(const std::string &path, double x, double y, double z)
{
    std::lock_guard lock(impl_->mutex);
    if (impl_->rerun)
        impl_->rerun->point3d(path, x, y, z);
}

} // namespace rm_utils
