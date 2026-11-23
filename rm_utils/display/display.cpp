#include "display.hpp"
#include "mas_log.hpp"
#include "rerun/rerun_display.hpp"
#include "sdl/sdl_display.hpp"
#include "rm_utils/io/playback_control.hpp"

#include <cmath>
#include <format>
#include <mutex>
#include <stdexcept>
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
    struct Layout
    {
        std::vector<DisplayView> views;
        std::vector<unsigned char> blueprint;
    };

    std::mutex mutex;
    std::unique_ptr<SdlDisplay> sdl;
    std::unique_ptr<RerunDisplay> rerun;

    double frame_time = NAN;
    std::optional<double> last_image_time;
    bool draw            = false;
    bool playback_paused = false;

    // 启动配置和后端生命周期只由主线程管理。
    YAML::Node deferred_settings;
    std::filesystem::path config_path, output;
    bool deferred_start = false;

    // 算法线程可注册布局，读写均需持 mutex。
    std::unordered_map<std::string, Layout> layouts;
    std::string selected_layout;
    bool layout_changed = false;

    bool image_due(double seconds, std::optional<double> last) const
    {
        if ((!sdl && !rerun) || !std::isfinite(seconds))
            return false;

        if ((sdl && playback_paused) || !last || seconds < *last)
            return true;

        const double fps = sdl ? Display::kSdlImageFps : Display::kImageFps;
        return std::floor(seconds * fps) != std::floor(*last * fps);
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

void Display::start(bool spawn, const std::string &file, const std::string &address, const std::string &backend,
                    const std::string &image_encoding, const std::string &hud_encoding)
{
    if (backend != "rerun" && backend != "sdl")
        throw std::runtime_error("display.backend must be rerun or sdl");
    {
        std::lock_guard lock(impl_->mutex);
        if (impl_->sdl || impl_->rerun)
            throw std::runtime_error("Display session already started");
    }

    // 创建后端时不持提交锁；成功后再发布，失败不留下半开会话。
    auto sdl = backend == "sdl" ? std::make_unique<SdlDisplay>() : nullptr;
    std::unique_ptr<RerunDisplay> rerun;
    if (!sdl)
        rerun = std::make_unique<RerunDisplay>(spawn, file, address, image_encoding, hud_encoding);

    std::lock_guard lock(impl_->mutex);
    impl_->sdl        = std::move(sdl);
    impl_->rerun      = std::move(rerun);
    impl_->frame_time = NAN;
    impl_->last_image_time.reset();
    impl_->draw            = false;
    impl_->playback_paused = false;
}

void Display::start(const std::filesystem::path &config_path, const std::filesystem::path &output)
{
    const auto settings = YAML::LoadFile(config_path.string())["display"];
    std::lock_guard lock(impl_->mutex);
    if (impl_->sdl || impl_->rerun || impl_->deferred_start)
        throw std::runtime_error("Display session already started");

    // 等当前领域注册非空布局后，再由主线程创建后端。
    impl_->deferred_settings = settings;
    impl_->config_path       = config_path;
    impl_->output            = output;
    impl_->deferred_start    = true;
    impl_->layout_changed    = true;
}

void Display::start(const YAML::Node &settings, const std::filesystem::path &config_path,
                    const std::filesystem::path &output)
{
    const auto backend = settings["backend"].as<std::string>("sdl");
    std::filesystem::path file;

    // 只有 Rerun 使用录制路径。
    if (backend == "rerun")
    {
        const auto value = settings["recording"].as<std::string>("");
        if (!value.empty())
            file = config_path.parent_path() / value;
        else if (!output.empty())
            file = output / "debug.rrd";

        if (!file.empty())
        {
            file = std::filesystem::absolute(file).lexically_normal();
            std::filesystem::create_directories(file.parent_path());
        }
    }

    if (backend == "sdl")
        start(false, {}, {}, backend);
    else
        start(settings["spawn"].as<bool>(true), file.string(), settings["address"].as<std::string>(""), backend,
              settings["image_encoding"].as<std::string>("jpeg"), settings["hud_encoding"].as<std::string>("png"));

    MAS_LOG_INFO("Debug display: {}", backend);
    if (!file.empty())
        MAS_LOG_INFO("Debug recording: {}", file.string());
}

void Display::shutdown() noexcept
{
    std::lock_guard lock(impl_->mutex);
    auto rerun = std::move(impl_->rerun);
    impl_->sdl.reset();
    impl_->draw       = false;
    impl_->frame_time = NAN;
    impl_->last_image_time.reset();
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
            MAS_LOG_WARN("Display shutdown: {}", error.what());
        }
    }
}

void Display::update_layout()
{
    Impl::Layout selected;
    bool launch = false;

    {
        std::lock_guard lock(impl_->mutex);
        const auto found = impl_->layouts.find(impl_->selected_layout);
        if (!impl_->layout_changed || found == impl_->layouts.end())
            return;

        selected = found->second;
        launch   = impl_->deferred_start && !selected.views.empty();
        if (launch)
            impl_->deferred_start = false;

        impl_->layout_changed = false;
    }

    // 启动配置由主线程独占，布局接口自行加锁。
    if (launch)
        start(impl_->deferred_settings, impl_->config_path, impl_->output);

    if (impl_->sdl || impl_->rerun)
        layout(selected.views, selected.blueprint);
}

void Display::poll(tool::PlaybackControl *playback, const std::function<void(int)> &on_key)
{
    update_layout();

    // 后端生命周期由主线程独占；SDL 自行同步最新帧，绘制时不占提交锁。
    if (impl_->sdl)
        impl_->sdl->poll(playback, on_key);

    // 暂停单步仍显示当前帧，不受图像采样间隔限制。
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
        // 图像采样不影响算法时间和数值记录。
        impl_->draw = impl_->image_due(seconds, impl_->last_image_time);
        if (impl_->draw)
            impl_->last_image_time = seconds;
        if (impl_->sdl)
            impl_->sdl->begin_frame();
    }
}

bool Display::image_due(double seconds, std::optional<double> last_image_time) const
{
    std::lock_guard lock(impl_->mutex);
    return impl_->image_due(seconds, last_image_time);
}

bool Display::image_due() const
{
    std::lock_guard lock(impl_->mutex);
    return impl_->draw;
}

bool Display::values_enabled() const
{
    std::lock_guard lock(impl_->mutex);
    return bool(impl_->rerun);
}

void Display::end_frame()
{
    std::lock_guard lock(impl_->mutex);
    if (impl_->sdl)
        impl_->sdl->end_frame();
}

void Display::display_add(const std::string &name, const cv::Mat &image, const std::vector<DisplayText> &texts,
                          const std::vector<DisplayPoint> &points, const std::vector<DisplayLine> &lines, int width,
                          int)
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
