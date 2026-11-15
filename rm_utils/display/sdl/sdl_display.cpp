#include "sdl_display.hpp"
#include "rm_utils/io/playback_control.hpp"

#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace rm_utils
{
namespace
{
void check(bool ok)
{
    if (!ok)
        throw std::runtime_error(std::string("SDL: ") + SDL_GetError());
}

cv::Scalar bgr(DisplayColor color)
{
    return {double(color.b), double(color.g), double(color.r)};
}
} // namespace

SdlDisplay::SdlDisplay()
{
    SDL_SetMainReady();
    check(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
}

SdlDisplay::~SdlDisplay()
{
    for (auto &[name, view] : windows_)
    {
        SDL_DestroyTexture(view.texture);
        SDL_DestroyRenderer(view.renderer);
        SDL_DestroyWindow(view.window);
    }
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
}

void SdlDisplay::layout(const std::vector<DisplayView> &views)
{
    std::lock_guard lock(mutex_);
    layout_set_ = true;
    names_.clear();
    for (const auto &view : views)
        names_.push_back(view.name);
}

void SdlDisplay::begin_frame()
{
    panels_.clear();
    raw_points_.clear();
    raw_labels_.clear();
}

void SdlDisplay::points(const std::string &path, const std::vector<cv::Point2d> &points,
                        const std::vector<std::string> &labels)
{
    const auto name = path.substr(0, path.find("/image/"));
    if (hidden(name))
        return;
    auto &dots = raw_points_[name];
    auto &texts = raw_labels_[name];
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        dots.push_back({cvRound(points[i].x), cvRound(points[i].y), {255, 80, 255, 255}, 3});
        if (i < labels.size())
            texts.push_back({labels[i], cvRound(points[i].x) + 4, cvRound(points[i].y) - 16,
                             16, {255, 80, 255, 255}});
    }
}

void SdlDisplay::add(const std::string &name, const cv::Mat &image, const std::vector<DisplayText> &texts,
                     const std::vector<DisplayPoint> &points, const std::vector<DisplayLine> &lines, int width)
{
    if (hidden(name) || image.empty())
        return;
    if (image.type() != CV_8UC3)
        throw std::runtime_error("SDL expects an 8-bit BGR image");
    // 先缩放再绘制，避免全尺寸叠图、JPEG 和透明 PNG 的编码成本。
    const double scale = double(std::max(1, width)) / image.cols;
    cv::Mat panel;
    cv::resize(image, panel, {std::max(1, width), std::max(1, cvRound(image.rows * scale))});
    const auto point = [&](int x, int y) { return cv::Point(cvRound(x * scale), cvRound(y * scale)); };
    for (const auto &line : lines)
        cv::line(panel, point(line.x1, line.y1), point(line.x2, line.y2), bgr(line.color),
                 std::max(1, cvRound(line.thickness * scale)), cv::LINE_AA);
    const auto draw_points = [&](const auto &dots)
    {
        for (const auto &dot : dots)
            cv::circle(panel, point(dot.x, dot.y), std::max(1, cvRound(dot.size * scale)),
                       bgr(dot.color), cv::FILLED, cv::LINE_AA);
    };
    draw_points(points);
    draw_points(raw_points_[name]);
    cv::Size hud;
    for (const auto &text : texts)
    {
        if (!text.image_overlay)
            continue;
        int baseline = 0;
        const auto size = cv::getTextSize(text.content, cv::FONT_HERSHEY_SIMPLEX,
                                          text.size / 24. * scale, 1, &baseline);
        const auto origin = point(text.x, text.y);
        hud.width = std::max(hud.width, origin.x + size.width + 5);
        hud.height = std::max(hud.height, origin.y + size.height + baseline + 4);
    }
    const cv::Rect background = cv::Rect({}, hud) & cv::Rect({}, panel.size());
    if (!background.empty())
        panel(background) *= .3;
    const auto draw_texts = [&](const auto &labels)
    {
        for (const auto &text : labels)
        {
            const double font_scale = text.size / 24. * scale;
            const int thickness = std::max(1, cvRound(text.thickness * scale));
            int baseline = 0;
            const auto size = cv::getTextSize(text.content, cv::FONT_HERSHEY_SIMPLEX, font_scale,
                                              thickness, &baseline);
            auto origin = point(text.x, text.y);
            origin.y += size.height;
            cv::putText(panel, text.content, origin + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX,
                         font_scale, {0, 0, 0}, thickness + 2, cv::LINE_AA);
            cv::putText(panel, text.content, origin, cv::FONT_HERSHEY_SIMPLEX,
                         font_scale, bgr(text.color), thickness, cv::LINE_AA);
        }
    };
    draw_texts(raw_labels_[name]);
    draw_texts(texts);
    panels_[name] = std::move(panel);
}

bool SdlDisplay::hidden(const std::string &name)
{
    std::lock_guard lock(mutex_);
    return closed_ || closed_names_.contains(name);
}

void SdlDisplay::end_frame()
{
    // panels_/raw_*仅生产者使用；pending_交给主线程，落后时整组替换，避免图像队列累积。
    if (closed_ || panels_.empty())
        return;
    std::lock_guard lock(mutex_);
    std::unordered_map<std::string, cv::Mat> frames;
    if (!layout_set_)
        frames = std::move(panels_); // 独立相机预览按自己的名称创建窗口。
    else
        for (const auto &name : names_)
            if (auto found = panels_.find(name); found != panels_.end())
                frames.emplace(name, std::move(found->second));
    for (const auto &name : closed_names_)
        frames.erase(name);
    pending_ = std::move(frames); // 原子发布同一曝光帧，旧的待显示图像直接替换。
}

void SdlDisplay::poll(tool::PlaybackControl *playback, const std::function<void(int)> &on_key)
{
    // 只由主线程调用；关闭事件按 windowID 路由，关闭一个模块不影响其余模块。
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        if (on_key)
        {
            if (event.type == SDL_KEYDOWN && !event.key.repeat)
                on_key(event.key.keysym.sym);
            else if (event.type == SDL_QUIT ||
                     (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE))
                on_key(27);
        }
        if (playback && event.type == SDL_KEYDOWN && !event.key.repeat)
        {
            if (event.key.keysym.sym == SDLK_SPACE) playback->key(' ');
            if (event.key.keysym.sym == SDLK_n) playback->key('n');
            if (event.key.keysym.sym == SDLK_q) playback->key('q');
            if (event.key.keysym.sym == SDLK_ESCAPE) playback->key(27);
        }
        if (event.type == SDL_QUIT)
        {
            if (playback) playback->quit = true;
            closed_ = true;
            for (auto &[name, view] : windows_)
                SDL_HideWindow(view.window);
        }
        else if ((event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE) ||
                 (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE))
        {
            const auto id = event.type == SDL_KEYDOWN ? event.key.windowID : event.window.windowID;
            for (auto &[name, view] : windows_)
                if (SDL_GetWindowID(view.window) == id)
                {
                    std::lock_guard lock(mutex_);
                    closed_names_.insert(name);
                    pending_.erase(name);
                    SDL_HideWindow(view.window);
                    break;
                }
        }
    }
    if (closed_)
        return;
    std::unordered_map<std::string, cv::Mat> images;
    std::vector<std::string> order;
    bool layout_set;
    {
        std::lock_guard lock(mutex_);
        images = std::move(pending_);
        order      = names_;
        layout_set = layout_set_;
        for (const auto &name : closed_names_)
            images.erase(name);
    }
    if (!layout_set)
    {
        for (const auto &[name, image] : images)
            order.push_back(name);
        std::sort(order.begin(), order.end());
    }
    for (const auto &name : order)
    {
        const auto found = images.find(name);
        if (found == images.end() || found->second.empty())
            continue;
        const auto &image = found->second;
        auto &view = windows_[name];
        if (!view.window)
        {
            SDL_Rect bounds{};
            const bool bounded = SDL_GetDisplayUsableBounds(0, &bounds) == 0;
            const double scale = bounded ? std::min({1., bounds.w * .8 / image.cols, bounds.h * .8 / image.rows}) : 1.;
            const int width = std::max(1, int(image.cols * scale));
            const int height = std::max(1, int(image.rows * scale));
            const int offset = int(windows_.size() - 1) * 32;
            const int x = bounded ? bounds.x + std::min(offset, std::max(0, bounds.w - width)) : SDL_WINDOWPOS_CENTERED;
            const int y = bounded ? bounds.y + std::min(offset, std::max(0, bounds.h - height)) : SDL_WINDOWPOS_CENTERED;
            const auto title   = "mas_vision | " + name;
            view.window = SDL_CreateWindow(title.c_str(), x, y, width, height,
                                           SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
            check(view.window != nullptr);
            view.renderer = SDL_CreateRenderer(view.window, -1, SDL_RENDERER_ACCELERATED);
            if (!view.renderer)
                view.renderer = SDL_CreateRenderer(view.window, -1, SDL_RENDERER_SOFTWARE);
            check(view.renderer != nullptr);
            SDL_RendererInfo info{};
            if (SDL_GetRendererInfo(view.renderer, &info) == 0)
                std::cout << "SDL renderer [" << name << "]: " << info.name
                          << ((info.flags & SDL_RENDERER_ACCELERATED) ? " (accelerated)\n" : " (software)\n");
        }
        if (view.texture_size != image.size())
        {
            SDL_DestroyTexture(view.texture);
            view.texture = SDL_CreateTexture(view.renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING,
                                             image.cols, image.rows);
            check(view.texture != nullptr);
            view.texture_size = image.size();
            check(SDL_RenderSetLogicalSize(view.renderer, image.cols, image.rows) == 0);
        }
        check(SDL_UpdateTexture(view.texture, nullptr, image.data, int(image.step)) == 0);
        check(SDL_RenderClear(view.renderer) == 0);
        check(SDL_RenderCopy(view.renderer, view.texture, nullptr, nullptr) == 0);
        SDL_RenderPresent(view.renderer); // 不启用 VSync，每个模块呈现在自己的系统窗口。
    }
}

} // namespace rm_utils
