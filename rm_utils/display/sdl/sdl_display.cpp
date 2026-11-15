#include "sdl_display.hpp"
#include "rm_utils/io/playback_control.hpp"

#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
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
        for (const auto &text : view.text_cache)
            SDL_DestroyTexture(text.texture);
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
    // 不缩放、不叠图；同帧三个窗口只增加图像引用，像素存储由 cv::Mat 共同持有。
    Panel panel{image, texts, points, lines, std::max(1, width)};
    panel.points.insert(panel.points.end(), raw_points_[name].begin(), raw_points_[name].end());
    panel.texts.insert(panel.texts.end(), raw_labels_[name].begin(), raw_labels_[name].end());
    panels_[name] = std::move(panel);
}

bool SdlDisplay::hidden(const std::string &name)
{
    std::lock_guard lock(mutex_);
    return closed_ || closed_names_.contains(name);
}

void SdlDisplay::end_frame()
{
    // 发布完整同帧任务；落后时整组替换，算法线程不等待主线程绘制。
    if (closed_ || panels_.empty())
        return;
    std::lock_guard lock(mutex_);
    std::unordered_map<std::string, Panel> frames;
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

const SdlDisplay::CachedText &SdlDisplay::text_texture(Window &view, const DisplayText &text)
{
    for (const auto &cached : view.text_cache)
    {
        const auto &style = cached.style;
        if (style.content == text.content && style.size == text.size && style.thickness == text.thickness &&
            style.color.r == text.color.r && style.color.g == text.color.g &&
            style.color.b == text.color.b && style.color.a == text.color.a)
            return cached;
    }

    // 借鉴 MAS 的文字纹理缓存；只在内容/样式变化时生成透明文字图。
    CachedText cached;
    cached.style = text;
    const int thickness = std::max(1, text.thickness);
    cached.padding = thickness + 3;
    cached.size = cv::getTextSize(text.content, cv::FONT_HERSHEY_SIMPLEX, text.size / 24.,
                                  thickness, &cached.baseline);
    cv::Mat glyph(cached.size.height + cached.baseline + cached.padding * 2,
                  cached.size.width + cached.padding * 2, CV_8UC4, cv::Scalar(0, 0, 0, 0));
    const cv::Point origin(cached.padding, cached.padding + cached.size.height);
    cv::putText(glyph, text.content, origin + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX,
                 text.size / 24., {0, 0, 0, 255}, thickness + 2, cv::LINE_AA);
    cv::putText(glyph, text.content, origin, cv::FONT_HERSHEY_SIMPLEX, text.size / 24.,
                 {double(text.color.b), double(text.color.g), double(text.color.r), double(text.color.a)},
                 thickness, cv::LINE_AA);
    cached.texture = SDL_CreateTexture(view.renderer, SDL_PIXELFORMAT_BGRA32, SDL_TEXTUREACCESS_STATIC,
                                        glyph.cols, glyph.rows);
    check(cached.texture != nullptr);
    // 先挂入所有权容器，后面的SDL操作失败时仍由窗口析构释放。
    if (view.text_cache.size() >= 50)
    {
        SDL_DestroyTexture(view.text_cache.front().texture);
        view.text_cache.erase(view.text_cache.begin());
    }
    view.text_cache.push_back(std::move(cached));
    auto &result = view.text_cache.back();
    check(SDL_SetTextureBlendMode(result.texture, SDL_BLENDMODE_BLEND) == 0);
    check(SDL_UpdateTexture(result.texture, nullptr, glyph.data, int(glyph.step)) == 0);
    return result;
}

void SdlDisplay::render(Window &view, const Panel &panel)
{
    const auto &image = panel.image;
    check(SDL_UpdateTexture(view.texture, nullptr, image.data, int(image.step)) == 0);
    check(SDL_SetRenderDrawColor(view.renderer, 0, 0, 0, 255) == 0);
    check(SDL_RenderClear(view.renderer) == 0);
    // 保持原图坐标，由SDL纹理缩放；不再为每个窗口在CPU缩放、复制整张底图。
    check(SDL_RenderCopy(view.renderer, view.texture, nullptr, nullptr) == 0);
    check(SDL_SetRenderDrawBlendMode(view.renderer, SDL_BLENDMODE_BLEND) == 0);
    for (const auto &line : panel.lines)
    {
        check(SDL_SetRenderDrawColor(view.renderer, line.color.r, line.color.g, line.color.b, line.color.a) == 0);
        const bool horizontal = std::abs(line.x2 - line.x1) > std::abs(line.y2 - line.y1);
        const int thickness = std::max(1, line.thickness);
        for (int i = 0; i < thickness; ++i)
        {
            const int dx = horizontal ? 0 : i - thickness / 2;
            const int dy = horizontal ? i - thickness / 2 : 0;
            check(SDL_RenderDrawLine(view.renderer, line.x1 + dx, line.y1 + dy,
                                      line.x2 + dx, line.y2 + dy) == 0);
        }
    }
    for (const auto &point : panel.points)
    {
        check(SDL_SetRenderDrawColor(view.renderer, point.color.r, point.color.g, point.color.b, point.color.a) == 0);
        const int radius = std::max(1, point.size);
        for (int y = -radius; y <= radius; ++y)
        {
            const int dx = int(std::sqrt(double(radius * radius - y * y)));
            check(SDL_RenderDrawLine(view.renderer, point.x - dx, point.y + y,
                                      point.x + dx, point.y + y) == 0);
        }
    }
    // 只有状态文字参与左上角背景范围；点旁标签保持原图亮度。
    cv::Size hud;
    for (const auto &text : panel.texts)
        if (text.image_overlay && !text.content.empty())
        {
            const auto &cached = text_texture(view, text);
            hud.width = std::max(hud.width, text.x + cached.size.width + 5);
            hud.height = std::max(hud.height, text.y + cached.size.height + cached.baseline + 4);
        }
    const cv::Rect background = cv::Rect({}, hud) & cv::Rect({}, image.size());
    if (!background.empty())
    {
        const SDL_Rect rect{background.x, background.y, background.width, background.height};
        check(SDL_SetRenderDrawColor(view.renderer, 0, 0, 0, 179) == 0);
        check(SDL_RenderFillRect(view.renderer, &rect) == 0);
    }
    for (const auto &text : panel.texts)
        if (!text.content.empty())
        {
            const auto &cached = text_texture(view, text);
            const SDL_Rect rect{text.x - cached.padding, text.y - cached.padding,
                                cached.size.width + cached.padding * 2,
                                cached.size.height + cached.baseline + cached.padding * 2};
            check(SDL_RenderCopy(view.renderer, cached.texture, nullptr, &rect) == 0);
        }
    SDL_RenderPresent(view.renderer);
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
    std::unordered_map<std::string, Panel> images;
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
        if (found == images.end() || found->second.image.empty())
            continue;
        const auto &panel = found->second;
        const auto &image = panel.image;
        const cv::Size display_size(panel.width, std::max(1, cvRound(double(image.rows) * panel.width / image.cols)));
        auto &view = windows_[name];
        if (!view.window)
        {
            SDL_Rect bounds{};
            const bool bounded = SDL_GetDisplayUsableBounds(0, &bounds) == 0;
            const double scale = bounded ? std::min({1., bounds.w * .8 / display_size.width, bounds.h * .8 / display_size.height}) : 1.;
            const int width = std::max(1, int(display_size.width * scale));
            const int height = std::max(1, int(display_size.height * scale));
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
        render(view, panel); // 已释放提交锁；纹理上传、缩放及绘制均由主线程完成。
    }
}

} // namespace rm_utils
