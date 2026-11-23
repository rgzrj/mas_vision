#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <chrono>

#include <algorithm>
#include <cmath>
#include <time.h>
#include <iomanip>
#include <memory>
#include <numeric>
#include <thread>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;

void check(bool ok)
{
    if (!ok)
        throw std::runtime_error(SDL_GetError());
}

struct View
{
    std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window{nullptr, SDL_DestroyWindow};
    std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> renderer{nullptr, SDL_DestroyRenderer};
    std::unique_ptr<SDL_Texture, decltype(&SDL_DestroyTexture)> texture{nullptr, SDL_DestroyTexture};
};

double milliseconds(Clock::time_point a, Clock::time_point b)
{
    return std::chrono::duration<double, std::milli>(b - a).count();
}

double mean(const std::vector<double> &values)
{
    return std::accumulate(values.begin(), values.end(), 0.) / values.size();
}

double process_cpu_seconds()
{
    timespec usage{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &usage) != 0)
        throw std::runtime_error("Cannot read process CPU time");
    return double(usage.tv_sec) + double(usage.tv_nsec) / 1e9;
}

// 独立比较上传内核；两种路径都复用纹理和CPU缓冲，同一底图上传到独立窗口。
void measure(cv::Mat &image, int windows, int frames, bool bgra, int round)
{
    const Uint32 format = bgra ? SDL_PIXELFORMAT_BGRA32 : SDL_PIXELFORMAT_BGR24;
    std::vector<View> views(windows);
    std::string renderer_name;
    for (int i = 0; i < windows; ++i)
    {
        auto &view = views[i];
        view.window.reset(SDL_CreateWindow("SDL upload profile", 40 + i * 40, 40 + i * 40, 720,
                                           std::max(1, 720 * image.rows / image.cols), SDL_WINDOW_SHOWN));
        check(bool(view.window));
        view.renderer.reset(SDL_CreateRenderer(view.window.get(), -1, SDL_RENDERER_ACCELERATED));
        if (!view.renderer)
            view.renderer.reset(SDL_CreateRenderer(view.window.get(), -1, SDL_RENDERER_SOFTWARE));
        check(bool(view.renderer));
        SDL_RendererInfo info{};
        check(SDL_GetRendererInfo(view.renderer.get(), &info) == 0);
        renderer_name = info.name;
        if (bgra && std::find(info.texture_formats, info.texture_formats + info.num_texture_formats, format) ==
                        info.texture_formats + info.num_texture_formats)
            throw std::runtime_error("Renderer does not advertise native BGRA32; comparison unavailable");
        view.texture.reset(
            SDL_CreateTexture(view.renderer.get(), format, SDL_TEXTUREACCESS_STREAMING, image.cols, image.rows));
        check(bool(view.texture));
    }

    std::vector<double> convert, upload, draw, total;
    cv::Mat converted;
    const auto interval = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1. / 30));
    auto deadline = Clock::now(), began = deadline;
    double cpu_began = process_cpu_seconds();
    for (int i = -30; i < frames; ++i)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
            if (event.type == SDL_QUIT ||
                (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE))
                throw std::runtime_error("Profiling stopped by window close");
        if (i == 0)
        {
            began     = Clock::now();
            deadline  = began;
            cpu_began = process_cpu_seconds();
        }
        const auto start = Clock::now();
        if (bgra)
            cv::cvtColor(image, converted, cv::COLOR_BGR2BGRA);
        const auto prepared = Clock::now();
        const auto &pixels  = bgra ? converted : image;
        for (auto &view : views)
            check(SDL_UpdateTexture(view.texture.get(), nullptr, pixels.data, int(pixels.step)) == 0);
        const auto uploaded = Clock::now();
        for (auto &view : views)
        {
            check(SDL_RenderClear(view.renderer.get()) == 0);
            check(SDL_RenderCopy(view.renderer.get(), view.texture.get(), nullptr, nullptr) == 0);
            SDL_RenderPresent(view.renderer.get());
        }
        const auto presented = Clock::now();
        if (i >= 0)
        {
            convert.push_back(milliseconds(start, prepared));
            upload.push_back(milliseconds(prepared, uploaded));
            draw.push_back(milliseconds(uploaded, presented));
            total.push_back(milliseconds(start, presented));
        }
        // 两种路径都按30Hz测量；超时不补画，避免用无限刷新制造不符合实机的负载。
        deadline = std::max(deadline + interval, presented);
        std::this_thread::sleep_until(deadline);
    }
    const double seconds = std::chrono::duration<double>(Clock::now() - began).count();
    const double cpu     = process_cpu_seconds() - cpu_began;
    std::sort(total.begin(), total.end());
    std::cout << (bgra ? "shared_bgra32" : "bgr24") << ',' << round << ',' << image.cols << ',' << image.rows << ','
              << windows << ',' << frames << ',' << frames / seconds << ',' << cpu / seconds * 100 << ','
              << mean(convert) << ',' << mean(upload) << ',' << mean(draw) << ','
              << total[std::size_t(std::ceil(frames * .95)) - 1] << ',' << renderer_name << std::endl;
}
} // namespace

int main(int argc, char *argv[])
{
    try
    {
        if (argc == 2 && std::string(argv[1]) == "--help")
        {
            std::cout << "sdl_upload_profile [width height windows frames]\n"
                         "Defaults: 1280 1024 3 300. Runs BGR/BGRA/BGRA/BGR at 30Hz, OpenCV threads=1.\n"
                         "Measures conversion, upload, render/present and process CPU; excludes algorithms and text.\n";
            return 0;
        }
        if (argc != 1 && argc != 5)
            throw std::runtime_error("Provide width height windows frames, or no arguments");
        const int width   = argc == 5 ? std::stoi(argv[1]) : 1280;
        const int height  = argc == 5 ? std::stoi(argv[2]) : 1024;
        const int windows = argc == 5 ? std::stoi(argv[3]) : 3;
        const int frames  = argc == 5 ? std::stoi(argv[4]) : 300;
        if (width <= 0 || height <= 0 || windows < 1 || windows > 3 || frames <= 0)
            throw std::runtime_error("Dimensions/frames must be positive; windows must be 1..3");
        cv::setNumThreads(1); // 独立进程，不修改实机的OpenCV线程策略。
        SDL_SetMainReady();
        check(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
        struct Quit
        {
            ~Quit()
            {
                SDL_QuitSubSystem(SDL_INIT_VIDEO);
            }
        } quit;
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        cv::Mat image(height, width, CV_8UC3, cv::Scalar(40, 60, 80));
        std::cout << "path,round,width,height,windows,frames,fps,cpu_percent,convert_mean_ms,upload_mean_ms,"
                     "draw_mean_ms,total_p95_ms,renderer\n"
                  << std::fixed << std::setprecision(6);
        int round = 0;
        for (bool bgra : {false, true, true, false})
            measure(image, windows, frames, bgra, ++round);
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "sdl_upload_profile: " << error.what() << '\n';
        return 1;
    }
}
