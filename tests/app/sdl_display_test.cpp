#include "display.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "rm_utils/io/playback_control.hpp"
#include "tests/common/test.hpp"
#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <opencv2/imgcodecs.hpp>
#include <array>
#include <iostream>

namespace
{
// 本用例仅创建少量窗口；扫描 ID，避免依赖哪个模块先创建窗口。
SDL_Window *window(const std::string &name)
{
    for (Uint32 id = 1; id <= 128; ++id)
        if (auto *value = SDL_GetWindowFromID(id); value && SDL_GetWindowTitle(value) == "mas_vision | " + name)
            return value;
    return nullptr;
}

int visible_windows()
{
    int count = 0;
    for (Uint32 id = 1; id <= 128; ++id)
        if (auto *value = SDL_GetWindowFromID(id); value && (SDL_GetWindowFlags(value) & SDL_WINDOW_SHOWN))
            ++count;
    return count;
}

cv::Mat capture(SDL_Window *value)
{
    tests::require(value != nullptr, "Missing independent SDL window");
    int width = 0, height = 0;
    SDL_GetWindowSize(value, &width, &height);
    tests::require(width == 320 && height == 240, "Module window contains a combined image");
    cv::Mat image(height, width, CV_8UC3);
    tests::require(SDL_RenderReadPixels(SDL_GetRenderer(value), nullptr, SDL_PIXELFORMAT_BGR24,
                                        image.data, int(image.step)) == 0, SDL_GetError());
    return image;
}
}

int main(int argc, char **argv)
{
    try
    {
        using tests::require;
        SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
        SDL_SetHint(SDL_HINT_RENDER_DRIVER, "software");
        tests::TemporaryDirectory temporary;
        const auto output = argc > 1 ? tests::argument_path(argv[1]) : temporary.path;
        std::filesystem::create_directories(output);
        auto &display = rm_utils::Display::getInstance();
        const auto file = output / "sdl_numeric.rrd";
        display.start(false, tool::utf8(file), {}, "sdl");
        require(display.image_due(0, std::nullopt) && !display.image_due(.001, 0.) &&
                    !display.image_due(.02, 0.) && display.image_due(.04, 0.) && display.image_due(0, 1.),
                "SDL image submission is not sampled at 30Hz or cannot rewind");
        display.layout({{"detect", 128}, {"track", 320}, {"shoot", 288}});
        cv::Mat detect(240, 320, CV_8UC3, cv::Scalar(10, 20, 30));
        cv::Mat track(240, 320, CV_8UC3, cv::Scalar(40, 50, 60));
        cv::Mat shoot(240, 320, CV_8UC3, cv::Scalar(70, 80, 90));
        display.set_time(0);
        display.points2d("detect/image/raw_pose", {{100, 100}}, {"raw P0"});
        display.display_add("shoot", shoot, {}, {}, {}, 320);
        display.display_add("track", track, {}, {}, {}, 320);
        display.display_add("detect", detect,
                            {{"Process FPS: 80", 10, 10, 16, {255, 255, 0, 255}, 1, true}},
                            {}, {{20, 150, 200, 150, 1, {0, 255, 0, 255}}}, 320);
        require(detect.u->refcount > 1 && track.u->refcount > 1 && shoot.u->refcount > 1,
                "SDL still builds pixel panels on the algorithm thread instead of retaining image references");
        display.end_frame();
        display.poll();
        auto *dw = window("detect"), *tw = window("track"), *sw = window("shoot");
        require(dw && tw && sw && dw != tw && tw != sw && dw != sw && visible_windows() == 3,
                "Enabled modules did not create three independent system windows");
        const auto first = capture(dw);
        require(first.at<cv::Vec3b>(200, 20) == cv::Vec3b(10, 20, 30) &&
                    capture(tw).at<cv::Vec3b>(200, 20) == cv::Vec3b(40, 50, 60) &&
                    capture(sw).at<cv::Vec3b>(200, 20) == cv::Vec3b(70, 80, 90),
                "Images leaked between module windows or BGR channels changed");
        require(first.at<cv::Vec3b>(100, 100) == cv::Vec3b(255, 80, 255), "Raw point missing");
        require(first.at<cv::Vec3b>(150, 100)[1] > 200, "Overlay line missing");
        cv::Mat yellow;
        cv::inRange(first(cv::Rect(0, 0, 320, 40)), cv::Scalar(0, 180, 180), cv::Scalar(80, 255, 255), yellow);
        require(cv::countNonZero(yellow) > 10, "Colored FPS text missing");
        require(cv::norm(detect, cv::Mat(detect.size(), detect.type(), cv::Scalar(10, 20, 30)), cv::NORM_INF) == 0,
                "SDL modified the original image");
        cv::imwrite(tool::utf8(output / "detect_window.png"), first);

        // 采样间隔内不提交新图；图像跳过不影响数值记录。
        display.set_time(.001);
        display.display_add("detect", cv::Mat::zeros(detect.size(), detect.type()), {}, {}, {}, 320);
        display.scalar("test/value", 0);
        display.end_frame();
        display.poll();
        require(capture(dw).at<cv::Vec3b>(200, 20) == cv::Vec3b(10, 20, 30),
                "A sampled-out image replaced the displayed frame");
        // 连续发布两组可显示帧但不poll；只保留最新帧，临时图像离开作用域后仍有效。
        for (int i = 1; i <= 2; ++i)
        {
            display.set_time(i / 25.);
            cv::Mat newer(240, 320, CV_8UC3, cv::Scalar(100 + i, 110, 120));
            for (const auto *name : {"detect", "track", "shoot"})
                display.display_add(name, newer, {}, {}, {}, 320);
            display.scalar("test/value", i);
            display.end_frame();
        }
        display.poll();
        for (auto *value : {dw, tw, sw})
        {
            const auto image = capture(value);
            require(image.at<cv::Vec3b>(100, 100) == cv::Vec3b(102, 110, 120) &&
                        image.at<cv::Vec3b>(150, 100) == cv::Vec3b(102, 110, 120),
                    "A module retained an old frame or stale overlays");
        }
        SDL_Event close{};
        close.type = SDL_WINDOWEVENT;
        close.window.event = SDL_WINDOWEVENT_CLOSE;
        close.window.windowID = SDL_GetWindowID(tw);
        SDL_PushEvent(&close);
        tool::PlaybackControl controls;
        display.poll(&controls);
        require(!controls.quit && visible_windows() == 2 && !(SDL_GetWindowFlags(tw) & SDL_WINDOW_SHOWN),
                "Closing track stopped playback or hid another module");
        display.set_time(1);
        for (const auto *name : {"detect", "track", "shoot"})
            display.display_add(name, detect, {}, {}, {}, 320);
        display.end_frame();
        display.poll();
        require(visible_windows() == 2 && !(SDL_GetWindowFlags(tw) & SDL_WINDOW_SHOWN) &&
                    capture(sw).at<cv::Vec3b>(200, 20) == cv::Vec3b(10, 20, 30),
                "Closed window reopened or blocked the remaining windows");
        const auto key = [&](SDL_Keycode value)
        {
            SDL_Event event{};
            event.type = SDL_KEYDOWN;
            event.key.windowID = SDL_GetWindowID(sw);
            event.key.keysym.sym = value;
            SDL_PushEvent(&event);
            display.poll(&controls);
        };
        key(SDLK_SPACE);
        require(controls.paused && !controls.take_step(), "Window keyboard did not pause playback");
        display.set_time(1.001); // 与上一帧1.0秒处于同一个采样区间。
        display.display_add("detect", cv::Mat(detect.size(), detect.type(), cv::Scalar(80, 90, 100)), {}, {}, {}, 320);
        display.end_frame();
        display.poll(&controls);
        require(capture(dw).at<cv::Vec3b>(200, 20) == cv::Vec3b(80, 90, 100),
                "Paused replay step was hidden by the SDL image sampling limit");
        key(SDLK_n);
        require(controls.take_step() && !controls.take_step(), "Window single step repeated");
        key(SDLK_SPACE);
        require(!controls.paused, "Window keyboard did not resume playback");
        key(SDLK_q);
        require(controls.quit, "Window keyboard did not quit playback");
        display.shutdown();
        require(visible_windows() == 0 && std::filesystem::file_size(file) > 100,
                "Shutdown leaked windows or lost numeric RRD data");

        // 三个 debug 开关的全部 8 种组合；即便错误提交了禁用模块，也不得打开它。
        const std::array<const char *, 3> names{"detect", "track", "shoot"};
        for (unsigned mask = 0; mask < 8; ++mask)
        {
            display.start(false, {}, {}, "sdl");
            std::vector<rm_utils::DisplayView> views;
            for (unsigned i = 0; i < names.size(); ++i)
                if (mask & (1u << i)) views.push_back({names[i]});
            display.layout(views);
            display.set_time(0);
            for (auto *name : names)
                display.display_add(name, detect, {}, {}, {}, 320);
            display.end_frame();
            display.poll();
            for (unsigned i = 0; i < names.size(); ++i)
                require(bool(window(names[i])) == bool(mask & (1u << i)), "Module debug switch ignored");
            display.shutdown();
            require(visible_windows() == 0, "Module mask test leaked a window");
        }
        // 通用布局不预设打符名称；其他应用也能复用，未列出的窗口不得出现。
        display.start(false, {}, {}, "sdl");
        display.layout({{"calibration"}});
        display.set_time(0);
        display.display_add("detect", detect, {}, {}, {}, 320);
        display.display_add("calibration", detect, {}, {}, {}, 320);
        display.end_frame();
        display.poll();
        require(window("calibration") && !window("detect") && visible_windows() == 1,
                "Display layout still depends on rune module names");
        display.shutdown();
        // 独立相机预览未设置固定布局，仍按自己的名称创建一个窗口。
        display.start(false, {}, {}, "sdl");
        display.set_time(0);
        cv::Mat full_size(480, 640, CV_8UC3, cv::Scalar(10, 20, 30));
        display.display_add("camera", full_size, {}, {{320, 240, {0, 255, 0, 255}, 6}}, {}, 320);
        full_size.release(); // 提交任务拥有引用，不依赖调用方cv::Mat对象的生命周期。
        display.end_frame();
        display.poll();
        require(window("camera") && visible_windows() == 1, "Independent camera preview broke");
        require(capture(window("camera")).at<cv::Vec3b>(120, 160)[1] > 200 &&
                    capture(window("camera")).at<cv::Vec3b>(200, 20) == cv::Vec3b(10, 20, 30),
                "SDL texture scaling lost original pixel coordinates or frame ownership");
        display.shutdown();
        std::cout << "SDL independent windows, flags, frame ownership, close routing and controls passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        rm_utils::Display::getInstance().shutdown();
        std::cerr << error.what() << '\n';
        return 1;
    }
}
