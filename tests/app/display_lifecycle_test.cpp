#define SDL_MAIN_HANDLED
#include <SDL.h>
#include "rm_utils/display/display.hpp"
#include "tests/common/test.hpp"
#include <fstream>

int main()
{
    return tests::run(
        "display_lifecycle_test",
        []
        {
            using tests::require;
            SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
            SDL_SetHint(SDL_HINT_RENDER_DRIVER, "software");
            tests::TemporaryDirectory temporary;
            const auto config = temporary.path / "display.yaml";
            std::ofstream(config) << "display: {backend: sdl, spawn: false}\n";
            auto &display = rm_utils::Display::getInstance();
            display.start(config, temporary.path / "output");
            display.select_layout("disabled");
            display.register_layout("disabled", {});
            display.poll();
            require(SDL_WasInit(SDL_INIT_VIDEO) == 0 && !std::filesystem::exists(temporary.path / "output"),
                    "Disabled debug initialized a renderer or recording session");
            display.shutdown();

            display.start(config);
            display.select_layout("armor");
            display.poll(); // 模块还未完成构造，等待注册即可。
            require(SDL_WasInit(SDL_INIT_VIDEO) == 0, "Session started before its module registered views");
            display.register_layout("armor", {{"HikCamera", 0}});
            display.poll();
            require(SDL_WasInit(SDL_INIT_VIDEO) != 0, "Registered view did not start the configured backend");
            display.set_time(1);
            display.display_add("HikCamera", cv::Mat::zeros(40, 60, CV_8UC3));
            display.end_frame();
            display.poll();
            int windows = 0;
            for (Uint32 i = 1; i <= 16; ++i)
                if (auto *window = SDL_GetWindowFromID(i))
                {
                    ++windows;
                    require(std::string(SDL_GetWindowTitle(window)) == "mas_vision | HikCamera", "Wrong module window");
                }
            require(windows == 1, "Module did not create exactly one independent window");
            display.shutdown();
            require(SDL_WasInit(SDL_INIT_VIDEO) == 0, "Shutdown retained SDL resources");
        });
}
