#define SDL_MAIN_HANDLED
#include <SDL.h>
#include "rm_utils/display/sdl/sdl_display.hpp"
#include "rm_utils/io/playback_control.hpp"
#include <iostream>
#include <stdexcept>

int main()
{
    try
    {
        SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
        rm_utils::SdlDisplay display;
        tool::PlaybackControl controls;
        std::vector<int> keys;
        const auto push_key = [](SDL_Keycode code, bool repeat = false)
        {
            SDL_Event event{};
            event.type           = SDL_KEYDOWN;
            event.key.keysym.sym = code;
            event.key.repeat     = repeat;
            if (SDL_PushEvent(&event) != 1)
                throw std::runtime_error(SDL_GetError());
        };
        push_key(SDLK_s);
        push_key(SDLK_s, true); // 按住保存键不能重复采样。
        push_key(SDLK_c);
        push_key(SDLK_SPACE);
        push_key(SDLK_n);
        const auto on_key = [&](int key) { keys.push_back(key); };
        display.poll(&controls, on_key);
        if (!controls.paused || !controls.take_step() || controls.take_step())
            throw std::runtime_error("Window pause or single step changed");
        push_key(SDLK_q);
        SDL_Event close{};
        close.type         = SDL_WINDOWEVENT;
        close.window.event = SDL_WINDOWEVENT_CLOSE;
        if (SDL_PushEvent(&close) != 1)
            throw std::runtime_error(SDL_GetError());
        display.poll(&controls, on_key);
        if (keys != std::vector<int>{'s', 'c', ' ', 'n', 'q', 27} || !controls.paused || !controls.quit ||
            controls.take_step())
            throw std::runtime_error("Calibration input or replay controls changed");
        std::cout << "SDL calibration keys, repeat suppression, close and replay controls passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
