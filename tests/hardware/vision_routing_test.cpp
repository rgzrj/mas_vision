#include "hardware/serial/vision_command.hpp"
#include <atomic>
#include <iostream>
#include <thread>
#include <stdexcept>

int main()
{
    try
    {
        const auto require = [](bool value, const char *message)
        {
            if (!value)
                throw std::runtime_error(message);
        };
        using namespace std::chrono_literals;
        VisionCommand commands;
        commands.set_mode(SMALL_RUNE_RED);
        const auto old = commands.state();
        require(commands.submit(old, VisionCommand::Clock::now(), 100ms, {true, .1f, .2f, 1}), "Valid result rejected");
        std::atomic<bool> switched{false}, accepted{true};
        std::thread delayed(
            [&]
            {
                while (!switched.load())
                    std::this_thread::yield();
                accepted = commands.submit(old, VisionCommand::Clock::now(), 100ms, {true, 9, 9, 1});
            });
        commands.set_mode(BIG_RUNE_RED);
        switched = true;
        delayed.join();
        require(!accepted, "Old in-flight result crossed a mode switch");
        require(!commands.packet(VisionCommand::Clock::now()).found, "Switch retained the previous command");
        commands.set_mode(SMALL_RUNE_RED);
        require(!commands.submit(old, VisionCommand::Clock::now(), 100ms, {true, 1, 1, 1}),
                "A->B->A accepted an old epoch");
        const auto current = commands.state();
        require(commands.submit(current, VisionCommand::Clock::now(), 1s, {true, .25f, -.5f, 1}),
                "Could not seed last command angles");
        require(!commands.clear(old), "Old frame cancelled the current command");
        require(commands.packet(VisionCommand::Clock::now()).found, "Current command was cleared by old frame");
        require(commands.clear(current), "Could not clear current command");
        const auto cleared = commands.packet(VisionCommand::Clock::now());
        require(!cleared.found && !cleared.fire_advice && cleared.target_yaw == .25f && cleared.target_pitch == -.5f,
                "Clearing a command copied zero angles or kept the command active");

        require(commands.submit(current, VisionCommand::Clock::now(), 100ms, {true, 1, 1, 1}),
                "Current epoch rejected");
        const auto expired = commands.packet(VisionCommand::Clock::now() + 101ms);
        require(!expired.found && !expired.fire_advice, "Camera silence retained a firing command");
        require(!commands.submit(current, VisionCommand::Clock::now() - 101ms, 100ms, {true, 1, 1, 1}),
                "Stale frame accepted");
        require(!commands.submit(current, VisionCommand::Clock::now() + 1s, 100ms, {}), "Future timestamp accepted");
        require(commands.submit(current, VisionCommand::Clock::now(), 100ms, {false, 0, 0, 1}),
                "No-target result rejected");
        require(!commands.packet(VisionCommand::Clock::now()).fire_advice, "No-target result retained fire");

        const auto before_reconnect = commands.state();
        commands.new_session();
        require(!commands.submit(before_reconnect, VisionCommand::Clock::now(), 100ms, {true, 1, 1, 1}),
                "Reconnect accepted an old command");

        std::cout << "Vision routing and stale-command checks passed\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
