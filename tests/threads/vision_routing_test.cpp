#include "threads/vision_router/vision_command.hpp"
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
        using threads::VisionCommand;
        VisionCommand commands;
        commands.update({SMALL_RUNE_RED, commands.state().epoch + 1});
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
        commands.update({BIG_RUNE_RED, commands.state().epoch + 1});
        switched = true;
        delayed.join();
        require(!accepted, "Old in-flight result crossed a mode switch");
        require(!commands.packet(VisionCommand::Clock::now()).found, "Switch retained the previous command");
        commands.update({SMALL_RUNE_RED, commands.state().epoch + 1});
        require(!commands.submit(old, VisionCommand::Clock::now(), 100ms, {true, 1, 1, 1}),
                "A->B->A accepted an old epoch");
        const auto current = commands.state();
        require(commands.submit(current, VisionCommand::Clock::now(), 1s, {true, .25f, -.5f, 1}),
                "Could not seed last command angles");
        commands.update(current);
        require(commands.packet(VisionCommand::Clock::now()).found, "Unchanged state cleared the command");
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

        require(commands.submit(current, VisionCommand::Clock::now(), 1s, {true, .25f, -.5f, 1}),
                "Could not seed command before pose loss");
        const auto no_pose = commands.packet(VisionCommand::Clock::now(), false);
        require(!no_pose.found && !no_pose.fire_advice && no_pose.target_yaw == .25f && no_pose.target_pitch == -.5f,
                "Pose loss retained firing or changed the last angles");
        require(!commands.packet(VisionCommand::Clock::now(), true).found,
                "Pose recovery revived an invalid command");

        const auto before_reconnect = commands.state();
        commands.update({before_reconnect.mode, before_reconnect.epoch + 1});
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
