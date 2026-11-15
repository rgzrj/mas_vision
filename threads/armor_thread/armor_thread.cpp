#include "armor_thread.hpp"
#include "auto_aim/armor_detector/armor_detector.hpp"
#include "auto_aim/armor_track/armor_track.hpp"
#include "auto_aim/armor_shoot/armor_shoot.hpp"
#include "auto_aim/debug/armor_debug.hpp"
#include "auto_aim/debug/visualize/track_debug.hpp"
#include "auto_aim/debug/visualize/shoot_debug.hpp"
#include "hardware/serial/user_serial.hpp"
#include "rm_utils/display/display.hpp"
#include "mas_log.hpp"
#include <atomic>
#include <thread>
extern std::atomic<bool> g_shutdown;
namespace threads
{
void armor_thread_func(VisionQueue &queue, const std::filesystem::path &config)
{
    MAS_LOG_INFO("Armor processing thread started");
    auto_aim::ArmorDetector detector(config.string());
    auto_aim::ArmorTrack tracker(config.string(), (config.parent_path() / "hikcamera.yaml").string());
    auto_aim::ArmorShoot shooter(config.string());
    const auto debug_views = auto_aim::ArmorDebug::configure_layout(config);
    auto &display = rm_utils::Display::getInstance();
    auto &serial = UserSerial::getInstance();
    std::optional<std::uint64_t> epoch;
    std::uint64_t frames = 0;
    while (!g_shutdown)
    {
        if (!queue.front())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        // 消费端丢弃旧帧；router 绝不 pop 工作队列。
        for (auto n = queue.size(); n > 1; --n)
            queue.pop();
        auto frame = std::move(*queue.front());
        queue.pop();
        const auto max_age = frame.max_age;
        if (frame.state.epoch != serial.getVisionState().epoch)
            continue;
        if (epoch != frame.state.epoch)
        {
            tracker.reset();
            shooter.reset();
            detector.ArmorDetector_Set_Color(frame.color == "blue" ? auto_aim::BLUE : auto_aim::RED);
            epoch = frame.state.epoch;
        }
        const auto now = std::chrono::steady_clock::now();
        if (frame.camera.timestamp > now || now - frame.camera.timestamp > max_age)
        {
            serial.submitVision(frame.state, frame.camera.timestamp, max_age, {});
            continue;
        }
        std::unique_lock display_frame(display.frame_mutex(), std::defer_lock);
        if (debug_views)
        {
            display_frame.lock();
            display.set_time(std::chrono::duration<double>(frame.camera.timestamp.time_since_epoch()).count());
        }
        auto armors = detector.ArmorDetect(frame.camera.frame);
        SendPacket packet{};
        if (!frame.pose)
        {
            tracker.reset();
            shooter.reset();
            auto_aim::ArmorDebug::unavailable(frame.camera.frame, debug_views, "Pose unavailable");
            if (debug_views & 2) auto_aim::TrackDebug::clear();
            if (debug_views & 4) auto_aim::ShootDebug::clear();
        }
        else
        {
            tracker.armor_pose().set_R_gimbal2world(*frame.pose);
            auto target = tracker.track(armors, frame.camera.timestamp, frame.camera.frame);
            packet      = shooter.shoot(target, frame.camera.timestamp, tracker.armor_pose().Get_R_gimbal2world(),
                                        std::chrono::steady_clock::now(), frame.camera.frame);
        }
        if (!serial.submitVision(frame.state, frame.camera.timestamp, max_age, packet))
            continue;
        ++frames;
        if (debug_views) display.end_frame();
    }
    MAS_LOG_INFO("Armor processing thread exited: {} frames", frames);
}
} // namespace threads
