#include "buff_thread.hpp"
#include "auto_buff/buff_detect/detection.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "auto_buff/buff_shoot/rune_shooter.hpp"
#include "auto_buff/debug/buff_debug.hpp"
#include "auto_buff/debug/visualize/track_debug.hpp"
#include "auto_buff/debug/visualize/shoot_debug.hpp"
#include "hardware/serial/user_serial.hpp"
#include "mas_log.hpp"
#include <atomic>
#include <thread>

extern std::atomic<bool> g_shutdown;

namespace threads
{
void buff_thread_func(VisionQueue &queue, const std::filesystem::path &config)
{
    MAS_LOG_INFO("Buff processing thread started");
    auto detector = auto_buff::make_detector(config);
    auto_buff::RmcsTracker tracker(config, config.parent_path() / "hikcamera.yaml");
    auto_buff::RuneShooter shooter(config);
    const auto debug_views = auto_buff::BuffDebug::configure_layout(config);
    auto &display          = rm_utils::Display::getInstance();
    auto &serial           = UserSerial::getInstance();
    std::optional<std::uint64_t> epoch;
    std::uint64_t frames = 0;
    while (!g_shutdown)
    {
        if (!queue.front())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        // 消费端只取最新帧，router 不操作算法工作队列的读端。
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
            detector->set_color(frame.color);
            epoch = frame.state.epoch;
        }
        const auto now = std::chrono::steady_clock::now();
        if (frame.camera.timestamp > now || now - frame.camera.timestamp > max_age)
        {
            serial.submitVision(frame.state, frame.camera.timestamp, max_age, {});
            continue;
        }
        std::unique_lock display_frame(display.frame_mutex(), std::defer_lock);
        const double seconds = std::chrono::duration<double>(frame.camera.timestamp.time_since_epoch()).count();
        if (debug_views)
        {
            display_frame.lock();
            display.set_time(seconds);
        }
        const auto mode = frame.task == VisionTask::big_rune ? auto_buff::RuneMode::big : auto_buff::RuneMode::small;
        const auto detection = detector->detect(frame.camera.frame, seconds, mode);
        SendPacket packet{};
        if (!frame.pose)
        {
            tracker.reset();
            shooter.reset();
            auto_buff::BuffDebug::unavailable(frame.camera.frame, debug_views, "Pose unavailable");
            if (debug_views & 2)
                auto_buff::TrackDebug::clear();
            if (debug_views & 4)
                auto_buff::ShootDebug::clear();
        }
        else
        {
            tracker.set_imu_orientation(*frame.pose);
            const auto tracking = tracker.update(detection, seconds, frame.camera.frame);
            // 实机在跟踪完成后读取当前时间，火控计入曝光后的真实处理延迟。
            if (tracking.initialized)
                shooter.update({std::chrono::steady_clock::now()});
            const auto aimed = shooter.shoot(tracking, frame.camera.frame);
            if (aimed)
                packet = {true, float(aimed->aim_yaw), float(aimed->aim_pitch), std::uint8_t(aimed->should_shoot)};
        }
        if (!serial.submitVision(frame.state, frame.camera.timestamp, max_age, packet))
            continue;
        if (debug_views)
            display.end_frame();
        ++frames;
    }
    MAS_LOG_INFO("Buff processing thread exited: {} frames", frames);
}
} // namespace threads
