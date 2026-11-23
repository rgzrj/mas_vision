#include "buff_thread.hpp"

#include "auto_buff/buff_detect/buff_detector.hpp"
#include "auto_buff/buff_shoot/rune_shooter.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "auto_buff/debug/buff_debug.hpp"
#include "hardware/serial/user_serial.hpp"
#include "mas_log.hpp"

#include <atomic>
#include <deque>
#include <optional>
#include <stdexcept>
#include <thread>

extern std::atomic<bool> g_shutdown;

namespace threads
{
void buff_thread_func(VisionQueue &queue, const std::filesystem::path &config)
{
    MAS_LOG_INFO("Buff processing thread started");

    auto detector = auto_buff::BuffDetector::create(config);
    
    auto_buff::RmcsTracker tracker(config, config.parent_path() / "hikcamera.yaml");
    auto_buff::RuneShooter shooter(config);

    const auto_buff::BuffDebug debug(config);
    UserSerial &serial = UserSerial::getInstance();

    using Clock = std::chrono::steady_clock;

    // GPU 最多两帧在途，元数据与检测请求按相同顺序收取。
    std::deque<VisionFrame> pending;

    std::optional<std::uint64_t> last_epoch;
    std::uint64_t frames = 0;

    // 检查模式版本和图像有效期。
    const auto check_frame = [&](const VisionFrame &frame)
    {
        if (frame.state.epoch != serial.getVisionState().epoch)
            return false;

        const auto now = Clock::now();
        if (frame.camera.timestamp > now || now - frame.camera.timestamp > frame.max_age)
        {
            serial.disableVision(frame.state);
            return false;
        }

        return true;
    };

    // 使用对应帧的图像、姿态和时间完成检测、跟踪与解算。
    const auto process = [&](const VisionFrame &frame, const auto_buff::BuffDetector::Detection &detection)
    {
        // 模式切换后清空上一轮跟踪和火控状态。
        if (frame.state.epoch != last_epoch)
        {
            tracker.reset();
            shooter.reset();

            last_epoch = frame.state.epoch;
        }

        const auto &image    = detection.image;
        const auto timestamp = frame.camera.timestamp;
        const double seconds = detection.seconds;

        std::optional<SendPacket> packet;
        if (!frame.pose)
        {
            tracker.reset();
            shooter.reset();
            debug.pose_unavailable(image);
        }
        else
        {
            tracker.set_imu_orientation(*frame.pose);
            const auto tracking = tracker.update(detection, seconds, image);

            // 实机在跟踪完成后读取当前时间，火控计入曝光后的真实处理延迟。
            if (tracking.initialized)
                shooter.update({std::chrono::steady_clock::now()});

            const auto aimed = shooter.shoot(tracking, image);
            if (aimed)
                packet =
                    SendPacket{true, float(aimed->aim_yaw), float(aimed->aim_pitch), std::uint8_t(aimed->should_shoot)};
        }

        // 无姿态或无解时撤销指令，保留上次角度。
        const bool accepted = packet ? serial.submitVision(frame.state, timestamp, frame.max_age, *packet)
                                     : serial.disableVision(frame.state);
        if (!accepted)
            return;

        debug.end_frame();
        ++frames;
    };

    while (!g_shutdown)
    {
        bool progressed = false;

        // 收取和绘制均使用队首对应帧；后一请求先完成时继续保留。
        if (!pending.empty())
        {
            const auto display_frame = debug.begin_frame(pending.front().camera.timestamp);
            if (auto detection = detector->collect())
            {
                if (check_frame(pending.front()))
                    process(pending.front(), *detection);
                pending.pop_front();
                progressed = true;
            }
        }

        // 仅保留本轮最新输入；满槽时保留该帧，下一轮继续尝试。
        for (auto backlog = queue.size(); backlog > 1; --backlog)
            queue.pop();

        if (auto *frame = queue.front())
        {
            if (!check_frame(*frame))
            {
                queue.pop();
                progressed = true;
            }
            else
            {
                if (frame->color == "blue")
                    detector->BuffDetector_Set_Color(auto_buff::EnemyColor::BLUE);
                else
                    detector->BuffDetector_Set_Color(auto_buff::EnemyColor::RED);

                const auto mode =
                    frame->task == VisionTask::big_rune ? auto_buff::RuneMode::big : auto_buff::RuneMode::small;
                const double seconds =
                    std::chrono::duration<double>(frame->camera.timestamp.time_since_epoch()).count();
                if (detector->submit(frame->camera.frame, seconds, mode))
                {
                    pending.push_back(std::move(*frame));
                    queue.pop();
                    progressed = true;
                }
            }
        }

        if (!progressed)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    MAS_LOG_INFO("Buff processing thread exited: {} frames", frames);
}

} // namespace threads
