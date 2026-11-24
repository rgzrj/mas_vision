#include "armor_thread.hpp"
#include "threads/vision_router/vision_command.hpp"

#include "auto_aim/armor_detector/armor_detector.hpp"
#include "auto_aim/armor_shoot/armor_shoot.hpp"
#include "auto_aim/armor_track/armor_track.hpp"
#include "auto_aim/debug/armor_debug.hpp"
#include "hardware/serial/user_serial.hpp"
#include "mas_log.hpp"

#include <atomic>
#include <thread>

extern std::atomic<bool> g_shutdown;

namespace threads
{
void armor_thread_func(VisionCommand &commands, VisionQueue &queue, const std::filesystem::path &config)
{
    MAS_LOG_INFO("Armor processing thread started");

    const auto armor_config  = config.string();
    const auto camera_config = (config.parent_path() / "hikcamera.yaml").string();

    auto_aim::ArmorDetector detector(armor_config);
    auto_aim::ArmorTrack    tracker(armor_config, camera_config);
    auto_aim::ArmorShoot    shooter(armor_config);

    UserSerial &serial                = UserSerial::getInstance();
    bool        last_quaternion_valid = false;

    const auto_aim::ArmorDebug debug(config);

    std::optional<std::uint64_t> last_epoch;
    std::uint64_t                frames = 0;

    while (!g_shutdown)
    {
        // 没有图像时休眠，避免空转占用 CPU。
        if (!queue.front())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 只处理本轮缓存中最新的一帧；模式和曝光姿态已由 router 配对。
        for (auto pending = queue.size(); pending > 1; --pending)
            queue.pop();

        VisionFrame input = std::move(*queue.front());
        queue.pop();

        const CameraFrame &frame   = input.camera;
        const auto         max_age = input.max_age;

        if (input.state.epoch != commands.state().epoch)
            continue;

        // 模式切换后清空上一轮跟踪和火控状态，再设置本帧颜色。
        if (input.state.epoch != last_epoch)
        {
            tracker.reset();
            shooter.reset();

            if (input.color == "blue")
            {
                detector.ArmorDetector_Set_Color(auto_aim::EnemyColor::BLUE);
                MAS_LOG_WARN("ArmorDetector_Set_Color BLUE");
            }
            else
            {
                detector.ArmorDetector_Set_Color(auto_aim::EnemyColor::RED);
                MAS_LOG_WARN("ArmorDetector_Set_Color RED");
            }

            last_epoch = input.state.epoch;
        }

        const auto now = std::chrono::steady_clock::now();
        if (frame.timestamp > now || now - frame.timestamp > max_age)
        {
            commands.clear(input.state);
            continue;
        }

        // 四元数流只在状态变化时记录；本帧姿态仍使用 router 的配对结果。
        const bool quaternion_valid = serial.isQuaternionValid();
        if (quaternion_valid != last_quaternion_valid)
        {
            MAS_LOG_WARN("Quaternion stream {} (age={} ms)", quaternion_valid ? "recovered" : "lost",
                         serial.quaternionAgeMs());
            last_quaternion_valid = quaternion_valid;
        }

        // 三个算法的调试图使用同一帧时间。
        const auto display_frame = debug.begin_frame(frame.timestamp);

        // 检测装甲板，使用曝光时刻的姿态完成跟踪和火控。
        auto armors = detector.ArmorDetect(frame.frame);

        if (input.pose)
        {
            tracker.armor_pose().set_R_gimbal2world(*input.pose);

            auto       target       = tracker.track(armors, frame.timestamp, frame.frame);
            const auto shoot_result = shooter.shoot(target, frame.timestamp, tracker.armor_pose().Get_R_gimbal2world(),
                                                     std::chrono::steady_clock::now(), frame.frame);

            if (!commands.submit(input.state, frame.timestamp, max_age, shoot_result))
                continue;
        }
        else
        {
            tracker.reset();
            shooter.reset();
            debug.pose_unavailable(frame.frame);

            if (!commands.clear(input.state))
                continue;
        }

        ++frames;
        debug.end_frame();
    }

    MAS_LOG_INFO("Armor processing thread exited: {} frames", frames);
}

} // namespace threads
