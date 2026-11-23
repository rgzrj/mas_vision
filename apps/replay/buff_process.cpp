#include "buff_process.hpp"
#include <cmath>
#include <stdexcept>
#include <thread>

namespace app
{
auto_buff::TrackResult process_buff_frame(BuffModules &modules, const BuffFrame &frame)
{
    const double seconds = std::chrono::duration<double>(frame.exposure_timestamp.time_since_epoch()).count();
    if (!std::isfinite(seconds) || seconds < 0)
        throw std::runtime_error("Invalid replay timestamp");
    if (!frame.color.empty())
    {
        if (frame.color != "red" && frame.color != "blue")
            throw std::invalid_argument("Replay color must be red or blue");
        modules.detector->BuffDetector_Set_Color(frame.color == "blue" ? auto_buff::EnemyColor::BLUE
                                                                       : auto_buff::EnemyColor::RED);
    }
    if (!modules.detector->submit(frame.image, seconds, frame.mode))
        throw std::logic_error("Replay frame submitted before previous result was collected");

    auto detection = modules.detector->collect();
    while (!detection)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        detection = modules.detector->collect();
    }
    if (frame.require_pose && !frame.imu_orientation)
    {
        modules.reset();
        auto_buff::TrackResult missing;
        missing.status = "waiting_for_pose";
        return missing;
    }
    if (frame.imu_orientation)
        modules.tracker.set_imu_orientation(*frame.imu_orientation);
    auto tracking = modules.tracker.update(*detection, detection->seconds, detection->image);
    // 回放固定使用记录时刻，暂停、单步和播放速度不改变火控提前量。
    if (tracking.initialized)
        modules.shooter.update({tracking.timestamp});
    modules.shooter.shoot(tracking, frame.image);
    return tracking;
}
} // namespace app
