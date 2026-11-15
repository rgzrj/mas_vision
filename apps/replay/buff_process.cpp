#include "buff_process.hpp"
#include <cmath>
#include <stdexcept>

namespace app
{
auto_buff::TrackResult process_buff_frame(BuffModules &modules, const BuffFrame &frame)
{
    const double seconds = std::chrono::duration<double>(frame.exposure_timestamp.time_since_epoch()).count();
    if (!std::isfinite(seconds) || seconds < 0)
        throw std::runtime_error("Invalid replay timestamp");
    if (!frame.color.empty())
        modules.detector->set_color(frame.color);
    const auto detection = modules.detector->detect(frame.image, seconds, frame.mode);
    if (frame.require_pose && !frame.imu_orientation)
    {
        modules.reset();
        auto_buff::TrackResult missing;
        missing.status = "waiting_for_pose";
        return missing;
    }
    if (frame.imu_orientation)
        modules.tracker.set_imu_orientation(*frame.imu_orientation);
    auto tracking = modules.tracker.update(detection, seconds, frame.image);
    // 回放固定使用记录时刻，暂停、单步和播放速度不改变火控提前量。
    if (tracking.initialized)
        modules.shooter.update({tracking.timestamp});
    modules.shooter.shoot(tracking, frame.image);
    return tracking;
}
} // namespace app
