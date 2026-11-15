#include "armor_process.hpp"
namespace app
{
ArmorResult process_armor_frame(ArmorModules &modules, const cv::Mat &image,
                                std::chrono::steady_clock::time_point exposure,
                                const std::optional<Eigen::Quaterniond> &pose, bool require_pose, bool realtime)
{
    ArmorResult result;
    result.armors         = modules.detector.ArmorDetect(image);
    if (require_pose && !pose)
    {
        modules.reset();
        result.status = "waiting_for_pose";
        return result;
    }
    if (pose)
        modules.tracker.armor_pose().set_R_gimbal2world(*pose);
    const auto target   = modules.tracker.track(result.armors, exposure, image);
    result.target_ready = target.has_value();
    result.aimed     = modules.shooter.shoot(target, exposure, modules.tracker.armor_pose().Get_R_gimbal2world(),
                                         realtime ? std::chrono::steady_clock::now() : exposure, image);
    result.status    = modules.tracker.state();
    return result;
}
} // namespace app
