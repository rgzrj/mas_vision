#pragma once
#include "auto_aim/armor_detector/armor_detector.hpp"
#include "auto_aim/armor_track/armor_track.hpp"
#include "auto_aim/armor_shoot/armor_shoot.hpp"
#include <filesystem>
namespace app
{
using auto_aim::ArmorDetector;
using auto_aim::ArmorShoot;
using auto_aim::ArmorTrack;
// 回放报告与标注导出需要的结果，不参与实机，也不收集调试快照。
struct ArmorResult
{
    std::vector<auto_aim::Armor> armors;
    SendPacket aimed{};
    std::string status;
    bool target_ready = false;
};
// 仅供独立回放装配三个生产算法，不参与实机线程管理。
struct ArmorModules
{
    ArmorDetector detector;
    ArmorTrack tracker;
    ArmorShoot shooter;
    explicit ArmorModules(const std::filesystem::path &config)
        : detector(config.string()), tracker(config.string(), (config.parent_path() / "hikcamera.yaml").string()),
          shooter(config.string())
    {
    }
    void reset()
    {
        tracker.reset();
        shooter.reset();
    }
};
ArmorResult process_armor_frame(ArmorModules &, const cv::Mat &, std::chrono::steady_clock::time_point,
                                const std::optional<Eigen::Quaterniond> &, bool require_pose = true,
                                bool realtime = true);
} // namespace app
