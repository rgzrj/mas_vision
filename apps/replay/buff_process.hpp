#pragma once

#include "auto_buff/buff_detect/buff_detector.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "auto_buff/buff_shoot/rune_shooter.hpp"
#include <filesystem>
#include <stdexcept>

namespace app
{
// 仅供离线回放装配；实机直接在 buff_thread 中持有三个模块。
struct BuffModules
{
    std::unique_ptr<auto_buff::BuffDetector> detector;
    auto_buff::RmcsTracker tracker;
    auto_buff::RuneShooter shooter;

    explicit BuffModules(const std::filesystem::path &config)
        : detector(auto_buff::BuffDetector::create(config)), tracker(config, config.parent_path() / "hikcamera.yaml"),
          shooter(config)
    {
        if (!detector)
            throw std::runtime_error("Replay detector could not be created");
    }
    void reset()
    {
        tracker.reset();
        shooter.reset();
    }
};

// 离线输入只描述该帧数据；普通媒体可用固定标定，配对输入要求同帧姿态。
struct BuffFrame
{
    cv::Mat image;
    rm_utils::Timestamp exposure_timestamp{};
    std::optional<Eigen::Quaterniond> imu_orientation;
    std::string color;
    auto_buff::RuneMode mode = auto_buff::RuneMode::unknown;
    bool require_pose        = false;
};

// 返回跟踪状态供回放计数与无效状态显示；不保存另一份检测/火控结果快照。
auto_buff::TrackResult process_buff_frame(BuffModules &modules, const BuffFrame &frame);
} // namespace app
