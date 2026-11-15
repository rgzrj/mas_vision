#pragma once

#include "auto_buff/debug/buff_debug.hpp"
#include "auto_buff/buff_shoot/rune_shooter.hpp"

namespace auto_buff
{
struct TrackResult;

// 火控现有 Impl 持有本对象，避免公开头文件相互包含。
class ShootDebug : public BuffDebug
{
public:
    explicit ShootDebug(bool enabled = false) : BuffDebug(enabled, "shoot") {}
    void draw(const TrackResult &track, const std::optional<RuneShooter::Aimed> &aimed, const cv::Mat &image);
    static void clear();
};
} // namespace auto_buff
