#pragma once

#include "auto_buff/debug/buff_debug.hpp"

namespace auto_buff
{
struct TrackResult;

// 展示本帧模型、匹配观测与未来投影；不修改跟踪状态。
class TrackDebug : public BuffDebug
{
public:
    explicit TrackDebug(bool enabled = false) : BuffDebug(enabled, "track") {}
    void draw(const TrackResult &result, const cv::Mat &image);
    // 丢失或缺姿态时清理曲线，防止继续展示上一帧有效状态。
    static void clear();
};
} // namespace auto_buff
