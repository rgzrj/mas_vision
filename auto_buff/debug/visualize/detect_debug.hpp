#pragma once

#include "auto_buff/debug/buff_debug.hpp"
#include "auto_buff/buff_detect/buff_detector.hpp"

namespace auto_buff
{

// 检测最终观测与原始分数；算法完成颜色、模式过滤后调用。
class DetectDebug : public BuffDebugBase
{
public:
    explicit DetectDebug(bool enabled = false) : BuffDebugBase(enabled, "detect") {}
    void draw(const BuffDetector::Detection &detection, const cv::Mat &image);
    static void clear();
};
} // namespace auto_buff
