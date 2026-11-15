#pragma once

#include "auto_buff/debug/buff_debug.hpp"

namespace auto_buff
{
struct Detection;

// 检测最终观测与原始分数；算法完成颜色、模式过滤后调用。
class DetectDebug : public BuffDebug
{
public:
    explicit DetectDebug(bool enabled = false) : BuffDebug(enabled, "detect") {}
    void draw(const Detection &detection, const cv::Mat &image);
    static void clear();
};
} // namespace auto_buff
