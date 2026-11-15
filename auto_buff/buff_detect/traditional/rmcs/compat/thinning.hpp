#pragma once
#include <opencv2/core.hpp>
#include <opencv2/core/check.hpp>

// OpenCV contrib 4.12 细化兼容实现，不要求额外链接 ximgproc。
namespace cv::rmcs_compat {
enum { THINNING_ZHANGSUEN = 0, THINNING_GUOHALL = 1 };
/**
 * @brief 将 0/255 二值前景细化为单像素骨架。
 * @param input CV_8UC1 二值图；函数会复制输入，不改变原始掩膜。
 * @param output 同尺寸、同类型的骨架图，前景仍为 255。
 * @param thinning_type Zhang-Suen 或 Guo-Hall；RMCS 的 R 检测使用 Zhang-Suen。
 */
void thinning(InputArray input, OutputArray output, int thinning_type);
}
