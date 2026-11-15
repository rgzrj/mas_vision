#pragma once
#include "../detection.hpp"
#include <span>

namespace auto_buff
{

/**
 * @brief 对 YOLO 检测的符板观测进行像素级几何精修。（仅针对自训练模型）
 * @param image 原始 BGR 图像；精修仅在目标附近的 ROI 内计算。
 * @param board 提供初始中心、半径、颜色和外观，同时接收原图坐标的精修结果。
 * @param contour 已按网络语义筛选的板面外轮廓，使用原图坐标。
 * @return 实测轴线取得板心返回 true；features_valid 为真才同时取得完整四端点。
 * @note 缺一臂时可由两条实测轴线求中心，缺失端点保持零；纯圆环不提供物理板心。
 */
bool refine_yolo_board_pixels(const cv::Mat &image, Board &board, std::span<const cv::Point> contour);

/**
 * @brief 精修五点观测，失败时保留有效的网络原始点，供 pose 几何验证（仅针对自训练模型）
 * @param image 原始 BGR 图像。
 * @param pose 网络五点仅用于选择符板轮廓，不作为精修后的实测端点。
 * @param board 原始网络中心和端点；精修按整组替换或回退，已有有效原始中心时不采用缺臂结果。
 * @param center_valid 原始网络中心是否有效。
 * @return 至少有可靠的原始或精修中心；缺失端点绝不按对称关系补齐。
 */
bool refine_yolo_pose_pixels(const cv::Mat &image, const PoseDetection &pose, Board &board, bool center_valid);
}
