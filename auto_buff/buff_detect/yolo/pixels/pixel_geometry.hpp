#pragma once
#include "../../buff_detector.hpp"

#include <span>

namespace auto_buff
{

/**
 * @brief 按指定红/蓝色差提取 ROI 内的外轮廓，供板面及灯臂几何处理共用。
 * @param image 原始 BGR 图像。
 * @param roi 轮廓搜索区域，内部裁剪至原图范围。
 * @param color 已确定的目标颜色，red 或 blue；此函数不重新判定颜色。
 * @return 原图坐标下的外轮廓点集合。
 */
std::vector<std::vector<cv::Point>> yolo_color_contours(const cv::Mat &image, cv::Rect roi,
                                                       EnemyColor color);

/**
 * @brief 用网络四端点及 R 的位置约束，选择板面外轮廓。
 * @param image 原始 BGR 图像。
 * @param pose 四个非R点须坐标有限且在图内；低分点允许作搜索先验，不直接作为有效端点。
 * @param color 已确定的板面颜色，red 或 blue。
 * @return 原图坐标下的板面轮廓；四点不完整或未找到符合约束的轮廓时为空。
 */
std::vector<cv::Point> yolo_board_contour(const cv::Mat &image, const PoseDetection &pose,
                                         EnemyColor color);

/**
 * @brief 提取 YOLO 检测的灯臂轮廓。
 * @param image 原始 BGR 图像。
 * @param arm 灯臂对象，包含待提取轮廓的信息。
 */
void extract_yolo_light_arm_contour(const cv::Mat &image, LightArm &arm);

/**
 * @brief 根据灯臂端点和圆心，估计灯臂的红蓝颜色。
 * @param image 原始 BGR 图像。
 * @param tips 灯臂端点集合，至少包含两个端点。
 * @param center 灯臂圆心位置。
 * @param box 灯臂的检测框，用于限定采样区域。
 * @return RED、BLUE；颜色证据不足返回 UNKNOWN，不按电控要求硬填颜色。
 * @note 该函数仅用于抑制候选和确定像素搜索区域，不作为跟踪圆心。
 */
EnemyColor yolo_observation_color(const cv::Mat &image, std::span<const cv::Point2d> tips,
                                   cv::Point2d center, const cv::Rect2d &box);
                                   
/**
 * @brief 从目标局部灯珠估计红蓝颜色，不依赖神经网络类别。
 * @param image 原始 BGR 图像。
 * @param board disk 优先采样可见端点附近；ring 采样自身圆环，避开框外装饰。
 * @param box 模型检测框，用于限定采样区域和估计尺度。
 * @return RED、BLUE；颜色证据不足返回 UNKNOWN，不按电控要求硬填颜色。
 */
EnemyColor yolo_board_color(const cv::Mat &image, const Board &board, const cv::Rect2d &box);
}
