#pragma once

#include "../../buff_detector.hpp"

#include <span>

namespace auto_buff
{

// 唯一模型协议：0 未激活、1 小符已激活、2 大符已激活；五点为 top、left、R、right、bottom。
inline constexpr std::size_t kPoseChannels = 18; // 三类分数与五组 (x, y, score)。

// 不同状态的四点对应不同结构：待击板端部、小符圆环或大符灯臂。
// 只有待击板四端点能直接使用当前的靶板物理尺寸进行 PnP。

/**
 * @brief 解码三类五点输出，还原原图坐标，并按区域中心距离去除重复候选。
 * @param data 输出张量 [1,18,N] 的连续内存视图，共 18*N 个 float，按通道存放。
 * @param count 候选数量 N，即输出张量的第三维长度；当前部署模型为 6300。
 * @param image_size 原始图像的宽高，用于判断还原后的关键点是否位于图像内。
 * @param scale 预处理时原图到网络输入的等比例缩放系数；还原坐标时除以此值。
 * @param pad 网络输入画布左侧和顶部的填充像素数；还原坐标时先减去此偏移。
 * @param confidence 目标置信度下限；每个候选取三个状态分数中的最大值。
 * @param point_confidence 单个关键点的置信度下限；低分、非有限或越界的点标为无效。
 * @param distance 去重距离门限，单位为原图像素；大符已激活类使用此值的三分之一。
 * @return 按目标置信度降序排列的去重结果；每个候选至少有三个有效点，坐标已还原到原图。
 */
std::vector<PoseDetection> decode_pose_output(std::span<const float> data, std::size_t count, cv::Size image_size,
                                              double scale, cv::Point pad, double confidence, double point_confidence,
                                              double distance);

/**
 * @brief 将解码后的五点转换为公共 R/符板观测，执行原图判色及可选像素精修。
 * @param poses decode_pose_output 返回的候选；点位与有效性均以原图为准。
 * @param image 对应的原始 BGR 图像，用于局部判色和几何精修。
 * @param color 符板颜色筛选条件：RED 或 BLUE；不筛选无色 R 候选。
 * @param refine 是否启用局部像素精修；精修失败时保留已有的有效原始几何。
 * @param profiling 是否在当前算法线程采集精修耗时；调用方须已确认 Tracy 客户端启动。
 * @return R、有效符板和独立的大符激活灯臂观测；灯臂保留原始点及单独提取的轮廓。
 */
BuffDetector::Detection make_pose_detection(const std::vector<PoseDetection> &poses, const cv::Mat &image, EnemyColor color,
                              bool refine, bool profiling = false);

} // namespace auto_buff
