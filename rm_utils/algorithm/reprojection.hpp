#pragma once
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <optional>
#include <span>
#include <vector>

namespace rm_utils::util {
// 输入已经位于 OpenCV 相机坐标系；此处只应用内参和畸变。
std::vector<cv::Point2f> reproject_points(std::span<const cv::Point3f> points,
                                        const cv::Mat &intrinsic, const cv::Mat &distortion);
std::optional<cv::Point2d> reproject_point(const Eigen::Vector3d &point,
                                        const cv::Mat &intrinsic, const cv::Mat &distortion);
} // namespace rm_utils::util
