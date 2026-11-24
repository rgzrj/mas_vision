#pragma once

#include <array>
#include <optional>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>

namespace auto_buff
{
struct SingleRunePnpSolution
{
    struct Input
    {
        cv::Mat intrinsic, distortion;

        cv::Point2d center;
        cv::Point2d icon;
        std::array<cv::Point2d, 4> corners;

        /**
         * @brief 按 R 位置将端点排序为 R、上、左、下、右。
         * @return 退化或两侧端点无法区分时为空，坐标为原图像素。
         */
        auto image_points() const -> std::optional<std::array<cv::Point2f, 5>>;
    } input;

    struct Result
    {
        Eigen::Vector3d translation;
        Eigen::Quaterniond orientation;
    } result;

    auto solve() -> bool;
};

} // namespace auto_buff
