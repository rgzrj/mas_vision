#pragma once
#include <array>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>
namespace auto_buff {
struct SingleRunePnpSolution {
    struct Input {
        cv::Mat intrinsic, distortion;

        cv::Point2d center;
        cv::Point2d icon;
        std::array<cv::Point2d, 4> corners;
    } input;

    struct Result {
        Eigen::Vector3d translation;
        Eigen::Quaterniond orientation;
    } result;

    auto solve() -> bool;
};

}
