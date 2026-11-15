#include "reprojection.hpp"
#include <opencv2/calib3d.hpp>
#include <array>

namespace rm_utils::util {
std::vector<cv::Point2f> reproject_points(std::span<const cv::Point3f> points,
                                        const cv::Mat &intrinsic, const cv::Mat &distortion)
{
    std::vector<cv::Point2f> pixels;
    cv::projectPoints(std::vector<cv::Point3f>(points.begin(), points.end()), cv::Vec3d{}, cv::Vec3d{},
                      intrinsic, distortion, pixels);
    return pixels;
}
std::optional<cv::Point2d> reproject_point(const Eigen::Vector3d &point,
                                        const cv::Mat &intrinsic, const cv::Mat &distortion)
{
    const std::array<cv::Point3f, 1> points{{{float(point.x()), float(point.y()), float(point.z())}}};
    const auto pixels = reproject_points(points, intrinsic, distortion);
    if (pixels.empty()) return {};
    return cv::Point2d(pixels.front());
}
} // namespace rm_utils::util
