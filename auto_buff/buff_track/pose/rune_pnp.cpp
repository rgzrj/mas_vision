#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT
#include "rune_pnp.hpp"

#include "rm_utils/algorithm/math_tools.hpp"
#include "rm_utils/algorithm/conversion.hpp"
#include "rm_utils/algorithm/reprojection.hpp"
#include "auto_buff/buff_types.hpp"

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <vector>

using namespace auto_buff;
using namespace rm_utils::util;

auto SingleRunePnpSolution::solve() -> bool {
    try {
        constexpr auto kEps = 1e-6;

        const auto camera_matrix = input.intrinsic;
        const auto distort_coeff = input.distortion;

        const auto fx = input.intrinsic.at<double>(0, 0);
        const auto fy = input.intrinsic.at<double>(1, 1);
        if (fx <= 0.0 || fy <= 0.0) return false;

        const auto icon   = cv::Point2f(input.icon);
        const auto center = cv::Point2f(input.center);

        auto corners = std::array<cv::Point2f, 4> { };
        std::ranges::copy(input.corners | std::views::transform([](const cv::Point2d& point) {
            return cv::Point2f(point);
        }),
            corners.begin());

        auto index_b      = std::size_t { 0 };
        auto index_t      = std::size_t { 0 };
        auto min_distance = std::numeric_limits<double>::max();
        auto max_distance = -std::numeric_limits<double>::max();
        for (const auto& [index, corner] : corners | std::views::enumerate) {
            const auto dx       = static_cast<double>(corner.x - icon.x);
            const auto dy       = static_cast<double>(corner.y - icon.y);
            const auto distance = std::hypot(dx, dy);

            if (distance < min_distance) {
                index_b      = static_cast<std::size_t>(index);
                min_distance = distance;
            }
            if (distance > max_distance) {
                index_t      = static_cast<std::size_t>(index);
                max_distance = distance;
            }
        }
        if (index_b == index_t) return false;

        auto remaining       = std::array<std::size_t, 2> { };
        auto remaining_count = std::size_t { 0 };
        for (std::size_t index = 0; index < corners.size(); index++) {
            if (index == index_b || index == index_t) continue;
            if (remaining_count >= remaining.size()) return false;

            remaining[remaining_count] = index;
            remaining_count++;
        }
        if (remaining_count != remaining.size()) return false;

        const auto b = corners[index_b];
        const auto t = corners[index_t];

        auto l = cv::Point2f { };
        auto r = cv::Point2f { };
        {
            const auto z_axis_raw = t - b;
            const auto z_axis_len =
                std::hypot(static_cast<double>(z_axis_raw.x), static_cast<double>(z_axis_raw.y));
            if (z_axis_len <= kEps) return false;

            const auto z_axis = cv::Point2d {
                static_cast<double>(z_axis_raw.x) / z_axis_len,
                static_cast<double>(z_axis_raw.y) / z_axis_len,
            };
            const auto y_axis = cv::Point2d { z_axis.y, -z_axis.x };

            const auto first  = corners[remaining[0]];
            const auto second = corners[remaining[1]];

            const auto first_projection = static_cast<double>(first.x - center.x) * y_axis.x
                + static_cast<double>(first.y - center.y) * y_axis.y;
            const auto second_projection = static_cast<double>(second.x - center.x) * y_axis.x
                + static_cast<double>(second.y - center.y) * y_axis.y;
            if (std::abs(first_projection - second_projection) <= kEps) return false;

            if (first_projection > second_projection) {
                l = first;
                r = second;
            } else {
                l = second;
                r = first;
            }
        }

        std::vector<cv::Point3f> object_points;
        object_points.reserve(RunePagePoints::kPoints.size());
        for (const auto& point : RunePagePoints::kPoints) {
            const auto p_ros = point;
            const auto p_ocv = ros2opencv_position(p_ros);
            object_points.push_back(cv::Point3f {
                static_cast<float>(p_ocv.x()),
                static_cast<float>(p_ocv.y()),
                static_cast<float>(p_ocv.z()),
            });
        }
        const auto image_points = std::vector { icon, t, l, b, r };

        auto rota_vec      = cv::Vec3d { };
        auto tran_vec      = cv::Vec3d { };
        const auto success = cv::solvePnP(object_points, image_points, camera_matrix, distort_coeff,
            rota_vec, tran_vec, false, cv::SOLVEPNP_EPNP);
        if (!success || tran_vec[2] <= 0.0) return false;

        cv::solvePnP(object_points, image_points, camera_matrix, distort_coeff, rota_vec, tran_vec,
            true, cv::SOLVEPNP_ITERATIVE);
        if (tran_vec[2] <= 0.0) return false;

        auto translation_eigen_opencv = Eigen::Vector3d { };
        cv::cv2eigen(tran_vec, translation_eigen_opencv);

        auto rotation_opencv = cv::Mat { };
        cv::Rodrigues(rota_vec, rotation_opencv);

        auto rotation_eigen_opencv = Eigen::Matrix3d { };
        cv::cv2eigen(rotation_opencv, rotation_eigen_opencv);

        result.translation = opencv2ros_position(translation_eigen_opencv);
        result.orientation =
            Eigen::Quaterniond { opencv2ros_rotation(rotation_eigen_opencv) }.normalized();

    } catch (cv::Exception const&) {
        return false;
    }
    return true;
}
