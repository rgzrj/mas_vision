#define OPENCV_DISABLE_EIGEN_TENSOR_SUPPORT
#include "rune_pnp.hpp"

#include "rm_utils/algorithm/conversion.hpp"
#include "auto_buff/buff_types.hpp"

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <vector>

namespace auto_buff
{
/**
 * @brief 按 R 位置将端点排序为 R、上、左、下、右。
 * @return 退化或两侧端点无法区分时为空，坐标为原图像素。
 */
auto SingleRunePnpSolution::image_points(const RuneBullseye &board, cv::Point2d icon)
    -> std::optional<std::array<cv::Point2f, 5>>
{
    constexpr auto kEps = 1e-6;

    const auto icon_point   = cv::Point2f(icon);
    const auto center_point = cv::Point2f(board.center);

    auto points = std::array<cv::Point2f, 4>{};
    std::ranges::copy(board.corners |
                          std::views::transform([](const cv::Point2d &point) { return cv::Point2f(point); }),
                      points.begin());

    auto index_b      = std::size_t{0};
    auto index_t      = std::size_t{0};
    auto min_distance = std::numeric_limits<double>::max();
    auto max_distance = -std::numeric_limits<double>::max();
    for (const auto &[index, corner] : points | std::views::enumerate)
    {
        const auto dx       = static_cast<double>(corner.x - icon_point.x);
        const auto dy       = static_cast<double>(corner.y - icon_point.y);
        const auto distance = std::hypot(dx, dy);

        if (distance < min_distance)
        {
            index_b      = static_cast<std::size_t>(index);
            min_distance = distance;
        }
        if (distance > max_distance)
        {
            index_t      = static_cast<std::size_t>(index);
            max_distance = distance;
        }
    }
    if (index_b == index_t)
        return std::nullopt;

    // 找到剩余左右两个端点的索引
    auto remaining       = std::array<std::size_t, 2>{};
    auto remaining_count = std::size_t{0};
    for (std::size_t index = 0; index < points.size(); ++index)
    {
        if (index == index_b || index == index_t)
            continue;
        if (remaining_count >= remaining.size())
            return std::nullopt;

        remaining[remaining_count] = index;
        remaining_count++;
    }
    if (remaining_count != remaining.size())
        return std::nullopt;

    const auto b = points[index_b];
    const auto t = points[index_t];

    auto l = cv::Point2f{};
    auto r = cv::Point2f{};
    {
        const auto z_axis_raw = t - b;
        const auto z_axis_len = std::hypot(static_cast<double>(z_axis_raw.x), static_cast<double>(z_axis_raw.y));
        if (z_axis_len <= kEps)
            return std::nullopt;

        const auto z_axis = cv::Point2d{
            static_cast<double>(z_axis_raw.x) / z_axis_len,
            static_cast<double>(z_axis_raw.y) / z_axis_len,
        };
        const auto y_axis = cv::Point2d{z_axis.y, -z_axis.x};

        const auto first  = points[remaining[0]];
        const auto second = points[remaining[1]];

        const auto first_projection  = static_cast<double>(first.x - center_point.x) * y_axis.x +
                                       static_cast<double>(first.y - center_point.y) * y_axis.y;
        const auto second_projection = static_cast<double>(second.x - center_point.x) * y_axis.x +
                                       static_cast<double>(second.y - center_point.y) * y_axis.y;
        if (std::abs(first_projection - second_projection) <= kEps)
            return std::nullopt;

        if (first_projection > second_projection)
        {
            l = first;
            r = second;
        }
        else
        {
            l = second;
            r = first;
        }
    }

    return std::array<cv::Point2f, 5>{icon_point, t, l, b, r};
}

/**
 * @brief 使用 OpenCV PnP 求解符面位姿。
 * @return 成功时返回 true，结果位姿存储在 result 中。
 */
auto SingleRunePnpSolution::solve(const RuneBullseye &board, cv::Point2d icon, const cv::Mat &camera_matrix,
                                  const cv::Mat &distort_coeffs) -> bool
{
    try
    {
        const auto fx = camera_matrix.at<double>(0, 0);
        const auto fy = camera_matrix.at<double>(1, 1);
        if (fx <= 0.0 || fy <= 0.0)
            return false;

        const auto ordered = image_points(board, icon);
        if (!ordered)
            return false;

        std::vector<cv::Point3f> object_points;
        object_points.reserve(RunePagePoints::kPoints.size());
        for (const auto &point : RunePagePoints::kPoints)
        {
            const auto p_flu = point;
            const auto p_ocv = rm_utils::util::flu2opencv_position(p_flu);
            object_points.push_back(cv::Point3f{
                static_cast<float>(p_ocv.x()),
                static_cast<float>(p_ocv.y()),
                static_cast<float>(p_ocv.z()),
            });
        }
        const auto image_points = std::vector(ordered->begin(), ordered->end());

        // 旋转向量和平移向量
        auto rota_vec      = cv::Vec3d{};
        auto tran_vec      = cv::Vec3d{};
        const auto success = cv::solvePnP(object_points, image_points, camera_matrix, distort_coeffs, rota_vec,
                                          tran_vec, false, cv::SOLVEPNP_EPNP);
        if (!success || tran_vec[2] <= 0.0)
            return false;

        cv::solvePnP(object_points, image_points, camera_matrix, distort_coeffs, rota_vec, tran_vec, true,
                     cv::SOLVEPNP_ITERATIVE);
        if (tran_vec[2] <= 0.0)
            return false;

        auto translation_eigen_opencv = Eigen::Vector3d{};
        cv::cv2eigen(tran_vec, translation_eigen_opencv);

        auto rotation_opencv = cv::Mat{};
        cv::Rodrigues(rota_vec, rotation_opencv);

        auto rotation_eigen_opencv = Eigen::Matrix3d{};
        cv::cv2eigen(rotation_opencv, rotation_eigen_opencv);

        result.xyz_in_camera = rm_utils::util::opencv2flu_position(translation_eigen_opencv);
        result.q_rune2camera =
            Eigen::Quaterniond{rm_utils::util::opencv2flu_rotation(rotation_eigen_opencv)}.normalized();
    }
    catch (cv::Exception const &)
    {
        return false;
    }
    return true;
}

} // namespace auto_buff
