#include "big_activated.hpp"

#include "rune_pose.hpp"
#include "szu_light_arm_model.hpp"
#include "rm_utils/algorithm/conversion.hpp"

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace auto_buff
{
namespace
{
struct Match
{
    double cost;
    double rms;
    double support;
};

// SZU 模型先转入 RMCS 符面，再投影到原图。
std::vector<cv::Point2d> project_model(std::span<const cv::Point3d> model, double angle,
                                       const Eigen::Matrix3d &face_to_camera, const Eigen::Vector3d &center_camera,
                                       const cv::Mat &intrinsic, const cv::Mat &distortion)
{
    const auto rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
    std::vector<cv::Point3d> optical;
    optical.reserve(model.size());
    for (auto p : model)
    {
        // SZU (x,y,z) 转 RMCS (z,-x,-y)，模型已包含 -11.1 mm 灯臂偏移。
        const Eigen::Vector3d local(p.z, -p.x, -p.y);
        const Eigen::Vector3d position = face_to_camera * rotation * local + center_camera;
        const auto point               = rm_utils::util::ros2opencv_position(position);
        if (!point.allFinite() || point.z() <= .1)
            return std::vector<cv::Point2d>{};
        optical.emplace_back(point.x(), point.y(), point.z());
    }

    std::vector<cv::Point2d> pixels;
    cv::projectPoints(optical, cv::Vec3d{}, cv::Vec3d{}, intrinsic, distortion, pixels);
    return pixels;
}

// 双线性采样轮廓距离，分别统计截断代价、RMS 和有效支持比例。
Match evaluate_contour(std::span<const cv::Point2d> pixels, const cv::Mat &distance, const cv::Rect &roi, double gate)
{
    if (pixels.empty())
        return {std::numeric_limits<double>::max(), 0, 0};

    double cost    = 0;
    double squared = 0;
    int support    = 0;
    for (auto p : pixels)
    {
        const double x = p.x - roi.x;
        const double y = p.y - roi.y;
        double value   = gate * 3;
        if (std::isfinite(x) && std::isfinite(y) && x >= 0 && y >= 0 && x < distance.cols - 1 && y < distance.rows - 1)
        {
            const int ix    = int(x);
            const int iy    = int(y);
            const double dx = x - ix;
            const double dy = y - iy;
            value = (1 - dx) * (1 - dy) * distance.at<float>(iy, ix) + dx * (1 - dy) * distance.at<float>(iy, ix + 1) +
                    (1 - dx) * dy * distance.at<float>(iy + 1, ix) + dx * dy * distance.at<float>(iy + 1, ix + 1);
        }
        support += value <= gate;
        squared += value * value;
        cost += std::min(value * value, gate * gate * 4);
    }
    return {cost / pixels.size(), std::sqrt(squared / pixels.size()), double(support) / pixels.size()};
}

} // namespace

std::vector<Board> reconstruct_big_activated(std::span<const LightArm> arms, const cv::Mat &intrinsic,
                                             const cv::Mat &distortion, const Eigen::Isometry3d &camera_transform,
                                             const RuneConfig &config, std::span<const RuneIcon> icons,
                                             std::span<const RuneBullseye> bullseyes)
{
    std::vector<Board> result;
    if (arms.empty())
        return result;

    // 只用同帧完整待击板端点建立符面。
    std::vector<RuneBullseye> seeds;
    for (const auto &board : bullseyes)
        if (!board.active &&
            std::ranges::all_of(board.corners, [](const auto &p)
                                { return std::isfinite(p.x) && std::isfinite(p.y) && (p.x != 0 || p.y != 0); }))
            seeds.push_back(board);

    const auto plane = solve_rune_pose(config, intrinsic, distortion, camera_transform, icons, seeds);
    if (!plane)
        return result;

    const auto face            = Eigen::AngleAxisd(plane->face_yaw_odom, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto camera_to_world = Eigen::Quaterniond(camera_transform.rotation());
    const auto origin          = Eigen::Vector3d(camera_transform.translation());
    const Eigen::Matrix3d face_to_camera = camera_to_world.conjugate().toRotationMatrix() * face;
    const Eigen::Vector3d center_camera  = camera_to_world.conjugate() * (plane->center_odom - origin);
    const std::array<cv::Point3d, 1> r_model{{{0, 0, -kRuneIconProminentDistance}}};
    const auto projected_r = project_model(r_model, 0, face_to_camera, center_camera, intrinsic, distortion);
    if (projected_r.empty())
        return result;

    for (const auto &arm : arms)
    {
        if (arm.pose.state != 2 || (arm.color != EnemyColor::RED && arm.color != EnemyColor::BLUE) ||
            arm.contour.size() < 8 || !arm.pose.valid[2] || !std::isfinite(arm.pose.points[2].x) ||
            !std::isfinite(arm.pose.points[2].y) ||
            cv::norm(arm.pose.points[2] - projected_r[0]) > config.init_center_gate)
            continue;

        std::vector<cv::Point2d> raw;
        for (int k : {0, 1, 3, 4})
            if (arm.pose.valid[k] && std::isfinite(arm.pose.points[k].x) && std::isfinite(arm.pose.points[k].y))
                raw.push_back(arm.pose.points[k]);
        if (raw.size() < 3)
            continue;

        std::vector<cv::Point2d> normalized;
        cv::undistortPoints(raw, normalized, intrinsic, distortion);
        Eigen::Vector2d radial            = Eigen::Vector2d::Zero();
        bool valid                        = true;
        const Eigen::Vector3d normal      = face.col(0);
        const Eigen::Vector3d lamp_origin = plane->center_odom - normal * .0111;
        for (auto p : normalized)
        {
            const Eigen::Vector3d ray = camera_to_world * Eigen::Vector3d(1, -p.x, -p.y);
            const double denominator  = normal.dot(ray);
            if (std::abs(denominator) < 1e-6)
            {
                valid = false;
                break;
            }
            const double depth          = normal.dot(lamp_origin - origin) / denominator;
            const Eigen::Vector3d local = face.transpose() * (origin + depth * ray - plane->center_odom);
            if (depth <= .1 || !local.allFinite())
                valid = false;
            radial += local.tail<2>();
        }
        radial /= double(normalized.size());
        if (!valid || radial.norm() < .08 || radial.norm() > .5)
            continue;

        const double initial = std::atan2(-radial.x(), radial.y());

        const auto bounds = cv::boundingRect(arm.contour);
        const auto roi    = bounds + cv::Size(8, 8) - cv::Point(4, 4);
        cv::Mat mask(roi.size(), CV_8UC1, cv::Scalar(255));
        cv::drawContours(mask, std::vector<std::vector<cv::Point>>{arm.contour}, 0, cv::Scalar(0), 1, cv::LINE_8,
                         cv::noArray(), 0, -roi.tl());
        cv::Mat distance;
        cv::distanceTransform(mask, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        const double gate   = std::max(2., std::min(bounds.width, bounds.height) * .10);
        const auto evaluate = [&](double angle)
        {
            const auto pixels =
                project_model(szu_light_arm_model, angle, face_to_camera, center_camera, intrinsic, distortion);
            return evaluate_contour(pixels, distance, roi, gate);
        };

        // 围绕实测初值进行有界角度搜索。
        constexpr double step = std::numbers::pi / 180;
        double angle          = initial;
        Match best{std::numeric_limits<double>::max(), 0, 0};
        int best_index = 0;
        for (int i = -30; i <= 30; ++i)
        {
            const auto match = evaluate(initial + i * step);
            if (match.cost < best.cost)
            {
                best       = match;
                angle      = initial + i * step;
                best_index = i;
            }
        }
        if (std::abs(best_index) == 30)
            continue;

        for (double delta : {step / 5, step / 25})
        {
            const double center_angle = angle;
            for (int i = -5; i <= 5; ++i)
            {
                const auto match = evaluate(center_angle + i * delta);
                if (match.cost < best.cost)
                {
                    best  = match;
                    angle = center_angle + i * delta;
                }
            }
        }
        if (best.support < .8 || best.rms > gate * .75 || evaluate(angle - step * 5).cost < best.cost + .1 ||
            evaluate(angle + step * 5).cost < best.cost + .1)
            continue;

        const std::array<cv::Point3d, 1> board_model{{{0, -kRuneGlobalRadius, 0}}};
        const auto pixel = project_model(board_model, angle, face_to_camera, center_camera, intrinsic, distortion);
        if (pixel.empty())
            continue;

        Board board;
        board.center            = pixel[0];
        board.color             = arm.color;
        board.source            = "yolo_big_activated_contour";
        board.appearance        = Appearance::big_activated;
        board.geometry_error_px = best.rms;
        board.geometry_support  = best.support;

        const bool direct = std::ranges::any_of(
            bullseyes, [&](const auto &b)
            { return cv::norm(board.center - cv::Point2d(b.center.x, b.center.y)) < config.init_center_gate; });
        const bool duplicate = std::ranges::any_of(
            result, [&](const auto &b) { return cv::norm(board.center - b.center) < config.init_center_gate; });
        if (!direct && !duplicate)
            result.push_back(std::move(board));
    }
    return result;
}

} // namespace auto_buff
