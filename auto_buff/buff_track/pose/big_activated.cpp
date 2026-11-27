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
// 模型轮廓与实测轮廓的匹配结果；距离均在原图像素尺度下计算。
struct Match
{
    double cost;    // 截断后的平均距离平方，用于比较候选转角，单位 px²。
    double rms;     // 未截断距离的均方根，单位 px。
    double support; // 距离不超过 gate 的模型点占比，范围 [0, 1]。
};

// 符面局部 FLU 点先绕 X 轴转过 angle（弧度），再变换到相机 FLU。
// 仅在投影前转换为 OpenCV 光学系：X 向右、Y 向下、Z 向前；三维距离单位均为米。
std::vector<cv::Point2d> project_model(std::span<const cv::Point3d> model, double angle,
                                       const Eigen::Matrix3d &R_rune2camera, const Eigen::Vector3d &center_in_camera,
                                       const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs)
{
    const Eigen::Matrix3d R_model2camera =
        R_rune2camera * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();

    std::vector<cv::Point3d> optical;
    optical.reserve(model.size());
    for (auto p : model)
    {
        const Eigen::Vector3d point_in_camera = R_model2camera * Eigen::Vector3d(p.x, p.y, p.z) + center_in_camera;
        if (!point_in_camera.allFinite() || point_in_camera.x() <= .1)
            return std::vector<cv::Point2d>{};
        const auto point = rm_utils::util::flu2opencv_position(point_in_camera);
        optical.emplace_back(point.x(), point.y(), point.z());
    }

    std::vector<cv::Point2d> pixels;
    // 点已在光学相机系中，因此这里的旋转、平移参数均为零。
    cv::projectPoints(optical, cv::Vec3d{}, cv::Vec3d{}, camera_matrix, distort_coeffs, pixels);
    return pixels;
}

/**
 * @brief 评估投影轮廓与轮廓距离图的匹配质量。
 * @param pixels 投影轮廓点，原图像素坐标。
 * @param distance 实测轮廓的距离图，CV_32F；每个像素存储到轮廓的最近距离。
 * @param roi 距离图对应的原图区域，用于将投影点转换为图内坐标。
 * @param gate 内点距离门限，单位像素；超出图像的点按较大误差处理。
 */
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
            // 投影点通常落在像素之间，插值读取距离，避免匹配代价随整像素取整跳变。
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

// 先用同帧 R 和完整待击板确定符面位姿，再匹配灯臂轮廓，恢复对应靶心的像素位置。
std::vector<Board> reconstruct_big_activated(std::span<const LightArm> arms, const cv::Mat &camera_matrix,
                                             const cv::Mat &distort_coeffs, const Eigen::Isometry3d &T_camera2world,
                                             const RuneConfig &config, std::span<const RuneIcon> icons,
                                             std::span<const RuneBullseye> bullseyes)
{
    std::vector<Board> result;
    if (arms.empty())
        return result;

    // 激活灯臂的四点不对应待击板端点，不能用它们初始化符面位姿。
    std::vector<RuneBullseye> seeds;
    for (const auto &board : bullseyes)
        if (!board.active &&
            std::ranges::all_of(board.corners, [](const auto &p)
                                { return std::isfinite(p.x) && std::isfinite(p.y) && (p.x != 0 || p.y != 0); }))
            seeds.push_back(board);

    const auto plane = solve_rune_pose(config, camera_matrix, distort_coeffs, T_camera2world, icons, seeds);
    if (!plane)
        return result;

    const auto R_rune2world = Eigen::AngleAxisd(plane->face_yaw_in_world, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    
    const auto q_camera2world              = Eigen::Quaterniond(T_camera2world.rotation());
    const auto t_camera2world              = Eigen::Vector3d(T_camera2world.translation());
    const Eigen::Matrix3d R_rune2camera    = q_camera2world.conjugate().toRotationMatrix() * R_rune2world;
    const Eigen::Vector3d center_in_camera = q_camera2world.conjugate() * (plane->center_in_world - t_camera2world);

    /*
     * 符面局部系：原点是转动中心，X 轴沿符面法线，零角度时靶心在 +Z 轴上。
     * 模型点先绕局部 X 轴旋转，再由 R_rune2camera 转到相机 FLU，最后加上 center_in_camera。
     * center_in_camera 表示转动中心相对相机光心的位置，单位米。
     * 灯臂在局部 x=-0.0111 m 平面上，这个偏移已写入点表，不需要再加一次。
     * 投影前将相机 FLU 转为 OpenCV 光学系，反投影射线则从光学系转回相机 FLU。
     */
    const Eigen::Vector3d normal_in_camera      = R_rune2camera.col(0);
    const Eigen::Vector3d lamp_origin_in_camera = center_in_camera - normal_in_camera * .0111;
    const std::array<cv::Point3d, 1> r_model{{{-kRuneIconProminentDistance, 0, 0}}};
    const auto projected_r = project_model(r_model, 0, R_rune2camera, center_in_camera, camera_matrix, distort_coeffs);
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
        cv::undistortPoints(raw, normalized, camera_matrix, distort_coeffs);
        Eigen::Vector2d radial = Eigen::Vector2d::Zero();
        bool valid             = true;
        for (auto p : normalized)
        {
            // 去畸变结果是光学系归一化坐标 (u,v)，对应相机 FLU 射线 (1,-u,-v)。
            const Eigen::Vector3d ray_in_camera = rm_utils::util::opencv2flu_position(Eigen::Vector3d(p.x, p.y, 1));
            const double denominator            = normal_in_camera.dot(ray_in_camera);
            if (std::abs(denominator) < 1e-6)
            {
                valid = false;
                break;
            }
            // 射线与灯臂平面求交；depth 是相机 FLU 的 X 方向深度，单位米，不是射线长度。
            const double depth = normal_in_camera.dot(lamp_origin_in_camera) / denominator;
            const Eigen::Vector3d point_in_rune =
                R_rune2camera.transpose() * (depth * ray_in_camera - center_in_camera);
            if (depth <= .1 || !point_in_rune.allFinite())
                valid = false;
            radial += point_in_rune.tail<2>(); // 符面 FLU 的 Y、Z 分量。
        }
        radial /= double(normalized.size());
        if (!valid || radial.norm() < .08 || radial.norm() > .5)
            continue;

        // radial 存储局部 (y,z)；绕 +X 旋转时 y=-r*sin(angle)、z=r*cos(angle)，故取 atan2(-y,z)。
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
            const auto pixels = project_model(szu_light_arm_model, angle, R_rune2camera, center_in_camera,
                                              camera_matrix, distort_coeffs);
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

        const std::array<cv::Point3d, 1> board_model{{{0, 0, kRuneGlobalRadius}}};
        const auto pixel =
            project_model(board_model, angle, R_rune2camera, center_in_camera, camera_matrix, distort_coeffs);
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
