#include "pose_output.hpp"

#include "../pixels/pixel_geometry.hpp"
#include "../pixels/pixel_refiner.hpp"
#include "../timing.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace auto_buff
{

namespace
{

// 四个网络端点必须均通过置信度及坐标检查，再以两条对径线的交点求板心。
bool leaf_center(const PoseDetection &pose, cv::Point2d &center)
{
    for (int i : PoseDetection::leaf_indices)
        if (!pose.valid[i])
            return false;
    const auto a = pose.points[0], b = pose.points[1];
    const auto u = pose.points[4] - a, v = pose.points[3] - b;

    // 2D叉积，判断两条线段是否平行
    const double cross = u.cross(v);
    if (std::abs(cross) < 1e-6)
        return false;
    const double t = (b - a).cross(v) / cross;
    const double s = (b - a).cross(u) / cross;
    if (t <= 0 || t >= 1 || s <= 0 || s >= 1)
        return false;
    center = a + t * u;
    return true;
}

// 对坐标有限的非R点求均值，允许不足四点，也不要求分数达标或位于图内
cv::Point2d region_center(const PoseDetection &pose)
{
    cv::Point2d center{};
    int count = 0;
    for (int i : PoseDetection::leaf_indices)
        if (std::isfinite(pose.points[i].x) && std::isfinite(pose.points[i].y))
        {
            center += pose.points[i];
            ++count;
        }
    return count ? center / count : center;
}

} // namespace

std::vector<PoseDetection> decode_pose_output(std::span<const float> data, std::size_t count, cv::Size size,
                                              double scale, cv::Point pad, double confidence, double point_confidence,
                                              double distance)
{
    if (!count || data.size() != kPoseChannels * count || !std::isfinite(scale) || scale <= 0 ||
        !std::isfinite(distance) || distance <= 0)
        throw std::runtime_error("Invalid SZU output/scale/NMS distance");

    std::vector<PoseDetection> candidates;
    for (std::size_t n = 0; n < count; ++n)
    {
        const auto value = [&](int c) {
            return data[c * count + n];
        };
        PoseDetection pose;
        pose.score = value(0);
        for (int c = 1; c < 3; ++c)
            if (value(c) > pose.score)
            {
                pose.state = c;
                pose.score = value(c);
            }

        if (!std::isfinite(pose.score) || pose.score < confidence)
            continue;

        for (int k = 0; k < 5; ++k)
        {
            pose.points[k]   = {(value(3 + 3 * k) - pad.x) / scale, (value(4 + 3 * k) - pad.y) / scale};
            const auto p     = pose.points[k];
            const auto score = value(5 + 3 * k);

            pose.point_scores[k] = score;
            pose.valid[k]        = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(score) &&
                                score >= point_confidence && p.x >= 0 && p.y >= 0 && p.x < size.width && p.y < size.height;
        }
        // 沿用上游至少三个有效点的要求；低分、越界点不会被裁到图像边缘后当真值。
        if (std::count(pose.valid.begin(), pose.valid.end(), true) >= 3)
            candidates.push_back(pose);
    }

    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const auto &a, const auto &b) { return a.score > b.score; });

    std::vector<PoseDetection> kept;

    // 按目标置信度降序抑制重复候选；状态 0/1 同组去重，状态 2 单独成组。
    for (const auto &pose : candidates)
    {
        const double gate    = pose.state == 2 ? distance / 3 : distance;
        const bool duplicate = std::ranges::any_of(kept, [&](const auto &other)
        {
            return (pose.state == 2) == (other.state == 2) &&
                   cv::norm(region_center(pose) - region_center(other)) < gate;
        });

        if (!duplicate)
            kept.push_back(pose);
    }
    return kept;
}

BuffDetector::Detection make_pose_detection(const std::vector<PoseDetection> &poses, const cv::Mat &image, EnemyColor color,
                              bool refine, bool profiling)
{
    BuffDetector::Detection result;
    if (color != EnemyColor::RED && color != EnemyColor::BLUE)
        throw std::invalid_argument("YOLO observation color must be red or blue");
    for (const auto &pose : poses)
    {
        // 多片符板可以共享同一个 R；保留高分位置并合并相邻重复候选。
        if (pose.valid[2] && !std::ranges::any_of(result.markers, [&](const auto &marker){
                return cv::norm(marker.center - pose.points[2]) < 5;
            }))
            {
                // R 无颜色类别，不能填筛选策略。
                result.markers.push_back({pose.points[2], pose.score, EnemyColor::UNKNOWN, true});
            }

        // 处理大符激活灯臂
        if (pose.state == 2)
        {
            LightArm arm;
            arm.pose = pose;
            std::vector<cv::Point2d> samples;
            std::vector<cv::Point2f> tips;

            for (int k = 0; k < 4; ++k)
                if (pose.valid[PoseDetection::leaf_indices[k]])
                {
                    samples.push_back(pose.points[PoseDetection::leaf_indices[k]]);
                    tips.emplace_back(samples.back());
                }

            if (!tips.empty())
                arm.color = yolo_observation_color(image, samples, {}, cv::boundingRect(tips));

            if (arm.color != EnemyColor::UNKNOWN && arm.color != color)
                continue;

            // 灯臂轮廓提取。
            extract_yolo_light_arm_contour(image, arm);

            result.light_arms.push_back(std::move(arm));
            continue;
        }

        std::vector<cv::Point2f> points;
        for (int i : PoseDetection::leaf_indices)
            if (pose.valid[i])
                points.emplace_back(pose.points[i]);

        const cv::Rect2d bounds = cv::boundingRect(points);

        Board board;
        board.appearance  = pose.state == 0 ? Appearance::full_mark : Appearance::single_ring;
        bool center_valid = leaf_center(pose, board.center);

        if (!center_valid)
            board.center = region_center(pose); // 只作为精修搜索初值。

        board.radius       = std::min(bounds.width, bounds.height) / 2;
        board.ellipse_axes = bounds.size();

        for (std::size_t i = 0; i < PoseDetection::leaf_indices.size(); ++i)
            if (pose.valid[PoseDetection::leaf_indices[i]])
                board.corners[i] = pose.points[PoseDetection::leaf_indices[i]];

        board.features_valid = pose.state == 0 && center_valid;
        board.color          = yolo_board_color(image, board, bounds);

        if (color != board.color)
            continue;

        if (refine)
        {
            TracyCZoneN(refine_zone, "PixelRefine", profiling);
            TimedZone refine_timer(yolo_timing.refine_ms, refine_zone, yolo_timing.enabled);

            if (yolo_timing.enabled)
                ++yolo_timing.refine_calls;

            center_valid = refine_yolo_pose_pixels(image, pose, board, center_valid);
        }

        if (!center_valid)
            continue;

        // 状态来自网络；像素几何只精修位置，不把已激活板改成待击板。
        board.features_valid &= pose.state == 0;
        board.source = pose.state == 0 ? "yolo_inactive" : "yolo_small_activated";
        result.boards.push_back(board);
    }
    return result;
}

} // namespace auto_buff
