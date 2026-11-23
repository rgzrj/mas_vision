#include "pixel_geometry.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_buff
{

std::vector<std::vector<cv::Point>> yolo_color_contours(const cv::Mat &image, cv::Rect roi,
                                                       EnemyColor color)
{
    roi &= cv::Rect(0, 0, image.cols, image.rows);

    if (roi.empty())
        return {};

    std::vector<cv::Mat> channels;
    cv::split(image(roi), channels);

    cv::Mat mask = color == EnemyColor::RED ? channels[2] - channels[0] : channels[0] - channels[2];
    cv::GaussianBlur(mask, mask, {5, 5}, 0);
    cv::threshold(mask, mask, color == EnemyColor::RED ? 50 : 62, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, roi.tl());
    return contours;
}

std::vector<cv::Point> yolo_board_contour(const cv::Mat &image, const PoseDetection &pose,
                                         EnemyColor color)
{
    std::vector<cv::Point2f> points;
    cv::Point2d search_center{};

    for (int k : PoseDetection::leaf_indices)
    {
        const auto p = pose.points[k];

        // 搜索中心和面积初值依赖完整四点，低分但坐标可用的点仍可作先验，最终端点须从像素重新实测。
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
            p.x >= image.cols || p.y >= image.rows)
            return {};
            
        points.emplace_back(p);
        search_center += p * .25; 
    }

    const double area = .25 * CV_PI * cv::norm(pose.points[0] - pose.points[4]) *
                        cv::norm(pose.points[1] - pose.points[3]);

    if (area < 16)
        return {};

    if (pose.valid[2])
        points.emplace_back(pose.points[2]);

    auto rectangle  = cv::minAreaRect(points);
    rectangle.size *= 1.4f;
    const auto roi  = rectangle.boundingRect() & cv::Rect(0, 0, image.cols, image.rows);

    std::vector<cv::Point> best;
    double best_error = .35; // 面积相对误差上限。
    for (const auto &contour : yolo_color_contours(image, roi, color))
    {
        if (contour.size() < 6 || cv::pointPolygonTest(contour, search_center, false) <= 0 ||
            (pose.valid[2] && cv::pointPolygonTest(contour, pose.points[2], false) >= 0))
            continue;

        const auto bounds = cv::boundingRect(contour);

        if (bounds.x <= roi.x || bounds.y <= roi.y || bounds.br().x >= roi.br().x || bounds.br().y >= roi.br().y)
            continue;

        // 凸包面积比过小的轮廓可能是噪点或被遮挡的残缺轮廓，排除。
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        const double measured_area = cv::contourArea(contour);

        if (measured_area < cv::contourArea(hull) * .8)
            continue;

        const double error = std::abs(measured_area - area) / area;
        if (error < best_error)
        {
            best_error = error;
            best = contour;
        }
    }
    return best;
}

EnemyColor yolo_board_color(const cv::Mat &image, const Board &board, const cv::Rect2d &box)
{
    return yolo_observation_color(image,
                                  board.appearance == Appearance::full_mark
                                      ? std::span<const cv::Point2d>(board.corners)
                                      : std::span<const cv::Point2d>{},
                                  board.center, box);
}

EnemyColor yolo_observation_color(const cv::Mat &image, std::span<const cv::Point2d> points, cv::Point2d center,
                                   const cv::Rect2d &box)
{
    const auto bounds   = box & cv::Rect2d(0, 0, image.cols, image.rows);
    const double radius = std::min(box.width, box.height) / 2;

    if (bounds.empty() || radius <= 0)
        return EnemyColor::UNKNOWN;

    double red = 0, blue = 0;
    // 采样点数
    int votes = 0;
    const auto sample = [&](int x, int y, double weight = 1.)
    {
        const auto p         = image.at<cv::Vec3b>(y, x);
        const int difference = int(p[2]) - p[0];

        // 排除暗背景、近白过曝核及绿色干扰；灯珠周围有色亮核/边沿提供颜色证据。
        if (std::max(p[0], p[2]) < 20 || std::abs(difference) < 8 || int(std::max(p[0], p[2])) - p[1] < 5)
            return;

        red += std::max(difference, 0) * weight;
        blue += std::max(-difference, 0) * weight;
        ++votes;
    };

    // 提示点数
    int tips = 0;
    if (!points.empty())
    {
        const int half = std::max(2, cvRound(radius * .12));
        for (const auto point : points)
        {
            if (point == cv::Point2d{} || !bounds.contains(point))
                continue;
                
            ++tips;
            const auto patch = cv::Rect(cvRound(point.x) - half, cvRound(point.y) - half, half * 2 + 1, half * 2 + 1) &
                               cv::Rect(bounds) & cv::Rect(0, 0, image.cols, image.rows);
            for (int y = patch.y; y < patch.y + patch.height; ++y)
                for (int x = patch.x; x < patch.x + patch.width; ++x)
                    sample(x, y);
        }
    }

    if (!tips)
    {
        // 环形 ROI 随框尺寸缩放；不扫描整个框，否则红圆外侧的蓝色结构会干扰。
        const auto pixels = cv::Rect(bounds) & cv::Rect(0, 0, image.cols, image.rows);
        for (int y = pixels.y; y < pixels.y + pixels.height; ++y)
            for (int x = pixels.x; x < pixels.x + pixels.width; ++x)
            {
                const double nx = (x - center.x) / (box.width / 2);
                const double ny = (y - center.y) / (box.height / 2);
                const double r2 = nx * nx + ny * ny;
                // 框边缘更可能混入外侧装饰；连续降权而非再缩框裁掉真实圆弧。
                if (r2 >= .36 && r2 < 1.)
                    sample(x, y, 1. - r2);
            }
    }
    if (votes < 3 || std::abs(red - blue) < .25 * (red + blue))
        return EnemyColor::UNKNOWN;

    return red > blue ? EnemyColor::RED : EnemyColor::BLUE;
}

void extract_yolo_light_arm_contour(const cv::Mat &image, LightArm &arm)
{
    arm.contour.clear();

    std::vector<cv::Point2f> points;
    for (int k : PoseDetection::leaf_indices)
        if (arm.pose.valid[k] && std::isfinite(arm.pose.points[k].x) && std::isfinite(arm.pose.points[k].y))
            points.emplace_back(arm.pose.points[k]);

    if (points.size() < 3)
        return;

    if (arm.color != EnemyColor::RED && arm.color != EnemyColor::BLUE)
        return;

    const auto box   = cv::boundingRect(points);
    const int margin = std::max(5, cvRound(std::max(box.width, box.height) * .2));
    const auto roi   =
        (box + cv::Size(margin * 2, margin * 2) - cv::Point(margin, margin)) & cv::Rect(0, 
            0, image.cols, image.rows);

    if (roi.empty())
        return;

    // SZU RuneObservationRefiner: color difference, 5x5 blur, external contours.
    // Class 2 points locate the arm itself; never treat their mean as an armor center.
    const auto contours = yolo_color_contours(image, roi, arm.color);
    const double gate   = std::max(2., std::min(box.width, box.height) * .12);
    double best         = std::numeric_limits<double>::max(), second = best;
    for (const auto &contour : contours)
    {
        if (contour.size() < 8 || cv::contourArea(contour) < 8)
            continue;

        const auto bounds = cv::boundingRect(contour);
        if (bounds.x <= roi.x || bounds.y <= roi.y || bounds.br().x >= roi.br().x || bounds.br().y >= roi.br().y)
            continue; 

        if (arm.pose.valid[2] && cv::pointPolygonTest(contour, arm.pose.points[2], false) >= 0)
            continue;

        int support  = 0;
        double error = 0;
        for (auto point : points)
        {
            const double distance = std::abs(cv::pointPolygonTest(contour, point, true));
            support += distance <= gate;
            error += std::min(distance, gate * 2);
        }
        if (support < 3)
            continue;

        error /= points.size();
        if (error < best)
        {
            second      = best;
            best        = error;
            arm.contour = contour;
        }
        else
            second = std::min(second, error);
    }
    // 两个候选误差接近，存在歧义，不输出轮廓。
    if (!arm.contour.empty() && second - best < .5)
        arm.contour.clear(); 
}

} // namespace auto_buff
