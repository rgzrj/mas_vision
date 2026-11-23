#include "pixel_refiner.hpp"

#include "pixel_geometry.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace auto_buff
{

namespace
{

constexpr double pi      = std::numbers::pi;
constexpr int angle_bins = 360;

/**
 * @brief 将角度索引限制在 [0, angle_bins) 范围内。
 * @param i 角度索引。
 * @return 限制后的角度索引。
 */
int wrap_angle_index(int i)
{
    return (i % angle_bins + angle_bins) % angle_bins;
}

/**
 * @brief 按椭圆极坐标对亮度图重采样，输出行表示径向位置，列表示角度。
 * @param energy ROI 内的单通道浮点亮度图。
 * @param center 采样中心，使用 ROI 局部像素坐标。
 * @param axis_x 椭圆第一条半轴向量；单位圆采样时为 (1, 0)。
 * @param axis_y 椭圆第二条半轴向量；单位圆采样时为 (0, 1)。
 * @param start 起始径向系数；实际偏移为半轴方向向量乘以该系数。
 * @param step 径向系数的采样步长。
 * @param count 径向采样行数。
 * @param columns 一周的角度采样列数。
 * @return 双线性插值后的亮度图；原图外的采样位置填零。
 */
cv::Mat polar_sample(const cv::Mat &energy, cv::Point2d center, cv::Point2d axis_x, cv::Point2d axis_y, double start,
                     double step, int count, int columns)
{
    cv::Mat map_x(count, columns, CV_32F), map_y(count, columns, CV_32F);
    for (int a = 0; a < columns; ++a)
    {
        double theta   = a * 2 * pi / columns;
        auto direction = axis_x * std::cos(theta) + axis_y * std::sin(theta);
        for (int r = 0; r < count; ++r)
        {
            auto point            = center + direction * (start + r * step);
            map_x.at<float>(r, a) = float(point.x);
            map_y.at<float>(r, a) = float(point.y);
        }
    }
    cv::Mat polar;
    cv::remap(energy, polar, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return polar;
}
} // namespace

bool refine_yolo_pose_pixels(const cv::Mat &image, const PoseDetection &pose, Board &board, bool center_valid)
{
    const auto contour = yolo_board_contour(image, pose, board.color);
    if (contour.empty())
        return center_valid;
    const cv::Rect2d box = cv::boundingRect(contour);
    Board measured     = board;
    measured.center    = box.tl() + cv::Point2d(box.width / 2, box.height / 2);
    measured.radius    = std::min(box.width, box.height) / 2;
    const bool refined = refine_yolo_board_pixels(image, measured, contour);
    // 部分实测轴线可以提供中心；已有完整原始几何时仍按整组回退，避免混用端点。
    if (!refined || (!measured.features_valid && center_valid))
        return center_valid;
    board = std::move(measured);
    return true;
}

/**
 * @brief 从 YOLO 框附近的像素精修符板中心、椭圆及四臂端点。
 * @param image 原始 BGR 图像；输出点始终使用原图像素坐标。
 * @param board 输入框提供的中心、半径估计、颜色和外观状态，并接收精修结果。
 * @return 两条实测轴线相交得到物理板心返回 true；完整四端点才置 features_valid。
 * @note 椭圆只用于搜索，不能单独提供物理板心。
 */
bool refine_yolo_board_pixels(const cv::Mat &image, Board &board, std::span<const cv::Point> contour)
{
    // 每帧先清空端点，避免精修中途退出时沿用上一轮的四点。
    board.features_valid = false;
    board.corners        = {};

    // 1. 围绕检测框的初始中心取 ROI，后续采样均先使用 ROI 局部坐标。
    double radius = board.radius;
    if (radius < 5 || contour.empty())
        return false;
    auto bounds = cv::Rect(cvFloor(board.center.x - radius * 1.5), cvFloor(board.center.y - radius * 1.5),
                           cvCeil(radius * 3 + 2), cvCeil(radius * 3 + 2)) &
                  cv::Rect(0, 0, image.cols, image.rows);
    if (bounds.empty())
        return false;

    // 2. 构造目标颜色的亮度图：排除明显异色像素，保留过曝后的白色灯珠亮核。
    cv::Mat energy(bounds.size(), CV_32F);
    int channel = board.color == EnemyColor::BLUE ? 0 : 2;
    for (int y = 0; y < bounds.height; ++y)
        for (int x = 0; x < bounds.width; ++x)
        {
            auto p = image.at<cv::Vec3b>(y + bounds.y, x + bounds.x);
            // 直接用目标通道减去其余通道会抹掉白色亮核，使拟合在亮核与光晕间跳动。
            energy.at<float>(y, x) =
                int(p[2 - channel]) - p[channel] > 20 ? 0.f : float(.114 * p[0] + .587 * p[1] + .299 * p[2]);
        }

    // 3. 平滑图仅用于连起相邻灯珠的圆弧支撑；端点测量仍使用未模糊的 energy。
    // 此处不做形态学膨胀，避免人为扩大后续端点范围。
    cv::Mat arc_energy;
    cv::GaussianBlur(energy, arc_energy, {}, std::max(.8, radius * .025));
    auto local_center = board.center - cv::Point2d(bounds.tl());
    // 仅使用已按网络语义选出的板面轮廓，不重复搜索R和灯臂。
    std::vector<cv::Point> selected(contour.begin(), contour.end());
    for (auto &point : selected)
        point -= bounds.tl();
    cv::Mat mask = cv::Mat::zeros(bounds.size(), CV_8UC1);
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{selected}, 0, 255, cv::FILLED);
    energy.setTo(0, mask == 0);
    arc_energy.setTo(0, mask == 0);
    int count         = std::max(2, int(std::ceil(radius * .9 / .5)));
    auto polar        = polar_sample(arc_energy, local_center, {1, 0}, {0, 1}, radius * .45, .5, count, 180);
    std::vector<cv::Point2f> points;
    for (int a = 0; a < polar.cols; ++a)
    {
        double maximum = 0;
        for (int r = 0; r < count; ++r)
            maximum = std::max(maximum, double(polar.at<float>(r, a)));
        double threshold = std::max(4., maximum * .15);
        int end          = -1;
        for (int r = 0; r < count; ++r)
            if (polar.at<float>(r, a) >= threshold)
                end = r;
        if (end < 0)
            continue;
        double weight = 0, length = 0;
        // 每条射线只在最外侧有效圆弧附近加权，避免多层同心环把半径拉向内侧。
        for (int r = end; r >= std::max(0, end - 4) && polar.at<float>(r, a) >= threshold; --r)
        {
            double value = polar.at<float>(r, a);
            weight += value;
            length += (radius * .45 + r * .5) * value;
        }
        double theta = a * 2 * pi / polar.cols;
        points.emplace_back(local_center + cv::Point2d(std::cos(theta), std::sin(theta)) * (length / weight));
    }

    if (points.size() < 90)
        return false;

    // 4. 迭代拟合椭圆：以归一化径向残差剔除突出臂、缺灯射线和无关像素。
    cv::RotatedRect ellipse;
    double fit_error = 0;
    for (int iteration = 0; iteration < 4; ++iteration)
    {
        ellipse      = cv::fitEllipse(points);
        double angle = ellipse.angle * pi / 180, cos_angle = std::cos(angle), sin_angle = std::sin(angle);
        double semi_axis_x = ellipse.size.width / 2, semi_axis_y = ellipse.size.height / 2;
        if (semi_axis_x <= 0 || semi_axis_y <= 0 || !std::isfinite(semi_axis_x + semi_axis_y))
            return false;
        std::vector<double> residuals;
        for (auto point : points)
        {
            auto d = point - ellipse.center;
            residuals.push_back(std::abs(std::hypot((d.x * cos_angle + d.y * sin_angle) / semi_axis_x,
                                                    (-d.x * sin_angle + d.y * cos_angle) / semi_axis_y) -
                                         1));
        }

        // 用残差中位数自适应限制内点；误差为归一化半径差，不是像素距离。
        auto sorted = residuals;
        std::ranges::sort(sorted);
        double limit = std::max(.025, sorted[sorted.size() / 2] * 2.5);
        std::vector<cv::Point2f> inliers;
        fit_error = 0;
        for (std::size_t i = 0; i < points.size(); ++i)
            if (residuals[i] <= limit)
            {
                inliers.push_back(points[i]);
                fit_error += residuals[i] * residuals[i];
            }
        if (inliers.size() < 90)
            return false;
        fit_error = std::sqrt(fit_error / inliers.size());
        if (inliers.size() == points.size())
            break;
        points = std::move(inliers);
    }

    // 5. 检查中心偏移、椭圆尺度和拟合误差，再要求圆弧覆盖至少 9/12 个扇区。
    auto center  = cv::Point2d(ellipse.center) + cv::Point2d(bounds.tl());
    double major = std::max(ellipse.size.width, ellipse.size.height) / 2;
    double minor = std::min(ellipse.size.width, ellipse.size.height) / 2;
    if (cv::norm(center - board.center) > radius * .4 || minor < radius * .35 || major > radius * 1.4 ||
        fit_error > .08)
        return false;
    std::array<bool, 12> sectors{};
    for (auto p : points)
    {
        auto d          = cv::Point2d(p) - cv::Point2d(ellipse.center);
        int sector      = int((std::atan2(d.y, d.x) + pi) / (2 * pi) * 12) % 12;
        sectors[sector] = true;
    }
    if (std::count(sectors.begin(), sectors.end(), true) < 9)
        return false;

    // 先写入椭圆搜索几何；实测轴线通过后才会写入物理板心。
    board.center        = center;
    board.radius        = major;
    board.ellipse_axes  = ellipse.size;
    board.ellipse_angle = ellipse.angle;
    board.fit_error     = fit_error;
    board.arc_coverage  = double(points.size()) / 180;
    if (board.appearance != Appearance::full_mark)
        return false; // 单一椭圆中心不冒充透视下的物理圆心。

    // 边缘截断板不能提供可靠的实测轴线。
    if (center.x - major < 1 || center.y - major < 1 || center.x + major >= image.cols - 1 ||
        center.y + major >= image.rows - 1)
        return false;

    // 6. 用椭圆半轴建立归一化极坐标，减小透视压缩对四臂方向搜索的影响。
    double rotation = ellipse.angle * pi / 180;
    cv::Point2d axis_x(std::cos(rotation) * ellipse.size.width / 2, std::sin(rotation) * ellipse.size.width / 2);
    cv::Point2d axis_y(-std::sin(rotation) * ellipse.size.height / 2, std::cos(rotation) * ellipse.size.height / 2);
    int radial_steps = std::max(40, int(std::ceil(major * 1.35 / .5)));
    double step      = 1.35 / radial_steps;
    polar = polar_sample(energy, cv::Point2d(ellipse.center), axis_x, axis_y, 0, step, radial_steps + 1, angle_bins);
    // 7. 角向亮度的四次谐波给出四臂粗相位，随后分别求每条臂的角向质心。
    // 图像中的四臂不必严格正交，也不直接把最亮角度采样格当作端点方向。
    // 粗相位沿用 RMCS compute_bullseye_feature 的思路：
    // Copyright 2025 Alliance Algorithm Team (MIT), see auto_buff/buff_track/LICENSE.
    std::array<double, angle_bins> profile{}, smoothed_profile{};
    for (int a = 0; a < angle_bins; ++a)
        for (int r = 0; r <= radial_steps; ++r)
            if (r * step >= .55 && r * step <= 1.3)
                profile[a] += polar.at<float>(r, a) * r * step;
    double sine = 0, cosine = 0;
    for (int a = 0; a < angle_bins; ++a)
    {
        for (int d = -3; d <= 3; ++d)
            smoothed_profile[a] += profile[wrap_angle_index(a + d)] / 7;
        sine += smoothed_profile[a] * std::sin(4 * a * 2 * pi / angle_bins);
        cosine += smoothed_profile[a] * std::cos(4 * a * 2 * pi / angle_bins);
    }
    double phase = std::atan2(sine, cosine) / 4, peak = 0, valley = 0;
    auto angle_index = [](double angle) { return wrap_angle_index(int(std::llround(angle * angle_bins / (2 * pi)))); };
    for (int k = 0; k < 4; ++k)
        for (int d = -2; d <= 2; ++d)
        {
            peak += smoothed_profile[wrap_angle_index(angle_index(phase + k * pi / 2) + d)] / 20;
            valley += smoothed_profile[wrap_angle_index(angle_index(phase + k * pi / 2 + pi / 4) + d)] / 20;
        }

    // PVD 比较臂方向与臂间方向的亮度，不是神经网络置信度。
    board.pvd = (peak - valley) / (peak + valley + 1e-10);
    if (board.pvd < .2)
        return false;

    // 8. 每条臂独立精修方向、验证亮度，再沿该方向搜索径向末端。
    std::array<cv::Point2d, 4> tips{};
    std::array<double, 4> lengths{};
    std::array<std::vector<cv::Point2f>, 4> arm_points;
    const auto measure_arm = [&](int k)
    {
        int expected    = angle_index(phase + k * pi / 2);
        double baseline = smoothed_profile[expected];
        for (int d = -22; d <= 22; ++d)
            baseline = std::min(baseline, smoothed_profile[wrap_angle_index(expected + d)]);
        double mass = 0, moment = 0;
        for (int d = -22; d <= 22; ++d)
        {
            double value = smoothed_profile[wrap_angle_index(expected + d)] - baseline;
            mass += value;
            moment += value * d;
        }
        if (mass <= 0)
            return false;

        // 去除局部基线后，以亮度质心细化相位，保留亚采样格的角度偏移。
        double offset = moment / mass, angle = (expected + offset) * 2 * pi / angle_bins;
        int best          = wrap_angle_index(int(std::llround(expected + offset)));
        double arm_energy = 0, between_energy = 0;
        for (int d = -2; d <= 2; ++d)
        {
            arm_energy += smoothed_profile[wrap_angle_index(best + d)] / 5;
            between_energy += (smoothed_profile[wrap_angle_index(best + 45 + d)] +
                               smoothed_profile[wrap_angle_index(best - 45 + d)]) /
                              10;
        }
        // 缺一臂的图案也可能有较强的四次谐波，因此每条臂都须比两侧纯圆环区域更亮。
        if (arm_energy < between_energy * 1.18)
            return false;
        // 在多条径线上独立测量臂的亮度质心；不能把所有端点约束到椭圆中心发出的射线。
        for (int r = 0; r <= radial_steps; ++r)
        {
            if (r * step < .55 || r * step > 1.2)
                continue;
            double background = 0;
            for (int d = -2; d <= 2; ++d)
                background += (polar.at<float>(r, wrap_angle_index(best + 45 + d)) +
                               polar.at<float>(r, wrap_angle_index(best - 45 + d))) / 10;
            double weight = 0, moment = 0, peak = 0;
            for (int d = -12; d <= 12; ++d)
            {
                double value = std::max(0., double(polar.at<float>(r, wrap_angle_index(best + d))) - background);
                weight += value;
                moment += value * d;
                peak = std::max(peak, value);
            }
            if (peak < std::max(8., background * .18) || weight <= 0)
                continue;
            double theta = (best + moment / weight) * 2 * pi / angle_bins;
            arm_points[k].emplace_back(center + (axis_x * std::cos(theta) + axis_y * std::sin(theta)) * (r * step));
        }
        if (arm_points[k].size() < 6 || cv::norm(arm_points[k].back() - arm_points[k].front()) < 5)
            return false;
        std::vector<double> raw_radial(radial_steps + 1), smoothed_radial(radial_steps + 1);
        for (int r = 0; r <= radial_steps; ++r)
            for (int d = -3; d <= 3; ++d)
                raw_radial[r] += polar.at<float>(r, wrap_angle_index(best + d)) / 7;
        // 径向窗口约覆盖 2.5 像素，抑制离散灯珠造成的起伏；
        // 保留细采样网格，便于在最后两个采样点之间插值求阈值交点。
        for (int r = 0; r <= radial_steps; ++r)
            for (int d = -2; d <= 2; ++d)
                if (r + d >= 0 && r + d <= radial_steps)
                    smoothed_radial[r] += raw_radial[r + d] / 5;
        double threshold = std::max(8., *std::max_element(smoothed_radial.begin(), smoothed_radial.end()) * .25);
        int end          = -1;
        for (int r = 0; r <= radial_steps; ++r)
            if (r * step >= .65 && smoothed_radial[r] >= threshold)
                end = r;
        if (end < 0 || end == radial_steps)
            return false;

        // 端点外至少约两像素保持低亮度，拒绝采样边界和紧邻的第二个亮结构。
        int outside = std::max(2, int(std::ceil(2. / (minor * step))));
        if (end + outside > radial_steps)
            return false;
        for (int r = end + 1; r <= end + outside; ++r)
            if (smoothed_radial[r] >= threshold)
                return false;
        double fraction = std::clamp((smoothed_radial[end] - threshold) /
                                         std::max(1e-10, smoothed_radial[end] - smoothed_radial[end + 1]),
                                     0., 1.);
        lengths[k]      = (end + fraction) * step;
        tips[k]         = center + (axis_x * std::cos(angle) + axis_y * std::sin(angle)) * lengths[k];
        if (tips[k].x < 1 || tips[k].y < 1 || tips[k].x >= image.cols - 1 || tips[k].y >= image.rows - 1)
            return false;
        return true;
    };
    std::array<bool, 4> measured{};
    for (int k = 0; k < 4; ++k)
    {
        measured[k] = measure_arm(k);
        if (!measured[k])
            tips[k] = {};
    }
    const int count_measured = std::count(measured.begin(), measured.end(), true);
    if (count_measured < 3)
        return false;

    // 三条实测臂足以提供两组轴线；缺失端点保持零，不补造PnP端点。
    double mean = 0, variance = 0;
    for (int k = 0; k < 4; ++k)
        if (measured[k]) mean += lengths[k] / count_measured;
    for (int k = 0; k < 4; ++k)
        if (measured[k]) variance += (lengths[k] - mean) * (lengths[k] - mean) / count_measured;
    if (std::sqrt(variance) > mean * .25)
        return false;
    std::array<cv::Vec4f, 2> lines;
    for (int k = 0; k < 2; ++k)
    {
        std::vector<cv::Point2f> points;
        for (int j : {k, k + 2})
            if (measured[j]) points.insert(points.end(), arm_points[j].begin(), arm_points[j].end());
        cv::fitLine(points, lines[k], cv::DIST_HUBER, 0, .01, .01);
        const cv::Point2d direction(lines[k][0], lines[k][1]), origin(lines[k][2], lines[k][3]);
        double residual = 0;
        for (auto point : points)
            residual += std::abs((cv::Point2d(point) - origin).cross(direction));
        if (residual / points.size() > std::max(1.5, minor * .04))
            return false;
        for (int j : {k, k + 2})
        {
            if (!measured[j]) continue;
            const auto projected = origin + direction * (tips[j] - origin).dot(direction);
            if (cv::norm(projected - tips[j]) > minor * .12 || projected.x < 1 || projected.y < 1 ||
                projected.x >= image.cols - 1 || projected.y >= image.rows - 1)
                return false;
            tips[j] = projected;
        }
    }
    const cv::Point2d u(lines[0][0], lines[0][1]), v(lines[1][0], lines[1][1]);
    const cv::Point2d a(lines[0][2], lines[0][3]), b(lines[1][2], lines[1][3]);
    const double cross = u.cross(v);
    if (std::abs(cross) < .15)
        return false;
    const auto physical_center = a + ((b - a).cross(v) / cross) * u;
    if (cv::norm(physical_center - center) > minor * .25)
        return false;
    for (int k = 0; k < 2; ++k)
        if (measured[k] && measured[k + 2] && (tips[k] - physical_center).dot(tips[k + 2] - physical_center) >= 0)
            return false;
    board.center = physical_center;
    board.corners = tips;
    board.features_valid = count_measured == 4;
    return true;
}
} // namespace auto_buff
