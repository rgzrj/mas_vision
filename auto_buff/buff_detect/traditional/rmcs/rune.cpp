#include "rune.hpp"
#include "auto_buff/buff_detect/traditional/process.hpp"

#include <opencv2/imgproc.hpp>
#include "compat/thinning.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <ranges>
#include <vector>

namespace auto_buff {

namespace details {
    constexpr auto kBullseyeRadius = 0.15;
    constexpr auto kRuneIconRadius = 0.05;

    /**
     * @brief 根据 R 的骨架连通性、端点、分支点及孔洞数计算候选评分。
     * @param roi R 候选的灰度局部图像。
     * @return 拓扑评分；名称沿用上游，实际不是几何距离或概率。
     */
    auto compute_distance_with_icon(const cv::Mat& roi) -> double {
        // 1. Otsu 二值化
        auto binary = cv::Mat { };
        cv::threshold(roi, binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

        // 2. Zhang-Suen 骨架提取，并要求只有一个前景连通分量。
        auto skel = cv::Mat { };
        cv::rmcs_compat::thinning(binary, skel, cv::rmcs_compat::THINNING_ZHANGSUEN);

        auto labels = cv::Mat { };
        if (cv::connectedComponents(skel, labels, 8) - 1 != 1) {
            return 0.0;
        }

        // 3. 统计骨架端点与分支点。
        const auto h  = skel.rows;
        const auto cy = h / 2;

        auto total_ep      = 0;
        auto lower_ep      = 0;
        auto branch_points = 0;

        for (auto y : std::views::iota(1, h - 1)) {
            for (auto x : std::views::iota(1, skel.cols - 1)) {
                if (skel.at<std::uint8_t>(y, x) == 0) continue;

                auto neighbors = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dy == 0 && dx == 0) continue;
                        if (skel.at<std::uint8_t>(y + dy, x + dx) > 0) ++neighbors;
                    }
                }

                if (neighbors == 1) {
                    ++total_ep;
                    if (y >= cy) ++lower_ep;
                } else if (neighbors >= 3) {
                    ++branch_points;
                }
            }
        }

        // 4. 洞检测：统计最外层轮廓的内部孔洞数。
        auto contours  = std::vector<std::vector<cv::Point>> { };
        auto hierarchy = std::vector<cv::Vec4i> { };
        cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        auto holes = 0;
        if (!contours.empty() && !hierarchy.empty()) {
            auto outer_idx = std::size_t { 0 };
            auto max_area  = 0.0;
            for (const auto& [i, h] : hierarchy | std::views::enumerate) {
                if (h[3] != -1) continue;
                const auto area = cv::contourArea(contours[i]);
                if (area > max_area) {
                    max_area  = area;
                    outer_idx = i;
                }
            }
            for (const auto& [i, h] : hierarchy | std::views::enumerate) {
                if (h[3] == static_cast<int>(outer_idx)) {
                    ++holes;
                }
            }
        }

        // 5. 沿用上游的拓扑门限，得到 R 候选评分。
        if (total_ep >= 1 && lower_ep >= 1 && branch_points >= 8 && branch_points <= 50
            && holes <= 2) {
            return 0.5 + 0.1 * total_ep;
        }

        return 0.0;
    }

    /**
     * @brief 用角向四次谐波和峰谷亮度判别完整符板/圆环，并提取径向端点。
     * @param roi 符板轮廓掩膜内的 BGR 或灰度图。
     * @param center 圆心，使用 ROI 局部坐标。
     * @param endpoints 接收 ROI 局部四点；圆环分支返回的只是半径占位点。
     * @param score 接收 PVD，供外层判定 active 状态。
     * @param active_threshold PVD 低于此值时按打击后圆环处理。
     * @return 图案几何有效时为 true；四臂长度不一致时为 false。
     */
    auto compute_bullseye_feature(const cv::Mat& roi, cv::Point2f center,
        std::array<cv::Point2f, 4>& endpoints, double& score, double active_threshold) -> bool {

        constexpr auto kRadialBins = 32;
        constexpr auto kThetaBins  = 180;

        constexpr auto kArmLengthStddevMax = 0.3;

        auto square = cv::Mat { };
        {
            const auto square_size = std::max(roi.cols, roi.rows);

            square = cv::Mat(square_size, square_size, roi.type(), cv::Scalar { 0, 0, 0 });
            roi.copyTo(square(cv::Rect { 0, 0, roi.cols, roi.rows }));
        }

        auto gray = cv::Mat { };
        if (square.channels() == 3) {
            cv::cvtColor(square, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = square;
        }

        const auto max_radius = std::min(square.cols, square.rows) * 0.5;

        auto polar_hist = std::array<float, static_cast<std::size_t>(kRadialBins * kThetaBins)> { };
        {
            for (int y = 0; y < gray.rows; ++y) {
                for (int x = 0; x < gray.cols; ++x) {
                    const auto dx = static_cast<double>(x) - center.x;
                    const auto dy = static_cast<double>(y) - center.y;
                    const auto r  = std::hypot(dx, dy);
                    if (r >= max_radius) continue;

                    const auto theta = std::atan2(dy, dx);
                    const auto r_bin = std::clamp(
                        static_cast<int>(r / max_radius * kRadialBins), 0, kRadialBins - 1);
                    const auto t_bin = std::clamp<double>(
                        (theta + std::numbers::pi) / (2 * std::numbers::pi) * kThetaBins, 0,
                        kThetaBins - 1);

                    polar_hist[static_cast<int>(r_bin * kThetaBins + t_bin)] +=
                        static_cast<float>(gray.at<std::uint8_t>(y, x));
                }
            }
        }

        // 径向亮度按半径加权汇总成角向曲线，四次谐波给出四臂粗相位。
        auto angular_proj = std::array<double, static_cast<std::size_t>(kThetaBins)> { };
        {
            for (int t = 0; t < kThetaBins; ++t) {
                for (int r = 0; r < kRadialBins; ++r) {
                    angular_proj[t] += polar_hist[r * kThetaBins + t] * static_cast<float>(r + 1);
                }
            }
        }
        auto sum_cos = 0.0;
        auto sum_sin = 0.0;
        for (int t = 0; t < kThetaBins; ++t) {
            const auto angle = 4.0 * static_cast<double>(t) / kThetaBins * 2.0 * std::numbers::pi;
            sum_cos += angular_proj[t] * std::cos(angle);
            sum_sin += angular_proj[t] * std::sin(angle);
        }
        const auto phase = std::atan2(sum_sin, sum_cos) / 4.0;

        constexpr auto kWindow = 2;

        auto peaks   = std::array<double, 4> { };
        auto valleys = std::array<double, 4> { };

        for (std::size_t p = 0; p < 4; ++p) {
            const auto theta_peak   = phase + static_cast<double>(p) * std::numbers::pi * 0.5;
            const auto theta_valley = theta_peak + std::numbers::pi * 0.25;

            const auto pi = std::numbers::pi;
            const auto t_peak =
                ((static_cast<int>(std::llround((theta_peak + pi) / (2.0 * pi) * kThetaBins))
                     % kThetaBins)
                    + kThetaBins)
                % kThetaBins;
            const auto t_valley =
                ((static_cast<int>(std::llround((theta_valley + pi) / (2.0 * pi) * kThetaBins))
                     % kThetaBins)
                    + kThetaBins)
                % kThetaBins;

            double p_sum = 0.0, v_sum = 0.0;
            for (int dt = -kWindow; dt <= kWindow; ++dt) {
                p_sum += angular_proj[(t_peak + dt + kThetaBins) % kThetaBins];
                v_sum += angular_proj[(t_valley + dt + kThetaBins) % kThetaBins];
            }
            peaks[p]   = p_sum / (2 * kWindow + 1);
            valleys[p] = v_sum / (2 * kWindow + 1);
        }

        const auto avg_peak   = (peaks[0] + peaks[1] + peaks[2] + peaks[3]) * 0.25;
        const auto avg_valley = (valleys[0] + valleys[1] + valleys[2] + valleys[3]) * 0.25;

        const auto pvd = (avg_peak - avg_valley) / (avg_peak + avg_valley + 1e-10);

        score = pvd;
        if (pvd < active_threshold) {
            // 圆环没有真实四臂，以下四点只表示估计半径，适配层不会把它们交给 PnP。

            auto radial_total = std::array<double, static_cast<std::size_t>(kRadialBins)> { };
            for (int r = 0; r < kRadialBins; ++r) {
                for (int t = 0; t < kThetaBins; ++t) {
                    radial_total[r] += polar_hist[r * kThetaBins + t];
                }
            }

            const auto outer_radius_bin = static_cast<std::size_t>(
                std::ranges::max_element(radial_total) - radial_total.begin());
            const auto radius =
                max_radius * static_cast<double>(outer_radius_bin + 1) / kRadialBins;

            endpoints = {
                center + cv::Point2f { 0.0f, -static_cast<float>(radius) },
                center + cv::Point2f { +static_cast<float>(radius), 0.0f },
                center + cv::Point2f { 0.0f, +static_cast<float>(radius) },
                center + cv::Point2f { -static_cast<float>(radius), 0.0f },
            };
            return true;
        }

        auto tips = std::array<cv::Point2f, 4> { };
        {
            constexpr auto pi    = std::numbers::pi;
            constexpr auto twopi = 2.0 * pi;

            for (std::size_t p = 0; p < 4; ++p) {
                const auto theta = phase + static_cast<double>(p) * pi * 0.5;
                const auto t_idx =
                    static_cast<int>(std::llround((theta + pi) / twopi * kThetaBins)) % kThetaBins;
                const auto dx = std::cos(theta);
                const auto dy = std::sin(theta);

                auto radial_profile = std::array<double, static_cast<std::size_t>(kRadialBins)> { };
                for (int r = 0; r < kRadialBins; ++r) {
                    for (int dt = -3; dt <= 3; ++dt)
                        radial_profile[r] +=
                            polar_hist[r * kThetaBins + ((t_idx + dt + kThetaBins) % kThetaBins)];
                }

                const auto max_energy = *std::ranges::max_element(radial_profile);
                const auto threshold  = max_energy * 0.25;

                auto boundary = -1;
                for (int r = kRadialBins - 1; r >= 0; --r) {
                    if (radial_profile[r] >= threshold) {
                        boundary = r;
                        break;
                    }
                }

                auto exact_r = 0.0;
                if (boundary >= kRadialBins - 1) {
                    exact_r = 1.0;
                } else if (boundary < 0) {
                    exact_r = static_cast<double>(
                                  std::ranges::max_element(radial_profile) - radial_profile.begin())
                        / kRadialBins;
                } else {
                    const auto e_in  = radial_profile[boundary];
                    const auto e_out = radial_profile[boundary + 1];
                    if (e_in > e_out) {
                        const auto frac = (threshold - e_out) / (e_in - e_out);
                        exact_r         = (static_cast<double>(boundary + 1) - frac) / kRadialBins;
                    } else {
                        exact_r = static_cast<double>(boundary + 1) / kRadialBins;
                    }
                }

                const auto arm_len = max_radius * exact_r;
                tips[p]            = cv::Point2f {
                    center.x + static_cast<float>(arm_len * dx),
                    center.y + static_cast<float>(arm_len * dy),
                };
            }
        }

        {
            auto lengths = std::array<double, 4> { };
            auto mean    = 0.0;
            for (std::size_t i = 0; i < 4; ++i) {
                lengths[i] = std::hypot(tips[i].x - center.x, tips[i].y - center.y);
                mean += lengths[i];
            }
            mean *= 0.25;

            auto variance = 0.0;
            for (const auto l : lengths) {
                const auto diff = l - mean;
                variance += diff * diff;
            }
            variance *= 0.25;
            if (std::sqrt(variance) > mean * kArmLengthStddevMax) return false;
        }

        score     = pvd;
        endpoints = tips;
        return true;
    }

    /**
     * @brief 按物理尺寸、距离和透视范围过滤轮廓，合并同心符板并排除板内伪 R。
     * @param config 相机标定及物理过滤参数。
     * @param mat 目标颜色的单通道二值图。
     * @param icons 接收可能的 R 轮廓，稍后还需骨架评分。
     * @param bullseyes 接收合并后的符板外轮廓，点均为原图坐标。
     */
    auto find_contours(const RuneDetector::Config& config, const cv::Mat& mat,
        std::vector<std::vector<cv::Point>>& icons,
        std::vector<std::vector<cv::Point>>& bullseyes) noexcept {

        icons.clear();
        bullseyes.clear();
        if (mat.empty() || mat.type() != CV_8UC1) return;

        const auto fx = config.intrinsic.at<double>(0, 0);
        const auto fy = config.intrinsic.at<double>(1, 1);
        if (fx <= 0.0 || fy <= 0.0) return;
        const auto focal = (fx + fy) * 0.5;

        const auto perspective_rad = config.max_perspective * std::numbers::pi / 180.0;
        const auto cos_theta       = std::cos(perspective_rad);
        if (cos_theta <= 0.0) return;

        const auto radius_range = [=](double physical_radius, double min_distance,
                                      double max_distance) -> std::array<double, 2> {
            constexpr auto kMargin = 0.1;

            const auto r_min = focal * physical_radius * cos_theta / max_distance * (1.0 - kMargin);
            const auto r_max =
                focal * physical_radius / (min_distance * cos_theta) * (1.0 + kMargin);

            return { r_min, r_max };
        };
        const auto area_range = [=](double physical_radius, double min_distance,
                                    double max_distance,
                                    double fill_ratio) -> std::array<double, 2> {
            constexpr auto kMargin = 0.1;

            const auto a_min = focal * focal * physical_radius * physical_radius * cos_theta
                / (max_distance * max_distance) * (1.0 - kMargin) * std::numbers::pi * fill_ratio;
            const auto a_max = focal * focal * physical_radius * physical_radius
                / (min_distance * min_distance * cos_theta) * (1.0 + kMargin) * std::numbers::pi;

            return { a_min, a_max };
        };

        const auto bullseye_range =
            radius_range(kBullseyeRadius, config.min_distance, config.max_distance);
        const auto icon_area_range =
            area_range(kRuneIconRadius, config.min_distance, config.max_distance, 0.2);

        const auto min_circularity = 2.0 * cos_theta / (1.0 + cos_theta * cos_theta);

        auto contours  = std::vector<std::vector<cv::Point>> { };
        auto hierarchy = std::vector<cv::Vec4i> { };
        cv::findContours(mat.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        struct Ellipse {
            cv::Point center;
            double major;
            double minor;
            std::size_t contour_index;
        };
        auto visited = std::vector<Ellipse> { };

        for (const auto& [i, contour] : contours | std::views::enumerate) {
            if (contour.size() < 5) continue;

            const auto area = cv::contourArea(contour);
            if (area <= 0.0) continue;

            const auto perimeter = cv::arcLength(contour, true);
            if (perimeter <= 0.0) continue;

            const auto radius = std::sqrt(area / std::numbers::pi);

            /*^^*/ if (radius >= bullseye_range[0] && radius <= bullseye_range[1]) {
                constexpr auto kEllipseAreaRatioMin = 0.7;
                constexpr auto kEllipseAreaRatioMax = 1.3;
                constexpr auto kCircularityMargin   = 0.7;

                const auto ellipse = cv::fitEllipse(contour);
                const auto major   = std::max(ellipse.size.width, ellipse.size.height);
                const auto minor   = std::min(ellipse.size.width, ellipse.size.height);
                if (major <= 0.0 || minor <= 0.0) continue;

                const auto aspect_ratio = minor / major;
                if (aspect_ratio < cos_theta) continue;

                const auto ellipse_area = std::numbers::pi * (major * 0.5) * (minor * 0.5);
                const auto area_ratio   = area / ellipse_area;

                if (area_ratio < kEllipseAreaRatioMin || area_ratio > kEllipseAreaRatioMax)
                    continue;

                const auto circularity = 4.0 * std::numbers::pi * area / (perimeter * perimeter);
                if (circularity < min_circularity * kCircularityMargin) continue;

                const auto moment = cv::moments(contour);
                visited.push_back(Ellipse {
                    .center =
                        cv::Point {
                            static_cast<int>(moment.m10 / moment.m00),
                            static_cast<int>(moment.m01 / moment.m00),
                        },
                    .major = static_cast<double>(major),
                    .minor = static_cast<double>(minor),

                    .contour_index = static_cast<std::size_t>(i),
                });
            } else if (area >= icon_area_range[0] && area <= icon_area_range[1]) {
                const auto ellipse = cv::fitEllipse(contour);
                const auto major   = std::max(ellipse.size.width, ellipse.size.height);
                const auto minor   = std::min(ellipse.size.width, ellipse.size.height);
                if (major <= 0.0 || minor <= 0.0) continue;

                const auto aspect_ratio = minor / major;
                if (aspect_ratio < cos_theta) continue;

                icons.push_back(contour);
            }
        }

        {
            constexpr auto kConcentricTolerance = 0.3;

            auto seen = std::vector<bool>(visited.size(), false);

            auto indices = std::views::iota(std::size_t { 0 }, visited.size())
                | std::views::filter([&](std::size_t n) { return !seen[n]; });

            for (auto i : indices) {
                auto group = std::vector<std::size_t> { i };
                seen[i]    = true;

                auto concentric = std::views::iota(i + 1, visited.size())
                    | std::views::filter([&](std::size_t j) {
                          if (seen[j]) return false;
                          const auto& a   = visited[i];
                          const auto& b   = visited[j];
                          const auto dist = std::hypot(static_cast<double>(a.center.x - b.center.x),
                              static_cast<double>(a.center.y - b.center.y));
                          return dist < std::max(a.major, b.major) * 0.5 * kConcentricTolerance;
                      });

                for (auto j : concentric) {
                    group.push_back(j);
                    seen[j] = true;
                }

                if (group.empty()) continue;

                const auto largest =
                    std::ranges::max(group, { }, [&](std::size_t n) { return visited[n].major; });
                bullseyes.push_back(contours[visited[largest].contour_index]);
            }
        }

        auto filtered_icons = std::vector<std::vector<cv::Point>> { };
        std::ranges::copy(icons | std::views::filter([&bullseyes](const auto& icon) {
            const auto moment = cv::moments(icon);
            const auto center = cv::Point2f {
                static_cast<float>(moment.m10 / moment.m00),
                static_cast<float>(moment.m01 / moment.m00),
            };

            return !std::ranges::any_of(bullseyes, [&center](const auto& bull) {
                return cv::pointPolygonTest(bull, center, false) >= 0;
            });
        }),
            std::back_inserter(filtered_icons));

        icons = std::move(filtered_icons);
    }

}

auto RuneDetector::detect(const cv::Mat& mat) const -> Elements {
    if (mat.empty()) return { };

    auto binary = cv::Mat { };
    auto_buff::extract_channel(mat, config.color, binary);

    auto icons     = std::vector<std::vector<cv::Point>> { };
    auto bullseyes = std::vector<std::vector<cv::Point>> { };
    details::find_contours(config, binary, icons, bullseyes);

    auto result = Elements { };

    // 符板只在扩大后的局部区域提取特征，完成后将圆心和端点一起加回 ROI 偏移。
    for (const auto& bull : bullseyes) {
        const auto br          = cv::boundingRect(bull);
        constexpr auto kMargin = 20;
        auto roi               = cv::Rect {
            br.x - kMargin,
            br.y - kMargin,
            br.width + 2 * kMargin,
            br.height + 2 * kMargin,
        };
        roi &= cv::Rect { 0, 0, mat.cols, mat.rows };
        if (roi.width <= 0 || roi.height <= 0) continue;

        const auto moment     = cv::moments(bull);
        const auto center_roi = cv::Point2f {
            static_cast<float>(moment.m10 / moment.m00 - roi.x),
            static_cast<float>(moment.m01 / moment.m00 - roi.y),
        };

        auto masked_roi = cv::Mat { };
        {
            auto contour_local = bull;
            for (auto& p : contour_local) {
                p.x -= roi.x;
                p.y -= roi.y;
            }

            auto mask = cv::Mat { };
            mask      = cv::Mat::zeros(roi.size(), CV_8UC1);
            auto ctns = std::vector<std::vector<cv::Point>> { contour_local };
            cv::drawContours(mask, ctns, -1, cv::Scalar { 255 }, cv::FILLED);
            mat(roi).copyTo(masked_roi, mask);
        }

        auto endpoints = std::array<cv::Point2f, 4> { };
        auto score     = double { };
        if (!details::compute_bullseye_feature(
                masked_roi, center_roi, endpoints, score, config.active_threshold))
            continue;

        for (auto& p : endpoints) {
            p.x += static_cast<float>(roi.x);
            p.y += static_cast<float>(roi.y);
        }

        result.bullseyes.push_back({
            .center =
                cv::Point2d {
                    center_roi.x + static_cast<float>(roi.x),
                    center_roi.y + static_cast<float>(roi.y),
                },
            .corners = { {
                cv::Point2d { endpoints[0] },
                cv::Point2d { endpoints[1] },
                cv::Point2d { endpoints[2] },
                cv::Point2d { endpoints[3] },
            } },
            .active  = score < config.active_threshold,
            .score   = score,
        });
    }
    // R 轮廓独立评分，保留所有通过门限的候选供几何初始化选择。
    for (const auto& icon : icons) {
        constexpr auto kIconMargin = 5;

        auto box  = cv::boundingRect(icon) & cv::Rect { 0, 0, mat.cols, mat.rows };
        auto rect = cv::Rect {
            box.x - kIconMargin,
            box.y - kIconMargin,
            box.width + 2 * kIconMargin,
            box.height + 2 * kIconMargin,
        };
        rect &= cv::Rect { 0, 0, mat.cols, mat.rows };

        auto roi = cv::Mat { };
        cv::cvtColor(mat(rect), roi, cv::COLOR_BGR2GRAY);

        const auto score = details::compute_distance_with_icon(roi);
        if (score < config.match_threshold) continue;

        const auto rc = cv::minAreaRect(icon);
        result.icons.emplace_back(cv::Point2d { rc.center }, score);
    }

    return result;
}

}
