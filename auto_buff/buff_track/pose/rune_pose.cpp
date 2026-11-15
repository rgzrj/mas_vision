#include "rune_pose.hpp"
#include "rune_projection.hpp"
#include "rm_utils/algorithm/math_tools.hpp"
#include "rm_utils/algorithm/conversion.hpp"
#include "rm_utils/algorithm/reprojection.hpp"
#include "auto_buff/buff_track/pose/rune_pnp.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>

using namespace rm_utils;
using namespace auto_buff;
using namespace rm_utils::util;

namespace {
// 单次初始化的临时求解器；候选遍历结束后不保留几何求解状态。
struct PoseSolver
{
    using StateVector = RuneStateVector;
    using Observable  = RuneObservable;
    const RuneConfig &config;
    const cv::Mat &intrinsic, &distortion;
    const Eigen::Isometry3d &camera_transform;
    Observable observable;

    // 一个 R × 种子板组合的空间解与误差；最终选择同时参考其他待击板的布局支持。
    struct InitCandidate
    {
        Eigen::Vector3d center_odom{Eigen::Vector3d::Zero()};
        double face_yaw_odom  = 0.0;
        double rotation_angle = 0.0;

        cv::Point2d icon_pixel;
        cv::Point2d seed_pixel;

        int inactive_inliers     = 0;
        double icon_error        = std::numeric_limits<double>::max();
        double seed_center_error = std::numeric_limits<double>::max();

        double seed_sse       = std::numeric_limits<double>::max();
        double seed_max_error = std::numeric_limits<double>::max();

        double inactive_center_sse = std::numeric_limits<double>::max();
    };

    struct InitLayoutScore
    {
        double icon_error          = std::numeric_limits<double>::max();
        double seed_center_error   = std::numeric_limits<double>::max();
        int inactive_inliers       = 0;
        double inactive_center_sse = std::numeric_limits<double>::max();
    };

    /**
     * @brief 根据 R 标位置重排四端点，得到 R 标、上/左/下/右端点的物理点序。
     * @param icon 被选中的 R 标
     * @param bullseye 被选中的种子板
     * @return 退化、重复或无法区分两侧端点时为空；不使用检测框角点补齐。
     */
    static auto make_seed_image_points(const auto_buff::RuneIcon &icon, const auto_buff::RuneBullseye &bullseye)
        -> std::optional<std::array<cv::Point2f, 5>>
    {
        constexpr auto kEps = 1e-6;

        const auto icon_point   = cv::Point2f(icon.center);
        const auto center_point = cv::Point2f(bullseye.center);

        auto corners = std::array<cv::Point2f, 4>{};
        std::ranges::copy(bullseye.corners |
                              std::views::transform([](const cv::Point2d &point) { return cv::Point2f(point); }),
                          corners.begin());

        auto index_b      = std::size_t{0};
        auto index_t      = std::size_t{0};
        auto min_distance = std::numeric_limits<double>::max();
        auto max_distance = -std::numeric_limits<double>::max();
        for (const auto &[index, corner] : corners | std::views::enumerate)
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

        auto remaining       = std::array<std::size_t, 2>{};
        auto remaining_count = std::size_t{0};
        for (std::size_t index = 0; index < corners.size(); ++index)
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

        const auto b = corners[index_b];
        const auto t = corners[index_t];

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

            const auto first  = corners[remaining[0]];
            const auto second = corners[remaining[1]];

            const auto first_projection = static_cast<double>(first.x - center_point.x) * y_axis.x +
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

    // 用完整五点重新投影，返回误差平方和与最大像素误差，避免只信任 PnP 成功标志。
    auto evaluate_seed_reprojection(const auto_buff::RuneIcon &icon, const auto_buff::RuneBullseye &bullseye,
                                    const Eigen::Vector3d &t_camera_rune, const Eigen::Quaterniond &q_camera_rune) const
        -> std::optional<std::pair<double, double>>
    {

        const auto image_points_opt = make_seed_image_points(icon, bullseye);
        if (!image_points_opt)
            return std::nullopt;

        const auto rotation_opencv    = rm_utils::util::ros2opencv_rotation(q_camera_rune.toRotationMatrix());
        const auto translation_opencv = rm_utils::util::ros2opencv_position(t_camera_rune);

        auto camera_points = std::array<cv::Point3f, 5>{};
        for (const auto &[index, point] : auto_buff::RunePagePoints::kPoints | std::views::enumerate)
        {
            const auto p_local_ros                         = point;
            const auto p_local_ocv                         = rm_utils::util::ros2opencv_position(p_local_ros);
            const auto p_camera                            = rotation_opencv * p_local_ocv + translation_opencv;
            camera_points[static_cast<std::size_t>(index)] = cv::Point3f{
                static_cast<float>(p_camera.x()),
                static_cast<float>(p_camera.y()),
                static_cast<float>(p_camera.z()),
            };
        }

        const auto projected_points = reproject_points(camera_points, intrinsic, distortion);
        if (projected_points.size() != image_points_opt->size())
            return std::nullopt;

        auto sse       = 0.0;
        auto max_error = 0.0;
        for (const auto &[projected, detected] :
             std::views::zip(projected_points, *image_points_opt))
        {
            const auto dx   = static_cast<double>(projected.x - detected.x);
            const auto dy   = static_cast<double>(projected.y - detected.y);
            const auto err2 = dx * dx + dy * dy;
            sse += err2;
            max_error = std::max(max_error, std::sqrt(err2));
        }
        return std::pair{sse, max_error};
    }

    // 将解降为当前模型的中心、符面偏航和转角，验证种子板心并统计其他待击板的支持。
    // 后续 EKF 采用相同投影模型，故初始化不能只满足一个任意六自由度 PnP 解。
    auto evaluate_reduced_layout(const auto_buff::RuneIcon &icon, const auto_buff::RuneBullseye &seed,
                                 std::span<const auto_buff::RuneBullseye> other_inactive_bullseyes,
                                 const Eigen::Vector3d &center_odom, double face_yaw_odom, double rotation_angle) const
        -> std::optional<InitLayoutScore>
    {

        auto simulated    = observable;
        StateVector state = StateVector::Zero();
        state[kX]         = center_odom.x();
        state[kY]         = center_odom.y();
        state[kZ]         = center_odom.z();
        state[kA]         = rotation_angle;
        state[kPsi]       = face_yaw_odom;
        simulated.update(state);

        constexpr auto kSeedBlade = 1;
        if (!simulated.visible[Observable::kFeatureIcon])
            return std::nullopt;
        if (!simulated.visible[kSeedBlade])
            return std::nullopt;

        const auto gate2 = config.init_center_gate * config.init_center_gate;

        auto score                  = InitLayoutScore{};
        const auto icon_observation = Eigen::Vector2d{icon.center.x, icon.center.y};
        score.icon_error            = (icon_observation - simulated.projected[Observable::kFeatureIcon]).squaredNorm();

        const auto seed_observation = Eigen::Vector2d{seed.center.x, seed.center.y};
        score.seed_center_error     = (seed_observation - simulated.projected[kSeedBlade]).squaredNorm();
        if (score.seed_center_error > gate2)
            return std::nullopt;

        if (other_inactive_bullseyes.empty())
        {
            score.inactive_inliers    = 0;
            score.inactive_center_sse = 0.0;
            return score;
        }

        constexpr auto kBladeBegin = 2;
        constexpr auto kBladeEnd   = Observable::kFeatureCount;

        auto total_sse = 0.0;
        auto inliers   = 0;
        for (const auto &bullseye : other_inactive_bullseyes)
        {
            const auto observation = Eigen::Vector2d{bullseye.center.x, bullseye.center.y};
            auto best_err2         = std::numeric_limits<double>::max();
            for (int blade = kBladeBegin; blade < kBladeEnd; ++blade)
            {
                if (!simulated.visible[blade])
                    continue;
                best_err2 = std::min(best_err2, (observation - simulated.projected[blade]).squaredNorm());
            }
            if (best_err2 > gate2)
                continue;

            inliers += 1;
            total_sse += best_err2;
        }

        score.inactive_inliers    = inliers;
        score.inactive_center_sse = inliers > 0 ? total_sse : std::numeric_limits<double>::max();
        return score;
    }

    // 先求相机系 PnP，再转世界系；依次检查五点误差、符面俯仰、法线朝向和整体布局。
    auto make_init_candidate(const auto_buff::RuneIcon &icon, const auto_buff::RuneBullseye &seed,
                             std::span<const auto_buff::RuneBullseye> other_inactive_bullseyes) const
        -> std::optional<InitCandidate>
    {

        auto pnp          = SingleRunePnpSolution{};
        pnp.input.intrinsic = intrinsic;
        pnp.input.distortion = distortion;
        pnp.input.center  = seed.center;
        pnp.input.icon    = icon.center;
        pnp.input.corners = seed.corners;
        if (!pnp.solve())
            return std::nullopt;

        const auto t_camera_rune = pnp.result.translation;
        const auto q_camera_rune = pnp.result.orientation.normalized();

        const auto seed_error = evaluate_seed_reprojection(icon, seed, t_camera_rune, q_camera_rune);
        if (!seed_error)
            return std::nullopt;

        const auto seed_mean_error = std::sqrt(seed_error->first / 5.0);
        if (seed_mean_error > config.init_seed_mean_error)
            return std::nullopt;
        if (seed_error->second > config.init_seed_max_error)
            return std::nullopt;

        const auto q_odom_camera = Eigen::Quaterniond(camera_transform.rotation());
        const auto t_odom_camera = Eigen::Vector3d(camera_transform.translation());

        auto candidate         = InitCandidate{};
        const auto r_page_odom = (q_odom_camera * q_camera_rune).normalized().toRotationMatrix();
        const auto face_x_odom = r_page_odom.col(0);
        constexpr auto kYawEps = 1e-6;
        if (face_x_odom.head<2>().squaredNorm() <= kYawEps)
            return std::nullopt;

        const auto pitch = std::asin(std::abs(face_x_odom.z()));
        if (pitch > rm_utils::deg2rad(config.init_pitch_bound))
            return std::nullopt;

        const auto base_yaw      = std::atan2(face_x_odom.y(), face_x_odom.x());
        candidate.center_odom    = q_odom_camera * t_camera_rune + t_odom_camera;
        candidate.seed_sse       = seed_error->first;
        candidate.seed_max_error = seed_error->second;

        {
            const auto face_x_camera =
                q_odom_camera.conjugate() * Eigen::Vector3d{std::cos(base_yaw), std::sin(base_yaw), 0.0};
            if (face_x_camera.x() <= 0.0)
                return std::nullopt;

            const auto r_yaw_inv    = Eigen::AngleAxisd{-base_yaw, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
            const auto r_seed_local = r_yaw_inv * r_page_odom;
            const auto seed_angle   = std::atan2(-r_seed_local(1, 2), r_seed_local(2, 2));

            const auto layout_score = evaluate_reduced_layout(icon, seed, other_inactive_bullseyes,
                                                              candidate.center_odom, base_yaw, seed_angle);
            if (!layout_score)
                return std::nullopt;

            candidate.face_yaw_odom       = base_yaw;
            candidate.rotation_angle      = seed_angle;
            candidate.inactive_inliers    = layout_score->inactive_inliers;
            candidate.icon_error          = layout_score->icon_error;
            candidate.seed_center_error   = layout_score->seed_center_error;
            candidate.inactive_center_sse = layout_score->inactive_center_sse;
        }

        return candidate;
    }

    // 优先保留支持更多待击板的解，再依次比较板心、R 和种子点误差；不按 R 分数一票决定。
    static auto better_init_candidate(const InitCandidate &lhs, const InitCandidate &rhs)
    {
        if (lhs.inactive_inliers != rhs.inactive_inliers)
        {
            return lhs.inactive_inliers > rhs.inactive_inliers;
        }
        if (lhs.seed_center_error != rhs.seed_center_error)
        {
            return lhs.seed_center_error < rhs.seed_center_error;
        }
        if (lhs.icon_error != rhs.icon_error)
        {
            return lhs.icon_error < rhs.icon_error;
        }
        if (lhs.seed_max_error != rhs.seed_max_error)
        {
            return lhs.seed_max_error < rhs.seed_max_error;
        }
        if (lhs.seed_sse != rhs.seed_sse)
        {
            return lhs.seed_sse < rhs.seed_sse;
        }
        return lhs.inactive_center_sse < rhs.inactive_center_sse;
    }

    // 遍历所有 R 和最多两个待击板种子。打击后圆环可用于后续校正，不作为初始 PnP 种子。
    auto solve(std::span<const auto_buff::RuneIcon> icons, std::span<const auto_buff::RuneBullseye> bullseyes) const
        -> std::optional<RunePose>
    {
        if (icons.empty())
            return std::nullopt;

        auto inactive_bullseyes = std::vector<const auto_buff::RuneBullseye *>{};
        for (const auto &bullseye : bullseyes)
        {
            if (!bullseye.active)
                inactive_bullseyes.push_back(&bullseye);
        }

        if (inactive_bullseyes.empty())
            return std::nullopt;
        if (inactive_bullseyes.size() > 2)
            return std::nullopt;

        auto best_candidate = std::optional<InitCandidate>{};
        for (const auto &icon : icons)
        {
            for (const auto *seed : inactive_bullseyes)
            {
                if (!seed)
                    continue;

                auto other_inactive_storage = std::array<auto_buff::RuneBullseye, 1>{};
                auto other_inactive_count   = std::size_t{0};
                for (const auto *item : inactive_bullseyes)
                {
                    if (item == seed)
                        continue;
                    if (other_inactive_count >= other_inactive_storage.size())
                        break;

                    other_inactive_storage[other_inactive_count] = *item;
                    other_inactive_count++;
                }
                const auto other_inactive = std::span<const auto_buff::RuneBullseye>{
                    other_inactive_storage.data(),
                    other_inactive_count,
                };

                auto candidate = make_init_candidate(icon, *seed, other_inactive);
                if (!candidate)
                    continue;
                if (!best_candidate || better_init_candidate(*candidate, *best_candidate))
                {
                    candidate->icon_pixel = icon.center;
                    candidate->seed_pixel = seed->center;
                    best_candidate        = *candidate;
                }
            }
        }

        if (!best_candidate)
            return std::nullopt;

        return RunePose{best_candidate->center_odom, best_candidate->face_yaw_odom, best_candidate->rotation_angle,
                        best_candidate->icon_pixel, best_candidate->seed_pixel};
    }
};
} // namespace

auto auto_buff::solve_rune_pose(const RuneConfig &config, const cv::Mat &intrinsic, const cv::Mat &distortion, const Eigen::Isometry3d &camera_transform, std::span<const auto_buff::RuneIcon> icons,
                           std::span<const auto_buff::RuneBullseye> bullseyes) -> std::optional<RunePose>
{
    auto solver = PoseSolver{config, intrinsic, distortion, camera_transform, {}};
    solver.observable.intrinsic = intrinsic;
    solver.observable.distortion = distortion;
    solver.observable.camera_to_world = camera_transform;
    return solver.solve(icons, bullseyes);
}
