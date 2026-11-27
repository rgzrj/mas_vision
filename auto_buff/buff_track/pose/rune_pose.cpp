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

namespace auto_buff
{

namespace
{
// 单次初始化的临时求解器；候选遍历结束后不保留几何求解状态。
struct PoseSolver
{
    using StateVector = RuneStateVector;
    using Observable  = RuneObservable;
    const RuneConfig &config;
    Observable observable;

    // 一个 R × 种子板组合的空间解与误差；最终选择同时参考其他待击板的布局支持。
    struct InitCandidate
    {
        Eigen::Vector3d center_in_world{Eigen::Vector3d::Zero()};
        double face_yaw_in_world = 0.0;
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
        double icon_error          = std::numeric_limits<double>::max(); // R 标识像素重投影误差平方和
        double seed_center_error   = std::numeric_limits<double>::max(); // 种子板圆心像素重投影误差
        int inactive_inliers       = 0;                                  // 其余待击板落在误差门限内的内点数量
        double inactive_center_sse = std::numeric_limits<double>::max(); // 其余待击板圆心重投影误差平方和(SSE)
    };

    // 用完整五点重新投影，返回误差平方和与最大像素误差，避免只信任 PnP 成功标志。
    auto evaluate_seed_reprojection(const RuneBullseye &board, cv::Point2d icon, const Eigen::Vector3d &xyz_in_camera,
                                    const Eigen::Quaterniond &q_rune2camera) const
        -> std::optional<std::pair<double, double>>
    {
        const auto image_points_opt = SingleRunePnpSolution::image_points(board, icon);
        if (!image_points_opt)
            return std::nullopt;

        const auto rotation_opencv    = rm_utils::util::flu2opencv_rotation(q_rune2camera.toRotationMatrix());
        const auto translation_opencv = rm_utils::util::flu2opencv_position(xyz_in_camera);

        auto camera_points = std::array<cv::Point3f, 5>{};
        for (const auto &[index, point] : RunePagePoints::kPoints | std::views::enumerate)
        {
            const auto p_local_flu = point;
            const auto p_local_ocv = rm_utils::util::flu2opencv_position(p_local_flu);
            const auto p_camera    = rotation_opencv * p_local_ocv + translation_opencv;

            camera_points[static_cast<std::size_t>(index)] = cv::Point3f{
                static_cast<float>(p_camera.x()),
                static_cast<float>(p_camera.y()),
                static_cast<float>(p_camera.z()),
            };
        }

        // 计算五点的重投影误差平方和与最大误差，避免只信任 PnP 成功标志。
        const auto projected_points =
            rm_utils::util::reproject_points(camera_points, observable.camera_matrix, observable.distort_coeffs);
        if (projected_points.size() != image_points_opt->size())
            return std::nullopt;

        auto sse       = 0.0;
        auto max_error = 0.0;
        for (const auto &[projected, detected] : std::views::zip(projected_points, *image_points_opt))
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
    auto evaluate_reduced_layout(const RuneIcon &icon, const RuneBullseye &seed,
                                 std::span<const RuneBullseye> other_inactive_bullseyes,
                                 const Eigen::Vector3d &center_in_world, double face_yaw_in_world,
                                 double rotation_angle) const -> std::optional<InitLayoutScore>
    {
        auto simulated    = observable;
        StateVector state = StateVector::Zero();
        state[kX]         = center_in_world.x();
        state[kY]         = center_in_world.y();
        state[kZ]         = center_in_world.z();
        state[kA]         = rotation_angle;
        state[kPsi]       = face_yaw_in_world;
        simulated.update(state);

        constexpr auto kSeedBlade = 1;
        if (!simulated.visible[Observable::kFeatureIcon])
            return std::nullopt;
        if (!simulated.visible[kSeedBlade])
            return std::nullopt;

        const auto gate2 = config.init_center_gate * config.init_center_gate;

        auto score                  = InitLayoutScore{};
        const auto icon_observation = Eigen::Vector2d{icon.center.x, icon.center.y};
        score.icon_error = (icon_observation - simulated.points_in_image[Observable::kFeatureIcon]).squaredNorm();

        const auto seed_observation = Eigen::Vector2d{seed.center.x, seed.center.y};
        score.seed_center_error     = (seed_observation - simulated.points_in_image[kSeedBlade]).squaredNorm();
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
                best_err2 = std::min(best_err2, (observation - simulated.points_in_image[blade]).squaredNorm());
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
    auto make_init_candidate(const RuneIcon &icon, const RuneBullseye &seed,
                             std::span<const RuneBullseye> other_inactive_bullseyes) const
        -> std::optional<InitCandidate>
    {
        auto pnp = SingleRunePnpSolution{};
        if (!pnp.solve(seed, icon.center, observable.camera_matrix, observable.distort_coeffs))
            return std::nullopt;

        const auto xyz_in_camera = pnp.result.xyz_in_camera;
        const auto q_rune2camera = pnp.result.q_rune2camera.normalized();

        const auto seed_error = evaluate_seed_reprojection(seed, icon.center, xyz_in_camera, q_rune2camera);
        if (!seed_error)
            return std::nullopt;

        const auto seed_mean_error = std::sqrt(seed_error->first / 5.0);
        if (seed_mean_error > config.init_seed_mean_error)
            return std::nullopt;
        if (seed_error->second > config.init_seed_max_error)
            return std::nullopt;

        const auto q_camera2world = Eigen::Quaterniond(observable.T_camera2world.rotation());
        const auto t_camera2world = Eigen::Vector3d(observable.T_camera2world.translation());

        auto candidate         = InitCandidate{};
        const auto R_rune2world = (q_camera2world * q_rune2camera).normalized().toRotationMatrix();
        const auto face_x_odom  = R_rune2world.col(0);
        constexpr auto kYawEps = 1e-6;
        if (face_x_odom.head<2>().squaredNorm() <= kYawEps)
            return std::nullopt;

        const auto pitch = std::asin(std::abs(face_x_odom.z()));
        if (pitch > rm_utils::deg2rad(config.init_pitch_bound))
            return std::nullopt;

        const auto base_yaw      = std::atan2(face_x_odom.y(), face_x_odom.x());
        candidate.center_in_world = q_camera2world * xyz_in_camera + t_camera2world;
        candidate.seed_sse       = seed_error->first;
        candidate.seed_max_error = seed_error->second;

        {
            const auto face_x_camera =
                q_camera2world.conjugate() * Eigen::Vector3d{std::cos(base_yaw), std::sin(base_yaw), 0.0};
            if (face_x_camera.x() <= 0.0)
                return std::nullopt;

            const auto r_yaw_inv    = Eigen::AngleAxisd{-base_yaw, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
            const auto r_seed_local = r_yaw_inv * R_rune2world;
            const auto seed_angle   = std::atan2(-r_seed_local(1, 2), r_seed_local(2, 2));

            const auto layout_score = evaluate_reduced_layout(icon, seed, other_inactive_bullseyes,
                                                              candidate.center_in_world, base_yaw, seed_angle);
            if (!layout_score)
                return std::nullopt;

            candidate.face_yaw_in_world   = base_yaw;
            candidate.rotation_angle      = seed_angle;
            candidate.inactive_inliers    = layout_score->inactive_inliers;
            candidate.icon_error          = layout_score->icon_error;
            candidate.seed_center_error   = layout_score->seed_center_error;
            candidate.inactive_center_sse = layout_score->inactive_center_sse;
        }

        return candidate;
    }

    // 优先保留支持更多待击板的解，再依次比较板心、R 和种子点误差；不按 R 分数一票决定。
    bool better_init_candidate(const InitCandidate &lhs, const InitCandidate &rhs) const
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
    auto solve(std::span<const RuneIcon> icons, std::span<const RuneBullseye> bullseyes) const
        -> std::optional<RunePose>
    {
        if (icons.empty())
            return std::nullopt;

        auto inactive_bullseyes = std::vector<const RuneBullseye *>{};
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

                auto other_inactive_storage = std::array<RuneBullseye, 1>{};
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
                const auto other_inactive = std::span<const RuneBullseye>{
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

        return RunePose{best_candidate->center_in_world, best_candidate->face_yaw_in_world,
                        best_candidate->rotation_angle, best_candidate->icon_pixel, best_candidate->seed_pixel};
    }
};
} // namespace

/**
 * @brief 由相机内参、畸变、相机位姿和符板布局求解符面世界系位姿。
 * @param config 符板配置参数。
 * @param camera_matrix 相机内参矩阵，3x3，CV_64F。
 * @param distort_coeffs 相机畸变系数，1x5，CV_64F。
 * @param T_camera2world 相机在世界系的位姿变换。
 * @param icons 当前帧检测到的 R 标识。
 * @param bullseyes 当前帧检测到的符板圆环。
 * @return 成功时返回符面世界系位姿，否则为空。
 */
auto solve_rune_pose(const RuneConfig &config, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs,
                     const Eigen::Isometry3d &T_camera2world, std::span<const RuneIcon> icons,
                     std::span<const RuneBullseye> bullseyes) -> std::optional<RunePose>
{
    auto solver = PoseSolver{config, {camera_matrix, distort_coeffs, T_camera2world}};
    return solver.solve(icons, bullseyes);
}

} // namespace auto_buff
