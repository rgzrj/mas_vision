#include "rune_model.hpp"

#include "mas_log.hpp"
#include "../pose/rune_pose.hpp"
#include "../pose/rune_projection.hpp"
#include "../predict/rune_predictor.hpp"
#include "rm_utils/clock.hpp"
#include "rm_utils/algorithm/math_tools.hpp"
#include <opencv2/core.hpp>
#include "rm_utils/algorithm/conversion.hpp"
#include "rm_utils/algorithm/hungarian.hpp"
#include "rm_utils/algorithm/kalman.hpp"
#include "rm_utils/algorithm/mahalanobis.hpp"

#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <ranges>

using namespace rm_utils;
using namespace auto_buff;
using namespace rm_utils::util;

// 模型内部状态顺序为 [世界 x,y,z,角速度,转角,符面偏航]。
// pose 只负责初始几何；此处独占 EKF、板位有效期和用于导出的拟合结果。

struct RuneModel::Impl
{
    using StateVector      = RuneStateVector;
    using Observable       = RuneObservable;
    using Covariance       = Eigen::Matrix<double, 6, 6>;
    using ProcessNoise     = Eigen::Matrix<double, 6, 6>;
    using ObservationNoise = Eigen::Matrix<double, 2, 2>;
    using KalmanGain       = Eigen::Matrix<double, 6, 2>;
    using ObsJacobian      = Eigen::Matrix<double, 2, 6>;

    static constexpr auto kInactiveTimeout = std::chrono::milliseconds{100}; // 待击板观测有效期。

    // EKF 当前后验和运动拟合分开保存，拟合不反写滤波状态向量。
    struct Context
    {
        StateVector posteriors_state     = StateVector::Zero();
        Covariance posteriors_covariance = Covariance::Identity();
        ProcessNoise noise_process       = ProcessNoise::Zero();

        RunePrediction prediction;

        auto set_noise(const RuneConfig &config) noexcept
        {
            noise_process.diagonal() << config.noise_x, config.noise_y, config.noise_z, config.noise_rotation_speed,
                config.noise_rotation_angle, config.noise_face_yaw;
        }

        // 沿用 RMCS 初始不确定度，使位置、角速度、转角和符面朝向可分别收敛。
        auto reset_covariance() noexcept
        {
            auto diag = StateVector{};
            diag << 64.0, 64.0, 64.0, 100.0, 25.0, 10.0;
            posteriors_covariance = diag.asDiagonal();
        }

        // 导出值副本给预测和火控；角速度可用拟合值，位置和当前转角仍取 EKF 后验。
        auto get_state(const std::array<bool, 5> &inactive, rm_utils::Timestamp start_timestamp) const noexcept
        {
            return RuneState{
                .x               = posteriors_state[kX],
                .y               = posteriors_state[kY],
                .z               = posteriors_state[kZ],
                .start_timestamp = start_timestamp,
                .rotation_speed =
                    prediction.sine_valid
                        ? prediction.sine_v + prediction.sine_a * std::sin(prediction.sine_phase)
                        : (prediction.use_prediction_speed ? prediction.prediction_speed : posteriors_state[kW]),
                .rotation_angle       = posteriors_state[kA],
                .face_yaw             = posteriors_state[kPsi],
                .inactive             = inactive,
                .use_prediction_speed = prediction.use_prediction_speed,
                .prediction_cost      = prediction.sine_valid ? prediction.sine_cost : prediction.prediction_cost,
                .sine_C               = prediction.sine_C,
                .sine_v               = prediction.sine_v,
                .sine_a               = prediction.sine_a,
                .sine_omega           = prediction.sine_omega,
                .sine_phase           = prediction.sine_phase,
                .sine_t               = prediction.sine_t,
                .sine_valid           = prediction.sine_valid,
                .update_count         = prediction.update_count,
            };
        }
    };

    RuneConfig config{};
    Observable observable;

    Context context;
    std::array<bool, 5> blade_inactive{};                        // true 表示该板位仍可击打，沿用原版 inactive 命名。
    std::array<rm_utils::Timestamp, 5> blade_inactive_timeout{}; // 最近待击板校正延长该板位有效期。
    RuneAddition addition{};
    rm_utils::Timestamp init_timestamp{};
    rm_utils::Timestamp current_stamp;
    RunePredictor predictor;

    explicit Impl(const RuneConfig &cfg) noexcept
    {
        config = cfg;
        context.set_noise(config);
        context.reset_covariance();
    }

    // 初始化求解和跟踪投影共享同一份内参与畸变，避免两条几何路径不一致。
    auto configure_camera(const cv::Mat &matrix, const cv::Mat &coeff) noexcept -> void
    {
        observable.intrinsic  = matrix;
        observable.distortion = coeff;
    }

    auto update_transform(const Eigen::Isometry3d &t) noexcept -> void
    {
        observable.camera_to_world = t;
    }

    // EKF 预测只用恒角速度推进转角；正弦/恒速拟合用于导出角速度及未来状态。

    static auto predict_state(double dt, const StateVector &last) -> StateVector
    {
        auto next  = last;
        next[kA]   = last[kA] + last[kW] * dt;
        next[kPsi] = rm_utils::limit_rad(last[kPsi]);
        return next;
    }

    // 转角对角速度的导数为 dt；其余状态在本轮保持恒定，再叠加过程噪声。
    auto predict_covariance(double dt, const Covariance &covariance) const -> Covariance
    {
        auto jacobian    = Covariance{Covariance::Identity()};
        jacobian(kA, kW) = dt;
        return rm_utils::util::predict_covariance(covariance, jacobian, context.noise_process);
    }

    // 观测雅可比：针孔像素投影 × 轴约定转换 × 世界到相机旋转 × 局部特征对状态的导数。
    // 雅可比沿用上游针孔近似，实际像素投影仍通过公共投影函数应用畸变。

    auto make_observation_jacobian(int feature_id) const -> ObsJacobian
    {
        const auto &state = context.posteriors_state;
        if (state.hasNaN())
            return ObsJacobian::Zero();

        const auto theta           = state[kA];
        const auto psi             = state[kPsi];
        const auto r_cam_from_odom = observable.camera_to_world.rotation().transpose().eval();

        const auto &cam_pt   = observable.in_camera[feature_id];
        const auto point_ocv = rm_utils::util::ros2opencv_position(cam_pt);
        const auto depth     = point_ocv.z();
        constexpr auto kMinZ = 0.1;
        if (depth < kMinZ)
            return ObsJacobian::Zero();

        Eigen::Matrix<double, 2, 3> proj_jac;
        // clang-format off
        proj_jac <<
            observable.intrinsic.at<double>(0, 0) / depth,                        0, -observable.intrinsic.at<double>(0, 0) * point_ocv.x() / (depth * depth),
                                                  0, observable.intrinsic.at<double>(1, 1) / depth, -observable.intrinsic.at<double>(1, 1) * point_ocv.y() / (depth * depth);
        // clang-format on

        Eigen::Matrix3d cam_ros_jac;
        // clang-format off
        cam_ros_jac <<  0, -1,  0,
                        0,  0, -1,
                        1,  0,  0;
        // clang-format on

        Eigen::Matrix<double, 3, 6> pos_jac;
        pos_jac.setZero();
        pos_jac(0, kX) = 1.0;
        pos_jac(1, kY) = 1.0;
        pos_jac(2, kZ) = 1.0;

        const auto r_face = Eigen::AngleAxisd{psi, Eigen::Vector3d::UnitZ()}.toRotationMatrix();

        Eigen::Matrix3d d_r_face_d_psi;
        // clang-format off
        d_r_face_d_psi <<
            -std::sin(psi), -std::cos(psi), 0,
             std::cos(psi), -std::sin(psi), 0,
                        0,              0, 0;
        // clang-format on

        Eigen::Vector3d local_point = Eigen::Vector3d::Zero();

        if (feature_id == Observable::kFeatureIcon)
        {
            local_point = Eigen::Vector3d(-auto_buff::kRuneIconProminentDistance, 0, 0);
        }
        else
        {
            static constexpr std::array kBladeDeg = {0.0, 72.0, 144.0, 216.0, 288.0};
            const auto alpha                      = theta + rm_utils::deg2rad(kBladeDeg[feature_id - 1]);
            local_point                           = Eigen::Vector3d(0, -auto_buff::kRuneGlobalRadius * std::sin(alpha),
                                                                    auto_buff::kRuneGlobalRadius * std::cos(alpha));
            Eigen::Vector3d d_local_d_theta(0, -auto_buff::kRuneGlobalRadius * std::cos(alpha),
                                            -auto_buff::kRuneGlobalRadius * std::sin(alpha));
            pos_jac.col(kA) = r_face * d_local_d_theta;
        }
        pos_jac.col(kPsi) = d_r_face_d_psi * local_point;

        ObsJacobian J;
        J.noalias() = proj_jac * cam_ros_jac * r_cam_from_odom * pos_jac;
        return J;
    }

    // 单个二维像素观测的 EKF 校正；只有数值有效时才提交新状态和协方差。

    auto update(const Eigen::Vector2d &observed, int feature_id) noexcept -> bool
    {
        const auto prior_state      = context.posteriors_state;
        const auto prior_covariance = context.posteriors_covariance;
        const auto pred             = observable.projected[feature_id];

        auto innovation = Eigen::Vector2d{observed - pred};
        if (innovation.hasNaN())
            return false;

        auto jacobian = make_observation_jacobian(feature_id);
        if (jacobian.hasNaN())
            return false;

        ObservationNoise noise_obs = ObservationNoise::Identity() * config.noise_observation;

        auto innovation_covariance = jacobian * prior_covariance * jacobian.transpose() + noise_obs;
        auto innovation_cov_inv    = innovation_covariance.inverse();
        if (innovation_cov_inv.hasNaN())
            return false;

        KalmanGain kalman_gain = prior_covariance * jacobian.transpose() * innovation_cov_inv;

        auto posterior_state = StateVector{prior_state};
        posterior_state.noalias() += kalman_gain * innovation;

        if (posterior_state.hasNaN() || !posterior_state.allFinite())
            return false;
        posterior_state[kPsi] = rm_utils::limit_rad(posterior_state[kPsi]);

        const auto posterior_covariance =
            rm_utils::util::correct_covariance(prior_covariance, jacobian, kalman_gain, noise_obs);

        context.posteriors_state      = posterior_state;
        context.posteriors_covariance = posterior_covariance;

        observable.update(posterior_state);
        return true;
    }

    // 初始化：PnP 给出中心、符面偏航和相位；初始角速度置零，再用 R/种子中心校正。

    auto init(std::span<const auto_buff::RuneIcon> icons, std::span<const auto_buff::RuneBullseye> bullseyes,
              rm_utils::Timestamp timestamp) noexcept -> bool
    {
        // pose 只返回几何解；滤波状态和协方差仍由 track 初始化并持有。
        const auto best_candidate = solve_rune_pose(config, observable.intrinsic, observable.distortion,
                                                    observable.camera_to_world, icons, bullseyes);
        if (!best_candidate)
            return false;

        context.posteriors_state       = StateVector::Zero();
        context.posteriors_state[kX]   = best_candidate->center_odom.x();
        context.posteriors_state[kY]   = best_candidate->center_odom.y();
        context.posteriors_state[kZ]   = best_candidate->center_odom.z();
        context.posteriors_state[kW]   = 0.0;
        context.posteriors_state[kA]   = best_candidate->rotation_angle;
        context.posteriors_state[kPsi] = best_candidate->face_yaw_odom;

        context.reset_covariance();
        init_timestamp     = timestamp;
        context.prediction = RunePrediction{};
        predictor.reset();

        blade_inactive.fill(false);
        blade_inactive_timeout.fill(rm_utils::Timestamp{});

        observable.update(context.posteriors_state);

        update(Eigen::Vector2d{best_candidate->icon_pixel.x, best_candidate->icon_pixel.y}, 0);
        update(Eigen::Vector2d{best_candidate->seed_pixel.x, best_candidate->seed_pixel.y}, 1);

        addition.tracked.clear();
        addition.predicted.clear();
        for (int feature_id = 0; feature_id < Observable::kFeatureCount; ++feature_id)
        {
            if (!observable.visible[feature_id])
                continue;
            addition.predicted.push_back({
                feature_id,
                cv::Point2d{observable.projected[feature_id].x(), observable.projected[feature_id].y()},
            });
        }

        return true;
    }

    // 正常帧先 predict，再 correct；即使没有观测也要推进时间、协方差和正弦相位。

    auto predict(double dt, rm_utils::Timestamp stamp) noexcept -> void
    {
        current_stamp = stamp;

        const auto prior_state      = predict_state(dt, context.posteriors_state);
        const auto prior_covariance = predict_covariance(dt, context.posteriors_covariance);

        context.posteriors_state      = prior_state;
        context.posteriors_covariance = prior_covariance;

        if (context.prediction.sine_valid)
        {
            context.prediction.sine_t += dt;
            context.prediction.sine_phase += context.prediction.sine_omega * dt;
        }

        observable.update(prior_state);
    }

    // 校正：更新投影与板位时限 → 类型约束关联 → 顺序 EKF 更新 → 用真实校正样本拟合。

    auto correct(std::span<const auto_buff::RuneIcon> icons,
                 std::span<const auto_buff::RuneBullseye> bullseyes) noexcept -> bool
    {

        observable.update(context.posteriors_state);
        for (auto &&[inactive, timeout] : std::views::zip(blade_inactive, blade_inactive_timeout))
        {
            if (current_stamp >= timeout)
                inactive = false;
        }

        addition.predicted.clear();
        for (int feature_id = 0; feature_id < Observable::kFeatureCount; ++feature_id)
        {
            if (!observable.visible[feature_id])
                continue;
            addition.predicted.push_back({
                feature_id,
                cv::Point2d{observable.projected[feature_id].x(), observable.projected[feature_id].y()},
            });
        }

        // R 只能匹配特征 0；其余观测只能匹配板位 1～5。颜色已在适配层统一。
        struct Obs
        {
            Eigen::Vector2d pixel;
            bool is_icon;
            bool is_inactive;
        };

        auto observations = std::vector<Obs>{};
        for (const auto &ic : icons)
            observations.push_back({Eigen::Vector2d(ic.center.x, ic.center.y), true, false});
        for (const auto &bs : bullseyes)
            observations.push_back({Eigen::Vector2d(bs.center.x, bs.center.y), false, !bs.active});

        if (observations.empty())
        {
            // 空观测只刷新投影和状态有效期，不校正、不向拟合器添加预测点。
            addition.tracked.clear();
            return false;
        }
        const auto N = static_cast<int>(observations.size());
        const auto M = Observable::kFeatureCount;

        struct PredInfo
        {
            ObsJacobian H;
            ObservationNoise S;
        };

        auto pred_infos = std::array<PredInfo, M>{};

        ObservationNoise R = ObservationNoise::Identity() * config.noise_observation;
        const auto &P      = context.posteriors_covariance;

        for (int j = 0; j < M; ++j)
        {
            pred_infos[j].H = make_observation_jacobian(j);
            pred_infos[j].S = pred_infos[j].H * P * pred_infos[j].H.transpose() + R;
        }

        // 用平方马氏距离构造代价矩阵；不可见点和类型不符的组合保持无穷大。
        // 匈牙利算法的虚拟匹配负责拒绝超出门限或没有对应物的观测。
        constexpr auto kInf = std::numeric_limits<double>::max() / 4;
        auto cost           = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>(N, M);
        cost.setConstant(kInf);

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < M; ++j)
            {
                if (!observable.visible[j])
                    continue;

                bool mismatch =
                    observations[i].is_icon ? (j != Observable::kFeatureIcon) : (j == Observable::kFeatureIcon);
                if (mismatch)
                    continue;

                auto innovation = observations[i].pixel - observable.projected[j];
                auto d2         = rm_utils::util::mahalanobis_distance(innovation, pred_infos[j].S);
                if (!d2)
                    continue;
                cost(i, j) = *d2;
            }
        }

        auto assignments = rm_utils::util::hungarian_assign(cost, config.gate_threshold);

        addition.tracked.clear();
        auto inactive_corrected = 0, active_corrected = 0;
        for (int i = 0; i < N; ++i)
        {
            if (!assignments[i])
                continue;
            auto j = *assignments[i];
            if (j < 0 || j >= M)
                continue;

            // 前一个观测可能已经改变状态与协方差，此处按最新结果检查创新的数值有效性。
            auto innovation = observations[i].pixel - observable.projected[j];
            auto d2         = rm_utils::util::mahalanobis_distance(
                innovation, pred_infos[j].H * context.posteriors_covariance * pred_infos[j].H.transpose() + R);
            if (!d2)
                continue;

            if (!update(observations[i].pixel, j))
                continue;
            if (!observations[i].is_icon && j != Observable::kFeatureIcon)
            {
                const auto blade = static_cast<std::size_t>(j - 1);
                if (observations[i].is_inactive)
                {
                    blade_inactive[blade] = true;
                    inactive_corrected++;
                    blade_inactive_timeout[blade] = current_stamp + kInactiveTimeout;
                }
                else
                {
                    active_corrected++;
                }
            }
            addition.tracked.push_back({j, cv::Point2d{observations[i].pixel.x(), observations[i].pixel.y()}});

            // 刷新雅可比供下一个已分配观测使用；S 只在初始代价矩阵中读取，无需重复计算。
            for (int jj = 0; jj < M; ++jj)
            {
                pred_infos[jj].H = make_observation_jacobian(jj);
            }
        }

        const auto elapsed = std::chrono::duration<double>(current_stamp - init_timestamp).count();
        predictor.correct(inactive_corrected, active_corrected, current_stamp, elapsed, context.posteriors_state[kA],
                          context.prediction);

        return inactive_corrected > 0;
    }

    // 发散检查仅依据当前后验；预热和能否击打由 RuneState::get_aimpoints 判断。

    auto diverged() const -> bool
    {
        const auto &state = context.posteriors_state;
        const auto &cov   = context.posteriors_covariance;

        // 符面法线（约定：指向远离车的方向）与 odom原点->符心 连线的夹角，应远小于 45 度
        auto face_angle = std::numeric_limits<double>::quiet_NaN();
        if (const auto r_xy = std::hypot(state[kX], state[kY]); r_xy > 0.5)
        {
            const auto to_center = std::atan2(state[kY], state[kX]);
            face_angle           = std::abs(rm_utils::limit_rad(state[kPsi] - to_center));
        }

        auto checks = std::array{
            cov(kX, kX) > 150.0,
            cov(kY, kY) > 150.0,
            std::abs(state[kX]) > 15.0,
            std::abs(state[kY]) > 15.0,
            std::abs(state[kZ]) > 5.0,
            std::abs(state[kW]) > 10.0 * std::numbers::pi,
            face_angle > rm_utils::deg2rad(config.diverge_face_angle),
            state.hasNaN() == true,
            state.allFinite() == false,
        };
        if (std::ranges::any_of(checks, std::identity{}))
        {
            // 限频跨越模型重建，避免不稳定观测导致每帧输出整组状态。
            static thread_local std::chrono::steady_clock::time_point log_time{};
            const auto now = std::chrono::steady_clock::now();
            if (log_time == std::chrono::steady_clock::time_point{} || now - log_time >= std::chrono::seconds(5))
            {
                constexpr std::array reasons{"cov_x", "cov_y",      "range_x",   "range_y",        "range_z",
                                             "speed", "face_angle", "state_nan", "state_nonfinite"};
                const auto failed = std::ranges::find(checks, true) - checks.begin();
                MAS_LOG_WARN("Rune diverged: reason={}, state=[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}], "
                             "cov_diag=[{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}], face_angle={:.4f}, update_count={}",
                             reasons[failed], state[kX], state[kY], state[kZ], state[kW], state[kA], state[kPsi],
                             cov(kX, kX), cov(kY, kY), cov(kZ, kZ), cov(kW, kW), cov(kA, kA), cov(kPsi, kPsi),
                             face_angle, context.prediction.update_count);
                log_time = now;
            }
            return true;
        }
        return false;
    }
};

// 对外接口只转发到模型实现，保持 EKF 与关联缓存的唯一所有权。

RuneModel::RuneModel(const RuneConfig &cfg) noexcept : pimpl{std::make_unique<Impl>(cfg)} {}

RuneModel::~RuneModel() noexcept = default;

auto RuneModel::update_camera(const cv::Mat &matrix, const cv::Mat &coeff) noexcept -> void
{
    pimpl->configure_camera(matrix, coeff);
}

auto RuneModel::update_transform(const Eigen::Isometry3d &t) noexcept -> void
{
    pimpl->update_transform(t);
}

auto RuneModel::init(std::span<const auto_buff::RuneIcon> icons, std::span<const auto_buff::RuneBullseye> bullseyes,
                     rm_utils::Timestamp timestamp) noexcept -> bool
{
    return pimpl->init(icons, bullseyes, timestamp);
}

auto RuneModel::predict(double dt, rm_utils::Timestamp now) noexcept -> void
{
    pimpl->predict(dt, now);
}

auto RuneModel::correct(std::span<const auto_buff::RuneIcon> icons,
                        std::span<const auto_buff::RuneBullseye> bullseyes) noexcept -> bool
{
    return pimpl->correct(icons, bullseyes);
}

auto RuneModel::diverged() const -> bool
{
    return pimpl->diverged();
}

auto RuneModel::state() const noexcept -> RuneState
{
    return pimpl->context.get_state(pimpl->blade_inactive, pimpl->init_timestamp);
}

auto RuneModel::addition() const -> const RuneAddition &
{
    return pimpl->addition;
}

auto RuneModel::ekf_snapshot() const noexcept -> RuneEkfSnapshot
{
    RuneEkfSnapshot result;
    for (int i = 0; i < 6; ++i)
    {
        result.state[i]    = pimpl->context.posteriors_state[i];
        result.variance[i] = pimpl->context.posteriors_covariance(i, i);
    }
    return result;
}
