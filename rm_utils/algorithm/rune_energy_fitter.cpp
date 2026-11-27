#include "rune_energy_fitter.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rm_utils {

namespace {

    auto sample_weight(double t, double reference_t) -> double {
        return std::exp2((t - reference_t) / RuneEnergyFitter::kWeightHalfLifeSeconds);
    }

} // namespace

RuneEnergyFitter::~RuneEnergyFitter() = default;

void RuneEnergyFitter::push(double t, double theta) {
    buffer_.push_back({ t, theta });

    while (!buffer_.empty() && buffer_.back().t - buffer_.front().t > kWindowSeconds)
        buffer_.pop_front();
}

void RuneEnergyFitter::reset() { buffer_.clear(); }

template <typename Pred>
auto RuneEnergyFitter::compute_weighted_cost(const std::deque<Point>& buffer, Pred&& pred_fn)
    -> double {

    const auto reference_t = buffer.back().t;
    double weighted_sum    = 0.0;
    double weight_sum      = 0.0;
    for (const auto& pt : buffer) {
        const auto weight = sample_weight(pt.t, reference_t);
        const auto diff   = pt.theta - pred_fn(pt.t);
        weighted_sum += weight * diff * diff;
        weight_sum += weight;
    }
    return weighted_sum / weight_sum;
}

auto RuneEnergyFitter::fit_linear() const -> std::optional<LinearResult> {
    if (buffer_.size() < 2) return std::nullopt;
    if (buffer_.back().t - buffer_.front().t < kMinFitSeconds) return std::nullopt;

    const auto reference_t = buffer_.back().t;

    double sum_weight = 0.0, sum_t = 0.0, sum_theta = 0.0, sum_tt = 0.0, sum_t_theta = 0.0;
    for (const auto& pt : buffer_) {
        const auto weight = sample_weight(pt.t, reference_t);
        sum_weight += weight;
        sum_t += weight * pt.t;
        sum_theta += weight * pt.theta;
        sum_tt += weight * pt.t * pt.t;
        sum_t_theta += weight * pt.t * pt.theta;
    }

    double denom = sum_weight * sum_tt - sum_t * sum_t;
    if (std::abs(denom) < 1e-12) return std::nullopt;

    double speed = (sum_weight * sum_t_theta - sum_t * sum_theta) / denom;
    double C     = (sum_theta - speed * sum_t) / sum_weight;

    auto pred   = [&](double dt) { return C + speed * dt; };
    double cost = compute_weighted_cost(buffer_, pred);

    return LinearResult { C, speed, cost };
}

auto RuneEnergyFitter::fit_sine() const -> std::optional<FitResult> {
    // 五个待估参数至少需要足够多的观测；时间跨度继续沿用 RMCS 的预热要求。
    if (buffer_.size() < 6 || buffer_.back().t - buffer_.front().t < kMinFitSeconds)
        return std::nullopt;

    // 参考 SZURPVision/RP-26Rune 的可分离 LM–IRLS；来源和许可证见 rm_utils/SOURCE_SZU.json。
    // 拟合 theta = A*cos(omega*tau) + B*sin(omega*tau) + v*tau + C。
    // 时间、转角均去中心化，避免运行时间很长后 t 与常数项的数值耦合。
    const auto count = static_cast<Eigen::Index>(buffer_.size());
    const double reference_t = buffer_[buffer_.size() / 2].t;
    const double reference_angle = buffer_[buffer_.size() / 2].theta;
    Eigen::VectorXd time(count), angle(count), time_weights(count);
    for (Eigen::Index i = 0; i < count; ++i) {
        time(i) = buffer_[i].t - reference_t;
        angle(i) = buffer_[i].theta - reference_angle;
        time_weights(i) = sample_weight(buffer_[i].t, buffer_.back().t);
    }
    const double weight_sum = time_weights.sum();
    Eigen::MatrixXd design(count, 4);
    design.col(2) = time;
    design.col(3).setOnes();

    struct Candidate {
        Eigen::Vector4d coefficients;
        Eigen::VectorXd residual;
        double cost;
    };
    double cauchy_scale = 1;
    // 固定频率后，其余四项是线性的；IRLS 用残差权重抑制短时错误相位。
    const auto solve_linear = [&](double omega, bool robust) -> std::optional<Candidate> {
        design.col(0) = (omega * time.array()).cos().matrix();
        design.col(1) = (omega * time.array()).sin().matrix();
        Eigen::VectorXd weights = time_weights;
        Eigen::Vector4d coefficients = Eigen::Vector4d::Zero();
        Eigen::VectorXd residual(count);
        for (int iteration = 0; iteration < (robust ? 12 : 1); ++iteration) {
            Eigen::Matrix4d normal = design.transpose() * weights.asDiagonal() * design;
            normal.diagonal().array() += 1e-10;
            const Eigen::LDLT<Eigen::Matrix4d> solver(normal);
            if (solver.info() != Eigen::Success)
                return std::nullopt;
            const Eigen::Vector4d next = solver.solve(design.transpose() * weights.cwiseProduct(angle));
            if (!next.allFinite())
                return std::nullopt;
            residual = angle - design * next;
            const double change = (next - coefficients).norm();
            coefficients = next;
            if (!robust || change < 1e-8)
                break;
            weights = (time_weights.array() / (1 + (residual.array() / cauchy_scale).square())).matrix();
        }
        // 所有频率共用同一 Cauchy 尺度，使 LM 比较的是同一个目标函数。
        const double cost = (time_weights.array() * cauchy_scale * cauchy_scale *
                             (1 + (residual.array() / cauchy_scale).square()).log()).sum() / weight_sum;
        if (!std::isfinite(cost))
            return std::nullopt;
        return Candidate{coefficients, std::move(residual), cost};
    };

    constexpr double kOmegaMin = 1.80, kOmegaMax = 2.20; // 保留原 RMCS 搜索范围。
    double omega = 1.942;
    const auto seed = solve_linear(omega, false);
    if (!seed)
        return std::nullopt;
    // 用 MAD 估计尺度，避免短时大误差抬高 RMS 后反过来削弱异常点抑制。
    Eigen::VectorXd deviations = seed->residual;
    auto *middle = deviations.data() + count / 2;
    std::nth_element(deviations.data(), middle, deviations.data() + count);
    const double median_residual = *middle;
    deviations = (seed->residual.array() - median_residual).abs().matrix();
    std::nth_element(deviations.data(), middle, deviations.data() + count);
    cauchy_scale = std::max(1e-6, 2.5 * 1.4826 * *middle);
    auto best = solve_linear(omega, true);
    if (!best)
        return std::nullopt;

    // 外层仅优化频率；阻尼和步长限制避免一次更新跨过合理频率区间。
    double damping = 1e-2;
    constexpr double kDifferenceStep = 1e-3;
    for (int iteration = 0; iteration < 20; ++iteration) {
        const auto plus = solve_linear(omega + kDifferenceStep, true);
        const auto minus = solve_linear(omega - kDifferenceStep, true);
        if (!plus || !minus)
            return std::nullopt;
        const double gradient = (plus->cost - minus->cost) / (2 * kDifferenceStep);
        const double curvature = (plus->cost - 2 * best->cost + minus->cost) /
                                 (kDifferenceStep * kDifferenceStep);
        const double step = std::clamp(-gradient / (std::abs(curvature) + damping), -.05, .05);
        const double candidate_omega = std::clamp(omega + step, kOmegaMin, kOmegaMax);
        if (std::abs(candidate_omega - omega) < 1e-5)
            break;
        auto candidate = solve_linear(candidate_omega, true);
        if (candidate && candidate->cost < best->cost) {
            omega = candidate_omega;
            best = std::move(candidate);
            damping = std::max(1e-6, damping * .5);
        } else {
            damping = std::min(1e6, damping * 2);
        }
    }

    const auto &coefficients = best->coefficients;
    const double speed = coefficients(2);
    // 转回旧 FitResult 的绝对时间语义，下游仍按当前相位积分，无需修改 EKF 或火控。
    const double constant = reference_angle + coefficients(3) - speed * reference_t;
    const double amplitude = omega * std::hypot(coefficients(0), coefficients(1));
    const double phase = std::remainder(std::atan2(coefficients(1), -coefficients(0)) - omega * reference_t,
                                        2 * std::numbers::pi);
    // 导出代价保持原来的时间加权 MSE，与恒速拟合可直接比较；不混用鲁棒损失与平方损失。
    const double cost = (time_weights.array() * best->residual.array().square()).sum() / weight_sum;
    return FitResult{constant, speed, amplitude, omega, phase, cost};
}

} // namespace rm_utils
