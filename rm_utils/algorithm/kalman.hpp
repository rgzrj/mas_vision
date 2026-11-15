#pragma once

namespace rm_utils::util {

// 从 RMCS RuneModel 提取的协方差公式。状态转移和观测雅可比由具体模型提供。
template <class Covariance>
auto predict_covariance(const Covariance& prior, const Covariance& jacobian,
                        const Covariance& process_noise) -> Covariance {
    return jacobian * prior * jacobian.transpose() + process_noise;
}

// Joseph 形式保留观测噪声项，再对称化，沿用原版数值计算顺序。
template <class Covariance, class Jacobian, class Gain, class Noise>
auto correct_covariance(const Covariance& prior, const Jacobian& jacobian, const Gain& gain,
                        const Noise& observation_noise) -> Covariance {
    auto complement = Covariance::Identity() - gain * jacobian;
    auto posterior = Covariance {};
    posterior.noalias() = complement * prior * complement.transpose();
    posterior += (gain * observation_noise * gain.transpose()).eval();
    posterior = 0.5 * (posterior + posterior.transpose());
    return posterior;
}

} // namespace rm_utils::util
