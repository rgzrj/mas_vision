#include "auto_buff/buff_track/buff_state.hpp"
#include "rm_utils/algorithm/math_tools.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <numbers>
#include <ranges>

namespace auto_buff
{
// 原版运动积分及瞄准点几何。未来预测调用前先复制 RuneState。
// 仅推进状态副本；跟踪器的 EKF 后验和协方差不受未来外推影响。
auto RuneState::transition(double seconds) -> void
{
    if (sine_valid)
    {
        // 对速度 v+a*sin(phase) 作解析积分，避免把当前瞬时角速度误当成未来恒速。
        const auto old_phase = sine_phase;

        sine_t += seconds;
        sine_phase += sine_omega * seconds;

        if (std::abs(sine_omega) > 1e-12)
        {
            rotation_angle += sine_v * seconds + sine_a / sine_omega * (std::cos(old_phase) - std::cos(sine_phase));
        }
        else
        {
            rotation_angle += rotation_speed * seconds;
        }

        rotation_speed = sine_v + sine_a * std::sin(sine_phase);
    }
    else
    {
        rotation_angle = rotation_angle + rotation_speed * seconds;
    }
}

auto RuneState::get_direction() const -> Eigen::Vector3d
{
    return {x, y, z};
}

auto RuneState::get_aimpoints() const -> AimPoints
{
    // 预热由记录/实机时钟和模型初始化时刻共同决定，不依赖程序执行了多少帧。
    const auto converge_duration = std::chrono::seconds{sine_valid ? 6 : 3};
    if (rm_utils::playback_now() - start_timestamp < converge_duration)
        return {};

    const auto r_face = Eigen::AngleAxisd{face_yaw, Eigen::Vector3d::UnitZ()};

    static constexpr std::array kBladeAnglesDeg = {0.0, 72.0, 144.0, 216.0, 288.0};

    auto result = AimPoints{};
    for (const auto &[deg, inactive] : std::views::zip(kBladeAnglesDeg, inactive))
    {
        if (!inactive)
            continue;

        auto aimpoint = AimPoint{};
        {
            const auto alpha = rotation_angle + rm_utils::deg2rad(deg);
            const auto sin_a = std::sin(alpha);
            const auto cos_a = std::cos(alpha);

            // 符叶绕符面法线旋转（符中心静止、符面朝向固定）
            const auto omega     = rotation_speed;
            const auto alpha_acc = sine_valid ? sine_a * sine_omega * std::cos(sine_phase) : 0.0;

            const auto local = Eigen::Vector3d{
                0.0,
                -kRuneGlobalRadius * sin_a,
                +kRuneGlobalRadius * cos_a,
            };
            const auto local_v = Eigen::Vector3d{
                0.0,
                -kRuneGlobalRadius * cos_a * omega,
                -kRuneGlobalRadius * sin_a * omega,
            };
            const auto local_a = Eigen::Vector3d{
                0.0,
                -kRuneGlobalRadius * cos_a * alpha_acc + kRuneGlobalRadius * sin_a * omega * omega,
                -kRuneGlobalRadius * sin_a * alpha_acc - kRuneGlobalRadius * cos_a * omega * omega,
            };

            const auto world   = (Eigen::Vector3d{x, y, z} + r_face * local).eval();
            const auto world_v = (r_face * local_v).eval();
            const auto world_a = (r_face * local_a).eval();

            aimpoint = AimPoint{world};

            // 方向向量 d = world/|world| 的角速度 ω 与角加速度 α（射线假设）
            if (const auto distance = world.norm(); distance > 1e-6)
            {
                const auto ff_v = (world.cross(world_v) / (distance * distance)).eval();
                const auto ff_a = (world.cross(world_a) / (distance * distance) -
                                   ff_v * (2.0 * world.dot(world_v) / (distance * distance)))
                                      .eval();
                aimpoint.ff_v   = ff_v;
                aimpoint.ff_a   = ff_a;
            }
        }

        // 与 RMCS 保持一致：每次仅取遍历顺序中的首个待击板，不在此新增选板策略。
        result.emplace_back(aimpoint);
        break;
    }
    return result;
}

} // namespace auto_buff
