#include "rune_shooter.hpp"
#include "auto_buff/debug/visualize/shoot_debug.hpp"

#include "rm_utils/algorithm/math_tools.hpp"
#include "rm_utils/algorithm/trajectory.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

using namespace rm_utils;
using namespace auto_buff;

namespace
{
// 火控参数在模块内读取；偏置在 Impl 初始化时统一从度转换为弧度。
RuneShooter::Config load_shoot_config(const std::filesystem::path &path)
{
    const auto settings = YAML::LoadFile(path.string())["auto_buff"]["shoot"];
    RuneShooter::Config config;
    config.bullet_speed        = settings["bullet_speed"].as<double>(config.bullet_speed);
    config.shoot_delay         = settings["shoot_delay"].as<double>(config.shoot_delay);
    config.offset_yaw          = settings["offset_yaw"].as<double>(config.offset_yaw);
    config.offset_pitch        = settings["offset_pitch"].as<double>(config.offset_pitch);
    config.rune_idle_duration  = settings["rune_idle_duration"].as<double>(config.rune_idle_duration);
    config.rune_shoot_duration = settings["rune_shoot_duration"].as<double>(config.rune_shoot_duration);
    return config;
}
} // namespace

struct RuneShooter::Impl
{
    ShootDebug shoot_debug;
    Config config;
    State state;
    rm_utils::Timestamp cycle_start{};
    bool cycle_started         = false;
    std::size_t cycle_progress = 0;
    double next_cycle_time     = 0.0;

    explicit Impl(const Config &source) : config{source}
    {
        if (!std::isfinite(config.bullet_speed) || config.bullet_speed <= 0.0)
            throw std::runtime_error("shoot.bullet_speed must be finite and positive");
        if (!std::isfinite(config.shoot_delay) || config.shoot_delay < 0.0)
            throw std::runtime_error("shoot.shoot_delay must be finite and nonnegative");
        if (!std::isfinite(config.offset_yaw) || !std::isfinite(config.offset_pitch))
            throw std::runtime_error("shoot angle offsets must be finite");
        if (!std::isfinite(config.rune_idle_duration) || config.rune_idle_duration <= 0.0 ||
            !std::isfinite(config.rune_shoot_duration) || config.rune_shoot_duration <= 0.0)
            throw std::runtime_error("shoot rune durations must be finite and positive");

        config.offset_yaw   = deg2rad(config.offset_yaw);
        config.offset_pitch = deg2rad(config.offset_pitch);
    }

    // 以解算时间推进 idle/shoot 两阶段；暂停、丢失或换模式由外层 reset 清周期。
    auto in_idle_phase() -> bool
    {
        if (!cycle_started)
        {
            cycle_start     = state.timestamp;
            cycle_started   = true;
            cycle_progress  = 0;
            next_cycle_time = config.rune_idle_duration;
        }
        const double elapsed = std::chrono::duration<double>(state.timestamp - cycle_start).count();
        // 对照 RMCS Repeat：跨过边界时只推进一个动作；回到 idle 后从当前时刻重启周期。
        if (elapsed > next_cycle_time)
        {
            cycle_progress = (cycle_progress + 1) % 2;
            if (cycle_progress == 0)
            {
                cycle_start     = state.timestamp;
                next_cycle_time = config.rune_idle_duration;
            }
            else
            {
                next_cycle_time += config.rune_shoot_duration;
            }
        }
        return cycle_progress == 0;
    }

    auto aim(const RuneState &track, rm_utils::Timestamp track_timestamp) -> std::optional<Aimed>
    {
        const auto direction        = track.get_direction();
        const auto distance         = direction.norm();
        const auto initial_fly_time = distance / config.bullet_speed;
        // track_timestamp 是曝光时刻，state.timestamp 是解算时刻；两者之差是处理延迟。
        const auto stale_time  = std::chrono::duration<double>(state.timestamp - track_timestamp).count();
        const auto fixed_delay = config.shoot_delay + stale_time;

        // 原版先以中心距离估算飞行时间，用该时刻的首个待击符板决定攻击/空闲状态。
        auto selected = false;
        auto idle     = true;
        {
            auto future = track;
            future.transition(initial_fly_time + fixed_delay);
            selected = !future.get_aimpoints().valid_indices().empty();
            if (selected)
                idle = in_idle_phase();
        }

        constexpr std::size_t kMaxIterate = 5;
        constexpr double kEpsilon         = 0.001;

        Eigen::Vector3d attack = Eigen::Vector3d::Zero();
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        auto target_point      = auto_buff::AimPoint{};
        auto yaw               = 0.0;
        auto pitch             = 0.0;
        auto fly_time          = initial_fly_time;

        // 飞行时间会改变符板未来位置；按 RMCS 原版最多迭代五次至 1 ms 收敛。
        for (std::size_t i = 0; i < kMaxIterate; ++i)
        {
            auto future = track;
            future.transition(fly_time + fixed_delay);
            center = future.get_direction();

            const auto aimpoints = future.get_aimpoints();
            if (selected && !aimpoints.empty())
            {
                target_point = aimpoints.front();
                attack       = target_point.position;
            }
            else
            {
                // 预热未完成或画面没有待击板时只瞄准中心，绝不允许发弹。
                attack       = center;
                target_point = auto_buff::AimPoint{attack};
            }

            const auto trajectory =
                rm_utils::Trajectory(config.bullet_speed, attack.head<2>().norm(), attack.z(), 9.81, .003, 80.0 / 57.3);
            if (trajectory.unsolvable)
                return std::nullopt;

            const auto previous = fly_time;
            fly_time            = trajectory.fly_time;
            yaw                 = std::atan2(attack.y(), attack.x());
            pitch               = -trajectory.pitch;
            if (std::abs(fly_time - previous) < kEpsilon)
                break;
        }

        yaw += config.offset_yaw;
        pitch += config.offset_pitch;

        // 将已补偿的绝对 yaw/pitch 转为单位方向；空间击打点另放在 attack 字段。
        auto target = auto_buff::AimPoint{
            +std::cos(pitch) * std::cos(yaw),
            +std::cos(pitch) * std::sin(yaw),
            -std::sin(pitch),
        };
        target.ff_v = target_point.ff_v;
        target.ff_a = target_point.ff_a;

        return Aimed{
            .aim_yaw         = yaw,
            .aim_pitch       = pitch,
            .fly_time        = fly_time,
            .prediction_time = fly_time + fixed_delay,
            .should_shoot    = selected && !idle,
            .idle            = idle,
            .target          = target,
            .center          = center,
            .attack          = attack,
        };
    }
};

RuneShooter::RuneShooter(const Config &config) : pimpl{std::make_unique<Impl>(config)} {}

RuneShooter::RuneShooter(const std::filesystem::path &config_path) : RuneShooter(load_shoot_config(config_path))
{
    pimpl->shoot_debug.set_enabled(YAML::LoadFile(config_path.string())["auto_buff"]["shoot"]["debug"].as<bool>(false));
}

void RuneShooter::disable_debug() noexcept
{
    pimpl->shoot_debug.set_enabled(false);
}

RuneShooter::~RuneShooter() noexcept = default;

auto RuneShooter::update(State state) -> void
{
    pimpl->state                 = state;
    rm_utils::playback_timestamp = state.timestamp;
}

auto RuneShooter::aim(const RuneState &track, rm_utils::Timestamp track_timestamp) -> std::optional<Aimed>
{
    return pimpl->aim(track, track_timestamp);
}

auto RuneShooter::reset() noexcept -> void
{
    pimpl->cycle_started   = false;
    pimpl->cycle_start     = rm_utils::Timestamp{};
    pimpl->cycle_progress  = 0;
    pimpl->next_cycle_time = 0.0;
}

auto RuneShooter::shoot(const auto_buff::TrackResult &track, const cv::Mat &image) -> std::optional<Aimed>
{
    std::optional<Aimed> aimed;
    if (track.initialized)
        aimed = aim(track.state, track.timestamp);
    else
        reset();
    pimpl->shoot_debug.draw(track, aimed, image);
    return aimed;
}
