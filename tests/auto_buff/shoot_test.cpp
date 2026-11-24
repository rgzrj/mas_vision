#include "auto_buff/buff_shoot/rune_shooter.hpp"
#include "rm_utils/algorithm/trajectory.hpp"
#include "tests/common/test.hpp"
#include <cmath>
#include <fstream>

using tests::require;

int main() {
    return tests::run("buff_shoot_test", [] {
        auto_buff::AimPoints points;
        points.emplace_back(1, 2, 3, false);
        points.emplace_back(1, 2, 3, true);
        require(points.valid_indices() == std::vector<std::size_t>{1}, "Valid aimpoint indices changed");
        auto_buff::RuneState state{};
        state.x = 5;
        state.rotation_speed = .8;
        state.rotation_angle = 3.9;
        state.inactive[0] = true;
        const auto stamp = rm_utils::Timestamp{} + std::chrono::milliseconds(4500);
        const auto ballistic = rm_utils::Trajectory(22.5, 5., 0., 9.81, .003, 80.0 / 57.3);
        require(!ballistic.unsolvable && ballistic.fly_time > .2 && ballistic.fly_time < .3 && ballistic.pitch > 0,
                "Shared MAS ballistic solution changed");
        // 约41°且能在原10轮迭代内收敛，单独检验上限而非迭代次数。
        require(rm_utils::Trajectory(22.5, .5, .425, 9.81, .003).unsolvable &&
                    !rm_utils::Trajectory(22.5, .5, .425, 9.81, .003, 80.0 / 57.3).unsolvable,
                "MAS default pitch limit or explicit buff pitch limit changed");
        require(rm_utils::Trajectory(22.5, .1, .1).unsolvable,
                "Initial launch angle bypassed the configured pitch limit");
        require(rm_utils::Trajectory(0, 5, 0).unsolvable &&
                    rm_utils::Trajectory(22.5, 10000, 0).unsolvable,
                "Invalid speed or unreachable target acquired a trajectory");
        auto_buff::RuneShooter shooter(auto_buff::RuneShooter::Config{});
        shooter.update({stamp});
        const auto idle_aim = shooter.aim(state, stamp);
        require(idle_aim && !idle_aim->should_shoot && idle_aim->idle &&
                    idle_aim->prediction_time > idle_aim->fly_time,
                "Rune fire control did not begin with the RMCS idle window");
        const auto shoot_stamp = stamp + std::chrono::milliseconds(410);
        shooter.update({shoot_stamp});
        const auto shoot_aim = shooter.aim(state, stamp);
        require(shoot_aim && shoot_aim->should_shoot && !shoot_aim->idle &&
                    std::hypot(shoot_aim->attack.y() - shoot_aim->center.y(),
                               shoot_aim->attack.z() - shoot_aim->center.z()) > .6,
                "Rune fire control missed the RMCS shoot window or future blade");

        // 文件入口与直接配置使用相同参数、单位和解算时钟。
        tests::TemporaryDirectory directory;
        const auto config_path = directory.path / "auto_buff.yaml";
        std::ofstream(config_path)
            << "auto_buff: {shoot: {bullet_speed: 24.0, fire_delay_time: 0.05, "
               "offset_yaw: 2.0, offset_pitch: -1.0, debug: false}}\n";

        auto config            = auto_buff::RuneShooter::Config{};
        config.bullet_speed    = 24.0;
        config.fire_delay_time = 0.05;
        config.offset_yaw      = 2.0;
        config.offset_pitch    = -1.0;

        auto_buff::RuneShooter configured(config);
        auto_buff::RuneShooter loaded(config_path);
        std::filesystem::remove(config_path);

        configured.update({stamp});
        loaded.update({stamp});
        const auto expected = configured.aim(state, stamp);
        const auto actual   = loaded.aim(state, stamp);
        require(expected && actual && std::abs(actual->aim_yaw - expected->aim_yaw) < 1e-12 &&
                    std::abs(actual->aim_pitch - expected->aim_pitch) < 1e-12 &&
                    std::abs(actual->fly_time - expected->fly_time) < 1e-12 &&
                    std::abs(actual->prediction_time - expected->prediction_time) < 1e-12 &&
                    actual->idle == expected->idle && actual->should_shoot == expected->should_shoot,
                "File and direct fire-control configuration differ");

        require(std::abs(shoot_aim->aim_yaw - .134059) < 1e-6 &&
                    std::abs(shoot_aim->aim_pitch + .0133239) < 1e-6 &&
                    std::abs(shoot_aim->fly_time - .225999) < 1e-6,
                "MAS trajectory integration changed the recorded buff fire-control baseline");
        shooter.update({stamp + std::chrono::milliseconds(610)});
        const auto next_idle = shooter.aim(state, stamp);
        require(next_idle && !next_idle->should_shoot && next_idle->idle,
                "Rune fire control did not return to the RMCS idle window");

        std::cout << "yaw=" << shoot_aim->aim_yaw << " pitch=" << shoot_aim->aim_pitch
                  << " fly_time=" << shoot_aim->fly_time << " prediction=" << shoot_aim->prediction_time
                  << '\n';
    });
}
