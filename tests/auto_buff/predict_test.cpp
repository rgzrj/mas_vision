#include "auto_buff/buff_track/predict/rune_predictor.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "tests/common/synthetic_rune.hpp"
#include "tests/common/test.hpp"
#include <cmath>
#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <limits>

namespace {
void check_robust_sine_fit() {
    struct Scenario {
        const char *name;
        double direction;
        double time_origin;
        bool outliers;
        double omega = 1.937;
    };
    constexpr std::array scenarios{
        Scenario{"clean", 1, 0, false}, Scenario{"outliers", 1, 0, true},
        Scenario{"reverse_outliers", -1, 0, true}, Scenario{"large_timestamp", 1, 1e6, false},
        Scenario{"lower_frequency", 1, 0, false, 1.80}, Scenario{"upper_frequency", 1, 0, false, 2.20}};
    double worst_prediction_error = 0;
    for (const auto &scenario : scenarios) {
        rm_utils::RuneEnergyFitter fitter;
        // 使用非网格频率和解析真值，独立检查两个旋向与异常观测下的未来角增量。
        const auto truth = [&](double t) {
            return scenario.direction * (.4 + 1.13 * t - .86 / scenario.omega * std::cos(scenario.omega * t + .7));
        };
        for (int i = 0; i <= 480; ++i) {
            const double time = i / 80.;
            double measured = truth(time) + .002 * std::sin(7 * time);
            if (scenario.outliers && i % 23 == 0)
                measured += (i / 23 % 2 ? -.6 : .6);
            // 接近窗口末端的短时偏移，模拟误关联或冲击造成的连续异常相位。
            if (scenario.outliers && i >= 430 && i < 438)
                measured += .6;
            fitter.push(scenario.time_origin + time, measured);
        }
        std::array<double, 7> timings{};
        std::optional<rm_utils::RuneEnergyFitter::FitResult> fit;
        for (auto &milliseconds : timings) {
            const auto start = std::chrono::steady_clock::now();
            fit = fitter.fit_sine();
            milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        }
        tests::require(fit.has_value(), "Sine fitter failed a six-second observation window");
        const auto predicted = [&](double relative_time) {
            const double time = scenario.time_origin + relative_time;
            return fit->C + fit->v * time - fit->a / fit->omega * std::cos(fit->omega * time + fit->phi);
        };
        double error = 0;
        for (double horizon : {.02, .1, .3}) {
            const double increment = predicted(6 + horizon) - predicted(6);
            error = std::max(error, std::abs(increment - (truth(6 + horizon) - truth(6))));
        }
        worst_prediction_error = std::max(worst_prediction_error, error);
        std::sort(timings.begin(), timings.end());
        std::cout << "fit_case=" << scenario.name << " future_error_rad=" << error
                  << " omega=" << fit->omega << " median_fit_ms=" << timings[timings.size() / 2] << '\n';
    }
    tests::require(worst_prediction_error < .01, "Outliers or large timestamps corrupted future angle increments");
}

void check_tracker_sine_prediction() {
    tests::TemporaryDirectory directory;
    const auto camera = directory.path / "camera.yaml";
    const auto image  = cv::Mat::zeros(540, 960, CV_8UC3);
    std::ofstream(camera) << "calibration: {camera_matrix: [900,0,480,0,900,270,0,0,1], "
                             "distort_coeffs: [0,0,0,0,0]}\n"
                             "handeye_calibration: {R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0], "
                             "t_camera2gimbal: [0,0,0], R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]}\n";
    auto_buff::RmcsTracker tracker(camera, YAML::Node{});
    const auto truth = [](double time) {
        return .4 + 1.13 * time - .86 / 1.937 * std::cos(1.937 * time + .7);
    };
    auto_buff::TrackResult result;
    for (int i = 0; i <= 800; ++i) {
        const double time = i / 80.;
        // synthetic 的参数只控制几何转角；曝光时间仍使用真实 time，不把相位当时间。
        const auto observation = tests::synthetic((truth(time) - .3) / .8, tests::intrinsics);
        result = tracker.update(observation, time, image);
        tests::require(result.initialized, "RMCS lost the known sine-motion target");
    }
    tests::require(result.state.sine_valid, "RMCS EKF phase did not reach the robust sine fitter");
    auto future = result.state;
    future.transition(.3);
    const double error = std::abs(future.rotation_angle - truth(10.3));
    std::cout << "ekf_sine_future_error_rad=" << error << '\n';
    tests::require(error < .03, "EKF-to-sine integration lost the known future position");
}
} // namespace

int main() {
    return tests::run("buff_predict_test", [] {
        using tests::require;
        auto_buff::RunePredictor predictor;
        auto_buff::RunePrediction linear{};
        for (int i = 0; i <= 200; ++i) {
            const double time = i / 50.;
            predictor.correct(1, 0, tests::stamp(time), time, .3 + .8 * time, linear);
        }
        require(linear.use_prediction_speed && !linear.sine_valid &&
                    std::abs(linear.prediction_speed - .8) < .01,
                "Constant-speed fit changed");
        const auto samples = linear.update_count;
        predictor.correct(0, 0, tests::stamp(4.1), 4.1, 100, linear);
        require(linear.update_count == samples, "Prediction-only input entered the fitting window");
        predictor.reset();
        auto_buff::RunePrediction sine{};
        // theta 是速度 v+a*sin(w*t+phi) 的解析积分；拟合与真值来源独立。
        const auto angle = [](double time) {
            return .3 + 1.12 * time - .84 / 1.9 * std::cos(1.9 * time + .3);
        };
        for (int i = 0; i <= 400; ++i) {
            const double time = i / 50.;
            predictor.correct(1, 0, tests::stamp(time), time, angle(time) + .001 * std::sin(7 * time), sine);
        }
        require(sine.sine_valid && !sine.use_prediction_speed && std::abs(sine.sine_v - 1.12) < .05 &&
                    std::abs(sine.sine_a - .84) < .05 && std::abs(sine.sine_omega - 1.9) < .03,
                "Sine fit failed the known motion");
        auto_buff::RuneState state{};
        state.x = 5;
        state.rotation_angle = angle(8);
        state.rotation_speed = sine.sine_v + sine.sine_a * std::sin(sine.sine_phase);
        state.sine_valid = true;
        state.sine_v = sine.sine_v;
        state.sine_a = sine.sine_a;
        state.sine_omega = sine.sine_omega;
        state.sine_phase = sine.sine_phase;
        state.sine_t = sine.sine_t;
        double worst_error = 0;
        for (double horizon : {.02, .1, .3}) {
            auto future = state;
            future.transition(horizon);
            worst_error = std::max(worst_error, std::abs(future.rotation_angle - angle(8 + horizon)));
        }
        require(worst_error < .01 && state.rotation_angle == angle(8),
                "Future integration changed the source or lost accuracy");
        cv::Mat intrinsic(3, 3, CV_64F, const_cast<double *>(tests::intrinsics.data()));
        cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
        require(auto_buff::predict_rune_features(state, .1, intrinsic, distortion, Eigen::Isometry3d::Identity()).size() == 5,
                "Predictor did not project all five board slots");
        state.x = std::numeric_limits<double>::quiet_NaN();
        require(auto_buff::predict_rune_features(state, .1, intrinsic, distortion, Eigen::Isometry3d::Identity()).empty(),
                "Predictor emitted non-finite image coordinates");
        std::cout << "constant_speed=" << linear.prediction_speed << " sine_v=" << sine.sine_v
                  << " a=" << sine.sine_a << " omega=" << sine.sine_omega
                  << " future_error_rad=" << worst_error << '\n';
        check_robust_sine_fit();
        check_tracker_sine_prediction();
    });
}
