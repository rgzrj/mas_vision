#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "tests/common/synthetic_rune.hpp"
#include "tests/common/test.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>

using namespace auto_buff;
using tests::require;
using tests::synthetic;

int main() {
    return tests::run("buff_track_test", [] {
        tests::TemporaryDirectory directory;
        const auto temporary = directory.path;
        auto camera = temporary / "camera.yaml";
        std::ofstream(camera) << "image_width: 960\nimage_height: "
                                 "540\ncamera_matrix: "
                                 "[900.,0.,480.,0.,900.,270.,0.,0.,1.]\ndistortion:"
                                 " [0.,0.,0.,0.,0.]\n";
        auto hik = temporary / "hikcamera.yaml";
        std::ofstream(hik) << "camera: {exposuretime: 4000, gain: 15, transfer_latency_ms: "
                              "10.96}\n"
                              "calibration: {image_width: 960, image_height: 540, camera_matrix: "
                              "[900,0,480,0,900,270,0,0,1], distort_coeffs: [0,0,0,0,0]}\n"
                              "handeye_calibration: {R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0], "
                              "t_camera2gimbal: [0.1,0.2,0.3]}\n";
        const cv::Mat image = cv::Mat::zeros(540, 960, CV_8UC3);
        std::array<double, 9> k{900, 0, 480, 0, 900, 270, 0, 0, 1};
        // mas_vision 的原始相机 YAML 没有图像宽高，仍须能直接读取并完成位姿初始化。
        auto host_yaml = YAML::LoadFile(hik.string());
        host_yaml["calibration"].remove("image_width");
        host_yaml["calibration"].remove("image_height");
        const auto host_camera = temporary / "host_camera.yaml";
        std::ofstream(host_camera) << host_yaml;
        auto_buff::RmcsTracker host_tracker(host_camera, YAML::Node{});
        const auto host_state = host_tracker.update(
            synthetic(0, k, Eigen::Quaterniond::Identity(), Eigen::Vector3d(.1, .2, .3)), 0, image);
        require(host_state.initialized &&
                    cv::norm(cv::Vec3d(host_state.state.x - 5, host_state.state.y, host_state.state.z)) < .05,
                "Host calibration without image dimensions cannot initialize the RMCS model");
        auto_buff::RmcsTracker calibrated_tracker(hik, YAML::Node{});
        const auto calibrated = calibrated_tracker.update(
            synthetic(0, k, Eigen::Quaterniond::Identity(), Eigen::Vector3d(.1, .2, .3)), 0, image);
        require(calibrated.initialized &&
                    cv::norm(cv::Vec3d(calibrated.state.x - 5, calibrated.state.y, calibrated.state.z)) < .05,
                "Optical-to-RMCS handeye conversion changed");
        auto options = YAML::Load("track: { lost_timeout_s: 1.5, prediction_seconds: 0.1 }");
        auto_buff::RmcsTracker tracker(camera, options["track"]);
        auto only_single = synthetic(0, k);
        for (auto &b : only_single.boards)
            b.appearance = auto_buff::Appearance::single_ring;
        require(!tracker.update(only_single, 0, image).initialized,
                "Single rings must not initialize a pose");
        auto clipped_seed = synthetic(.02, k);
        clipped_seed.boards[0].features_valid = false;
        require(!tracker.update(clipped_seed, .02, image).initialized,
                "Clipped endpoints must not seed PnP");
        tracker.reset();
        auto neutral_r = synthetic(.03, k);
        neutral_r.markers[0].color = auto_buff::CampColor::UNKNOWN;
        const auto neutral_state = tracker.update(neutral_r, .03, image);
        require(neutral_state.initialized && neutral_state.color == auto_buff::CampColor::RED,
                "Neutral R must initialize using the selected board color");
        auto neutral_update = synthetic(.04, k);
        neutral_update.markers[0].color = auto_buff::CampColor::UNKNOWN;
        require(tracker.update(neutral_update, .04, image).observed,
                "Neutral R or selected board disappeared after initialization");
        tracker.reset();
        auto multiple_r = synthetic(.04, k);
        // A high glyph score must not hide the lower-scored geometrically correct
        // R.
        multiple_r.markers.insert(multiple_r.markers.begin(), auto_buff::Marker{{80, 60}, auto_buff::CampColor::RED, 2, true});
        const auto multi_state = tracker.update(multiple_r, .04, image);
        require(multi_state.initialized && cv::norm(cv::Vec3d(multi_state.state.x - 5, multi_state.state.y,
                                                              multi_state.state.z)) < .05,
                "Higher-scored false R suppressed the correct PnP candidate");
        tracker.reset();
        auto_buff::TrackResult state;
        std::size_t count_before_occlusion = 0;
        double angle_before_occlusion = 0;
        for (int i = 0; i <= 225; ++i) {
            double time = i / 50.;
            auto_buff::Detection input = synthetic(time, k);
            if (i >= 100 && i < 104)
                input = {}; // Occlusion must predict, not inject held measurements.
            state = tracker.update(input, time, image);
            require(state.initialized, "RMCS failed known-calibration initialization/tracking");
            require(std::abs(state.ekf.state[0] - state.state.x) < 1e-10 &&
                        std::abs(state.ekf.state[4] - state.state.rotation_angle) < 1e-10 &&
                        std::abs(state.ekf.state[5] - state.state.face_yaw) < 1e-10,
                    "EKF snapshot order differs from the posterior state");
            require(std::ranges::all_of(state.ekf.variance, [](double p) { return std::isfinite(p) && p >= 0; }),
                    "EKF covariance display contains invalid variances");
            if (i == 99) {
                count_before_occlusion = state.state.update_count;
                angle_before_occlusion = state.state.rotation_angle;
            }
            if (i >= 100 && i < 104) {
                require(!state.observed && state.matched.empty(), "Occlusion became a fake observation");
                require(state.state.update_count == count_before_occlusion,
                        "Prediction-only frames entered motion fitting");
                require(state.state.rotation_angle > angle_before_occlusion,
                        "Occlusion stopped time prediction");
            }
        }
        require(cv::norm(cv::Vec3d(state.state.x - 5, state.state.y, state.state.z)) < .05,
                "RMCS center estimate regressed");
        require(std::abs(state.state.rotation_speed - .8) < .08, "Media-time angular speed is wrong");
        require(state.ready, "RMCS warmup used wall time instead of media time");
        require(state.future.size() == 5, "RMCS must predict all five board slots");
        // 相机转动、手眼有平移且 IMU 基不同，空间神符中心仍应固定在已知世界位置。
        const auto moving_camera = temporary / "moving_camera.yaml";
        std::ofstream(moving_camera) << "calibration: {image_width: 960, image_height: 540, camera_matrix: "
                                        "[900,0,480,0,900,270,0,0,1], distort_coeffs: [0,0,0,0,0]}\n"
                                        "handeye_calibration:\n"
                                        "  R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0]\n"
                                        "  t_camera2gimbal: [0.1,0.02,0.03]\n"
                                        "  R_gimbal2imubody: [0,1,0,-1,0,0,0,0,1]\n";
        auto_buff::RmcsTracker moving_tracker(moving_camera, YAML::Load("lost_timeout_s: 0.5"));
        const Eigen::Quaterniond basis(Eigen::AngleAxisd(-std::numbers::pi / 2, Eigen::Vector3d::UnitZ()));
        auto_buff::TrackResult moving_state;
        for (int i = 0; i <= 225; ++i) {
            const double time = i / 50.;
            const Eigen::Quaterniond gimbal =
                Eigen::AngleAxisd(.12 * std::sin(time), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(.04 * std::cos(time), Eigen::Vector3d::UnitY());
            const Eigen::Vector3d translation = gimbal * Eigen::Vector3d(.1, .02, .03);
            moving_tracker.set_imu_orientation(basis * gimbal * basis.conjugate());
            moving_state = moving_tracker.update(synthetic(time, k, gimbal, translation), time, image);
            require(moving_state.initialized, "Dynamic handeye failed to initialize or track");
            require(moving_state.gimbal_orientation && moving_state.gimbal_orientation->angularDistance(gimbal) < 1e-8,
                    "Shoot display lost the exposure-paired gimbal orientation");
            require(cv::norm(cv::Vec3d(moving_state.state.x - 5, moving_state.state.y,
                                       moving_state.state.z)) < .05,
                    "Camera motion leaked into the world rune center");
        }
        require(std::abs(moving_state.state.rotation_speed - .8) < .08,
                "Camera motion corrupted the fitted rune speed");
        require(!moving_tracker.update({}, 4.52, image).gimbal_orientation,
                "Shoot display reused the previous frame's gimbal observation");
        auto future = synthetic(4.6, k);
        double worst = 0;
        for (const auto &p : state.future) {
            double best = 1e9;
            for (const auto &b : future.boards)
                best = std::min(best, cv::norm(b.center - cv::Point2d(p.point.x, p.point.y)));
            worst = std::max(worst, best);
        }
        require(worst < 3, "RMCS future pixels disagree with the known motion");
        auto clipped_update = synthetic(4.52, k);
        clipped_update.boards[0].features_valid = false;
        auto updated = tracker.update(clipped_update, 4.52, image);
        require(updated.initialized &&
                    std::ranges::any_of(updated.matched,
                                        [&](const auto &p) {
                                            return p.feature_id > 0 &&
                                                   cv::norm(cv::Point2d(p.point.x, p.point.y) -
                                                            clipped_update.boards[0].center) < 1;
                                        }),
                "An initialized RMCS tracker discarded a valid clipped-ring center");
        require(!tracker.update({}, 6.1, image).initialized, "Lost tracker failed to reset");
        require(tracker.update(synthetic(6.2, k), 6.2, image).initialized, "RMCS failed reacquisition");
        bool mismatch = false;
        try {
            (void)tracker.update({}, 6.3, cv::Mat::zeros(480, 640, CV_8UC3));
        } catch (const std::runtime_error &) {
            mismatch = true;
        }
        require(mismatch, "Calibration resolution mismatch was accepted");
        std::cout << "RMCS: known 3D geometry, 0.8 rad/s, occlusion, replay "
                     "warmup, prediction and reacquisition passed.\n";
    });
}
