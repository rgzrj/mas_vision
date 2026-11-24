#include "tests/common/detection.hpp"
#include "display.hpp"
#include "auto_buff/debug/visualize/detect_debug.hpp"
#include "auto_buff/debug/visualize/track_debug.hpp"
#include "auto_buff/debug/visualize/shoot_debug.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "auto_buff/buff_detect/traditional/traditional.hpp"
#include "auto_buff/debug/buff_debug.hpp"
#include "tests/common/test.hpp"
#include <iostream>
#include <fstream>
#include <stdexcept>

int main()
{
    try
    {
        using namespace auto_buff;
        using tests::require;
        cv::Mat blank(540, 960, CV_8UC3, cv::Scalar(0));
        auto_buff::TrackResult overlay;
        require(auto_buff::rune_motion_text(overlay).empty(), "Uninitialized tracker displayed a motion model");
        overlay.initialized          = true;
        overlay.state.rotation_angle = .25;
        require(auto_buff::rune_motion_text(overlay) == "theta_ekf=+0.25", "EKF display fallback changed");
        overlay.state.use_prediction_speed = true;
        overlay.state.update_count         = 7;
        overlay.state.rotation_speed       = .8;
        overlay.state.prediction_cost      = .002;
        require(auto_buff::rune_motion_text(overlay) == "spd_7(t)=+0.80, e=0.002", "Constant speed formula changed");
        overlay.state.sine_valid      = true;
        overlay.state.sine_v          = -1.12;
        overlay.state.sine_a          = .84;
        overlay.state.sine_phase      = 20.24;
        overlay.state.sine_omega      = 1.9;
        overlay.state.prediction_cost = .001;
        require(auto_buff::rune_motion_text(overlay) == "spd_7(t)=-1.12+0.84*sin(+20.24+1.90t), e=0.001",
                "Sine formula no longer matches RMCS state fields");
        tests::TemporaryDirectory temporary;
        const auto recording = temporary.path / "debug.rrd";
        auto &display        = rm_utils::Display::getInstance();
        display.start(false, tool::utf8(recording)); // 测试只落盘，不启动桌面 Viewer。
        require(display.image_due(0, std::nullopt) && !display.image_due(.001, 0.) && display.image_due(.02, 0.) &&
                    display.image_due(0, 1.),
                "Rerun image sampling or rewind handling changed");
        display.set_time(0);
        const auto camera = temporary.path / "camera.yaml";
        std::ofstream(camera) << "image_width: 960\nimage_height: 540\n"
                                 "camera_matrix: [900,0,480,0,900,270,0,0,1]\ndistortion: [0,0,0,0,0]\n";
        const auto config_path = temporary.path / "buff.yaml";
        std::ofstream(config_path) << "auto_buff:\n  track: {debug: true}\n  shoot: {debug: true}\n";
        auto_buff::TraditionalDetector detector(tests::traditional_config("red", YAML::Load("debug: true")), YAML::LoadFile(camera.string()));
        const auto observation = tests::detect(detector, blank, 0);
        detector.draw(observation);
        auto_buff::RmcsTracker tracker(config_path, camera);
        const auto tracking = tracker.update(observation, 0, blank);
        auto_buff::RuneShooter shooter(config_path);
        require(!shooter.shoot(tracking, blank), "Uninitialized shoot produced an aim");
        auto_buff::TraditionalDetector off_detector(tests::traditional_config(), YAML::LoadFile(camera.string()));
        const auto off_observation = tests::detect(off_detector, blank, 0);
        auto_buff::RmcsTracker off_tracker(camera, YAML::Node{});
        const auto off_tracking = off_tracker.update(off_observation, 0, blank);
        auto_buff::RuneShooter off_shooter(auto_buff::RuneShooter::Config{});
        require(!off_shooter.shoot(off_tracking) && tracking.status == off_tracking.status &&
                    observation.boards.size() == off_observation.boards.size(),
                "Debug display changed the module computation");
        auto valid_track                 = tracking;
        valid_track.initialized          = true;
        valid_track.state.x              = 5;
        valid_track.state.rotation_speed = .8;
        valid_track.state.rotation_angle = 3.9;
        valid_track.state.inactive[0]    = true;
        valid_track.gimbal_orientation   = Eigen::Quaterniond::Identity(); // 当前角、目标角与差值的显示分支。
        shooter.update({valid_track.timestamp});
        off_shooter.update({valid_track.timestamp});
        const auto debug_aim = shooter.shoot(valid_track, blank);
        const auto plain_aim = off_shooter.shoot(valid_track, blank);
        require(debug_aim && plain_aim && debug_aim->aim_yaw == plain_aim->aim_yaw &&
                    debug_aim->aim_pitch == plain_aim->aim_pitch && debug_aim->should_shoot == plain_aim->should_shoot,
                "Module debug flag changed the ballistic result");
        auto_buff::DetectDebug(true).draw(observation, blank);
        auto_buff::TrackDebug(true).draw(valid_track, blank);
        auto_buff::ShootDebug(true).draw(valid_track, debug_aim, blank);
        cv::Mat frame(240, 320, CV_8UC3, cv::Scalar(20, 30, 40));
        display.set_time(1);
        display.display_add("test", frame, {{"Rerun check", 10, 10, 24, {}, 1, true}}, {{100, 100}},
                            {{10, 20, 100, 100}});
        require(frame.at<cv::Vec3b>(10, 10) == cv::Vec3b(20, 30, 40), "Rerun HUD changed the original frame");
        display.scalar("test/value", 1.25);
        display.point3d("world/test", 1, 2, 3);
        frame.setTo(cv::Scalar(0));
        display.set_time(2);
        display.display_add("test", frame); // 空图元必须清除旧观测。
        display.shutdown();
        require(std::filesystem::file_size(recording) > 1024, "Rerun recording is empty");
        std::ifstream rrd(recording, std::ios::binary);
        char magic[4]{};
        rrd.read(magic, 4);
        require(std::string(magic, 4) == "RRF2", "Invalid Rerun file header");
        std::cout << "Rerun recording and debug computation checks passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
