#include "auto_aim/debug/armor_debug.hpp"
#include "auto_aim/debug/visualize/detect_debug.hpp"
#include "auto_aim/debug/visualize/pose_debug.hpp"
#include "auto_aim/debug/visualize/track_debug.hpp"
#include "auto_aim/debug/visualize/shoot_debug.hpp"
#include "auto_aim/armor_detector/armor_detector.hpp"
#include "auto_aim/armor_track/armor_track.hpp"
#include "auto_aim/armor_shoot/armor_shoot.hpp"
#include "auto_buff/debug/buff_debug.hpp"
#include "auto_buff/debug/visualize/detect_debug.hpp"
#include "auto_buff/debug/visualize/track_debug.hpp"
#include "auto_buff/debug/visualize/shoot_debug.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "rm_utils/display/display.hpp"
#include "tests/common/test.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <atomic>
#include <type_traits>
#include <map>
#include <cmath>

namespace
{
struct Panel
{
    std::string name;
    std::vector<rm_utils::DisplayText> texts;
    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine> lines;
};
std::vector<Panel> panels;
std::vector<rm_utils::DisplayView> views;
std::map<std::string, double> series;
std::size_t scalar_calls = 0;
bool text_contains(const Panel &panel, const std::string &value)
{
    return std::any_of(panel.texts.begin(), panel.texts.end(),
                       [&](const auto &text) { return text.content.find(value) != std::string::npos; });
}
} // namespace

// 只替换显示接收端，执行生产绘制代码，检查原诊断信息而不依赖GPU或Viewer。
namespace rm_utils
{
struct Display::Impl
{
};
Display::Display() : impl_(std::make_unique<Impl>()) {}
Display::~Display() = default;
Display &Display::getInstance()
{
    static Display value;
    return value;
}
void Display::layout(const std::vector<DisplayView> &value, std::span<const unsigned char>)
{
    views = value;
}
void Display::set_time(double) {}
void Display::register_layout(const std::string &, const std::vector<DisplayView> &value,
                              std::span<const unsigned char>)
{
    views = value;
}
void Display::select_layout(const std::string &) {}
bool Display::image_due(double, std::optional<double>) const
{
    return true;
}
void Display::points2d(const std::string &, const std::vector<cv::Point2d> &, const std::vector<std::string> &) {}
void Display::point3d(const std::string &, double, double, double) {}
std::string processing_fps_text(double value)
{
    return "Process FPS: " + std::to_string(value);
}
void Display::scalar(const std::string &path, double value)
{
    series[path] = value;
    ++scalar_calls;
}
void Display::end_frame() {}
void Display::display_add(const std::string &name, const cv::Mat &, const std::vector<DisplayText> &texts,
                          const std::vector<DisplayPoint> &points, const std::vector<DisplayLine> &lines, int, int)
{
    panels.push_back({name, texts, points, lines});
}
} // namespace rm_utils

int main()
{
    return tests::run(
        "armor_visual_test",
        []
        {
            using tests::require;
            using namespace auto_aim;
            static_assert(std::is_base_of_v<ArmorDebug, DetectDebug>);
            static_assert(std::is_base_of_v<ArmorDebug, TrackDebug>);
            static_assert(std::is_base_of_v<ArmorDebug, PoseDebug>);
            static_assert(std::is_base_of_v<ArmorDebug, ShootDebug>);
            DetectDebug detect_view(true);
            TrackDebug track_view(true);
            PoseDebug pose_view(true);
            ShootDebug shoot_view(true);
            const std::filesystem::path root(BUFF_PROJECT_DIR);
            tests::TemporaryDirectory temporary;
            auto config                             = YAML::LoadFile((root / "config/auto_aim.yaml").string());
            auto aim                                = config["auto_aim"];
            aim["armor_detector"]["debug"]          = false;
            aim["armor_track"]["armor_track_debug"] = false;
            aim["armor_track"]["armor_pose_debug"]  = false;
            aim["armor_shoot"]["debug"]             = false;
            aim["armor_track"]["plotter_enable"]    = false;
            aim["armor_shoot"]["plotter_enable"]    = false;
            for (const auto *key : {"model_path", "label_path"})
                aim["armor_detector"]["number_params"][key] =
                    std::filesystem::absolute(root / "config" /
                                              aim["armor_detector"]["number_params"][key].as<std::string>())
                        .string();
            const auto path = temporary.path / "auto_aim.yaml";
            std::ofstream(path) << config;
            ArmorPose pose((root / "config/hikcamera.yaml").string());
            pose.set_R_gimbal2world(Eigen::Quaterniond::Identity());
            for (const auto *name : {"1", "3", "base", "outpost"})
            {
                Armor sample{};
                sample.number = name;
                sample.points = pose.reproject_armor({3, 0, 0}, 0, sample.number);
                require(sample.points.size() == 4 && pose.GetArmorPose(sample), "Small armor pose failed");
                require((sample.xyz_in_world - Eigen::Vector3d{3, 0, 0}).norm() < .01,
                        "Small armor pose uses inconsistent physical dimensions");
                const auto projected = pose.reproject_armor(sample.xyz_in_world, sample.ypr_in_world[0], sample.number);
                for (std::size_t i = 0; i < projected.size(); ++i)
                    require(cv::norm(projected[i] - sample.points[i]) < .1, "Pose and debug projection disagree");
            }
            Armor observed{};
            observed.number            = "3";
            observed.confidence        = .9f;
            observed.ratio             = 2;
            observed.side_ratio        = 1;
            observed.rectangular_error = 0;
            observed.points            = pose.reproject_armor({3, 0, 0}, 0, observed.number);
            observed.center            = (observed.points[0] + observed.points[2]) * .5f;
            observed.xyz_in_world      = {3, 0, 0};
            observed.xyz_in_gimbal     = {3, 0, 0};
            const cv::Mat image        = cv::Mat::zeros(480, 640, CV_8UC3);
            const std::vector<LightBar> lights{LightBar(cv::RotatedRect({120, 120}, {8, 40}, 5))};
            std::vector<Armor> armors{observed};
            detect_view.draw(image, armors, lights);
            require(text_contains(panels.back(), "Lights: 1") && panels.back().points.size() >= 6 &&
                        panels.back().lines.size() >= 8,
                    "Detection lost lightbars, corners or counts");
            require(panels.back().name == "HikCamera" &&
                        std::all_of(panels.back().texts.begin(), panels.back().texts.end(),
                                    [](const auto &text) { return text.image_overlay; }),
                    "Base display submission changed window naming or text overlays");
            const auto before_disable = panels.size();
            ArmorDebug &base_debug    = detect_view;
            base_debug.set_enabled(false);
            detect_view.draw(image, armors, lights);
            require(!detect_view.enabled() && panels.size() == before_disable,
                    "Base debug switch did not control derived drawing");
            detect_view.set_enabled(true);
            pose_view.draw(image, armors, pose);
            for (const auto *label :
                 {"IMU YPR", "Gimbal XYZ", "Gimbal YPR", "World XYZ", "World YPR", "World YPD", "Reproj Error"})
                require(text_contains(panels.back(), label), "Pose diagnostic missing");
            require(panels.back().lines.size() >= 8, "Pose contours missing");
            ArmorTrack tracker(path.string(), (root / "config/hikcamera.yaml").string());
            tracker.armor_pose().set_R_gimbal2world(Eigen::Quaterniond::Identity());
            const auto time  = std::chrono::steady_clock::now();
            const auto ready = tracker.track(armors, time, image);
            require(!ready && tracker.target_state(), "Synthetic sample did not enter tracker warm-up");
            track_view.draw(image, armors, tracker);
            for (const auto *label :
                 {"detecting", "Target: 3", "Updates:", "type: SMALL", "jumped:", "switch:", "armor_num:", "x:", "y:",
                  "z:", "vx:", "vy:", "vz:", "yaw:", "w:", "r:", "l:", "h:"})
                require(text_contains(panels.back(), label), "Requested track diagnostic missing");
            for (const auto *id : {"0", "3"})
                require(std::any_of(panels.back().texts.begin(), panels.back().texts.end(),
                                    [&](const auto &t) { return t.content == id; }),
                        "Model board ID missing");
            require(std::any_of(panels.back().lines.begin(), panels.back().lines.end(),
                                [](const auto &l) { return l.color.g == 255 && l.thickness == 3; }) &&
                        std::any_of(panels.back().lines.begin(), panels.back().lines.end(),
                                    [](const auto &l) { return l.color.r == 255 && l.color.g == 0; }),
                    "Selected and other boards lost their distinction");
            const AimPoint point{true, Eigen::Vector4d{3, 0, 0, 0}, true, 0, AimMode::CENTER};
            shoot_view.draw(image, ready, {}, point, pose.Get_R_gimbal2world());
            require(text_contains(panels.back(), "No Target"), "Warm-up model exposed as a shoot target");
            shoot_view.draw(image, tracker.target_state(), {true, .1f, .2f, 1}, point, pose.Get_R_gimbal2world());
            for (const auto *label : {"Gimbal", "Target", "Diff_yaw", "Diff_pitch", "CENTER", "Dist:", "ArmorID:"})
                require(text_contains(panels.back(), label), "Shoot diagnostic missing");
            Armor outpost            = observed;
            outpost.number           = "outpost";
            outpost.xyz_in_world.z() = .3;
            Target three_plates(outpost, time, .2765, .2765, 0, 3, Eigen::VectorXd::Constant(11, .01));
            require(std::abs(three_plates.armor_xyza_list()[0].z() - .3) < 1e-9, "Outpost M0 height changed");
            const auto labels = temporary.path / "small_labels.txt";
            for (const auto *name : {"1", "base"})
            {
                std::ifstream original(aim["armor_detector"]["number_params"]["label_path"].as<std::string>());
                std::ofstream mapped(labels);
                std::string line;
                while (std::getline(original, line))
                    mapped << name << '\n';
                mapped.close();
                NumberClassifier classifier(aim["armor_detector"]["number_params"]["model_path"].as<std::string>(),
                                            labels.string(), 0);
                Armor small(LightBar(cv::RotatedRect({120, 120}, {8, 40}, 0)),
                            LightBar(cv::RotatedRect({200, 120}, {8, 40}, 0)));
                require(classifier.classify(image, small) == NumberClassifier::RejectReason::PASS &&
                            small.number == name && small.number_img.size() == cv::Size(28, 28),
                        "Small armor class/ROI changed");
            }
            ArmorDetector detector(path.string());
            ArmorShoot shooter(path.string());
            panels.clear();
            require(ArmorDebug::enabled_views(config) == 0, "Disabled YAML flags enabled a view");
            auto empty        = detector.ArmorDetect(image);
            const auto absent = tracker.track(empty, time + std::chrono::milliseconds(20), image);
            shooter.shoot(absent, time, Eigen::Matrix3d::Identity(), time, image);
            require(panels.empty(), "Disabled module debug generated a panel");
            aim["armor_detector"]["debug"]          = true;
            aim["armor_track"]["armor_track_debug"] = true;
            aim["armor_track"]["armor_pose_debug"]  = true;
            aim["armor_shoot"]["debug"]             = true;
            std::ofstream(path) << config;
            ArmorDetector on_detector(path.string());
            ArmorTrack on_tracker(path.string(), (root / "config/hikcamera.yaml").string());
            ArmorShoot on_shooter(path.string());
            const auto mask = ArmorDebug::enabled_views(config);
            ArmorDebug::configure_layout(mask, false);
            require(mask == 15 && views.size() == 4, "YAML layout lost a requested view");
            panels.clear();
            empty             = on_detector.ArmorDetect(image);
            const auto target = on_tracker.track(empty, time, image);
            const auto packet = on_shooter.shoot(target, time, Eigen::Matrix3d::Identity(), time, image);
            static_assert(std::is_same_v<std::remove_cv_t<decltype(packet)>, SendPacket>);
            require(panels.size() == 4 && text_contains(panels[0], "Armors: 0") &&
                        text_contains(panels.back(), "No Target"),
                    "Internal direct debug did not draw each stage");
            panels.clear();
            on_detector.disable_debug();
            on_tracker.disable_debug();
            on_shooter.disable_debug();
            empty = on_detector.ArmorDetect(image);
            on_tracker.track(empty, time, image);
            on_shooter.shoot({}, time, Eigen::Matrix3d::Identity(), time, image);
            require(panels.empty(), "Replay display=false failed to suppress direct module drawing");
            TrackDebug::plot(three_plates);
            ShootDebug::plot({true, .1f, .2f, 1});
            const auto before_status = scalar_calls;
            ArmorDebug::unavailable(image, mask, "Pose unavailable");
            require(panels.size() == 3 && text_contains(panels.back(), "Pose unavailable"),
                    "Missing pose retained old panels");
            require(scalar_calls == before_status, "Base status display modified derived curve data");
            TrackDebug::clear();
            ShootDebug::clear();
            require(series.at("armor/track/valid") == 0 && std::isnan(series.at("armor/track/position/x_m")) &&
                        std::isnan(series.at("armor/track/covariance/0")),
                    "Track clear retained old model curves");
            require(series.at("armor/shoot/angles/yaw_rad") == 0 && series.at("armor/shoot/angles/pitch_rad") == 0 &&
                        series.at("armor/shoot/state/fire") == 0,
                    "Shoot clear retained old aim or fire state");
            detect_view.draw(image, armors, lights);
            DetectDebug::clear();
            require(series.at("armor/detect/count/armors") == 0 && series.at("armor/detect/count/lights") == 0,
                    "Inactive mode retained old detector counts");
            panels.clear();
            {
                static_assert(std::is_base_of_v<auto_buff::BuffDebug, auto_buff::DetectDebug>);
                static_assert(std::is_base_of_v<auto_buff::BuffDebug, auto_buff::TrackDebug>);
                static_assert(std::is_base_of_v<auto_buff::BuffDebug, auto_buff::ShootDebug>);
                struct Detector final : auto_buff::Detector
                {
                    Detector()
                    {
                        detect_debug_.set_enabled(true);
                    }
                    auto_buff::Detection detect_impl(const cv::Mat &, double) override
                    {
                        auto_buff::Detection result;
                        result.markers.push_back({{40, 50}, auto_buff::CampColor::RED, .9, true});
                        auto_buff::Board ring;
                        ring.color      = auto_buff::CampColor::RED;
                        ring.appearance = auto_buff::Appearance::single_ring;
                        result.boards.push_back(ring);
                        return result;
                    }
                } detector;
                const auto detection = detector.detect(image, 2, auto_buff::RuneMode::big);
                require(detection.boards.empty() && panels.size() == 1 && text_contains(panels.front(), "boards=0"),
                        "Detector drew before mode filtering");
                require(std::count_if(panels.front().texts.begin(), panels.front().texts.end(),
                                      [](const auto &text) { return text.image_overlay; }) == 3 &&
                            std::any_of(panels.front().texts.begin(), panels.front().texts.end(),
                                        [](const auto &text) {
                                            return text.content.starts_with("R: ") && !text.image_overlay;
                                        }),
                        "Target labels expanded the detect HUD background over the image");
                auto_buff::RmcsTracker rune(root / "config/hikcamera.yaml", YAML::Load("debug: true"));
                const auto track       = rune.update(detection, 2, image);
                const auto buff_config = temporary.path / "buff_shoot.yaml";
                std::ofstream(buff_config) << "auto_buff: {shoot: {debug: true}}\n";
                auto_buff::RuneShooter fire(buff_config);
                fire.shoot(track, image);
                require(panels.size() == 3 && panels[1].name == "track" && panels[2].name == "shoot",
                        "Algorithm-owned debug did not submit all three stage windows");
                const auto count = panels.size();
                detector.disable_debug();
                rune.disable_debug();
                fire.disable_debug();
                detector.detect(image, 3);
                rune.update(detection, 3, image);
                fire.shoot(track, image);
                require(panels.size() == count, "Disabled module debug still drew a frame");
                // 模型投影依特征 ID 连线，不按观测向量排列；替代已删除的视频标注检查。
            auto_buff::TrackResult overlay;
            overlay.initialized = true;
            overlay.predicted = {{3, {350, 420}}, {0, {480, 270}}, {5, {680, 210}},
                                 {2, {280, 210}}, {1, {480, 80}}, {4, {610, 420}}};
            auto_buff::TrackDebug track_view(true);
            track_view.draw(overlay, image);
            require(panels.back().lines.size() == 6 && panels.back().lines[0].x1 == 480 &&
                        panels.back().lines[0].y1 == 80 && panels.back().lines[0].x2 == 280 &&
                        panels.back().lines[0].y2 == 210, "Buff polygon did not follow feature IDs");
            overlay.predicted[0].feature_id = 2;
            track_view.draw(overlay, image);
            require(panels.back().lines.empty(), "Duplicate feature IDs produced a misleading polygon");
            auto_buff::TrackDebug::clear();
                auto_buff::ShootDebug::clear();
                require(series.at("track/ready") == 0 && std::isnan(series.at("track/ekf/x_m")) &&
                            series.at("shoot/should_shoot") == 0 && std::isnan(series.at("shoot/yaw_rad")),
                        "Invalid buff state retained old curves or firing advice");
            }
        });
}
