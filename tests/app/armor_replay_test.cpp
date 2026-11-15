#include "apps/replay/replay.hpp"
#include "apps/replay/armor_process.hpp"
#include <thread>
#include "rm_utils/io/paired_frame.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "tests/common/test.hpp"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <fstream>
#include <iostream>
std::atomic<bool> g_shutdown{false};

int main()
{
    return tests::run(
        "armor_replay_test",
        []
        {
            namespace fs        = std::filesystem;
            const fs::path root = tool::from_utf8(BUFF_PROJECT_DIR);
            tests::TemporaryDirectory temporary;
            const auto input = temporary.path / "images";
            fs::create_directory(input);
            auto algorithm  = YAML::LoadFile((root / "config/auto_aim.yaml").string());
            auto parameters = algorithm["auto_aim"];
            for (const auto *name : {"armor_detector", "armor_track", "armor_shoot"})
                parameters[name]["debug"] = false;
            auto numbers = parameters["armor_detector"]["number_params"];
            for (const auto *name : {"model_path", "label_path"})
                numbers[name] =
                    tool::utf8(fs::absolute(root / "config" / tool::from_utf8(numbers[name].as<std::string>())));
            std::ofstream(temporary.path / "auto_aim.yaml") << algorithm;
            fs::copy_file(root / "config/hikcamera.yaml", temporary.path / "hikcamera.yaml");
            // 用有速度的合成目标检验离线火控不读取播放主机时间。
            app::ArmorModules modules(temporary.path / "auto_aim.yaml");
            auto_aim::Armor observed;
            observed.number       = "3";
            observed.xyz_in_world = {3, 0, 0};
            observed.ypr_in_world = {0, 0, 0};
            const auto time       = std::chrono::steady_clock::time_point(std::chrono::seconds(1));
            auto_aim::Target moving(observed, time, .2, .2, 0, 4, Eigen::VectorXd::Constant(11, .01));
            // 仅设置测试目标速度；不为测试给生产类添加可写接口。
            const_cast<rm_utils::ExtendedKalmanFilter &>(moving.ekf()).x[3] = 1;
            const auto first = modules.shooter.shoot(moving, time, Eigen::Matrix3d::Identity(), time);
            modules.shooter.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            const auto second = modules.shooter.shoot(moving, time, Eigen::Matrix3d::Identity(), time);
            tests::require(first.found && second.found && first.target_yaw == second.target_yaw &&
                               first.target_pitch == second.target_pitch,
                           "Offline armor fire-control depends on host playback time");
            // 中速、没有可见候选时，复制的火控仍应使用默认板，不能读取空列表。
            observed.ypr_in_world[0] = CV_PI;
            auto_aim::Target hidden(observed, time, .2, .2, 0, 1, Eigen::VectorXd::Constant(11, .01));
            hidden.jumped = true;
            const_cast<rm_utils::ExtendedKalmanFilter &>(hidden.ekf()).x[7] = 4;
            modules.shooter.reset();
            const auto fallback = modules.shooter.shoot(hidden, time, Eigen::Matrix3d::Identity(), time);
            tests::require(fallback.found && modules.shooter.debug_aim_point.valid &&
                               modules.shooter.debug_aim_point.mode == auto_aim::AimMode::COMING,
                           "Copied shooter did not handle an empty visible-candidate list");
            modules.shooter.reset();
            const auto lost = modules.shooter.shoot(std::nullopt, time, Eigen::Matrix3d::Identity(), time);
            tests::require(!lost.found && !lost.fire_advice && !modules.shooter.debug_aim_point.valid,
                           "Copied shooter retained a valid target after reset");
            std::ofstream csv(temporary.path / "frames.csv");
            tool::write_metadata_header(csv);
            for (int i = 0; i < 3; ++i)
            {
                tool::write_image(input / (std::to_string(i) + ".png"), cv::Mat::zeros(64, 96, CV_8UC3));
                tool::RecordedFrame frame;
                frame.index        = i;
                frame.timestamp_ns = 1000000000LL + i * 20000000LL;
                frame.t_recv_ns    = frame.timestamp_ns + 1000000;
                frame.mode         = i == 0 ? 0 : (i == 1 ? 2 : 1);
                if (i < 2)
                    frame.pose = Eigen::Quaterniond::Identity();
                tool::write_frame_metadata(csv, frame);
            }
            csv.close();
            YAML::Node config;
            config["pipeline"]                 = "armor";
            config["input"]["type"]            = "paired";
            config["input"]["path"]            = "images";
            config["input"]["frames_csv"]      = "frames.csv";
            config["algorithm_config"]         = "auto_aim.yaml";
            config["output"]["directory"]      = "paired_output";
            config["playback"]["speed"]        = 0;
            config["playback"]["display"]      = false;
            const auto run                     = [&]
            {
                const auto path = temporary.path / "replay.yaml";
                std::ofstream(path) << config;
                tests::require(app::run_replay(path) == 0, "Armor replay failed");
                fs::path summary;
                for (const auto &entry :
                     fs::directory_iterator(temporary.path / config["output"]["directory"].as<std::string>()))
                    if (entry.is_directory())
                        summary = entry.path() / "summary.yaml";
                return YAML::LoadFile(summary.string());
            };
            const auto paired = run();
            tests::require(paired["frames"].as<int>() == 3 && paired["mode_inactive_frames"].as<int>() == 1 &&
                               paired["missing_pose_frames"].as<int>() == 1,
                           "Armor mode or missing-pose handling changed");
            config["input"]["type"]       = "media";
            config["input"]["media_type"] = "image";
            config["input"]["path"]       = "images/0.png";
            config["output"]["directory"] = "media_output";
            const auto media              = run();
            tests::require(media["frames"].as<int>() == 1 && media["mode_inactive_frames"].as<int>() == 0,
                           "Armor media input incorrectly required paired mode/pose");
        });
}
