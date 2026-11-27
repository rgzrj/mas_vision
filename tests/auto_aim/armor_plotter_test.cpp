#include "auto_aim/armor_track/armor_track.hpp"
#include "auto_aim/armor_shoot/armor_shoot.hpp"
#include "tests/common/test.hpp"
#include <yaml-cpp/yaml.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fstream>
#include <iostream>

int main()
{
    const int listener = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (listener < 0)
        return 1;
    sockaddr_in address{};
    address.sin_family      = AF_INET;
    address.sin_port        = htons(9870);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
    {
        ::close(listener);
        std::cout << "Plotter port 9870 is already in use; run without the monitor to test UDP output\n";
        return 77;
    }
    timeval timeout{1, 0};
    ::setsockopt(listener, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    const int result = tests::run(
        "armor_plotter_test",
        [&]
        {
            using tests::require;
            const std::filesystem::path root(BUFF_PROJECT_DIR);
            tests::TemporaryDirectory directory;
            auto config                                = YAML::LoadFile((root / "config/auto_aim.yaml").string());
            auto track                                 = config["auto_aim"]["armor_track"];
            track["debug"]                             = false;
            track["armor_track_debug"]                 = false;
            track["armor_pose_debug"]                  = false;
            track["plotter_enable"]                    = true;
            config["auto_aim"]["armor_shoot"]["debug"] = false;
            config["auto_aim"]["armor_shoot"]["plotter_enable"] = true;
            const auto path                                     = directory.path / "auto_aim.yaml";
            std::ofstream(path) << config;
            const auto receive = [&]
            {
                char buffer[65536];
                const auto count = ::recv(listener, buffer, sizeof(buffer), 0);
                require(count > 0, "No UDP diagnostic received with GUI disabled");
                return nlohmann::json::parse(buffer, buffer + count);
            };
            auto_aim::ArmorTrack tracker(path.string(), (root / "config/hikcamera.yaml").string());
            tracker.reset();
            tracker.armor_pose().set_R_gimbal2world(
                Eigen::Quaterniond(Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitZ())));
            const auto rotation = tracker.armor_pose().Get_R_gimbal2world();
            auto_aim::Armor observed{};
            observed.number = "3";
            observed.points = {{600, 480}, {660, 480}, {660, 520}, {600, 520}};
            observed.center = {630, 500};
            std::vector<auto_aim::Armor> armors{observed};
            const auto time   = std::chrono::steady_clock::now();
            const auto target = tracker.track(armors, time, cv::Mat::zeros(1024, 1280, CV_8UC3));
            require(!target, "Synthetic sample unexpectedly bypassed tracker warm-up");
            const auto packet = receive();
            require(packet["handeye"]["valid"].get<bool>(), "Raw hand-eye sample was gated on EKF readiness");
            require(packet.size() == 1 && packet.contains("handeye"),
                    "Track output contains non-calibration diagnostics");
            require(!packet["handeye"]["armor_big"].get<bool>(), "Small armor hand-eye sample marked as big");
            for (int i = 0; i < 4; ++i)
            {
                const auto &point = packet["handeye"]["p" + std::to_string(i)];
                require(point["x"] == observed.points[i].x && point["y"] == observed.points[i].y,
                        "Hand-eye raw corner changed");
            }
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    require(std::abs(packet["handeye"]["R_gimbal2world"]["r" + std::to_string(r) + std::to_string(c)]
                                         .get<double>() -
                                     rotation(r, c)) < 1e-12,
                            "Hand-eye rotation does not match the same-frame pose");
            armors.clear();
            tracker.track(armors, time + std::chrono::milliseconds(20), cv::Mat::zeros(1024, 1280, CV_8UC3));
            const auto missing = receive();
            require(!missing["handeye"]["valid"].get<bool>(), "Missing observation retained a valid hand-eye sample");
            for (int i = 0; i < 4; ++i)
            {
                const auto &point = missing["handeye"]["p" + std::to_string(i)];
                require(point["x"] == 0 && point["y"] == 0, "Missing observation retained old hand-eye corners");
            }
            require(packet["handeye"].size() == missing["handeye"].size(),
                    "Hand-eye output fields changed when observation was missing");
            observed.xyz_in_world = {3, 0, 0};
            observed.ypr_in_world = {0, 0, 0};
            auto_aim::Target moving(observed, time, .2, .2, 0, 4, Eigen::VectorXd::Constant(11, .01));
            auto_aim::ArmorShoot shooter(path.string());
            require(shooter.shoot(moving, time, Eigen::Matrix3d::Identity(), time).found,
                    "Synthetic shoot target invalid");
            const auto shoot = receive();
            require(shoot["SendPacket"]["found"] == 1 && shoot.contains("gimbal") && shoot.contains("distance") &&
                        shoot["aim_point"].contains("z"),
                    "Original shoot telemetry fields missing");
        });
    ::close(listener);
    return result;
}
