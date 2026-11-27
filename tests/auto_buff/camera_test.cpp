#include "auto_buff/buff_track/pose/rune_camera.hpp"
#include "tests/common/test.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>
#include <numbers>

int main()
{
    return tests::run("buff_camera_test", []
    {
        using tests::require;
        tests::TemporaryDirectory directory;
        const auto path = directory.path / "hikcamera.yaml";
        const auto config = YAML::Load(
            "calibration: {camera_matrix: [900,0,480,0,900,270,0,0,1], distort_coeffs: [0,0,0,0,0]}\n"
            "handeye_calibration: {R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0], "
            "t_camera2gimbal: [0.1,0.2,0.3], R_gimbal2imubody: [0,1,0,-1,0,0,0,0,1]}\n");
        const auto write = [&](const YAML::Node &value) { std::ofstream(path) << value; };
        write(config);

        auto_buff::RuneCamera camera(path);
        const Eigen::Vector3d offset(.1, .2, .3);
        require(camera.camera_matrix().at<double>(0, 0) == 900 && cv::norm(camera.distort_coeffs()) == 0,
                "Main calibration fields were not loaded");
        require((camera.camera_to_world().rotation() - Eigen::Matrix3d::Identity()).norm() < 1e-12 &&
                    (camera.camera_to_world().translation() - offset).norm() < 1e-12,
                "Initial optical-to-FLU handeye transform changed");

        const Eigen::Quaterniond basis(Eigen::AngleAxisd(-std::numbers::pi / 2, Eigen::Vector3d::UnitZ()));
        for (int i = 0; i <= 225; ++i)
        {
            const double t = i / 50.;
            const Eigen::Quaterniond gimbal = Eigen::AngleAxisd(.12 * std::sin(t), Eigen::Vector3d::UnitZ()) *
                                              Eigen::AngleAxisd(.04 * std::cos(t), Eigen::Vector3d::UnitY());
            const auto actual = camera.set_imu_orientation(basis * gimbal * basis.conjugate());
            require(actual.angularDistance(gimbal) < 1e-12 &&
                        (camera.camera_to_world().rotation() - gimbal.toRotationMatrix()).norm() < 1e-12 &&
                        (camera.camera_to_world().translation() - gimbal * offset).norm() < 1e-12,
                    "Exposure IMU did not rotate both camera orientation and handeye offset");
        }

        const auto rejects = [&](const YAML::Node &value)
        {
            write(value);
            try { auto_buff::RuneCamera invalid(path); }
            catch (const std::exception &) { return true; }
            return false;
        };
        auto missing = YAML::Clone(config);
        missing.remove("handeye_calibration");
        require(rejects(missing), "Missing handeye was silently accepted");
        missing = YAML::Clone(config);
        missing["handeye_calibration"].remove("R_gimbal2imubody");
        require(rejects(missing), "Missing IMU basis was silently accepted");
        require(rejects(YAML::Load("camera_matrix: [900,0,480,0,900,270,0,0,1]\ndistortion: [0,0,0,0,0]")),
                "Legacy flat camera format was still accepted");
        auto invalid = YAML::Clone(config);
        invalid["handeye_calibration"]["R_camera2gimbal"][0] = 2;
        require(rejects(invalid), "Invalid handeye rotation was accepted");
        invalid = YAML::Clone(config);
        invalid["calibration"]["camera_matrix"][0] = -900;
        require(rejects(invalid), "Negative focal length was accepted");

        bool rejected = false;
        try { camera.set_imu_orientation(Eigen::Quaterniond(0, 0, 0, 0)); }
        catch (const std::exception &) { rejected = true; }
        require(rejected, "Zero IMU quaternion was accepted");
    });
}
