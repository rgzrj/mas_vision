#include "auto_aim/armor_track/armor_pose.hpp"
#include "tests/common/test.hpp"

#include <array>
#include <fstream>
#include <limits>
#include <yaml-cpp/yaml.h>

int main()
{
    return tests::run("armor_pose_test", []
    {
        using tests::require;
        const std::filesystem::path root(BUFF_PROJECT_DIR);
        tests::TemporaryDirectory temporary;
        const auto path = temporary.path / "hikcamera.yaml";
        const auto config = YAML::LoadFile((root / "config/hikcamera.yaml").string());
        const auto require_failure = [&]
        {
            bool rejected = false;
            try
            {
                auto_aim::ArmorPose pose(path.string());
            }
            catch (const std::exception &)
            {
                rejected = true;
            }
            require(rejected, "Invalid pose calibration was silently accepted");
        };

        // 缺文件、损坏 YAML、缺配置段都不能产生半初始化对象。
        require_failure();
        for (const auto *text : {"calibration: [", "{}", "calibration: {}", "handeye_calibration: {}"})
        {
            std::ofstream(path) << text;
            require_failure();
        }
        struct Field
        {
            const char *group;
            const char *name;
            std::size_t size;
        };
        const Field fields[] = {
            {"handeye_calibration", "R_gimbal2imubody", 9},
            {"handeye_calibration", "R_camera2gimbal", 9},
            {"handeye_calibration", "t_camera2gimbal", 3},
            {"calibration", "camera_matrix", 9},
            {"calibration", "distort_coeffs", 5},
        };
        for (const auto &field : fields)
        {
            auto invalid = YAML::Clone(config);
            invalid[field.group].remove(field.name);
            std::ofstream(path) << invalid;
            require_failure();
            for (const auto size : {std::size_t{0}, field.size - 1, field.size + 1})
            {
                invalid[field.group][field.name] = std::vector<double>(size, 0);
                std::ofstream(path) << invalid;
                require_failure();
            }
            for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                                       std::numeric_limits<double>::infinity()})
            {
                invalid = YAML::Clone(config);
                invalid[field.group][field.name][0] = value;
                std::ofstream(path) << invalid;
                require_failure();
            }
        }
        // 缩放矩阵和镜像矩阵都不是有效旋转，不能带入坐标变换。
        for (const auto *name : {"R_gimbal2imubody", "R_camera2gimbal"})
        {
            for (const double value : {0.0, -1.0, 2.0})
            {
                auto invalid = YAML::Clone(config);
                invalid["handeye_calibration"][name] = std::array<double, 9>{value, 0, 0, 0, 1, 0, 0, 0, 1};
                std::ofstream(path) << invalid;
                require_failure();
            }
        }
        for (const int index : {0, 4, 8})
        {
            auto invalid = YAML::Clone(config);
            invalid["calibration"]["camera_matrix"][index] = 0;
            std::ofstream(path) << invalid;
            require_failure();
        }
        for (const int index : {1, 3, 6, 7})
        {
            auto invalid = YAML::Clone(config);
            invalid["calibration"]["camera_matrix"][index] = 1;
            std::ofstream(path) << invalid;
            require_failure();
        }

        // 正常标定不改变行列顺序、初始姿态和手眼平移。
        std::ofstream(path) << config;
        auto_aim::ArmorPose pose(path.string());
        require(pose.Get_R_gimbal2world().isIdentity(), "Initial world rotation changed");
        const auto basis_values = config["handeye_calibration"]["R_gimbal2imubody"].as<std::array<double, 9>>();
        const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> basis(basis_values.data());
        const auto translation_values = config["handeye_calibration"]["t_camera2gimbal"].as<std::array<double, 3>>();
        const Eigen::Vector3d translation(translation_values.data());
        const Eigen::Quaterniond orientation(Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitZ()));
        pose.set_R_gimbal2world(orientation);
        const Eigen::Matrix3d rotation = basis.transpose() * orientation.toRotationMatrix() * basis;
        require((pose.Get_R_gimbal2world() - rotation).norm() < 1e-12, "IMU basis changed on loading");
        require((pose.cameraCenterInWorld() - rotation * translation).norm() < 1e-12,
                "Handeye translation changed on loading");

        auto zero_distortion = YAML::Clone(config);
        zero_distortion["calibration"]["distort_coeffs"] = std::array<double, 5>{};
        std::ofstream(path) << zero_distortion;
        auto_aim::ArmorPose undistorted(path.string());
        require(undistorted.Get_R_gimbal2world().isIdentity(), "Zero distortion was rejected");
    });
}
