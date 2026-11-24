#include "rune_camera.hpp"

#include "rm_utils/algorithm/conversion.hpp"
#include "rm_utils/algorithm/orientation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace auto_buff
{
RuneCamera::RuneCamera(const std::filesystem::path &config_path)
{
    // 标定在 pose 模块初始化时直接读取，PnP 和跟踪投影共用。
    const auto yaml             = YAML::LoadFile(config_path.string());
    const auto calibration      = yaml["calibration"] ? yaml["calibration"] : yaml;
    const auto intrinsic_values = calibration["camera_matrix"].as<std::array<double, 9>>();
    const auto distortion_values =
        calibration[yaml["calibration"] ? "distort_coeffs" : "distortion"].as<std::array<double, 5>>();
    calibration_size_ = {calibration["image_width"].as<int>(0), calibration["image_height"].as<int>(0)};

    // 主工程的标定 YAML 不含宽高；未填写时按原内参使用，不猜测分辨率或缩放内参。
    // 已填写则要求宽高成对有效，并在 update 时核对原图尺寸。
    const bool invalid_size      = calibration_size_ != cv::Size{} && calibration_size_.empty();
    const bool invalid_intrinsic = intrinsic_values[0] <= 0 || intrinsic_values[4] <= 0 ||
                                   std::abs(intrinsic_values[8] - 1) > 1e-8 ||
                                   std::abs(intrinsic_values[1]) + std::abs(intrinsic_values[3]) +
                                           std::abs(intrinsic_values[6]) + std::abs(intrinsic_values[7]) >
                                       1e-8;
    const bool invalid_values =
        !std::ranges::all_of(intrinsic_values, [](double value) { return std::isfinite(value); }) ||
        !std::ranges::all_of(distortion_values, [](double value) { return std::isfinite(value); });

    if (invalid_size || invalid_intrinsic || invalid_values)
        throw std::runtime_error("Invalid rune pose camera calibration");

    intrinsic_  = cv::Mat(3, 3, CV_64F, const_cast<double *>(intrinsic_values.data())).clone();
    distortion_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(distortion_values.data())).clone();

    // 四元数字段仅用于固定相机离线配置，顺序为 w/x/y/z。
    const auto read_quaternion = [](const YAML::Node &node)
    {
        const auto values     = node.as<std::array<double, 4>>();
        const auto quaternion = Eigen::Quaterniond(values[0], values[1], values[2], values[3]);

        if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-8)
            throw std::runtime_error("Invalid camera quaternion");

        return quaternion.normalized();
    };

    const auto is_rotation = [](const Eigen::Matrix3d &matrix)
    {
        return matrix.allFinite() && (matrix.transpose() * matrix - Eigen::Matrix3d::Identity()).norm() <= 1e-3 &&
               std::abs(matrix.determinant() - 1) <= 1e-3;
    };

    const auto handeye = yaml["handeye_calibration"];
    if (handeye)
    {
        if (yaml["camera_to_world_quaternion_wxyz"] || yaml["camera_to_world_translation_m"])
            throw std::runtime_error("Use handeye_calibration OR camera_to_world fields, not both");

        const auto rotation_values = handeye["R_camera2gimbal"].as<std::array<double, 9>>();
        const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation(rotation_values.data());
        if (!is_rotation(rotation))
            throw std::runtime_error("R_camera2gimbal must be a proper rotation matrix");

        const auto translation_values = handeye["t_camera2gimbal"].as<std::array<double, 3>>();
        const Eigen::Vector3d translation(translation_values[0], translation_values[1], translation_values[2]);
        if (!translation.allFinite())
            throw std::runtime_error("Invalid t_camera2gimbal");

        // 内部相机轴为 RMCS 的前/左/上，手眼标定输入为 OpenCV 的右/下/前。
        const Eigen::Quaterniond camera_to_gimbal =
            Eigen::Quaterniond(rotation * rm_utils::util::kCoordTransformMatrix.transpose()).normalized();
        camera_to_gimbal_.linear()      = camera_to_gimbal.toRotationMatrix();
        camera_to_gimbal_.translation() = translation;

        const auto gimbal_to_world = yaml["fixed_gimbal_to_world_quaternion_wxyz"]
                                         ? read_quaternion(yaml["fixed_gimbal_to_world_quaternion_wxyz"])
                                         : Eigen::Quaterniond::Identity();
        transform_.linear()        = (gimbal_to_world * camera_to_gimbal).normalized().toRotationMatrix();
        transform_.translation()   = gimbal_to_world * translation;

        if (handeye["R_gimbal2imubody"])
        {
            const auto basis_values = handeye["R_gimbal2imubody"].as<std::array<double, 9>>();
            const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> basis(basis_values.data());
            if (!is_rotation(basis))
                throw std::runtime_error("R_gimbal2imubody must be a proper rotation matrix");

            gimbal_to_imu_       = Eigen::Quaterniond(basis);
            dynamic_calibration_ = true;
        }
    }
    else
    {
        if (yaml["camera_to_world_quaternion_wxyz"])
            transform_.linear() = read_quaternion(yaml["camera_to_world_quaternion_wxyz"]).toRotationMatrix();

        if (yaml["camera_to_world_translation_m"])
        {
            const auto values = yaml["camera_to_world_translation_m"].as<std::array<double, 3>>();
            const Eigen::Vector3d translation(values[0], values[1], values[2]);
            if (!translation.allFinite())
                throw std::runtime_error("Invalid camera_to_world_translation_m");

            transform_.translation() = translation;
        }
    }
}

Eigen::Quaterniond RuneCamera::set_imu_orientation(const Eigen::Quaterniond &orientation)
{
    if (!dynamic_calibration_)
        throw std::runtime_error("Dynamic pose requires R_camera2gimbal, t_camera2gimbal and R_gimbal2imubody");

    const auto norm = orientation.norm();
    if (!orientation.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-8)
        throw std::runtime_error("Invalid IMU quaternion");

    // 先从 IMU 本体系恢复云台姿态，再将手眼旋转和平移一起变换到世界系。
    const auto gimbal = rm_utils::util::gimbal_orientation(orientation.normalized(), gimbal_to_imu_);
    const auto camera = Eigen::Quaterniond(camera_to_gimbal_.rotation());

    transform_.linear()      = (gimbal * camera).normalized().toRotationMatrix();
    transform_.translation() = gimbal * camera_to_gimbal_.translation();

    return gimbal;
}

void RuneCamera::check_image_size(cv::Size size) const
{
    if (calibration_size_ != cv::Size{} && size != calibration_size_)
        throw std::runtime_error("Input resolution does not match camera.yaml; "
                                 "calibrate/scale intrinsics explicitly");
}
} // namespace auto_buff
