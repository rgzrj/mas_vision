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
    const auto yaml           = YAML::LoadFile(config_path.string());
    const auto calibration    = yaml["calibration"];
    const auto handeye        = yaml["handeye_calibration"];
    const auto camera_matrix_data  = calibration["camera_matrix"].as<std::array<double, 9>>();
    const auto distort_coeffs_data = calibration["distort_coeffs"].as<std::array<double, 5>>();

    const bool invalid_intrinsic = camera_matrix_data[0] <= 0 || camera_matrix_data[4] <= 0 ||
                                   std::abs(camera_matrix_data[8] - 1) > 1e-8 ||
                                   std::abs(camera_matrix_data[1]) + std::abs(camera_matrix_data[3]) +
                                           std::abs(camera_matrix_data[6]) + std::abs(camera_matrix_data[7]) >
                                       1e-8;
    const bool invalid_values =
        !std::ranges::all_of(camera_matrix_data, [](double value) { return std::isfinite(value); }) ||
        !std::ranges::all_of(distort_coeffs_data, [](double value) { return std::isfinite(value); });

    if (invalid_intrinsic || invalid_values)
        throw std::runtime_error("Invalid rune pose camera calibration");

    camera_matrix_  = cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix_data.data())).clone();
    distort_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(distort_coeffs_data.data())).clone();

    const auto is_rotation = [](const Eigen::Matrix3d &matrix)
    {
        return matrix.allFinite() && (matrix.transpose() * matrix - Eigen::Matrix3d::Identity()).norm() <= 1e-3 &&
               std::abs(matrix.determinant() - 1) <= 1e-3;
    };

    const auto R_camera2gimbal_data  = handeye["R_camera2gimbal"].as<std::array<double, 9>>();
    const auto t_camera2gimbal_data  = handeye["t_camera2gimbal"].as<std::array<double, 3>>();
    const auto R_gimbal2imubody_data = handeye["R_gimbal2imubody"].as<std::array<double, 9>>();

    const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_camera2gimbal(R_camera2gimbal_data.data());
    const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());
    const Eigen::Vector3d t_camera2gimbal(t_camera2gimbal_data.data());

    if (!is_rotation(R_camera2gimbal))
        throw std::runtime_error("R_camera2gimbal must be a proper rotation matrix");
    if (!is_rotation(R_gimbal2imubody))
        throw std::runtime_error("R_gimbal2imubody must be a proper rotation matrix");
    if (!t_camera2gimbal.allFinite())
        throw std::runtime_error("Invalid t_camera2gimbal");

    // 标定使用光学轴，跟踪内部使用前/左/上；手眼平移已在云台系中，无需换轴。
    const Eigen::Quaterniond q_camera_flu2gimbal =
        Eigen::Quaterniond(R_camera2gimbal * rm_utils::util::R_opencv2flu.transpose()).normalized();
    T_camera2gimbal_.linear()      = q_camera_flu2gimbal.toRotationMatrix();
    T_camera2gimbal_.translation() = t_camera2gimbal;
    q_gimbal2imubody_              = Eigen::Quaterniond(R_gimbal2imubody);

    // 首帧 IMU 到来前以单位云台姿态初始化；离线固定机位也使用同一份手眼标定。
    T_camera2world_ = T_camera2gimbal_;
}

Eigen::Quaterniond RuneCamera::set_imu_orientation(const Eigen::Quaterniond &q_imubody2world)
{
    const auto norm = q_imubody2world.norm();
    if (!q_imubody2world.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-8)
        throw std::runtime_error("Invalid IMU quaternion");

    // 先从 IMU 本体系恢复云台姿态，再将手眼旋转和平移一起变换到世界系。
    const auto q_gimbal2world = rm_utils::util::gimbal_orientation(q_imubody2world.normalized(), q_gimbal2imubody_);
    const auto q_camera_flu2gimbal = Eigen::Quaterniond(T_camera2gimbal_.rotation());

    T_camera2world_.linear()      = (q_gimbal2world * q_camera_flu2gimbal).normalized().toRotationMatrix();
    T_camera2world_.translation() = q_gimbal2world * T_camera2gimbal_.translation();

    return q_gimbal2world;
}

} // namespace auto_buff
