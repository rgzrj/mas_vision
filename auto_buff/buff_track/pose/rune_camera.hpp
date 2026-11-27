#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <filesystem>

namespace auto_buff
{
/**
 * @brief 位姿模块持有的相机几何；构造时读取内参、畸变和手眼标定。
 * @note PnP、EKF 投影和未来投影共用这些参数；不负责设备取帧，不逐帧读取 YAML。
 */
class RuneCamera
{
public:
    /// 读取主工程 calibration 和完整 handeye_calibration；实机与离线共用此格式。
    explicit RuneCamera(const std::filesystem::path &config_path);

    const cv::Mat &camera_matrix() const
    {
        return camera_matrix_;
    }
    const cv::Mat &distort_coeffs() const
    {
        return distort_coeffs_;
    }
    // 相机到世界 (odom) 的位姿
    const Eigen::Isometry3d &camera_to_world() const
    {
        return T_camera2world_;
    }

    /**
     * @brief 根据曝光时刻的 IMU 姿态更新相机到世界变换。
     * @return 转换后的云台到世界四元数，供本帧调试显示使用。
     * @note IMU 基变换、光学轴变换和手眼平移均在此处理，输入四元数顺序为 wxyz。
     */
    Eigen::Quaterniond set_imu_orientation(const Eigen::Quaterniond &q_imubody2world);

private:
    cv::Mat camera_matrix_;  // 行主序相机内参。
    cv::Mat distort_coeffs_; // k1、k2、p1、p2、k3。

    Eigen::Isometry3d T_camera2world_    = Eigen::Isometry3d::Identity(); // 当前相机到世界变换。
    Eigen::Isometry3d T_camera2gimbal_   = Eigen::Isometry3d::Identity(); // 内部 FLU 相机到云台的固定变换。
    Eigen::Quaterniond q_gimbal2imubody_ = Eigen::Quaterniond::Identity();
};
} // namespace auto_buff
