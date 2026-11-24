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
    /// 支持主工程 calibration/handeye_calibration 节点，以及原有平铺离线标定。
    explicit RuneCamera(const std::filesystem::path &config_path);

    const cv::Mat &intrinsic() const
    {
        return intrinsic_;
    }
    const cv::Mat &distortion() const
    {
        return distortion_;
    }
    const Eigen::Isometry3d &transform() const
    {
        return transform_;
    }

    /**
     * @brief 根据曝光时刻的 IMU 姿态更新相机到世界变换。
     * @return 转换后的云台到世界四元数，供本帧调试显示使用。
     * @note IMU 基变换、光学轴变换和手眼平移均在此处理，输入四元数顺序为 wxyz。
     */
    Eigen::Quaterniond set_imu_orientation(const Eigen::Quaterniond &orientation);

    /// 配置记录了标定宽高时核对输入；未记录时使用原内参，不自动缩放。
    void check_image_size(cv::Size size) const;

private:
    cv::Mat intrinsic_;  // 行主序相机内参。
    cv::Mat distortion_; // k1、k2、p1、p2、k3。
    cv::Size calibration_size_;

    Eigen::Isometry3d transform_        = Eigen::Isometry3d::Identity(); // 当前相机到世界变换。
    Eigen::Isometry3d camera_to_gimbal_ = Eigen::Isometry3d::Identity(); // 固定手眼外参。
    Eigen::Quaterniond gimbal_to_imu_   = Eigen::Quaterniond::Identity();

    bool dynamic_calibration_ = false; // 是否具备实时 IMU 所需的完整标定。
};
} // namespace auto_buff
