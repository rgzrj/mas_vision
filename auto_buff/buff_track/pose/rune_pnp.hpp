#pragma once

#include <array>
#include <optional>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>

namespace auto_buff
{
struct RuneBullseye;

// 单次符面 PnP 求解器；输入符中心、R、四个端点，输出相机系位姿。
struct SingleRunePnpSolution
{
    /**
     * @brief 按 R 位置将符板端点排序为 R、上、左、下、右，供 PnP 和重投影检查共用。
     * @param board 符板观测；使用中心和四端点，不修改原始点序。
     * @param icon 本次与符板配对的 R 点，坐标为原图像素。
     * @return 排序后的五点；退化或两侧端点无法区分时为空。
     */
    static auto image_points(const RuneBullseye &board, cv::Point2d icon) -> std::optional<std::array<cv::Point2f, 5>>;

    // 结果使用内部 FLU 相机系：位置单位米，四元数将符面旋转到该相机系。
    struct Result
    {
        Eigen::Vector3d xyz_in_camera;
        Eigen::Quaterniond q_rune2camera;
    } result;

    /**
     * @brief 使用本次观测求解相机系位姿，标定参数仅在调用期间使用。
     * @param board 符板观测；直接复用公共观测，不另存输入副本。
     * @param icon 本次与符板配对的 R 点，坐标为原图像素。
     * @param camera_matrix 相机内参，3x3，CV_64F。
     * @param distort_coeffs 相机畸变系数，沿用 RuneCamera 加载的标定。
     * @return 成功时返回 true，位姿写入 result。
     */
    auto solve(const RuneBullseye &board, cv::Point2d icon, const cv::Mat &camera_matrix, const cv::Mat &distort_coeffs)
        -> bool;
};

} // namespace auto_buff
