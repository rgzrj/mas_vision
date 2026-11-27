#pragma once
#include <Eigen/Geometry>

namespace rm_utils::util {

// FLU（Forward-Left-Up）坐标系：X 向前、Y 向左、Z 向上。
// OpenCV 相机坐标系：X 向右、Y 向下、Z 向前。
inline const Eigen::Matrix3d R_opencv2flu {
    { +0, +0, +1 },
    { -1, +0, +0 },
    { +0, -1, +0 },
};

inline auto opencv2flu_position(const Eigen::Vector3d& position) -> Eigen::Vector3d {
    auto result = Eigen::Vector3d { position.z(), -position.x(), -position.y() };
    return result;
}
inline auto opencv2flu_rotation(const Eigen::Matrix3d& rotation_matrix) -> Eigen::Matrix3d {
    return R_opencv2flu * rotation_matrix * R_opencv2flu.transpose();
}
inline auto flu2opencv_position(const Eigen::Vector3d& position) -> Eigen::Vector3d {
    auto result = Eigen::Vector3d { -position.y(), -position.z(), position.x() };
    return result;
}
inline auto flu2opencv_rotation(const Eigen::Matrix3d& rotation_matrix) -> Eigen::Matrix3d {
    return R_opencv2flu.transpose() * rotation_matrix * R_opencv2flu;
}


} // namespace rm_utils::util
