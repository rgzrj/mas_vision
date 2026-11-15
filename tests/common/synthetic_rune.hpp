#pragma once

#include "auto_buff/buff_detect/detection.hpp"
#include "rm_utils/algorithm/conversion.hpp"
#include "rm_utils/clock.hpp"
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <array>
#include <numbers>

namespace tests {
inline constexpr std::array<double, 9> intrinsics{900, 0, 480, 0, 900, 270, 0, 0, 1};
inline rm_utils::Timestamp stamp(double seconds) {
    return rm_utils::Timestamp{} +
           std::chrono::duration_cast<rm_utils::Duration>(std::chrono::duration<double>(seconds));
}

// 已知符中心 (5,0,0) m、半径 0.7 m、角速度 0.8 rad/s；直接投影生成观测，不经过检测器。
inline auto_buff::Detection
synthetic(double time, const std::array<double, 9> &intrinsics,
          const Eigen::Quaterniond &camera_orientation = Eigen::Quaterniond::Identity(),
          const Eigen::Vector3d &camera_translation = Eigen::Vector3d::Zero()) {
    auto_buff::Detection result;
    cv::Mat k(3, 3, CV_64F, const_cast<double *>(intrinsics.data()));
    auto project = [&](const Eigen::Vector3d &local) {
        Eigen::Vector3d camera =
            camera_orientation.conjugate() * (Eigen::Vector3d(5, 0, 0) + local - camera_translation);
        auto ocv = rm_utils::util::ros2opencv_position(camera);
        std::vector<cv::Point2d> projected;
        cv::projectPoints(std::vector<cv::Point3d>{{ocv.x(), ocv.y(), ocv.z()}}, cv::Vec3d{}, cv::Vec3d{}, k,
                          cv::Mat(), projected);
        return projected[0];
    };
    auto r = project(Eigen::Vector3d(-.1, 0, 0));
    result.markers.push_back(auto_buff::Marker{r, auto_buff::CampColor::RED, 1, true});
    for (int slot = 0; slot < 5; ++slot) {
        double theta = .3 + .8 * time + slot * 2 * std::numbers::pi / 5;
        Eigen::AngleAxisd rotate(theta, Eigen::Vector3d::UnitX());
        auto_buff::Board board;
        board.center = project(rotate * Eigen::Vector3d(0, 0, .7));
        board.radius = 27;
        board.color = auto_buff::CampColor::RED;
        board.features_valid = true;
        board.pvd = .8;
        board.appearance = slot == 0 ? auto_buff::Appearance::full_mark : auto_buff::Appearance::single_ring;
        std::array<Eigen::Vector3d, 4> endpoints{
            Eigen::Vector3d(0, 0, .85), {0, .15, .7}, {0, 0, .55}, {0, -.15, .7}};
        for (int i = 0; i < 4; ++i)
            board.corners[i] = project(rotate * endpoints[i]);
        result.boards.push_back(board);
    }
    return result;
}

} // namespace tests
