#include "rune_projection.hpp"

#include "rm_utils/algorithm/math_tools.hpp"
#include "rm_utils/algorithm/conversion.hpp"
#include "rm_utils/algorithm/reprojection.hpp"
#include "auto_buff/buff_types.hpp"

#include <Eigen/Geometry>
#include <cmath>

using namespace rm_utils;
using namespace auto_buff;

auto auto_buff::RuneObservable::update(const RuneStateVector &state) noexcept -> void
{
    const auto theta  = state[kA];
    const auto psi    = state[kPsi];
    const auto center = Eigen::Vector3d(state[kX], state[kY], state[kZ]);
    const auto r_face = Eigen::AngleAxisd{psi, Eigen::Vector3d::UnitZ()}.toRotationMatrix();

    // feature 保存相机到世界的变换；投影需要使用其逆变换得到相机坐标。
    const auto q_odom_from_cam = Eigen::Quaterniond(camera_to_world.rotation());
    const auto q_cam_from_odom = q_odom_from_cam.conjugate();
    const auto cam_position    = Eigen::Vector3d(camera_to_world.translation());

    static constexpr std::array kBladeDeg = {0.0, 72.0, 144.0, 216.0, 288.0};

    for (int i = 0; i < kFeatureCount; ++i)
    {
        Eigen::Vector3d local;
        // R 沿符面法线突出 0.1 m；五个板心则在半径 0.7 m 的圆周上等间隔分布。
        if (i == kFeatureIcon)
        {
            local = Eigen::Vector3d(-auto_buff::kRuneIconProminentDistance, 0, 0);
        }
        else
        {
            const auto alpha = theta + rm_utils::deg2rad(kBladeDeg[i - 1]);
            local << 0, -auto_buff::kRuneGlobalRadius * std::sin(alpha), auto_buff::kRuneGlobalRadius * std::cos(alpha);
        }

        Eigen::Vector3d world  = center + r_face * local;
        Eigen::Vector3d camera = q_cam_from_odom * (world - cam_position);

        in_camera[i] = camera;

        // 先从前/左/上转换为 OpenCV 右/下/前，再应用内参与畸变。
        // 无效点显式标为不可见，供后续关联跳过，不把 (0,0) 当作观测。
        constexpr auto kMinZ     = 0.1;
        const auto point_ocv     = rm_utils::util::ros2opencv_position(camera);
        const auto projected_opt = rm_utils::util::reproject_point(point_ocv, intrinsic, distortion);
        if (point_ocv.z() > kMinZ && projected_opt)
        {
            projected[i] = Eigen::Vector2d{projected_opt->x, projected_opt->y};
            visible[i]   = true;
        }
        else
        {
            projected[i].setZero();
            visible[i] = false;
        }
    }
}
