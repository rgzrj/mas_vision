#include "rune_projection.hpp"

#include "auto_buff/buff_types.hpp"
#include "rm_utils/algorithm/conversion.hpp"
#include "rm_utils/algorithm/math_tools.hpp"
#include "rm_utils/algorithm/reprojection.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace auto_buff
{
auto RuneObservable::update(const RuneStateVector &state) noexcept -> void
{
    constexpr double kMinDepth = 0.1;

    const auto angle    = state[kA];
    const auto face_yaw = state[kPsi];
    const auto center   = Eigen::Vector3d(state[kX], state[kY], state[kZ]);
    const auto R_rune2world = Eigen::AngleAxisd{face_yaw, Eigen::Vector3d::UnitZ()}.toRotationMatrix();

    // 投影使用相机到世界变换的逆变换，得到内部 FLU 相机坐标。
    const auto q_camera2world = Eigen::Quaterniond(T_camera2world.rotation());
    const auto q_world2camera = q_camera2world.conjugate();
    const auto t_camera2world = Eigen::Vector3d(T_camera2world.translation());

    static constexpr std::array kBladeDeg = {0.0, 72.0, 144.0, 216.0, 288.0};

    for (int i = 0; i < kFeatureCount; ++i)
    {
        Eigen::Vector3d local;
        // R 沿符面法线突出 0.1 m；五个板心则在半径 0.7 m 的圆周上等间隔分布。
        if (i == kFeatureIcon)
        {
            local = Eigen::Vector3d(-kRuneIconProminentDistance, 0.0, 0.0);
        }
        else
        {
            const auto alpha = angle + rm_utils::deg2rad(kBladeDeg[i - 1]);
            local = Eigen::Vector3d(0.0, -kRuneGlobalRadius * std::sin(alpha), kRuneGlobalRadius * std::cos(alpha));
        }

        const Eigen::Vector3d world  = center + R_rune2world * local;
        const Eigen::Vector3d camera = q_world2camera * (world - t_camera2world);

        points_in_camera[i] = camera;

        // 先从前/左/上转换为 OpenCV 右/下/前，再应用内参与畸变。
        // 无效点显式标为不可见，供后续关联跳过，不把 (0,0) 当作观测。
        const auto point_ocv     = rm_utils::util::flu2opencv_position(camera);
        const auto projected_opt = rm_utils::util::reproject_point(point_ocv, camera_matrix, distort_coeffs);
        if (point_ocv.z() > kMinDepth && projected_opt)
        {
            points_in_image[i] = Eigen::Vector2d{projected_opt->x, projected_opt->y};
            visible[i]   = true;
        }
        else
        {
            points_in_image[i].setZero();
            visible[i] = false;
        }
    }
}

} // namespace auto_buff
