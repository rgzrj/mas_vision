#include "pose_debug.hpp"
#include "math_tools.hpp"

#include <cmath>
#include <spdlog/fmt/fmt.h>

namespace auto_aim
{
void PoseDebug::draw(const cv::Mat &bgr_img, const std::vector<Armor> &armors, const ArmorPose &pose)
{
    if (!enabled() || bgr_img.empty() || !rm_utils::Display::getInstance().image_due())
        return;

    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine> lines;
    std::vector<rm_utils::DisplayText> texts;

    int text_y = 50;

    // 同帧 IMU 姿态：由云台到世界的旋转矩阵提取 YPR（度）。
    const auto imu_ypr = rm_utils::eulers(pose.Get_R_gimbal2world(), 2, 1, 0);
    texts.push_back({fmt::format("  IMU YPR: [{:.2f}, {:.2f}, {:.2f}] deg",
                                rm_utils::rad2deg(imu_ypr.x()), rm_utils::rad2deg(imu_ypr.y()),
                                rm_utils::rad2deg(imu_ypr.z())),
                     20, text_y, 32, {255, 255, 0, 255}});
    text_y += 32;

    // 首块装甲板：坐标为米，YPR及YPD的前两项由弧度换算为度。
    if (!armors.empty())
    {
        const auto &armor = armors[0];
        const auto &gimbal_xyz = armor.xyz_in_gimbal;
        const auto &gimbal_ypr = armor.ypr_in_gimbal;
        const auto &world_xyz = armor.xyz_in_world;
        const auto &world_ypr = armor.ypr_in_world;
        const auto &world_ypd = armor.ypd_in_world;
        texts.push_back({fmt::format("  Gimbal XYZ: [{:.3f}, {:.3f}, {:.3f}] m",
                                    gimbal_xyz.x(), gimbal_xyz.y(), gimbal_xyz.z()), 20, text_y, 32});
        text_y += 32;
        texts.push_back({fmt::format("  Gimbal YPR: [{:.2f}, {:.2f}, {:.2f}] deg",
                                    rm_utils::rad2deg(gimbal_ypr.x()), rm_utils::rad2deg(gimbal_ypr.y()),
                                    rm_utils::rad2deg(gimbal_ypr.z())), 20, text_y, 32});
        text_y += 32;
        texts.push_back({fmt::format("  World XYZ: [{:.3f}, {:.3f}, {:.3f}] m",
                                    world_xyz.x(), world_xyz.y(), world_xyz.z()), 20, text_y, 32});
        text_y += 32;
        texts.push_back({fmt::format("  World YPR: [{:.2f}, {:.2f}, {:.2f}] deg",
                                    rm_utils::rad2deg(world_ypr.x()), rm_utils::rad2deg(world_ypr.y()),
                                    rm_utils::rad2deg(world_ypr.z())), 20, text_y, 32});
        text_y += 32;
        texts.push_back({fmt::format("  World YPD: [{:.2f}, {:.2f}, {:.3f}] deg",
                                    rm_utils::rad2deg(world_ypd.x()), rm_utils::rad2deg(world_ypd.y()),
                                    world_ypd.z()), 20, text_y, 32});
        text_y += 32;

        if (armor.points.size() == 4)
        {
            // 绘制检测到的装甲板轮廓（绿色）
            for (int j = 0; j < 4; j++)
            {
                rm_utils::DisplayLine line;
                line.x1        = static_cast<int>(armor.points[j].x);
                line.y1        = static_cast<int>(armor.points[j].y);
                line.x2        = static_cast<int>(armor.points[(j + 1) % 4].x);
                line.y2        = static_cast<int>(armor.points[(j + 1) % 4].y);
                line.color     = {0, 255, 0, 255}; // 绿色：检测结果
                line.thickness = 3;
                lines.push_back(line);
            }

            const auto &object_points = SMALL_ARMOR_POINTS;

            // 重投影链路：装甲板局部四角点 → 世界坐标 → 图像像素。
            // 姿态按 Z-Y-X 顺序组合：yaw、pitch、roll。
            double yaw   = armor.ypr_in_world.x();
            double pitch = armor.ypr_in_world.y();
            double roll  = armor.ypr_in_world.z();

            Eigen::Matrix3d R_armor2world;
            R_armor2world = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

            // 将物体坐标系的点转换到世界坐标系
            std::vector<cv::Point3f> world_corners;
            for (const auto &p_local : object_points)
            {
                // 转换为 Eigen 向量
                Eigen::Vector3d local_point(p_local.x, p_local.y, p_local.z);

                // 注意顺序：先旋转，后平移
                Eigen::Vector3d world_point = R_armor2world * local_point + armor.xyz_in_world;

                world_corners.push_back(cv::Point3f(world_point.x(), world_point.y(), world_point.z()));
            }

            std::vector<cv::Point2f> projected_points = pose.world2pixel(world_corners);

            // 绘制投影点
            if (!projected_points.empty())
            {
                for (int j = 0; j < 4; j++)
                {
                    rm_utils::DisplayLine line;
                    line.x1        = static_cast<int>(projected_points[j].x);
                    line.y1        = static_cast<int>(projected_points[j].y);
                    line.x2        = static_cast<int>(projected_points[(j + 1) % 4].x);
                    line.y2        = static_cast<int>(projected_points[(j + 1) % 4].y);
                    line.color     = {0, 0, 255, 255}; // world2pixel投影结果
                    line.thickness = 2;
                    lines.push_back(line);
                }

                // 计算四个对应角点的平均像素距离；小于 5 px 时显示绿色。
                double reprojection_error = 0;
                for (int j = 0; j < 4; j++)
                {
                    float dx = projected_points[j].x - armor.points[j].x;
                    float dy = projected_points[j].y - armor.points[j].y;
                    reprojection_error += std::sqrt(dx * dx + dy * dy);
                }
                reprojection_error /= 4.0;

                texts.push_back({fmt::format("  Reproj Error: {:.3f} px", reprojection_error),
                                 20, text_y, 32, reprojection_error < 5.0
                                     ? rm_utils::DisplayColor{0, 255, 0, 255}
                                     : rm_utils::DisplayColor{0, 0, 255, 255}});
                text_y += 32;
            }
        }
    }

    show(bgr_img, texts, points, lines);
}

} // namespace auto_aim
