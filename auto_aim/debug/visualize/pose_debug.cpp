#include "pose_debug.hpp"
#include "math_tools.hpp"

#include <cmath>

namespace auto_aim
{
void PoseDebug::draw(const cv::Mat &bgr_img, const std::vector<Armor> &armors, const ArmorPose &pose)
{
    if (!enabled() || bgr_img.empty())
        return;

    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine> lines;
    std::vector<rm_utils::DisplayText> texts;

    int text_y = 50;

    // 同帧 IMU 姿态：由云台到世界的旋转矩阵提取 YPR（度）。
    Eigen::Matrix3d R_gimbal2world = pose.Get_R_gimbal2world();
    Eigen::Vector3d imu_ypr        = rm_utils::eulers(R_gimbal2world, 2, 1, 0);
    rm_utils::DisplayText imu_text;
    imu_text.content = "  IMU YPR: [" + std::to_string(imu_ypr.x() * 57.3f).substr(0, 5) + ", " +
                       std::to_string(imu_ypr.y() * 57.3f).substr(0, 5) + ", " +
                       std::to_string(imu_ypr.z() * 57.3f).substr(0, 5) + "] deg";
    imu_text.x         = 20;
    imu_text.y         = text_y;
    imu_text.size      = 32;
    imu_text.color     = {255, 255, 0, 0};
    imu_text.thickness = 1;
    texts.push_back(imu_text);
    text_y += imu_text.size;

    // 首块装甲板：云台/世界坐标（米），YPR（度）和 YPD（前两项为度，距离为米）。
    if (!armors.empty())
    {
        const auto &armor = armors[0];
        rm_utils::DisplayText gimbal_xyz_text;
        gimbal_xyz_text.content = "  Gimbal XYZ: [" + std::to_string(armor.xyz_in_gimbal.x()).substr(0, 5) + ", " +
                                  std::to_string(armor.xyz_in_gimbal.y()).substr(0, 5) + ", " +
                                  std::to_string(armor.xyz_in_gimbal.z()).substr(0, 5) + "] m";
        gimbal_xyz_text.x         = 20;
        gimbal_xyz_text.y         = text_y;
        gimbal_xyz_text.size      = 32;
        gimbal_xyz_text.color     = {255, 255, 255, 255};
        gimbal_xyz_text.thickness = 1;
        texts.push_back(gimbal_xyz_text);
        text_y += gimbal_xyz_text.size;

        rm_utils::DisplayText gimbal_ypr_text;
        gimbal_ypr_text.content = "  Gimbal YPR: [" + std::to_string(armor.ypr_in_gimbal.x() * 57.3f).substr(0, 5) +
                                  ", " + std::to_string(armor.ypr_in_gimbal.y() * 57.3f).substr(0, 5) + ", " +
                                  std::to_string(armor.ypr_in_gimbal.z() * 57.3f).substr(0, 5) + "] deg";
        gimbal_ypr_text.x         = 20;
        gimbal_ypr_text.y         = text_y;
        gimbal_ypr_text.size      = 32;
        gimbal_ypr_text.color     = {255, 255, 255, 255};
        gimbal_ypr_text.thickness = 1;
        texts.push_back(gimbal_ypr_text);
        text_y += gimbal_ypr_text.size;

        rm_utils::DisplayText world_xyz_text;
        world_xyz_text.content = "  World XYZ: [" + std::to_string(armor.xyz_in_world.x()).substr(0, 5) + ", " +
                                 std::to_string(armor.xyz_in_world.y()).substr(0, 5) + ", " +
                                 std::to_string(armor.xyz_in_world.z()).substr(0, 5) + "] m";
        world_xyz_text.x         = 20;
        world_xyz_text.y         = text_y;
        world_xyz_text.size      = 32;
        world_xyz_text.color     = {255, 255, 255, 255};
        world_xyz_text.thickness = 1;
        texts.push_back(world_xyz_text);
        text_y += world_xyz_text.size;

        rm_utils::DisplayText world_ypr_text;
        world_ypr_text.content = "  World YPR: [" + std::to_string(armor.ypr_in_world.x() * 57.3f).substr(0, 5) + ", " +
                                 std::to_string(armor.ypr_in_world.y() * 57.3f).substr(0, 5) + ", " +
                                 std::to_string(armor.ypr_in_world.z() * 57.3f).substr(0, 5) + "] deg";
        world_ypr_text.x         = 20;
        world_ypr_text.y         = text_y;
        world_ypr_text.size      = 32;
        world_ypr_text.color     = {255, 255, 255, 255};
        world_ypr_text.thickness = 1;
        texts.push_back(world_ypr_text);
        text_y += world_ypr_text.size;

        rm_utils::DisplayText world_ypd_text;
        world_ypd_text.content = "  World YPD: [" + std::to_string(armor.ypd_in_world.x() * 57.3f).substr(0, 5) + ", " +
                                 std::to_string(armor.ypd_in_world.y() * 57.3f).substr(0, 5) + ", " +
                                 std::to_string(armor.ypd_in_world.z()).substr(0, 5) + "] deg";
        world_ypd_text.x         = 20;
        world_ypd_text.y         = text_y;
        world_ypd_text.size      = 32;
        world_ypd_text.color     = {255, 255, 255, 255};
        world_ypd_text.thickness = 1;
        texts.push_back(world_ypd_text);
        text_y += world_ypd_text.size;

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

                rm_utils::DisplayText error_text;
                error_text.content   = "  Reproj Error: " + std::to_string(reprojection_error).substr(0, 5) + " px";
                error_text.x         = 20;
                error_text.y         = text_y;
                error_text.size      = 32;
                error_text.color     = (reprojection_error < 5.0) ? rm_utils::DisplayColor{0, 255, 0, 255}
                                                                  : rm_utils::DisplayColor{0, 0, 255, 255};
                error_text.thickness = 1;
                texts.push_back(error_text);
                text_y += error_text.size;
            }
        }
    }

    show(bgr_img, texts, points, lines);
}

} // namespace auto_aim
