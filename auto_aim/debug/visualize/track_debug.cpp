#include "track_debug.hpp"
#include "auto_aim/armor_track/armor_track.hpp"

#include <limits>

namespace auto_aim
{
void TrackDebug::draw(const cv::Mat &bgr_img, const std::vector<Armor> &armors, const ArmorTrack &tracker)
{
    if (!enabled() || bgr_img.empty())
        return;

    const auto state   = tracker.state();
    const auto &target = tracker.target_state();
    const auto &pose   = tracker.armor_pose();
    plot(target);
    const double fps = update_fps();
    if (!rm_utils::Display::getInstance().image_due())
        return;

    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine> lines;
    std::vector<rm_utils::DisplayText> texts;

    // 左上角状态：帧率、观测数量、跟踪状态和目标模型参数。
    rm_utils::DisplayText fps_text;
    fps_text.content = "FPS: " + std::to_string(static_cast<int>(fps)) + " Armors: " + std::to_string(armors.size()) +
                       " State: " + state;
    fps_text.x         = 20;
    fps_text.y         = 20;
    fps_text.size      = 24;
    fps_text.color     = {255, 255, 0, 255};
    fps_text.thickness = 2;
    texts.push_back(fps_text);

    // 显示跟踪目标参数
    if (target.has_value())
    {
        // 目标基本信息
        rm_utils::DisplayText target_text;
        target_text.content = "Target: " + target->name + " ID: " + std::to_string(target->last_id) +
                              " Updates: " + std::to_string(target->update_count);
        target_text.x         = 20;
        target_text.y         = 50;
        target_text.size      = 20;
        target_text.color     = {0, 255, 0, 255};
        target_text.thickness = 1;
        texts.push_back(target_text);

        // 目标类型和状态
        rm_utils::DisplayText target_info;
        target_info.content = "type: " + std::string("SMALL") + " jumped: " + std::to_string(target->jumped) +
                              " switch: " + std::to_string(target->is_switch) +
                              " armor_num: " + std::to_string(target->armor_num);
        target_info.x         = 20;
        target_info.y         = 75;
        target_info.size      = 18;
        target_info.color     = {255, 200, 0, 255};
        target_info.thickness = 1;
        texts.push_back(target_info);

        // EKF 状态顺序：[x, vx, y, vy, z, vz, yaw, omega, r, l, h]。
        if (target->ekf().x.size() >= 11)
        {
            // 位置
            rm_utils::DisplayText position_text;
            position_text.content = "x: " + std::to_string(target->ekf().x[0]).substr(0, 4) +
                                    " y: " + std::to_string(target->ekf().x[2]).substr(0, 4) +
                                    " z: " + std::to_string(target->ekf().x[4]).substr(0, 4);
            position_text.x         = 20;
            position_text.y         = 100;
            position_text.size      = 18;
            position_text.color     = {0, 255, 255, 255};
            position_text.thickness = 1;
            texts.push_back(position_text);

            // 速度
            rm_utils::DisplayText velocity_text;
            velocity_text.content = "vx: " + std::to_string(target->ekf().x[1]).substr(0, 4) +
                                    " vy: " + std::to_string(target->ekf().x[3]).substr(0, 4) +
                                    " vz: " + std::to_string(target->ekf().x[5]).substr(0, 4);
            velocity_text.x         = 20;
            velocity_text.y         = 125;
            velocity_text.size      = 18;
            velocity_text.color     = {0, 255, 255, 255};
            velocity_text.thickness = 1;
            texts.push_back(velocity_text);

            // 角度和角速度
            rm_utils::DisplayText rotation_text;
            rotation_text.content = "yaw: " + std::to_string(target->ekf().x[6]).substr(0, 4) +
                                    " w: " + std::to_string(target->ekf().x[7]).substr(0, 4);
            rotation_text.x         = 20;
            rotation_text.y         = 150;
            rotation_text.size      = 18;
            rotation_text.color     = {0, 255, 255, 255};
            rotation_text.thickness = 1;
            texts.push_back(rotation_text);

            // 半径参数
            rm_utils::DisplayText geometry_text;
            geometry_text.content = "r: " + std::to_string(target->ekf().x[8]).substr(0, 4) +
                                    " l: " + std::to_string(target->ekf().x[9]).substr(0, 4) +
                                    " h: " + std::to_string(target->ekf().x[10]).substr(0, 4);
            geometry_text.x         = 20;
            geometry_text.y         = 175;
            geometry_text.size      = 18;
            geometry_text.color     = {0, 255, 255, 255};
            geometry_text.thickness = 1;
            texts.push_back(geometry_text);
        }
    }

    // 观测标签：检测类别与置信度。
    for (const auto &armor : armors)
    {
        // 显示装甲板编号和置信度
        rm_utils::DisplayText armor_info_text;
        armor_info_text.content = "[" + armor.number + "]: [" + std::to_string(armor.confidence).substr(0, 5) + "]";
        armor_info_text.x       = static_cast<int>(armor.center.x) -
                            static_cast<int>(armor_info_text.size * armor_info_text.content.size() / 2);
        armor_info_text.y         = static_cast<int>(armor.center.y);
        armor_info_text.size      = 32;
        armor_info_text.color     = {255, 255, 255, 255};
        armor_info_text.thickness = 1;
        texts.push_back(armor_info_text);
    }

    if (target.has_value() && target->ekf().x.size() >= 11)
    {
        // 模型投影：从目标状态生成各板位，再通过当前相机位姿投回图像。
        auto xyza_list = target->armor_xyza_list();
        for (int i = 0; i < (int)xyza_list.size() && i < target->armor_num; i++)
        {
            const auto &xyza = xyza_list[i];
            Eigen::Vector3d armor_xyz_world(xyza[0], xyza[1], xyza[2]);
            double yaw = target->ekf().x[6] + i * (2 * CV_PI / target->armor_num);

            // 投影到图像平面
            auto image_points = pose.reproject_armor(armor_xyz_world, yaw, target->name);

            if (image_points.empty())
                continue;

            // 当前跟踪的装甲板用绿色，其他用红色
            bool is_tracking = (i == target->last_id);
            rm_utils::DisplayColor color =
                is_tracking ? rm_utils::DisplayColor{0, 255, 0, 255} : rm_utils::DisplayColor{255, 0, 0, 255};

            // 绘制矩形框
            for (int j = 0; j < 4; j++)
            {
                rm_utils::DisplayLine line;
                line.x1        = static_cast<int>(image_points[j].x);
                line.y1        = static_cast<int>(image_points[j].y);
                line.x2        = static_cast<int>(image_points[(j + 1) % 4].x);
                line.y2        = static_cast<int>(image_points[(j + 1) % 4].y);
                line.thickness = is_tracking ? 3 : 2;
                line.color     = color;
                lines.push_back(line);
            }

            // 当前跟踪的装甲板绘制X型对角线
            if (is_tracking)
            {
                rm_utils::DisplayLine diagonal_02;
                diagonal_02.x1        = static_cast<int>(image_points[0].x);
                diagonal_02.y1        = static_cast<int>(image_points[0].y);
                diagonal_02.x2        = static_cast<int>(image_points[2].x);
                diagonal_02.y2        = static_cast<int>(image_points[2].y);
                diagonal_02.thickness = 2;
                diagonal_02.color     = color;
                lines.push_back(diagonal_02);

                rm_utils::DisplayLine diagonal_13;
                diagonal_13.x1        = static_cast<int>(image_points[1].x);
                diagonal_13.y1        = static_cast<int>(image_points[1].y);
                diagonal_13.x2        = static_cast<int>(image_points[3].x);
                diagonal_13.y2        = static_cast<int>(image_points[3].y);
                diagonal_13.thickness = 2;
                diagonal_13.color     = color;
                lines.push_back(diagonal_13);
            }

            // 绘制装甲板ID
            rm_utils::DisplayText id_text;
            id_text.content   = std::to_string(i);
            id_text.x         = static_cast<int>(image_points[0].x);
            id_text.y         = static_cast<int>(image_points[0].y) - 5;
            id_text.size      = 32;
            id_text.color     = {255, 255, 0, 255};
            id_text.thickness = 1;
            texts.push_back(id_text);
        }

        // 旋转中心投影为紫色点，与装甲板中心区分。
        Eigen::Vector3d center_xyz(target->ekf().x[0], target->ekf().x[2], target->ekf().x[4]);
        std::vector<cv::Point3f> center_world = {cv::Point3f(center_xyz[0], center_xyz[1], center_xyz[2])};
        auto center_pixel                     = pose.world2pixel(center_world);
        if (!center_pixel.empty())
        {
            rm_utils::DisplayPoint center_point;
            center_point.x     = static_cast<int>(center_pixel[0].x);
            center_point.y     = static_cast<int>(center_pixel[0].y);
            center_point.size  = 15;
            center_point.color = {255, 0, 255, 255};
            points.push_back(center_point);
        }
    }

    // 统一提交当前窗口，不保存整帧结果。
    show(bgr_img, texts, points, lines);
}

void TrackDebug::plot(const std::optional<Target> &target)
{
    auto &display = rm_utils::Display::getInstance();
    if (!display.values_enabled())
        return;
    display.scalar("armor/track/valid", target.has_value());
    // 下标必须与 EKF 的 11 维状态一致，路径按物理量分组。
    const char *paths[]{"position/x_m",         "velocity/vx_mps",      "position/y_m",     "velocity/vy_mps",
                        "position/z_m",         "velocity/vz_mps",      "rotation/yaw_rad", "rotation/omega_radps",
                        "geometry/radius_02_m", "geometry/radius_13_m", "geometry/height_m"};
    for (int i = 0; i < 11; ++i)
    {
        const double missing = std::numeric_limits<double>::quiet_NaN();
        display.scalar(std::string("armor/track/") + paths[i], target ? target->ekf().x[i] : missing);
        display.scalar("armor/track/covariance/" + std::to_string(i), target ? target->ekf().P(i, i) : missing);
    }
}

void TrackDebug::clear()
{
    plot(std::nullopt);
}

} // namespace auto_aim
