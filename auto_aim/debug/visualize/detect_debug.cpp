#include "detect_debug.hpp"
#include <spdlog/fmt/fmt.h>

namespace auto_aim
{
void DetectDebug::draw(const cv::Mat &bgr_img, const std::vector<Armor> &armors, const std::vector<LightBar> &lights)
{
    if (!enabled() || bgr_img.empty())
        return;

    const double fps = update_fps();
    auto &display    = rm_utils::Display::getInstance();
    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine> lines;
    std::vector<rm_utils::DisplayText> texts;

    display.scalar("armor/detect/count/armors", armors.size());
    display.scalar("armor/detect/count/lights", lights.size());

    // 灯条观测：绿色旋转矩形，红色中心点。
    for (const auto &light : lights)
    {
        // 灯条边框
        cv::Point2f vertices[4];
        light.rotated_rect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            rm_utils::DisplayLine line;
            line.x1        = static_cast<int>(vertices[i].x);
            line.y1        = static_cast<int>(vertices[i].y);
            line.x2        = static_cast<int>(vertices[(i + 1) % 4].x);
            line.y2        = static_cast<int>(vertices[(i + 1) % 4].y);
            line.color     = {0, 255, 0, 255}; // 绿色 (RGBA)
            line.thickness = 2;
            lines.push_back(line);
        }

        // 灯条中心点
        rm_utils::DisplayPoint center_point;
        center_point.x     = static_cast<int>(light.center.x);
        center_point.y     = static_cast<int>(light.center.y);
        center_point.color = {255, 0, 0, 255}; // 红色 (RGBA)
        center_point.size  = 2;
        points.push_back(center_point);
    }

    // 装甲板观测：青色角点与中心，黄色轮廓和分类信息。
    for (const auto &armor : armors)
    {
        if (armor.points.size() != 4)
            continue;

        // 绘制左灯条角点
        rm_utils::DisplayPoint left_top;
        left_top.x     = static_cast<int>(armor.points[0].x);
        left_top.y     = static_cast<int>(armor.points[0].y);
        left_top.color = {0, 255, 255, 255}; // 青色 (RGBA)
        left_top.size  = 10;
        points.push_back(left_top);

        rm_utils::DisplayPoint left_bottom;
        left_bottom.x     = static_cast<int>(armor.points[3].x);
        left_bottom.y     = static_cast<int>(armor.points[3].y);
        left_bottom.color = {0, 255, 255, 255}; // 青色 (RGBA)
        left_bottom.size  = 10;
        points.push_back(left_bottom);

        // 绘制右灯条角点
        rm_utils::DisplayPoint right_top;
        right_top.x     = static_cast<int>(armor.points[1].x);
        right_top.y     = static_cast<int>(armor.points[1].y);
        right_top.color = {0, 255, 255, 255}; // 青色 (RGBA)
        right_top.size  = 10;
        points.push_back(right_top);

        rm_utils::DisplayPoint right_bottom;
        right_bottom.x     = static_cast<int>(armor.points[2].x);
        right_bottom.y     = static_cast<int>(armor.points[2].y);
        right_bottom.color = {0, 255, 255, 255}; // 青色 (RGBA)
        right_bottom.size  = 10;
        points.push_back(right_bottom);

        // 装甲板四边形（使用四个角点）
        if (armor.points.size() == 4)
        {
            for (int i = 0; i < 4; i++)
            {
                rm_utils::DisplayLine line;
                line.x1        = static_cast<int>(armor.points[i].x);
                line.y1        = static_cast<int>(armor.points[i].y);
                line.x2        = static_cast<int>(armor.points[(i + 1) % 4].x);
                line.y2        = static_cast<int>(armor.points[(i + 1) % 4].y);
                line.color     = {255, 255, 0, 255}; // 黄色 (RGBA)
                line.thickness = 3;
                lines.push_back(line);
            }
        }

        // 装甲板中心点
        rm_utils::DisplayPoint center_point;
        center_point.x     = static_cast<int>(armor.center.x);
        center_point.y     = static_cast<int>(armor.center.y);
        center_point.color = {0, 255, 255, 255}; // 青色 (RGBA)
        center_point.size  = 20;
        points.push_back(center_point);

        // 依次显示长宽比、侧边比、矩形角误差（度）、置信度、类别和尺寸。
        rm_utils::DisplayText armor_info;
        armor_info.content   = fmt::format("{:.2f} {:.2f} {:.1f} {:.2f} {} {}", armor.ratio, armor.side_ratio,
                                           armor.rectangular_error * 57.3, armor.confidence, armor.number, "SMALL");
        armor_info.x         = static_cast<int>(armor.center.x) + 25;
        armor_info.y         = static_cast<int>(armor.center.y);
        armor_info.size      = 24;
        armor_info.color     = {255, 255, 0, 255};
        armor_info.thickness = 1;
        texts.push_back(armor_info);
    }

    // 左上角状态：本模块绘制帧率、装甲板数和灯条数。
    rm_utils::DisplayText stats;
    stats.content = "FPS: " + std::to_string(static_cast<int>(fps)) + " Armors: " + std::to_string(armors.size()) +
                    " Lights: " + std::to_string(lights.size());
    stats.x         = 20;
    stats.y         = 20;
    stats.size      = 24;
    stats.color     = {255, 255, 0, 255}; // 黄色 (RGBA)
    stats.thickness = 2;
    texts.push_back(stats);

    show(bgr_img, texts, points, lines);
}

void DetectDebug::clear()
{
    auto &display = rm_utils::Display::getInstance();
    display.scalar("armor/detect/count/armors", 0);
    display.scalar("armor/detect/count/lights", 0);
}

} // namespace auto_aim
