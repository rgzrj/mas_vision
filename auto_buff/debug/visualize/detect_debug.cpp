#include "detect_debug.hpp"
#include "auto_buff/buff_detect/detection.hpp"
#include "display.hpp"
#include <iomanip>
#include <sstream>

namespace auto_buff
{
namespace
{
// 保留原始分数供曲线诊断；完整原始坐标仍由帧报告保存，不重复叠在最终观测上。
void log_raw_pose(const std::vector<PoseDetection> &objects, const Detection &result, const std::string &window_name)
{
    auto &display = rm_utils::Display::getInstance();
    // 候选序号只是当前帧排序，不代表跨帧目标 ID；R 分数与目标分类分数分别记录。
    for (std::size_t n = 0; n < objects.size(); ++n)
    {
        const auto &pose  = objects[n];
        const auto prefix = "detect/confidence/candidate_" + std::to_string(n);
        display.scalar("detect/classes/candidate_" + std::to_string(n), pose.state);
        display.scalar(prefix + "/object_score", pose.score);
        for (int k = 0; k < 5; ++k)
        {
            display.scalar(prefix + "/point_" + std::to_string(k) + "_score", pose.point_scores[k]);
        }
    }
    // 清空旧图层，避免复用显示路径时残留原始点和编号。
    display.points2d(window_name + "/image/raw_pose", {});
    display.scalar("detect/count/markers", double(result.markers.size()));
    display.scalar("detect/count/boards", double(result.boards.size()));
    display.scalar("detect/count/light_arms", double(result.light_arms.size()));
}

} // namespace

void DetectDebug::draw(const Detection &detection, const cv::Mat &image)
{
    if (!enabled() || image.empty())
        return;

    const double processing_fps = update_fps();
    auto &display               = rm_utils::Display::getInstance();
    if (display.values_enabled())
        log_raw_pose(detection.raw_poses, detection, "detect");
    if (!display.image_due())
        return;

    std::vector<rm_utils::DisplayText> texts;
    std::vector<rm_utils::DisplayPoint> points;
    std::vector<rm_utils::DisplayLine> lines;

    texts.push_back({rm_utils::processing_fps_text(processing_fps), 16, 12, 24, {255, 255, 0, 255}, 1, true});
    texts.push_back({"Detect: R candidates=" + std::to_string(detection.markers.size()) +
                         " boards=" + std::to_string(detection.boards.size()),
                     16,
                     44,
                     28,
                     {255, 255, 0, 255},
                     1,
                     true});
    texts.push_back({"Candidates=" + std::to_string(detection.raw_poses.size()) +
                         " arms=" + std::to_string(detection.light_arms.size()),
                     16,
                     78,
                     24,
                     {0, 255, 255, 255},
                     1,
                     true});

    const auto score_text = [](const char *prefix, double score)
    {
        std::ostringstream out;
        out << prefix << std::fixed << std::setprecision(3) << score;
        return out.str();
    };

    const Marker *reference = nullptr;
    for (const auto &marker : detection.markers)
    {
        if (!marker.observed)
            continue;
        if (!reference || marker.score > reference->score)
            reference = &marker;
    }
    // 仅绘制用于连线的参考 R，其他候选仍交给跟踪并保留在报告中。
    if (reference)
    {
        points.push_back({cvRound(reference->center.x), cvRound(reference->center.y), {0, 255, 0, 255}, 5});
        texts.push_back({score_text("R: ", reference->score),
                         cvRound(reference->center.x) + 8,
                         cvRound(reference->center.y),
                         20,
                         {0, 255, 0, 255}});
    }

    for (const auto &board : detection.boards)
    {
        const rm_utils::DisplayColor color = board.appearance == Appearance::full_mark
                                                 ? rm_utils::DisplayColor{0, 255, 0, 255}
                                                 : rm_utils::DisplayColor{255, 210, 0, 255};

        points.push_back({cvRound(board.center.x), cvRound(board.center.y), color, 4});
        texts.push_back(
            {appearance_name(board.appearance), cvRound(board.center.x) + 8, cvRound(board.center.y) - 24, 20, color});

        if (reference && (reference->color == CampColor::UNKNOWN || reference->color == board.color))
            lines.push_back({cvRound(reference->center.x),
                             cvRound(reference->center.y),
                             cvRound(board.center.x),
                             cvRound(board.center.y),
                             1,
                             {120, 120, 120, 255}});

        if (board.features_valid)
            for (int i = 0; i < 4; ++i)
            {
                const auto &point = board.corners[i];
                points.push_back({cvRound(point.x), cvRound(point.y), color, 4});
            }

        if (board.source == "rmcs_traditional")
            texts.push_back(
                {score_text("B: ", board.pvd), cvRound(board.center.x) + 8, cvRound(board.center.y) + 22, 20, color});
    }

    // 灯臂端点按自己的语义显示，不能因为关闭原始点图层而丢失第三类观测。
    for (const auto &arm : detection.light_arms)
    {
        const cv::Point2d *label_point = nullptr;
        for (int k : PoseDetection::leaf_indices)
            if (arm.pose.valid[k])
            {
                const auto &point = arm.pose.points[k];
                points.push_back({cvRound(point.x), cvRound(point.y), {0, 210, 255, 255}, 4});
                if (!label_point)
                    label_point = &point;
            }
        if (label_point)
            texts.push_back(
                {"big_activated", cvRound(label_point->x) + 8, cvRound(label_point->y) - 24, 20, {0, 210, 255, 255}});
    }

    show(image, texts, points, lines);
}

void DetectDebug::clear()
{
    if (rm_utils::Display::getInstance().values_enabled())
        log_raw_pose({}, Detection{}, "detect");
}

} // namespace auto_buff
