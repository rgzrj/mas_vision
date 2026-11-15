#pragma once

#include "auto_buff/buff_types.hpp"
#include "auto_buff/debug/visualize/detect_debug.hpp"

#include <opencv2/core.hpp>
#include <filesystem>
#include <memory>
#include <array>
#include <string>
#include <vector>

namespace auto_buff
{

enum class RuneMode
{
    unknown,
    small,
    big
};

// 传统检测与 YOLO 共用的单帧观测接口。
// 所有图像点和长度均使用原图像素坐标：左上角为原点，x 向右，y 向下。

enum class Appearance
{
    unknown,      // 无法判定
    full_mark,    // 待击打符板
    single_ring,  // 小符打击后圆环。
    big_activated // 大符激活灯臂对应的板心。
};

/**
 *  @brief 将外观状态转为显示与日志使用的名称。
 *  @param value Appearance 枚举值
 *  @return 静态字符串；full_mark 对应 "pending"，表示待击打。
 */
inline const char *appearance_name(Appearance value)
{
    switch (value)
    {
    case Appearance::full_mark:
        return "pending";
    case Appearance::single_ring:
        return "single_ring";
    case Appearance::big_activated:
        return "big_activated";
    default:
        return "unknown";
    }
}

// 单个 R 标候选。检测阶段保留多个候选，交给后续几何初始化选择。
struct Marker
{
    cv::Point2d center; // R 中心
    CampColor color = CampColor::UNKNOWN;  // 传统检测为 red/blue；无色 R 使用 UNKNOWN，不推断其所属阵营。

    // 传统检测为 R 骨架匹配评分；YOLO 为产生该 R 点的目标分类分数，不是 R 点自身的置信度。
    double score  = 0;
    bool observed = true; // 是否为本帧有效观测
};

// 单块符板
struct Board
{

    CampColor color = CampColor::UNKNOWN;       // 符板颜色
    std::string source;      // 观测分支
    cv::Point2d center;      // 符板圆心
    cv::Size2d ellipse_axes; // 椭圆两条完整轴长

    // 四个臂端点，顺序为 top、left、right、bottom；未观测到的点为 (0,0)
    std::array<cv::Point2d, 4> corners{};

    double radius         = 0;                   // 图像尺度估计（像素）
    double ellipse_angle  = 0;                   // 椭圆轴旋转角（度）
    double pvd            = 0;                   // 臂方向与臂间方向的峰谷亮度对比
    double arc_coverage   = 0;                   // YOLO 像素精修保留的圆弧采样点数 / 180，范围 [0,1]
    double fit_error      = 0;                   // YOLO 像素精修的归一化椭圆径向残差 RMS
    Appearance appearance = Appearance::unknown; // 当前帧识别出的外观状态

    bool features_valid      = false; // 四个真实臂端点是否全部可用
    double geometry_error_px = 0;     // 灯臂模型轮廓匹配 RMS，单位像素。
    double geometry_support  = 0;     // 灯臂模型点中得到实测轮廓支持的比例。
};

// YOLO 解码后的单个候选；用于观测转换，也可保留为精修前的调试快照。
struct PoseDetection
{
    // 五点协议中排除 R(索引2)后的环绕点序：top、left、bottom、right。
    static constexpr std::array leaf_indices{0, 1, 4, 3};

    int state    = 0;                     // 0 未激活、1 小符已激活、2 大符已激活
    double score = 0;                     // 三个状态分数中的最大值。
    std::array<cv::Point2d, 5> points{};  // 原图坐标，点序为 top、left、R、right、bottom。
    std::array<bool, 5> valid{};          // 点置信度达标，且坐标有限、位于原图范围内。
    std::array<double, 5> point_scores{}; // 原始逐点分数，供调试；不改变模型协议。
};

// 类别2的四点属于激活灯臂，不是靶板端点，交给 pose 转换
struct LightArm
{
    PoseDetection pose;
    CampColor color = CampColor::UNKNOWN;

    std::vector<cv::Point> contour; // 实测灯臂轮廓；提取失败时为空，不覆盖网络关键点。
};

// 当前帧的全部检测观测
struct Detection
{
    RuneMode mode = RuneMode::unknown;

    std::vector<Marker> markers;          // 全部 R 候选
    std::vector<Board> boards;            // 待击板和打击后圆环
    std::vector<LightArm> light_arms;     // 大符激活灯臂，不冒充靶板。
    std::vector<PoseDetection> raw_poses; // YOLO 原始三类五点快照，不受精修/debug 开关影响。
    cv::Mat mask;                         // 可选诊断用二值掩膜
};

/**
 * @brief 根据符板模式和颜色过滤观测结果。
 * @param detection 输入输出参数，过滤后保留的观测。
 * @param mode 符板模式，决定保留哪些外观状态的符板。
 * @param color 仅保留该颜色的符板和 R；必须为 red 或 blue，否则抛出异常。
 * @note 该函数不改变观测的坐标和时间戳约定
 */
void filter_rune_observations(Detection &detection, RuneMode mode, CampColor color);

// 统一检测器接口，主流程可在传统检测与 YOLO 之间切换。
class Detector
{
public:
    virtual ~Detector() = default;

    /**
     * @brief 切换下一帧要保留的目标颜色。
     * @param color 仅支持 "red"、"blue"；不改变坐标和时间戳约定。
     */
    void set_color(const std::string &color);

    CampColor color() const
    {
        return color_;
    }
    /**
     * @brief 检测、过滤并返回本帧观测；开启 debug 时在内部绘制最终结果。
     * @param bgr 输入图像，BGR 8UC3。
     * @param media_seconds 曝光/媒体时间戳，单位秒。
     * @param mode 大小符模式；unknown 只按颜色过滤，供普通离线媒体使用。
     * @return 检测结果，包含 R 标候选、符板观测和二值掩膜。
     */
    Detection detect(const cv::Mat &bgr, double media_seconds, RuneMode mode = RuneMode::unknown);
    void disable_debug() noexcept
    {
        detect_debug_.set_enabled(false);
    }

protected:
    virtual Detection detect_impl(const cv::Mat &bgr, double media_seconds) = 0;
    CampColor color_                                                       = CampColor::RED;
    DetectDebug detect_debug_; // 最终观测由公共入口统一绘制，后端不重复提交。
};

// 只负责选择传统或 YOLO 后端；三个算法的生命周期由实机/回放各自管理。
std::unique_ptr<Detector> make_detector(const std::filesystem::path &config_path);

} // namespace auto_buff
