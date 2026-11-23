#pragma once

#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <ranges>
#include <string>
#include <vector>

namespace auto_buff
{

// UNKNOWN 表示观测颜色未确定；目标检测颜色使用 RED 或 BLUE。
enum class EnemyColor
{
    UNKNOWN,
    RED,
    BLUE
};

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

// 单块符板
struct Board
{

    EnemyColor color = EnemyColor::UNKNOWN; // 符板颜色
    std::string source;                     // 观测分支
    cv::Point2d center;                     // 符板圆心
    cv::Size2d ellipse_axes;                // 椭圆两条完整轴长

    // 原图四个臂端点：YOLO按top/left/bottom/right，传统检测按环向顺序。
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
    EnemyColor color = EnemyColor::UNKNOWN;

    std::vector<cv::Point> contour; // 实测灯臂轮廓；提取失败时为空，不覆盖网络关键点。
};

// 观测到的能量机关靶心和 R 标
struct RuneBullseye
{
    cv::Point2d center;
    std::array<cv::Point2d, 4> corners;
    bool active;
    double score;
};
using RuneBullseyes = std::vector<RuneBullseye>;

// 单个 R 标候选；检测与跟踪共用原图像素坐标。
struct RuneIcon
{
    cv::Point2d center;
    // 传统检测为骨架匹配评分；YOLO 为产生该 R 点的目标分类分数，不是点置信度。
    double     score    = 0;
    EnemyColor color    = EnemyColor::UNKNOWN; // 无色 R 不推断所属阵营。
    bool       observed = true;               // 是否为本帧有效观测。
};
using RuneIcons = std::vector<RuneIcon>;

// 世界空间瞄准点与方向前馈，保留 RMCS 原接口。
struct AimPoint
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();

    bool valid           = true;
    Eigen::Vector3d ff_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d ff_a = Eigen::Vector3d::Zero();

    AimPoint() = default;
    AimPoint(double x, double y, double z, bool valid = true) : position(x, y, z), valid(valid) {}
    explicit AimPoint(const Eigen::Vector3d &point, bool valid = true) : position(point), valid(valid) {}
};

struct AimPoints : std::vector<AimPoint>
{
    using vector::vector;

    // 目前只有 Rune 需要有效性判断，为简化 Robot 和 Outpost 的实现逻辑，
    // 保留此显式转换以兼容接口
    explicit AimPoints(const std::vector<Eigen::Vector3d> &points)
    {
        vector::reserve(points.size());
        for (const auto point : points)
        {
            vector::emplace_back(point);
        }
    }

    auto same_valid(const AimPoints &o) const -> bool
    {
        if (vector::size() != o.size())
        {
            return false;
        }

        return std::ranges::all_of(std::views::zip(*this, o), [](const auto &pair)
            {
                const auto &[p1, p2] = pair;
                return p1.valid == p2.valid;
            });
    }

    auto valid_indices() const
    {
        std::vector<std::size_t> result;
        for (std::size_t i = 0; i < size(); ++i)
        {
            if ((*this)[i].valid) result.push_back(i);
        }

        return result;
    }
};

constexpr auto kRuneGlobalRadius         = 0.7;  // 能量机关整体的半径
constexpr auto kRuneBullseyeRadius       = 0.15; // 取十字端点的末端，最大半径
constexpr auto kRuneIconProminentDistance = 0.1; // R 标突出的距离，参考的是靶心的平面

// 局部坐标系，以大符中心为原点，X 正方向指向背面
struct RunePagePoints
{
    inline static const Eigen::Vector3d kIcon { -kRuneIconProminentDistance, 0., 0. };
    inline static const Eigen::Vector3d kT { 0, 0, kRuneGlobalRadius + kRuneBullseyeRadius };
    inline static const Eigen::Vector3d kL { 0, +kRuneBullseyeRadius, kRuneGlobalRadius };
    inline static const Eigen::Vector3d kB { 0, 0, kRuneGlobalRadius - kRuneBullseyeRadius };
    inline static const Eigen::Vector3d kR { 0, -kRuneBullseyeRadius, kRuneGlobalRadius };

    inline static const std::array kPoints { kIcon, kT, kL, kB, kR };
};

} // namespace auto_buff
