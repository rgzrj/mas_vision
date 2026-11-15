#pragma once

#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <ranges>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace auto_buff
{

// YAML 边界只接受 red/blue；UNKNOWN 表示观测本身没有可靠颜色，不是自动选色模式。
enum class CampColor : std::uint8_t
{
    UNKNOWN,
    RED,
    BLUE
};

inline CampColor parse_camp_color(std::string_view value)
{
    if (value == "red") return CampColor::RED;
    if (value == "blue") return CampColor::BLUE;
    throw std::invalid_argument("Detection color must be red or blue");
}

// 打符物理模型与几何观测；尺寸单位为米，观测坐标为原图像素。
constexpr auto kRuneGlobalRadius   = 0.7; // 能量机关整体的半径
constexpr auto kRuneBullseyeRadius = 0.15; // 取十字端点的末端，最大半径

constexpr auto kRuneIconProminentDistance = 0.1; // R 标突出的距离，参考的是靶心的平面

// 局部坐标系，以大符中心为原点，X 正方向指向背面
struct RunePagePoints {
    inline static const Eigen::Vector3d kIcon { -kRuneIconProminentDistance, 0., 0. };
    inline static const Eigen::Vector3d kT { 0, 0, kRuneGlobalRadius + kRuneBullseyeRadius };
    inline static const Eigen::Vector3d kL { 0, +kRuneBullseyeRadius, kRuneGlobalRadius };
    inline static const Eigen::Vector3d kB { 0, 0, kRuneGlobalRadius - kRuneBullseyeRadius };
    inline static const Eigen::Vector3d kR { 0, -kRuneBullseyeRadius, kRuneGlobalRadius };

    inline static const std::array kPoints { kIcon, kT, kL, kB, kR };
};

struct RuneBullseye {
    cv::Point2d center;
    std::array<cv::Point2d, 4> corners;
    bool active;
    double score;
};
using RuneBullseyes = std::vector<RuneBullseye>;

struct RuneIcon {
    cv::Point2d center;
    double score;
};
using RuneIcons = std::vector<RuneIcon>;

// 世界空间瞄准点与方向前馈，保留 RMCS 原接口。
struct AimPoint {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    bool valid = true;
    Eigen::Vector3d ff_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d ff_a = Eigen::Vector3d::Zero();

    AimPoint() = default;
    AimPoint(double x, double y, double z, bool valid = true) : position(x, y, z), valid(valid) {}
    explicit AimPoint(const Eigen::Vector3d &point, bool valid = true) : position(point), valid(valid) {}
};

struct AimPoints : std::vector<AimPoint> {
    using vector::vector;

    // 目前只有 Rune 需要有效性判断，为简化 Robot 和 Outpost 的实现逻辑，
    // 保留此显式转换以兼容接口
    explicit AimPoints(const std::vector<Eigen::Vector3d>& points) {
        vector::reserve(points.size());
        for (const auto point : points) {
            vector::emplace_back(point);
        }
    }

    auto same_valid(const AimPoints& o) const -> bool {
        if (vector::size() != o.size()) {
            return false;
        }
        return std::ranges::all_of(std::views::zip(*this, o), [](const auto& pair) {
            const auto& [p1, p2] = pair;
            return p1.valid == p2.valid;
        });
    }

    auto valid_indices() const {
        std::vector<std::size_t> result;
        for (std::size_t i = 0; i < size(); ++i) {
            if ((*this)[i].valid) result.push_back(i);
        }
        return result;
    }
};

} // namespace auto_buff
