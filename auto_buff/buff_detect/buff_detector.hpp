#pragma once

#include "auto_buff/buff_types.hpp"

#include <opencv2/core/mat.hpp>
#include <filesystem>
#include <memory>
#include <optional>

namespace YAML
{
class Node;
}

namespace auto_buff
{
class DetectDebug;

// 传统检测和 YOLO 的统一入口；仅由一个算法线程提交和收取。
class BuffDetector
{
public:
    struct Detection
    {
        RuneMode mode    = RuneMode::unknown;
        EnemyColor color = EnemyColor::RED;
        cv::Mat image;      // 对应原图，只读引用。
        double seconds = 0; // 曝光或媒体时间，单位秒。

        std::vector<RuneIcon> markers;
        std::vector<Board> boards;
        std::vector<LightArm> light_arms;
        std::vector<PoseDetection> raw_poses;
        cv::Mat mask;
    };

    virtual ~BuffDetector();

    BuffDetector(const BuffDetector &)            = delete;
    BuffDetector &operator=(const BuffDetector &) = delete;

    /**
     * @brief 读取一次业务配置，构造选定后端。
     * @param config_path 业务 YAML；相对资源路径以其所在目录为基准。
     * @return 检测器；初始化失败时记录日志并返回空指针。
     */
    static std::unique_ptr<BuffDetector> create(const std::filesystem::path &config_path);

    /**
     * @brief 接收一帧；满槽返回 false，不消耗输入或覆盖已有请求。
     * @param image BGR 8UC3 图像；接收后至收取前保持像素只读。
     * @param seconds 曝光或媒体时间，单位秒。
     * @param mode 本帧大小符模式；unknown 仅按颜色过滤。
     */
    virtual bool submit(const cv::Mat &image, double seconds, RuneMode mode = RuneMode::unknown) = 0;

    /**
     * @brief 按提交顺序收取、过滤并绘制完成帧，不等待未完成的推理。
     * @return 完成帧；队首未完成或无待收取帧时返回空。
     */
    std::optional<Detection> collect();

    void BuffDetector_Set_Color(EnemyColor color)
    {
        color_ = color;
    }
    void disable_debug();

protected:
    explicit BuffDetector(const YAML::Node &config);
    /**
     * @brief 实际收集检测结果的实现。
     * @return 完成帧；没有可收取的完成帧时返回空，未检出目标仍返回结果。
     */
    virtual std::optional<Detection> collect_impl() = 0;

    EnemyColor color() const noexcept
    {
        return color_;
    }

private:
    EnemyColor color_ = EnemyColor::RED;
    std::unique_ptr<DetectDebug> debug_;
};

/**
 * @brief 过滤符板、灯臂和 R 点观测结果。
 * @param detection 待过滤的检测结果。
 * @param mode 本帧大小符模式；unknown 仅按颜色过滤。
 * @param color 本帧敌方阵营，使用 RED 或 BLUE；无色 R 保留。
 */
void filter_rune_observations(BuffDetector::Detection &detection, RuneMode mode, EnemyColor color);

} // namespace auto_buff
