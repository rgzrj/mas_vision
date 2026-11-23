#pragma once

#include "../buff_detector.hpp"
#include <memory>

namespace auto_buff
{
class Inference;

// 解释三类五点输出，组织观测并执行像素精修。
class YoloDetector final : public BuffDetector
{
public:
    /**
     * @brief 从内存配置构造检测与推理模块。
     * @param config 业务 YAML 根节点。
     * @param resource_dir 模型相对路径的基准目录。
     */
    YoloDetector(const YAML::Node &config, const std::filesystem::path &resource_dir);
    ~YoloDetector() override;

    bool submit(const cv::Mat &image, double seconds, RuneMode mode = RuneMode::unknown) override;

private:
    // 完成帧解码与精修；颜色、模式过滤和绘制由基类统一处理。
    std::optional<Detection> collect_impl() override;

    std::unique_ptr<Inference> inference_;

    bool refine_            = true;
    float confidence_       = 0.25f;
    float point_confidence_ = 0.8f;
    float nms_distance_     = 30.0f;
};
} // namespace auto_buff
