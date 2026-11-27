#pragma once

#include "../buff_detector.hpp"
#include "rmcs/rune.hpp"

namespace auto_buff
{
// 保留 RMCS 识别流程，将观测转换为公共结果。
class TraditionalDetector final : public BuffDetector
{
public:
    /**
     * @brief 从内存中的业务配置和相机标定初始化传统检测。
     */
    TraditionalDetector(const YAML::Node &config, const YAML::Node &camera);

    bool submit(const cv::Mat &image, double seconds, RuneMode mode = RuneMode::unknown) override;

private:
    std::optional<Detection> collect_impl() override;
    Detection detect_impl(const cv::Mat &image, RuneMode mode);

    RuneDetector detector_;
    cv::Size calibration_size_;
    bool save_masks_ = false;
    std::optional<Detection> pending_;
};
} // namespace auto_buff
