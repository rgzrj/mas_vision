#pragma once
#include "../detection.hpp"
#include "rmcs/rune.hpp"
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace auto_buff
{

// RMCS 传统检测的无状态适配层：读取标定，并转换为与 YOLO 一致的 Detection。
class TraditionalDetector final : public Detector
{
public:
    // 正式入口传入业务配置和标定路径，参数在构造函数内部读取。
    TraditionalDetector(const std::filesystem::path &config_path, const std::filesystem::path &camera_config);

    /**
     * @brief 加载相机标定及传统检测参数。
     * @param camera_config 相机 YAML 路径；内参和图像尺寸用于物理尺寸过滤。
     * @param color red / blue。
     * @param settings detect 配置节点，覆盖距离、透视、PVD 和 R 评分门限。
     */
    TraditionalDetector(const std::filesystem::path &camera_config, std::string color, const YAML::Node &settings);
protected:
    /**
     * @brief 提取本帧全部 R 候选、待击板和打击后圆环。
     * @param bgr 原始 BGR 8UC3 图像，尺寸须与标定一致。
     * @param media_seconds 图像时间，单位秒；传统检测不保留时序状态。
     * @return 原图坐标中的观测；圆环不提供可初始化 PnP 的四端点。
     */
    Detection detect_impl(const cv::Mat &bgr, double media_seconds) override;

private:
    auto_buff::RuneDetector detector_; // 保留 RMCS 的轮廓、四臂和 R 拓扑识别实现。
    cv::Size calibration_size_;        // 相机标定对应的原图分辨率。
    bool save_masks_ = false;          // 是否输出颜色分割掩膜，供显示和离线诊断使用。
};

} // namespace auto_buff
