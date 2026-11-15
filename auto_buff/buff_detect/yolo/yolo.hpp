#pragma once
#include "../detection.hpp"

#include <openvino/openvino.hpp>
#include <filesystem>

namespace auto_buff
{

// OpenVINO CPU 五点检测。深大模型与自训练模型采用相同的输入、类别和点序。
class YoloDetector final : public Detector
{
public:
    /**
     * @brief 在构造函数内读取auto_buff.detect，加载模型并创建单流CPU推理请求。
     * @param config_path 业务配置路径；模型相对路径按该配置所在目录解析。
     */
    explicit YoloDetector(const std::filesystem::path &config_path);

protected:
    /**
     * @brief 核心检测接口，重写基类虚函数
     * @param image 输入相机原始图像
     * @param timestamp 图像时间戳，单位秒；本检测器无时序状态，不参与网络推理。
     * @return Detection 打包本帧所有识别结果（多个Board符板、Marker信息）
     */
    Detection detect_impl(const cv::Mat &image, double timestamp) override;

private:
    bool tracy_debug_   = false; // 应用启动 Tracy 后控制采集，与图像显示独立。
    bool reuse_buffers_ = false;
    cv::Mat input_, floating_; // 单算法线程复用；每帧仍完整覆盖 letterbox 和浮点数据。

    ov::Core core_;             // OpenVINO核心对象，负责加载插件、硬件（CPU/GPU/NPU）、读取模型、编译网络
    ov::CompiledModel network_; // 编译后的模型
    ov::InferRequest request_;  // 单处理线程复用请求和输入张量。
    float confidence_;          // 置信度阈值
    float point_confidence_;    // 单个原始关键点的有效性下限。
    float nms_distance_;        // 原图像素距离门限；本协议没有检测框，不使用 IoU 去重。

    static constexpr int kInputWidth  = 640;
    static constexpr int kInputHeight = 480;

    std::string feature_points_; // none 使用原始五点；rmcs_pixels 精修，失败保留有效原始点。
};

} // namespace auto_buff
