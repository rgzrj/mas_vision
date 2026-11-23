#pragma once

#include "auto_buff/buff_types.hpp"

#include <opencv2/core/mat.hpp>
#include <array>
#include <chrono>
#include <filesystem>
#include <memory>
#include <span>

namespace YAML
{
class Node;
}

namespace auto_buff
{
// YOLO 内部推理资源；CPU 单槽同步，GPU 双槽异步，单线程提交和收取。
class Inference
{
public:
    struct Slot
    {
        cv::Mat image;
        EnemyColor color = EnemyColor::RED;
        RuneMode mode    = RuneMode::unknown;
        double seconds   = 0.0;
        double scale     = 1.0;
        cv::Point pad;

        std::span<const float> output; // front 返回后有效，pop 或销毁后不可再访问。

        bool profiling    = false;
        double prepare_ms = 0.0;
        std::chrono::steady_clock::time_point submitted;
    };

    // settings 为已读入的 yolo 节点；资源相对路径以 resource_dir 为基准。
    Inference(const YAML::Node &settings, const std::filesystem::path &resource_dir);
    ~Inference();
    Inference(const Inference &)            = delete;
    Inference &operator=(const Inference &) = delete;

    // 满槽拒收；成功接收后，图像像素须保持只读直至 pop。
    bool submit(const cv::Mat &image, double seconds, RuneMode mode, EnemyColor color);
    const Slot *front(); // 仅返回已完成的只读队首，不等待推理。
    void pop();          // front 返回非空且后处理结束后调用。

private:
    using Clock = std::chrono::steady_clock;

    static constexpr int kInputWidth  = 640;
    static constexpr int kInputHeight = 480;

    // 由构建时选中的后端实现；厂商资源不进入公共头文件。
    struct Runtime;
    float *input(std::size_t index);
    void start(std::size_t index);
    bool ready(std::size_t index);
    std::span<const float> output(std::size_t index);
    void prepare(Slot &frame, const cv::Mat &image, float *data);

    std::unique_ptr<Runtime> runtime_;
    std::array<Slot, 2> slots_;

    std::size_t capacity_ = 1;
    std::size_t head_     = 0;
    std::size_t pending_  = 0;

    bool tracy_debug_   = false;
    bool reuse_buffers_ = false;

    cv::Mat input_, floating_;
};
} // namespace auto_buff
