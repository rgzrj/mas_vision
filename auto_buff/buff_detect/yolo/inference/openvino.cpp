#include "inference.hpp"
#include "../observation/pose_output.hpp"

#include "mas_log.hpp"

#include <openvino/openvino.hpp>
#include <opencv2/core/utility.hpp>
#include <yaml-cpp/yaml.h>
#include <stdexcept>

namespace auto_buff
{
struct Inference::Runtime
{
    ov::Core core;
    ov::CompiledModel network;
    std::array<ov::InferRequest, 2> requests;
    bool gpu = false;
};

Inference::Inference(const YAML::Node &settings, const std::filesystem::path &resource_dir)
    : runtime_(std::make_unique<Runtime>())
{
    const auto device = settings["device"].as<std::string>("CPU");
    if (device != "CPU" && device != "GPU")
        throw std::runtime_error("detect.yolo.device must be CPU or GPU");

    runtime_->gpu  = device == "GPU";
    capacity_      = runtime_->gpu ? slots_.size() : 1;
    tracy_debug_   = settings["tracy_debug"].as<bool>(false);
    reuse_buffers_ = settings["reuse_buffers"].as<bool>(false);

    const auto model_path = (resource_dir / settings["model"].as<std::string>()).string();
    const auto model      = runtime_->core.read_model(model_path);

    // INT8 模型仍使用 FP32 接口，仅接受三类五点协议。
    if (model->inputs().size() != 1 || model->outputs().size() != 1 ||
        model->input().get_element_type() != ov::element::f32 ||
        model->output().get_element_type() != ov::element::f32 ||
        model->input().get_shape() != ov::Shape{1, 3, kInputHeight, kInputWidth})
        throw std::runtime_error("Expected FP32 pose I/O and input [1,3,480,640]");
    if (model->output().get_shape() != ov::Shape{1, kPoseChannels, 6300})
        throw std::runtime_error("Expected SZU-compatible pose output [1,18,6300]");

    if (runtime_->gpu)
    {
        runtime_->network =
            runtime_->core.compile_model(model, "GPU", ov::num_streams(2), ov::hint::allow_auto_batching(false));
    }
    else
    {
        const auto threads = settings["threads"].as<int>(4);
        if (threads < 1)
            throw std::runtime_error("detect.yolo.threads must be positive");

        runtime_->network =
            runtime_->core.compile_model(model, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
                                         ov::num_streams(1), ov::inference_num_threads(threads));
        MAS_LOG_INFO("YOLO CPU: threads={}", runtime_->network.get_property(ov::inference_num_threads));
    }

    for (std::size_t i = 0; i < capacity_; ++i)
        runtime_->requests[i] = runtime_->network.create_infer_request();

    MAS_LOG_INFO("YOLO OpenVINO: model={}, device={}, requests={}, io=FP32 [1,3,480,640] -> [1,18,6300]", model_path,
                 device, capacity_);
    MAS_LOG_INFO("YOLO: opencv_threads={}, streams={}, reuse_buffers={}", cv::getNumThreads(),
                 static_cast<int>(runtime_->network.get_property(ov::num_streams)), reuse_buffers_);
}

Inference::~Inference()
{
    // 原图与张量必须存活到在途请求结束。
    for (std::size_t i = 0; i < pending_; ++i)
    {
        try
        {
            runtime_->requests[(head_ + i) % capacity_].wait();
        }
        catch (...)
        {
            // 执行异常由调用方处理；析构期间只完成资源收尾。
        }
    }
}

float *Inference::input(std::size_t index)
{
    return runtime_->requests[index].get_input_tensor().data<float>();
}

void Inference::start(std::size_t index)
{
    if (runtime_->gpu)
        runtime_->requests[index].start_async();
    else
        runtime_->requests[index].infer();
}

bool Inference::ready(std::size_t index)
{
    return !runtime_->gpu || runtime_->requests[index].wait_for(std::chrono::milliseconds(0));
}

std::span<const float> Inference::output(std::size_t index)
{
    const auto tensor = runtime_->requests[index].get_output_tensor();
    return {tensor.data<const float>(), tensor.get_size()};
}
} // namespace auto_buff
