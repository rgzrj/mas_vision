#include "inference.hpp"
#include "../observation/pose_output.hpp"

#include "mas_log.hpp"

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <opencv2/core/utility.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <vector>

namespace auto_buff
{
namespace
{
constexpr std::size_t kInputCount  = 3 * 480 * 640;
constexpr std::size_t kOutputCount = kPoseChannels * 6300;

void check_cuda(cudaError_t status, const char *operation)
{
    if (status != cudaSuccess)
        throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
}

class Logger final : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char *message) noexcept override
    {
        // SDK 回调禁止抛出异常；保留诊断，失败由对应 API 的返回值上报。
        try
        {
            if (severity <= Severity::kERROR)
                MAS_LOG_ERROR("TensorRT: {}", message);
            else if (severity == Severity::kWARNING)
                MAS_LOG_WARN("TensorRT: {}", message);
        }
        catch (...)
        {
        }
    }
};

void check_tensor(const nvinfer1::ICudaEngine &engine, const char *name, std::initializer_list<std::int64_t> expected)
{
    const auto shape = engine.getTensorShape(name);
    if (engine.getTensorDataType(name) != nvinfer1::DataType::kFLOAT ||
        engine.getTensorFormat(name) != nvinfer1::TensorFormat::kLINEAR ||
        engine.getTensorLocation(name) != nvinfer1::TensorLocation::kDEVICE ||
        shape.nbDims != static_cast<int>(expected.size()) || !std::equal(expected.begin(), expected.end(), shape.d))
        throw std::runtime_error(std::string("Unsupported TensorRT tensor: ") + name +
                                 "; expected static FP32/LINEAR [1,3,480,640] -> [1,18,6300]");
}
} // namespace

struct Inference::Runtime
{
    struct Request
    {
        std::unique_ptr<nvinfer1::IExecutionContext> context;
        cudaStream_t stream = nullptr;
        cudaEvent_t done    = nullptr;
        float *host_input   = nullptr;
        float *host_output  = nullptr;
        void *device_input  = nullptr;
        void *device_output = nullptr;

        ~Request()
        {
            // 也覆盖构造失败和提交中途失败，不能提前释放仍被 CUDA 使用的缓冲。
            if (stream)
                cudaStreamSynchronize(stream);
            context.reset();
            if (done)
                cudaEventDestroy(done);
            if (stream)
                cudaStreamDestroy(stream);
            if (device_output)
                cudaFree(device_output);
            if (device_input)
                cudaFree(device_input);
            if (host_output)
                cudaFreeHost(host_output);
            if (host_input)
                cudaFreeHost(host_input);
        }
    };

    Logger logger;
    std::unique_ptr<nvinfer1::IRuntime> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine> engine;
    std::array<Request, 2> requests; // 先释放上下文，再释放 engine、runtime 和 logger。
};

Inference::Inference(const YAML::Node &settings, const std::filesystem::path &resource_dir)
    : runtime_(std::make_unique<Runtime>())
{
    if (settings["device"].as<std::string>("GPU") != "GPU")
        throw std::runtime_error("TensorRT requires detect.yolo.device: GPU");

    capacity_             = slots_.size();
    tracy_debug_          = settings["tracy_debug"].as<bool>(false);
    reuse_buffers_        = settings["reuse_buffers"].as<bool>(false);
    const auto model_path = resource_dir / settings["model"].as<std::string>();
    if (model_path.extension() != ".engine")
        throw std::runtime_error("TensorRT requires a prebuilt .engine; build it on the target Jetson with trtexec");

    std::ifstream file(model_path, std::ios::binary | std::ios::ate);
    if (!file || file.tellg() <= 0)
        throw std::runtime_error("Cannot read TensorRT engine: " + model_path.string());
    std::vector<char> bytes(static_cast<std::size_t>(file.tellg()));
    file.seekg(0);
    if (!file.read(bytes.data(), static_cast<std::streamsize>(bytes.size())))
        throw std::runtime_error("Incomplete TensorRT engine: " + model_path.string());

    runtime_->runtime.reset(nvinfer1::createInferRuntime(runtime_->logger));
    if (!runtime_->runtime)
        throw std::runtime_error("Cannot create TensorRT runtime");
    runtime_->engine.reset(runtime_->runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!runtime_->engine)
        throw std::runtime_error("Cannot deserialize engine; rebuild with this Jetson's TensorRT: " +
                                 model_path.string());

    auto &engine = *runtime_->engine;
    if (engine.getNbIOTensors() != 2)
        throw std::runtime_error("TensorRT engine must have exactly one input and one output");
    const char *input_name  = nullptr;
    const char *output_name = nullptr;
    for (int i = 0; i < engine.getNbIOTensors(); ++i)
    {
        const auto *name = engine.getIOTensorName(i);
        if (engine.getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT)
            input_name = name;
        else if (engine.getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT)
            output_name = name;
    }
    if (!input_name || !output_name)
        throw std::runtime_error("TensorRT engine must have exactly one input and one output");
    check_tensor(engine, input_name, {1, 3, kInputHeight, kInputWidth});
    check_tensor(engine, output_name, {1, static_cast<std::int64_t>(kPoseChannels), 6300});

    for (auto &request : runtime_->requests)
    {
        request.context.reset(engine.createExecutionContext());
        if (!request.context)
            throw std::runtime_error("Cannot create TensorRT execution context");

        check_cuda(cudaStreamCreateWithFlags(&request.stream, cudaStreamNonBlocking), "Create CUDA stream");
        check_cuda(cudaEventCreateWithFlags(&request.done, cudaEventDisableTiming), "Create CUDA event");
        check_cuda(cudaMallocHost(reinterpret_cast<void **>(&request.host_input), kInputCount * sizeof(float)),
                   "Allocate input staging buffer");
        check_cuda(cudaMallocHost(reinterpret_cast<void **>(&request.host_output), kOutputCount * sizeof(float)),
                   "Allocate output staging buffer");
        check_cuda(cudaMalloc(&request.device_input, kInputCount * sizeof(float)), "Allocate device input");
        check_cuda(cudaMalloc(&request.device_output, kOutputCount * sizeof(float)), "Allocate device output");

        if (!request.context->setTensorAddress(input_name, request.device_input) ||
            !request.context->setTensorAddress(output_name, request.device_output))
            throw std::runtime_error("Cannot bind TensorRT input/output buffers");
    }

    MAS_LOG_INFO("YOLO TensorRT: model={}, device=GPU, requests={}, io=FP32 [1,3,480,640] -> [1,18,6300]",
                 model_path.string(), capacity_);
    MAS_LOG_INFO("YOLO: opencv_threads={}, reuse_buffers={}", cv::getNumThreads(), reuse_buffers_);
}

Inference::~Inference()
{
    runtime_.reset(); // 先等 CUDA 结束，再释放槽位保存的图像。
}

float *Inference::input(std::size_t index)
{
    return runtime_->requests[index].host_input;
}

void Inference::start(std::size_t index)
{
    auto &request = runtime_->requests[index];
    check_cuda(cudaMemcpyAsync(request.device_input, request.host_input, kInputCount * sizeof(float),
                               cudaMemcpyHostToDevice, request.stream),
               "Upload input");
    if (!request.context->enqueueV3(request.stream))
        throw std::runtime_error("TensorRT enqueueV3 failed");
    check_cuda(cudaMemcpyAsync(request.host_output, request.device_output, kOutputCount * sizeof(float),
                               cudaMemcpyDeviceToHost, request.stream),
               "Download output");
    check_cuda(cudaEventRecord(request.done, request.stream), "Record completion");
}

bool Inference::ready(std::size_t index)
{
    const auto status = cudaEventQuery(runtime_->requests[index].done);
    if (status == cudaErrorNotReady)
        return false;
    check_cuda(status, "Query completion");
    return true;
}

std::span<const float> Inference::output(std::size_t index)
{
    return {runtime_->requests[index].host_output, kOutputCount};
}
} // namespace auto_buff
