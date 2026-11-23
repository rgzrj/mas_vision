#include "yolo.hpp"

#include "inference/inference.hpp"
#include "observation/pose_output.hpp"
#include "rm_utils/yolo_timing.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>

namespace auto_buff
{
YoloDetector::YoloDetector(const YAML::Node &config, const std::filesystem::path &resource_dir) : BuffDetector(config)
{
    const auto settings = config["auto_buff"]["detect"]["yolo"];
    const auto features = settings["feature_points"].as<std::string>("rmcs_pixels");

    confidence_       = settings["confidence"].as<float>(0.25f);
    point_confidence_ = settings["point_confidence"].as<float>(0.8f);
    nms_distance_     = settings["nms_distance"].as<float>(30.0f);

    if (features != "none" && features != "rmcs_pixels")
        throw std::runtime_error("detect.yolo.feature_points must be none or rmcs_pixels");
    if (!std::isfinite(confidence_) || confidence_ < 0 || confidence_ > 1 || !std::isfinite(point_confidence_) ||
        point_confidence_ < 0 || point_confidence_ > 1 || !std::isfinite(nms_distance_) || nms_distance_ <= 0)
        throw std::runtime_error("Invalid YOLO confidence or NMS distance");

    refine_    = features == "rmcs_pixels";
    inference_ = std::make_unique<Inference>(settings, resource_dir);
}

YoloDetector::~YoloDetector() = default;

bool YoloDetector::submit(const cv::Mat &image, double seconds, RuneMode mode)
{
    return inference_->submit(image, seconds, mode, color());
}

std::optional<BuffDetector::Detection> YoloDetector::collect_impl()
{
    const auto *frame = inference_->front();
    if (!frame)
        return std::nullopt;

    auto &timing         = rm_utils::yolo_timing;
    timing               = {};
    timing.enabled       = frame->profiling;
    timing.preprocess_ms = frame->prepare_ms;
    // GPU 计时包含排队和等待收取，不代表纯设备执行耗时。
    if (timing.enabled)
        timing.infer_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frame->submitted).count();

    TracyCZoneN(zone, "YOLO decode", frame->profiling);
    rm_utils::TimedZone timer(timing.decode_ms, zone, frame->profiling);
    auto objects = decode_pose_output(frame->output, frame->output.size() / kPoseChannels, frame->image.size(),
                                      frame->scale, frame->pad, confidence_, point_confidence_, nms_distance_);

    timer.finish();
    timing.yolo_ms = timing.preprocess_ms + timing.infer_ms + timing.decode_ms;

    // 使用提交时的原图和颜色完成观测转换与精修。
    auto result      = make_pose_detection(objects, frame->image, frame->color, refine_, frame->profiling);
    result.raw_poses = std::move(objects);
    result.image     = frame->image;
    result.seconds   = frame->seconds;
    result.mode      = frame->mode;
    result.color     = frame->color;

    inference_->pop();
    return result;
}
} // namespace auto_buff
