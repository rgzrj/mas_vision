#include "inference.hpp"

#include "rm_utils/yolo_timing.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cassert>
#include <stdexcept>

namespace auto_buff
{
bool Inference::submit(const cv::Mat &image, double seconds, RuneMode mode, EnemyColor color)
{
    if (pending_ == capacity_)
        return false;

    const auto index = (head_ + pending_) % capacity_;
    auto &frame      = slots_[index];
    prepare(frame, image, input(index));
    frame.seconds = seconds;
    frame.mode    = mode;
    frame.color   = color;

    {
        ZoneNamedN(zone, "YOLO inference", frame.profiling);
        start(index);
    }

    ++pending_;
    return true;
}

const Inference::Slot *Inference::front()
{
    if (!pending_)
        return nullptr;

    auto &frame = slots_[head_];
    if (!ready(head_))
        return nullptr;

    frame.output = output(head_);
    return &frame;
}

void Inference::pop()
{
    assert(pending_ > 0 && !slots_[head_].output.empty());

    auto &frame  = slots_[head_];
    frame.output = {};
    frame.image.release();

    head_ = (head_ + 1) % capacity_;
    --pending_;
}

void Inference::prepare(Slot &frame, const cv::Mat &image, float *data)
{
    if (image.empty() || image.type() != CV_8UC3)
        throw std::runtime_error("YOLO needs a BGR image");

    frame.image      = image;
    frame.profiling  = tracy_debug_ && tracy::IsProfilerStarted();
    frame.prepare_ms = 0.0;

    TracyCZoneN(zone, "YOLO preprocess", frame.profiling);
    rm_utils::TimedZone timer(frame.prepare_ms, zone, frame.profiling);

    // 居中 letterbox，保存还原坐标所需的缩放和填充量。
    frame.scale = std::min(double(kInputWidth) / image.cols, double(kInputHeight) / image.rows);
    const cv::Size size(cvRound(image.cols * frame.scale), cvRound(image.rows * frame.scale));
    frame.pad = {(kInputWidth - size.width) / 2, (kInputHeight - size.height) / 2};

    if (!reuse_buffers_)
    {
        input_.release();
        floating_.release();
    }
    input_.create(kInputHeight, kInputWidth, CV_8UC3);
    input_.setTo(cv::Scalar(114, 114, 114));
    cv::resize(image, input_(cv::Rect(frame.pad, size)), size);
    input_.convertTo(floating_, CV_32F, 1.0 / 255.0);

    // 按反向平面写入 B/G/R，得到 RGB/NCHW；每个请求独占输入张量。
    constexpr auto plane = kInputWidth * kInputHeight;
    std::array<cv::Mat, 3> channels{cv::Mat(kInputHeight, kInputWidth, CV_32F, data + 2 * plane),
                                    cv::Mat(kInputHeight, kInputWidth, CV_32F, data + plane),
                                    cv::Mat(kInputHeight, kInputWidth, CV_32F, data)};
    cv::split(floating_, channels.data());

    timer.finish();
    frame.submitted = frame.profiling ? Clock::now() : Clock::time_point{};
}

} // namespace auto_buff
