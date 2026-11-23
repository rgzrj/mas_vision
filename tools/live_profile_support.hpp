#pragma once

#include <opencv2/imgproc.hpp>
#include <openvino/openvino.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace tool
{
using ProfileClock = std::chrono::steady_clock;

// 限制新帧接纳上限；预处理计入周期，错过的时段不补发。
class FramePacer
{
  public:
    explicit FramePacer(double fps)
    {
        if (!std::isfinite(fps) || fps <= 0 || fps > 1000)
            throw std::invalid_argument("fps must be in (0,1000]");
        interval_ = std::chrono::ceil<ProfileClock::duration>(std::chrono::duration<double>(1 / fps));
    }

    bool due(ProfileClock::time_point now) const
    {
        return now >= next_;
    }
    void accepted(ProfileClock::time_point now)
    {
        next_ = now + interval_;
    }

  private:
    ProfileClock::duration interval_{};
    ProfileClock::time_point next_{};
};

struct Letterbox
{
    double scale;
    cv::Point pad;
};

// 测试入口与生产YOLO采用相同的letterbox、RGB/NCHW/255；每个请求独占输入张量。
inline Letterbox prepare_profile_input(const cv::Mat &image, ov::Tensor tensor)
{
    if (image.empty() || image.type() != CV_8UC3)
        throw std::invalid_argument("Expected an 8-bit BGR frame");
    const auto shape = tensor.get_shape();
    if (shape.size() != 4 || shape[0] != 1 || shape[1] != 3 || tensor.get_element_type() != ov::element::f32)
        throw std::invalid_argument("Expected FP32 input [1,3,H,W]");

    const int width = int(shape[3]), height = int(shape[2]);
    const double scale = std::min(double(width) / image.cols, double(height) / image.rows);
    const cv::Size size(cvRound(image.cols * scale), cvRound(image.rows * scale));
    const cv::Point pad((width - size.width) / 2, (height - size.height) / 2);
    cv::Mat input(height, width, CV_8UC3, cv::Scalar(114, 114, 114)), floating;
    cv::resize(image, input(cv::Rect(pad, size)), size);
    input.convertTo(floating, CV_32F, 1. / 255.);

    auto *data      = tensor.data<float>();
    const int plane = width * height;
    std::array<cv::Mat, 3> channels{cv::Mat(height, width, CV_32F, data + 2 * plane),
                                    cv::Mat(height, width, CV_32F, data + plane), cv::Mat(height, width, CV_32F, data)};
    cv::split(floating, channels.data());
    return {scale, pad};
}
} // namespace tool
