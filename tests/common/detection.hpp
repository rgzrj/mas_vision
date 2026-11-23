#pragma once

#include "auto_buff/buff_detect/buff_detector.hpp"
#include "tests/common/test.hpp"
#include <yaml-cpp/yaml.h>
#include <thread>

namespace tests
{
// 离线检查逐帧等待，生产接口仍只有提交和收取。
inline auto_buff::BuffDetector::Detection detect(auto_buff::BuffDetector &detector, const cv::Mat &image,
                                                 double seconds,
                                                 auto_buff::RuneMode mode = auto_buff::RuneMode::unknown)
{
    require(detector.submit(image, seconds, mode), "Previous detection was not collected");
    auto result = detector.collect();
    while (!result)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        result = detector.collect();
    }
    return std::move(*result);
}

inline YAML::Node traditional_config(const std::string &color = "red", const YAML::Node &settings = YAML::Node{})
{
    auto root = YAML::Load("auto_buff: {detect: {backend: traditional, common: {}, traditional: {}}}");
    root["auto_buff"]["detect"]["common"]["color"] = color;
    root["auto_buff"]["detect"]["common"]["debug"] = settings["debug"].as<bool>(false);
    root["auto_buff"]["detect"]["traditional"]     = settings;
    return root;
}

struct FrameDetector : auto_buff::BuffDetector
{
    explicit FrameDetector(const YAML::Node &config = traditional_config()) : BuffDetector(config) {}
    Detection observation;
    std::optional<Detection> pending;

    bool submit(const cv::Mat &image, double seconds, auto_buff::RuneMode mode = auto_buff::RuneMode::unknown) override
    {
        if (pending)
            return false;
        pending          = observation;
        pending->image   = image;
        pending->seconds = seconds;
        pending->mode    = mode;
        pending->color   = color();
        return true;
    }

    std::optional<Detection> collect_impl() override
    {
        auto result = std::move(pending);
        pending.reset();
        return result;
    }
};
} // namespace tests
