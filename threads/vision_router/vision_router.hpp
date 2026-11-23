#pragma once
#include "threads/frame_pool.hpp"
#include "hardware/serial/vision_command.hpp"
#include <chrono>
#include <optional>
#include <string>

namespace threads
{
enum class VisionTask
{
    armor,
    small_rune,
    big_rune
};

struct VisionFrame
{
    CameraFrame camera;
    VisionState state;
    std::string color;
    std::optional<Eigen::Quaterniond> pose;
    VisionTask task = VisionTask::armor;
    // router从runtime读取，两个算法线程及串口验收使用同一个帧有效期。
    std::chrono::steady_clock::duration max_age = std::chrono::milliseconds(100);
};
using VisionQueue = rigtorp::SPSCQueue<VisionFrame>;

// camera_queue 只由 router 消费；两个工作队列各由对应算法线程消费。
void vision_router_func(rigtorp::SPSCQueue<CameraFrame> &camera, VisionQueue &armor, VisionQueue &buff);
} // namespace threads
