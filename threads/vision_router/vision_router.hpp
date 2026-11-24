#pragma once
#include "threads/frame_pool.hpp"
#include "hardware/serial/serial_types.hpp"
#include <chrono>
#include <optional>
#include <string>

namespace threads
{
class VisionCommand;

struct VisionFrame
{
    CameraFrame camera;
    std::optional<Eigen::Quaterniond> pose;

    VisionState state;
    std::string color;

    // router 从 runtime 读取；算法与指令验收共用这一帧有效期。
    std::chrono::steady_clock::duration max_age = std::chrono::milliseconds(100);
};
using VisionQueue = rigtorp::SPSCQueue<VisionFrame>;

// camera_queue 只由 router 消费；两个工作队列各由对应算法线程消费。
void vision_router_func(VisionCommand &commands, rigtorp::SPSCQueue<CameraFrame> &camera,
                        VisionQueue &armor, VisionQueue &buff);
} // namespace threads
