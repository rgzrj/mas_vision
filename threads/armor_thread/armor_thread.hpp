#pragma once
#include "threads/vision_router/vision_router.hpp"
#include <filesystem>
namespace threads
{
// 算法和Debug在线程内部装配；本线程负责逐帧计算、帧边界和结果提交。
void armor_thread_func(VisionCommand &commands, VisionQueue &, const std::filesystem::path &config);
} // namespace threads
