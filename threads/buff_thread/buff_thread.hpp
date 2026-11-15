#pragma once
#include "threads/vision_router/vision_router.hpp"
#include <filesystem>
namespace threads
{
// 算法由本线程持有，调试子类由各算法内部持有，不随电控模式切换重新创建。
void buff_thread_func(VisionQueue &, const std::filesystem::path &config);
} // namespace threads
