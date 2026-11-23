#include "mas_log.hpp"

#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <utility>

namespace third_party
{

namespace
{
const auto fallback = spdlog::default_logger();

std::string make_log_path(const std::filesystem::path &path)
{
    const auto directory = path.parent_path();
    if (!directory.empty() && !std::filesystem::exists(directory))
        std::filesystem::create_directories(directory);

    const auto now = std::time(nullptr);
    std::ostringstream timestamp;
    timestamp << std::put_time(std::localtime(&now), "%Y-%m-%d_%H%M%S");

    return (directory / (path.stem().string() + "_" + timestamp.str() + path.extension().string())).string();
}
} // namespace

std::shared_ptr<spdlog::logger> MasLog::logger_ = fallback;

void MasLog::init(const std::string &log_path, spdlog::level::level_enum filelevel,
                  spdlog::level::level_enum consolelevel)
{
    shutdown();

    try
    {
        const auto file = make_log_path(log_path);

        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
        console_sink->set_level(consolelevel);

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>(file, true);
        file_sink->set_level(filelevel);

        const std::array<spdlog::sink_ptr, 2> sinks{console_sink, file_sink};

        // 文件可用后再启动日志工作线程；单工作线程独占两个 _st sink。
        spdlog::init_thread_pool(65536, 1);
        auto logger =
            std::make_shared<spdlog::async_logger>("mas_logger", sinks.begin(), sinks.end(), spdlog::thread_pool(),
                                                   spdlog::async_overflow_policy::overrun_oldest);

        logger->set_level(std::min(filelevel, consolelevel));
        logger->set_pattern("[%H:%M:%S.%e] [%^%l%$] [%s:%#] %v");

        spdlog::flush_every(std::chrono::seconds(3));
        logger->flush_on(spdlog::level::warn);

        spdlog::set_default_logger(logger);
        logger_ = std::move(logger);

        // 统一处理正常退出和提前返回，注册一次即可。
        static const bool registered = std::atexit(&MasLog::shutdown) == 0;
        if (!registered)
            std::fprintf(stderr, "Cannot register log shutdown\n");
            
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        spdlog::shutdown();
        logger_ = fallback;
        spdlog::set_default_logger(logger_);
        std::fprintf(stderr, "Log initialization failed: %s\n", ex.what());
    }
}

void MasLog::shutdown()
{
    if (logger_ == fallback)
        return;

    // 刷新先于线程池销毁；之后恢复终端兜底，避免悬空的异步 logger。
    logger_->flush();
    spdlog::shutdown();
    logger_ = fallback;
    spdlog::set_default_logger(logger_);
}

} // namespace third_party
