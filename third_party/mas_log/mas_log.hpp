#pragma once

#include "spdlog/spdlog.h"
#include <memory>
#include <spdlog/common.h>
#include <string>

#define MAS_LOG_TRACE(...)                                                                         \
    SPDLOG_LOGGER_TRACE(third_party::MasLog::get_logger(), __VA_ARGS__)
#define MAS_LOG_DEBUG(...)                                                                         \
    SPDLOG_LOGGER_DEBUG(third_party::MasLog::get_logger(), __VA_ARGS__)
#define MAS_LOG_INFO(...)                                                                          \
    SPDLOG_LOGGER_CALL(third_party::MasLog::get_logger(), spdlog::level::info, __VA_ARGS__)
#define MAS_LOG_WARN(...)                                                                          \
    SPDLOG_LOGGER_CALL(third_party::MasLog::get_logger(), spdlog::level::warn, __VA_ARGS__)
#define MAS_LOG_ERROR(...)                                                                         \
    SPDLOG_LOGGER_CALL(third_party::MasLog::get_logger(), spdlog::level::err, __VA_ARGS__)
#define MAS_LOG_CRITICAL(...)                                                                      \
    SPDLOG_LOGGER_CALL(third_party::MasLog::get_logger(), spdlog::level::critical, __VA_ARGS__)

namespace third_party
{
class MasLog
{
  public:
    /**
     * @brief 初始化日志系统
     * @param log_path 日志文件保存路径
     * @param filelevel 文件日志的输出级别
     * @param consolelevel 控制台日志的输出级别
     * @note 在工作线程启动前调用；切换会话须先停止旧任务，旧日志会先刷新关闭。
     */
    static void init(const std::string        &log_path     = "logs/mas_vision.log",
                     spdlog::level::level_enum filelevel    = spdlog::level::debug,
                     spdlog::level::level_enum consolelevel = spdlog::level::info);

    /**
     * @brief 刷新并关闭日志；正常进程退出时自动调用，必须晚于所有日志生产者。
     */
    static void shutdown();

    /**
     * @brief 获取全局 logger 实例
     * @return logger 的只读 shared_ptr 引用，不允许调用方替换实例。
     */
    static const std::shared_ptr<spdlog::logger> &get_logger()
    {
        return logger_;
    }

  private:
    static std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace third_party
