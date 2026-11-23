#include "mas_log.hpp"
#include "tests/common/test.hpp"

#include <cstdlib>
#include <fstream>
#include <iterator>
#include <thread>
#include <type_traits>
#include <vector>

namespace
{
std::filesystem::path directory;

std::string read_log(const char *stem)
{
    for (const auto &entry : std::filesystem::directory_iterator(directory))
        if (entry.path().filename().string().starts_with(stem))
        {
            std::ifstream stream(entry.path());
            return {std::istreambuf_iterator<char>(stream), {}};
        }
    throw std::runtime_error("Missing log file");
}

// 先登记检查，再由 init 登记 shutdown；退出时先收日志，再检查文件尾部。
void check_exit()
{
    try
    {
        const auto text = read_log("exit_");
        tests::require(text.find("final-marker") != std::string::npos, "Exit lost final log");
        for (int worker = 0; worker < 4; ++worker)
            for (int frame = 0; frame < 256; ++frame)
                tests::require(text.find("worker=" + std::to_string(worker) + " frame=" + std::to_string(frame) +
                                         "\n") != std::string::npos,
                               "Exit lost queued log");
        std::filesystem::remove_all(directory);
    }
    catch (const std::exception &error)
    {
        std::cerr << "mas_log_test exit: " << error.what() << '\n';
        std::_Exit(EXIT_FAILURE);
    }
}
} // namespace

int main()
{
    return tests::run(
        "mas_log_test",
        []
        {
            static_assert(
                std::is_same_v<decltype(third_party::MasLog::get_logger()), const std::shared_ptr<spdlog::logger> &>);
            directory = std::filesystem::temp_directory_path() /
                        ("mas_log_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
            tests::require(std::filesystem::create_directory(directory), "Cannot create log directory");
            tests::require(std::atexit(check_exit) == 0, "Cannot register exit check");

            third_party::MasLog::init((directory / "filter.log").string(), spdlog::level::warn, spdlog::level::off);
            int debug_calls = 0, trace_calls = 0;
            MAS_LOG_DEBUG("hidden-debug {}", ++debug_calls);
            MAS_LOG_TRACE("hidden-trace {}", ++trace_calls);
            MAS_LOG_INFO("hidden-info");
            MAS_LOG_WARN("visible-warn");
            MAS_LOG_ERROR("visible-error");
            third_party::MasLog::shutdown();
            third_party::MasLog::shutdown();

            constexpr int expected_calls = SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_DEBUG ? 1 : 0;
            tests::require(debug_calls == expected_calls, "DEBUG argument evaluation did not match the active level");
            tests::require(trace_calls == 0, "Disabled TRACE evaluated argument");
            const auto filtered = read_log("filter_");
            tests::require(filtered.find("hidden-") == std::string::npos, "Runtime level filter failed");
            tests::require(filtered.find("visible-warn") != std::string::npos &&
                               filtered.find("visible-error") != std::string::npos,
                           "Missing warning/error");
            tests::require(filtered.find("mas_log_test.cpp:") != std::string::npos, "Missing source context");
            tests::require(bool(third_party::MasLog::get_logger()), "Shutdown lost fallback logger");

            third_party::MasLog::init((directory / "exit.log").string(), spdlog::level::debug, spdlog::level::off);
            std::vector<std::thread> workers;
            for (int worker = 0; worker < 4; ++worker)
                workers.emplace_back(
                    [worker]
                    {
                        for (int frame = 0; frame < 256; ++frame)
                            MAS_LOG_INFO("worker={} frame={}", worker, frame);
                    });
            for (auto &worker : workers)
                worker.join();

            MAS_LOG_INFO("final-marker"); // 不显式 shutdown，覆盖 main 提前返回的自动收尾。
        });
}
