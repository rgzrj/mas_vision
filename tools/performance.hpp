#pragma once

#include "rm_utils/clock.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace tool
{
// 测速程序独立持有，生产算法结果中不再夹带计时数据。
struct ProcessingTiming
{
    double detect_ms   = std::numeric_limits<double>::quiet_NaN();
    double track_ms    = std::numeric_limits<double>::quiet_NaN();
    double shoot_ms    = std::numeric_limits<double>::quiet_NaN();
    double pipeline_ms = std::numeric_limits<double>::quiet_NaN();
};

class PerformanceLog
{
public:
    explicit PerformanceLog(const std::filesystem::path &file) : file_(file)
    {
        if (!file_)
            throw std::runtime_error("Cannot create profiling CSV");
        file_ << "frame,host_s,pipeline_ms,detect_ms,track_ms,shoot_ms\n" << std::setprecision(12);
    }
    void write(int index, const ProcessingTiming &timing)
    {
        const auto host = std::chrono::duration<double>(rm_utils::Clock::now().time_since_epoch()).count();
        file_ << index << ',' << host << ',' << timing.pipeline_ms << ',' << timing.detect_ms << ',' << timing.track_ms
              << ',' << timing.shoot_ms << '\n';
        if (!file_)
            throw std::runtime_error("Cannot write profiling CSV");
    }

private:
    std::ofstream file_;
};
} // namespace tool
