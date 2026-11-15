#pragma once
#include <chrono>
#include <optional>

namespace rm_utils {

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration  = TimePoint::duration;
using Timestamp = TimePoint;

inline double elapsed_ms(Timestamp begin, Timestamp end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

// Offline frames use media time; live frames/commands use the same host steady_clock epoch.
inline thread_local std::optional<Timestamp> playback_timestamp;
inline Timestamp playback_now() { return playback_timestamp.value_or(Clock::now()); }

} // namespace rm_utils
