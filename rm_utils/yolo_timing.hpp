#pragma once
#include <chrono>
#include <tracy/TracyC.h>
#include <tracy/Tracy.hpp>

namespace rm_utils
{

// 同一算法线程内汇总一帧；普通计时不依赖 Tracy 是否连接。
struct YoloTiming
{
    bool enabled = false;
    double yolo_ms = 0, refine_ms = 0;
    double preprocess_ms = 0, infer_ms = 0, decode_ms = 0;
    unsigned refine_calls = 0;
};
inline thread_local YoloTiming yolo_timing;

inline void start_tracy()
{
    // 应用装配模块时显式启动；检测器不管理进程级客户端。
    static struct Session
    {
        Session() { tracy::StartupProfiler(); }
        ~Session()
        {
            if (tracy::IsProfilerStarted())
                tracy::ShutdownProfiler();
        }
    } session;
}

// 显式 finish 用于结束 YOLO 阶段；析构同时保证异常退出时 Tracy 区段成对结束。
class TimedZone
{
  public:
    TimedZone(double &elapsed, TracyCZoneCtx zone, bool measure = false)
        : elapsed_(elapsed), zone_(zone), measure_(measure || zone.active != 0),
          start_(measure_ ? Clock::now() : Clock::time_point{}) {}
    TimedZone(const TimedZone &) = delete;
    TimedZone &operator=(const TimedZone &) = delete;
    ~TimedZone() { finish(); }
    void finish()
    {
        if (finished_)
            return;
        if (measure_)
            elapsed_ += std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
        if (zone_.active)
            TracyCZoneEnd(zone_);
        finished_ = true;
    }

  private:
    using Clock = std::chrono::steady_clock;
    double &elapsed_;
    TracyCZoneCtx zone_;
    bool measure_;
    Clock::time_point start_;
    bool finished_ = false;
};
} // namespace rm_utils
