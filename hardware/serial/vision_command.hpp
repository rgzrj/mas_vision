#pragma once
#include "serial_types.hpp"
#include <cmath>
#include <mutex>

struct VisionState
{
    VisionMode mode     = AUTO_AIM_RED;
    std::uint64_t epoch = 0;
};

// 模式切换、结果验收和发送快照共用一把锁。算法计算期间不持锁。
class VisionCommand
{
public:
    using Clock = std::chrono::steady_clock;

    VisionState state() const
    {
        std::lock_guard lock(mutex_);
        return state_;
    }

    void set_mode(VisionMode mode)
    {
        std::lock_guard lock(mutex_);
        if (state_.mode == mode)
            return;
        state_.mode = mode;
        ++state_.epoch;
        invalidate();
    }

    bool submit(VisionState source, Clock::time_point exposure, Clock::duration max_age, SendPacket packet)
    {
        std::lock_guard lock(mutex_);
        const auto now = Clock::now();
        if (source.mode != state_.mode || source.epoch != state_.epoch)
            return false;
        if (exposure > now || max_age <= Clock::duration::zero() || now - exposure > max_age ||
            !std::isfinite(packet.target_yaw) || !std::isfinite(packet.target_pitch))
        {
            invalidate();
            return false;
        }
        packet.fire_advice = packet.found ? packet.fire_advice : 0;
        packet_            = packet;
        deadline_          = exposure + max_age;
        return true;
    }

    SendPacket packet(Clock::time_point now)
    {
        std::lock_guard lock(mutex_);
        if (now >= deadline_)
            invalidate();
        return packet_;
    }

    /**
     * @brief 开启新会话，丢弃旧快照。
     * @details 该函数必须在持锁的情况下调用。
     */
    void new_session()
    {
        std::lock_guard lock(mutex_);
        ++state_.epoch;
        invalidate();
    }

    void clear()
    {
        std::lock_guard lock(mutex_);
        invalidate();
    }

    bool clear(VisionState source)
    {
        std::lock_guard lock(mutex_);
        if (source.mode != state_.mode || source.epoch != state_.epoch)
            return false;

        invalidate();
        return true;
    }

private:
    /**
     * @brief 清空发送快照，避免过期数据被误用。
     * @details 该函数必须在持锁的情况下调用。
     */
    void invalidate()
    {
        packet_.found       = false;
        packet_.fire_advice = 0;
        deadline_           = {};
    }
    mutable std::mutex mutex_;
    VisionState state_;
    SendPacket packet_{};
    Clock::time_point deadline_{};
};
