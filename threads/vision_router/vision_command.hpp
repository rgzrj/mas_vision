#pragma once
#include "hardware/serial/serial_types.hpp"
#include <cmath>
#include <mutex>

namespace threads
{
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

    // 串口线程发布接收状态；模式切换或重连后撤销上一轮指令。
    void update(VisionState state)
    {
        std::lock_guard lock(mutex_);
        if (state.mode == state_.mode && state.epoch == state_.epoch)
            return;

        state_ = state;
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

    SendPacket packet(Clock::time_point now, bool pose_valid = true)
    {
        std::lock_guard lock(mutex_);
        if (!pose_valid || now >= deadline_)
            invalidate();
        return packet_;
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
     * @brief 撤销跟随和开火，保留最后一次目标角度。
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
} // namespace threads
