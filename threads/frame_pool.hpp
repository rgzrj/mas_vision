#pragma once

#include "hardware/common_def.hpp"
#include "SPSCQueue.h"

namespace threads {

inline bool take_latest_frame(rigtorp::SPSCQueue<CameraFrame> &buffer, CameraFrame &frame) {
    // 只跳过本轮已积压的旧帧；图像和时间戳一起移动，pop 只在消费端调用。
    for (auto pending = buffer.size(); pending > 1; --pending) {
        if (buffer.front())
            buffer.pop();
    }
    if (auto *latest = buffer.front()) {
        frame = std::move(*latest);
        buffer.pop();
        return true;
    }
    return false;
}

} // namespace threads
