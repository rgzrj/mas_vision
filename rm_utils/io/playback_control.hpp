#pragma once
#include <utility>

namespace tool
{
// 回放控制只决定是否取下一帧，不改变帧的记录时间，也不重复提交当前帧。
struct PlaybackControl
{
    bool paused = false;
    bool quit = false;

    void key(int value)
    {
        if (value == 'q' || value == 27)
            quit = true;
        else if (value == ' ')
        {
            paused = !paused;
            step_ = false;
        }
        else if (value == 'n' && paused)
            step_ = true;
    }

    bool take_step() { return !quit && paused && std::exchange(step_, false); }

private:
    bool step_ = false;
};
}
