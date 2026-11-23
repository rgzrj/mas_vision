#include "rm_utils/io/playback_control.hpp"
#include "tests/common/test.hpp"

TEST_MAIN
{
    return tests::run("playback_control_test", []
    {
        tool::PlaybackControl control;
        tests::require(!control.paused && !control.quit && !control.take_step(), "Unexpected initial state");
        control.key('n');
        control.key(' ');
        tests::require(control.paused && !control.take_step(), "Playing n leaked a step into pause");
        for (int i = 0; i < 100; ++i)
            tests::require(!control.take_step(), "Pause advanced without input");
        control.key('n');
        tests::require(control.take_step() && !control.take_step() && control.paused,
                       "Single step did not advance exactly once and remain paused");
        control.key('n');
        control.key(' ');
        tests::require(!control.paused && !control.take_step(), "Resume retained a stale step request");
        control.key(' ');
        control.key('n');
        control.key('q');
        tests::require(control.quit && !control.take_step(), "Quit allowed another frame");
    });
}
