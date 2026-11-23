#include "rm_utils/display/rerun/rerun_display.hpp"
#include "tests/common/test.hpp"

#include <iostream>
#include <limits>

TEST_MAIN
{
    if (argc != 2 || std::string(argv[1]) == "--help")
    {
        std::cout << "rerun_scalar_test OUTPUT.rrd\n"
                     "Use tests/rm_utils/display/check_rerun_scalars.py to verify the recording.\n";
        return argc == 2 ? 0 : 1;
    }
    return tests::run("rerun_scalar_test", [&]
    {
        rm_utils::RerunDisplay display(false, argv[1], "", "jpeg", "png");
        display.set_time(0);
        display.scalar("test/value", 1.25);
        display.scalar("test/value/child", 10);
        display.set_time(1);
        display.scalar("test/value", std::numeric_limits<double>::quiet_NaN());
        display.set_time(2);
        display.scalar("test/value", std::numeric_limits<double>::infinity());
        display.set_time(3);
        display.scalar("test/value", 2.5);
        display.flush();
    });
}
