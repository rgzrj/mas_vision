#include "auto_buff/debug/buff_debug.hpp"
#include "auto_buff/debug/visualize/detect_debug.hpp"
#include "auto_buff/debug/visualize/track_debug.hpp"
#include "auto_buff/debug/visualize/shoot_debug.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "tests/common/test.hpp"
#include "tests/common/detection.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <type_traits>

TEST_MAIN
{
    return tests::run(
        "buff_debug_test",
        [&]
        {
            using namespace auto_buff;
            static_assert(std::is_base_of_v<BuffDebugBase, DetectDebug>);
            static_assert(std::is_base_of_v<BuffDebugBase, TrackDebug>);
            static_assert(std::is_base_of_v<BuffDebugBase, ShootDebug>);
            tests::require(BuffDebug::enabled_views(YAML::Load(
                               "auto_buff: {detect: {common: {debug: true}}, track: {debug: false}, shoot: {debug: true}}")) == 5,
                           "Stage debug switches do not select independent windows");
            tests::TemporaryDirectory temporary;
            const auto file = temporary.path / "debug.rrd";
            auto &display   = rm_utils::Display::getInstance();
            display.start(false, tool::utf8(file));
            BuffDebug::configure_layout(7, false);
            cv::Mat image(240, 320, CV_8UC3, cv::Scalar(10, 20, 30));
            const auto original = image.clone();
            tests::FrameDetector detect(tests::traditional_config("red", YAML::Load("debug: true")));
            TrackDebug track(true);
            ShootDebug shoot(true);
            for (int i = 0; i < 3; ++i)
            {
                display.set_time(i / 50.);
                const auto detection = tests::detect(detect, image, i / 50.);
                detect.draw(detection);
                track.draw({}, image);
                shoot.draw({}, {}, image);
                display.end_frame();
            }
            BuffDebug::unavailable(image, 7, "Pose unavailable");
            TrackDebug::clear();
            ShootDebug::clear();
            display.end_frame();
            tests::require(cv::norm(image, original, cv::NORM_INF) == 0, "Debug modified the camera buffer");
            display.shutdown();
            tests::require(std::filesystem::file_size(file) > 1024, "Debug recording is empty");
            std::ifstream input(file, std::ios::binary);
            char magic[4]{};
            input.read(magic, 4);
            tests::require(std::string(magic, 4) == "RRF2", "Invalid Rerun recording");
        });
}
