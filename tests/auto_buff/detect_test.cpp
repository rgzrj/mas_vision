#include "auto_buff/buff_detect/traditional/traditional.hpp"
#include "auto_buff/buff_detect/yolo/yolo.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "rm_utils/display/display.hpp"
#include "tests/common/test.hpp"
#include "auto_buff/debug/buff_debug.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>

void check_traditional();
void check_yolo();

TEST_MAIN
{
    return tests::run(
        "buff_detect_test",
        [&]
        {
            using namespace auto_buff;
            const auto argument = argc > 1 ? tool::utf8(tests::argument_path(argv[1])) : "--self-test";
            if (argument == "--help")
            {
                std::cout << "buff_detect_test [--self-test [traditional|yolo] | config.yaml]\n"
                             "Video/image mode uses input, auto_buff.detect and display from the test config.\n";
                return;
            }
            if (argument == "--self-test")
            {
                const auto backend = argc > 2 ? tool::utf8(tests::argument_path(argv[2])) : "all";
                tests::require(backend == "all" || backend == "traditional" || backend == "yolo",
                               "Unknown self-test backend");
                if (backend != "yolo")
                    check_traditional();
                if (backend != "traditional")
                    check_yolo();
                return;
            }
            const auto config_path = std::filesystem::absolute(tests::argument_path(argv[1]));
            auto config            = YAML::LoadFile(config_path.string());
            const auto resolve     = [&](const std::string &value)
            { return config_path.parent_path() / tool::from_utf8(value); };
            auto detect = config["auto_buff"]["detect"];
            std::unique_ptr<auto_buff::Detector> detector;
            const auto backend = detect["backend"].as<std::string>("traditional");
            if (backend == "traditional")
                detector = std::make_unique<auto_buff::TraditionalDetector>(
                    resolve(detect["camera_config"].as<std::string>("hikcamera.yaml")),
                    detect["color"].as<std::string>("red"), detect);
            else if (backend == "yolo")
                detector = std::make_unique<auto_buff::YoloDetector>(config_path);
            else
                throw std::runtime_error("detect.backend must be traditional or yolo");
            cv::setNumThreads(detect["threads"].as<int>(4));
            tool::MediaReader reader(resolve(config["input"]["path"].as<std::string>()),
                                     config["input"]["type"].as<std::string>("video"));
            auto &display = rm_utils::Display::getInstance();
            if (detect["debug"].as<bool>(false))
                display.start(config["display"], config_path);
            auto debug_config                           = YAML::Clone(config);
            debug_config["auto_buff"]["track"]["debug"] = false;
            debug_config["auto_buff"]["shoot"]["debug"] = false;
            auto_buff::BuffDebug::configure_layout(auto_buff::BuffDebug::enabled_views(debug_config), false);
            tool::Frame frame;
            while (reader.next(frame))
            {
                display.poll();
                display.set_time(frame.seconds);
                const auto start       = std::chrono::steady_clock::now();
                const auto observation = detector->detect(frame.image, frame.seconds);
                const auto ms =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
                std::cout << "detect_ms=" << ms << '\n';
                display.end_frame();
            }
            display.shutdown();
        });
}
