#include "auto_buff/buff_detect/buff_detector.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "rm_utils/display/display.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
#include <chrono>
#include "auto_buff/debug/buff_debug.hpp"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <thread>

int main(int argc, char *argv[])
{
    try
    {
        using namespace auto_buff;
        if (argc != 2 || std::string(argv[1]) == "--help")
        {
            std::cout << "buff_preview config.yaml\n";
            return 0;
        }
        cv::setNumThreads(1);
        const auto config_path = std::filesystem::absolute(tool::from_utf8(argv[1]));
        auto config            = YAML::LoadFile(config_path.string());
        const auto resolve     = [&](const std::string &value)
        { return config_path.parent_path() / tool::from_utf8(value); };
        auto detect   = config["auto_buff"]["detect"];
        auto detector = auto_buff::BuffDetector::create(config_path);
        if (!detector)
            throw std::runtime_error("Cannot create detector");
        tool::MediaReader reader(resolve(config["input"]["path"].as<std::string>()),
                                 config["input"]["type"].as<std::string>("video"));
        auto &display = rm_utils::Display::getInstance();
        if (detect["common"]["debug"].as<bool>(false))
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
            const auto start = std::chrono::steady_clock::now();
            if (!detector->submit(frame.image, frame.seconds))
                throw std::runtime_error("Previous frame was not collected");
            while (!detector->collect())
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            const auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            std::cout << "detect_ms=" << ms << '\n';
            display.end_frame();
        }
        display.shutdown();

        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "buff_preview: " << error.what() << '\n';
        return 1;
    }
}
