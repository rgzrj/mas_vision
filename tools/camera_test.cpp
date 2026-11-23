#include "hardware/usbcamera/usbcamera.hpp"
#include "hardware/usbcamera/usbcamera_config.hpp"
#include "hardware/hikcamera/hikcamera.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "rm_utils/io/media_recorder.hpp"
#include "rm_utils/io/runtime_path.hpp"
#include "rm_utils/display/display.hpp"
#include "mas_log.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <csignal>
#include <memory>

namespace
{
volatile std::sig_atomic_t interrupted = 0;
void stop_signal(int)
{
    interrupted = 1;
}
} // namespace

int main(int argc, char *argv[])
{
    try
    {
        if (argc == 1 || std::string(argv[1]) == "--help")
        {
            std::cout << "camera_test config/auto_buff.yaml [seconds=10] [--display]\n"
                         "Uses input.type and input.camera_config; no detector or serial connection.\n";
            return 0;
        }
        const auto path   = std::filesystem::absolute(tool::from_utf8(argv[1]));
        auto config       = YAML::LoadFile(path.string());
        const auto source = config["input"]["type"].as<std::string>("hikcamera");
        const auto camera_path =
            path.parent_path() / tool::from_utf8(config["input"]["camera_config"].as<std::string>("hikcamera.yaml"));
        const double seconds = argc > 2 ? std::stod(std::string(argv[2])) : 10;
        if (!std::isfinite(seconds) || seconds < 0)
            throw std::runtime_error("Duration must be >= 0; zero runs until Ctrl+C");
        const bool show   = argc > 3 && std::string(argv[3]) == "--display";
        const auto output = tool::create_session_directory(rm_utils::project_root() / "output/profiling");
        third_party::MasLog::init(tool::utf8(output / "camera_test.log"));
        std::cout << "Output: " << tool::utf8(output) << '\n';
        std::unique_ptr<Base_Camera> camera;
        if (source == "usbcamera")
        {
            if (!UsbCameraConfigManager::loadConfig(tool::utf8(camera_path)) || !UsbCameraConfigManager::isEnabled())
                throw std::runtime_error("USB config is invalid or enable=false");
            camera = std::make_unique<hardware::UsbCamera>(UsbCameraConfigManager::getDevicePath());
        }
        else if (source == "hikcamera")
        {
            camera = std::make_unique<hardware::HikCamera>(tool::utf8(camera_path));
        }
        else
            throw std::runtime_error("input.type must be hikcamera or usbcamera");
        if (!camera->openCamera())
            throw std::runtime_error("Cannot open camera");
        if (show)
            rm_utils::Display::getInstance().start(config["display"], path);
        interrupted = 0;
        std::signal(SIGINT, stop_signal);
        const auto start = std::chrono::steady_clock::now();
        auto first_stamp = start, last_stamp = start;
        std::int64_t frames = 0;
        while (
            !interrupted &&
            (seconds == 0 || std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds))
        {
            const auto frame = camera->getImage();
            if (show)
                rm_utils::Display::getInstance().poll();
            if (frame.frame.empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (!frames)
                first_stamp = frame.timestamp;
            last_stamp = frame.timestamp;
            ++frames;
            if (frames % 30 == 0)
                std::cout
                    << "frames=" << frames << " timestamp_ns="
                    << std::chrono::duration_cast<std::chrono::nanoseconds>(frame.timestamp.time_since_epoch()).count()
                    << " receive_delay_ms="
                    << std::chrono::duration<double, std::milli>(frame.t_recv - frame.timestamp).count() << '\n';
            if (show)
            {
                auto &display = rm_utils::Display::getInstance();
                display.set_time(std::chrono::duration<double>(frame.timestamp.time_since_epoch()).count());
                display.display_add("camera", frame.frame, {}, {}, {}, 1280,
                                    std::max(1, 1280 * frame.frame.rows / frame.frame.cols));
                display.end_frame();
                display.poll();
            }
        }
        camera->closeCamera();
        rm_utils::Display::getInstance().shutdown();
        if (frames == 0)
            throw std::runtime_error("Camera produced no frames");
        const double span = std::chrono::duration<double>(last_stamp - first_stamp).count();
        std::cout << "frames=" << frames << " exposure_fps=" << (frames > 1 && span > 0 ? (frames - 1) / span : 0)
                  << '\n';
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "camera_test: " << error.what() << '\n';
        return 1;
    }
}
