#include "vision_router.hpp"
#include "vision_command.hpp"

#include "hardware/serial/user_serial.hpp"
#include "rm_utils/display/display.hpp"

#include <atomic>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <yaml-cpp/yaml.h>

extern std::atomic<bool> g_shutdown;

namespace threads
{
void vision_router_func(VisionCommand &commands, rigtorp::SPSCQueue<CameraFrame> &camera,
                        VisionQueue &armor, VisionQueue &buff)
{
    auto &serial = UserSerial::getInstance();
    
    const auto armor_color =
        YAML::LoadFile("config/auto_aim.yaml")["auto_aim"]["armor_detector"]["detect_color"].as<std::string>("RED")
            == "BLUE" ? "blue" : "red";

    const auto buff_config = YAML::LoadFile("config/auto_buff.yaml");
    const auto buff_color  = buff_config["auto_buff"]["detect"]["common"]["color"].as<std::string>("red");

    const double age     = buff_config["runtime"]["max_frame_age_ms"].as<double>(100);
    const double seconds = buff_config["runtime"]["max_runtime_s"].as<double>(0);
    if (!std::isfinite(age) || age < 1 || age > 1000 || !std::isfinite(seconds) || seconds < 0)
        throw std::runtime_error("Invalid runtime duration or frame age");

    const auto max_age =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double, std::milli>(age));
    const auto started = std::chrono::steady_clock::now();

    std::optional<bool> shown_armor;

    while (!g_shutdown)
    {
        // 即使没有相机帧，定时运行也能正常结束；主线程仍负责等待与资源收尾。
        if (seconds > 0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >= seconds)
        {
            g_shutdown = true;
            break;
        }

        // 即使没有图像也排出串口姿态队列，避免模式闲置时积满。
        serial.pollPose();
        const auto state = commands.state();
        const bool aim   = state.mode <= AUTO_AIM_BLUE;

        // 这里只更新显示布局，SDL 窗口由主线程操作。
        if (shown_armor != aim)
        {
            rm_utils::Display::getInstance().select_layout(aim ? "armor" : "buff");
            shown_armor = aim;
        }

        // 0／2／4 沿用 YAML，1／3／5 选择蓝色；接裁判时 YAML 使用红色。
        auto color = aim ? armor_color : buff_color;
        if (state.mode == AUTO_AIM_BLUE || state.mode == SMALL_RUNE_BLUE || state.mode == BIG_RUNE_BLUE)
            color = "blue";

        CameraFrame frame;
        if (!take_latest_frame(camera, frame) || frame.frame.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        VisionFrame routed{std::move(frame), {}, state, color};
        routed.max_age = max_age;
        routed.pose    = serial.awaitPose(routed.camera.timestamp);

        // 队列满时丢弃本次投递；生产者绝不 pop。消费端取最新帧并丢弃旧 epoch。
        (void)(aim ? armor : buff).try_push(std::move(routed));
    }
}
} // namespace threads
