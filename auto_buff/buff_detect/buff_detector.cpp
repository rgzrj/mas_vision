#include "buff_detector.hpp"

#include "traditional/traditional.hpp"
#include "yolo/yolo.hpp"
#include "auto_buff/debug/visualize/detect_debug.hpp"
#include "mas_log.hpp"
#include "rm_utils/yolo_timing.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <stdexcept>

namespace auto_buff
{
std::unique_ptr<BuffDetector> BuffDetector::create(const std::filesystem::path &config_path)
{
    try
    {
        const auto config  = YAML::LoadFile(config_path.string());
        const auto backend = config["auto_buff"]["detect"]["backend"].as<std::string>("traditional");

        MAS_LOG_INFO("Buff detector: backend={}, config={}", backend, config_path.string());

        if (backend == "yolo")
        {
            if (config["auto_buff"]["detect"]["yolo"]["tracy_debug"].as<bool>(false))
                rm_utils::start_tracy();

            return std::make_unique<YoloDetector>(config, config_path.parent_path());
        }
        if (backend == "traditional")
        {
            const auto camera = YAML::LoadFile((config_path.parent_path() / "hikcamera.yaml").string());
            return std::make_unique<TraditionalDetector>(config, camera);
        }

        throw std::runtime_error("detect.backend must be traditional or yolo");
    }
    catch (const std::exception &error)
    {
        MAS_LOG_ERROR("Buff detector initialization failed (config={}): {}", config_path.string(), error.what());
        return nullptr;
    }
}

BuffDetector::BuffDetector(const YAML::Node &config)
{
    const auto common = config["auto_buff"]["detect"]["common"];
    const auto name   = common["color"].as<std::string>("red");
    if (name != "red" && name != "blue")
        throw std::runtime_error("detect.common.color must be red or blue");

    color_ = name == "blue" ? EnemyColor::BLUE : EnemyColor::RED;
    debug_ = std::make_unique<DetectDebug>(common["debug"].as<bool>(false));
}

BuffDetector::~BuffDetector() = default;

void BuffDetector::disable_debug()
{
    debug_->set_enabled(false);
}

std::optional<BuffDetector::Detection> BuffDetector::collect()
{
    auto result = collect_impl();
    if (result)
    {
        filter_rune_observations(*result, result->mode, result->color);
        debug_->draw(*result, result->image);
    }
    return result;
}

void filter_rune_observations(BuffDetector::Detection &detection, RuneMode mode, EnemyColor color)
{
    detection.mode = mode;
    std::erase_if(detection.boards,
                  [&](const Board &board)
                  {
                      return board.color != color ||
                             (mode == RuneMode::big && board.appearance == Appearance::single_ring) ||
                             (mode == RuneMode::small && board.appearance == Appearance::big_activated);
                  });
    std::erase_if(detection.light_arms,
                  [&](const LightArm &arm) { return mode == RuneMode::small || arm.color != color; });
    std::erase_if(detection.markers,
                  [&](const RuneIcon &marker) { return marker.color != EnemyColor::UNKNOWN && marker.color != color; });
}
} // namespace auto_buff
