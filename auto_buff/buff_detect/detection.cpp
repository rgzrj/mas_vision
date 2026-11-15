#include "detection.hpp"
#include "traditional/traditional.hpp"
#include "yolo/yolo.hpp"
#include "rm_utils/yolo_timing.hpp"
#include <algorithm>
#include <stdexcept>

namespace auto_buff
{

void Detector::set_color(const std::string &color)
{
    color_ = parse_camp_color(color);
}

void filter_rune_observations(Detection &detection, RuneMode mode, CampColor color)
{
    if (color != CampColor::RED && color != CampColor::BLUE)
        throw std::invalid_argument("Observation color must be red or blue");

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
                  [&](const Marker &marker) { return marker.color != CampColor::UNKNOWN && marker.color != color; });
}

Detection Detector::detect(const cv::Mat &image, double seconds, RuneMode mode)
{
    auto detection = detect_impl(image, seconds);
    filter_rune_observations(detection, mode, color_);
    detect_debug_.draw(detection, image);
    return detection;
}

std::unique_ptr<Detector> make_detector(const std::filesystem::path &config_path)
{
    const auto detect = YAML::LoadFile(config_path.string())["auto_buff"]["detect"];
    if (detect["tracy_debug"].as<bool>(false))
        rm_utils::start_tracy();
    const auto backend = detect["backend"].as<std::string>("traditional");
    if (backend == "traditional")
        return std::make_unique<TraditionalDetector>(config_path, config_path.parent_path() / "hikcamera.yaml");
    if (backend == "yolo")
        return std::make_unique<YoloDetector>(config_path);
    throw std::runtime_error("detect.backend must be traditional or yolo");
}

} // namespace auto_buff
