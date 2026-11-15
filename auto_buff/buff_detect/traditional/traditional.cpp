#include "traditional.hpp"
#include "auto_buff/buff_detect/traditional/process.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace auto_buff
{

TraditionalDetector::TraditionalDetector(const std::filesystem::path &config_path,
                                         const std::filesystem::path &camera_config)
    : TraditionalDetector(camera_config,
                          YAML::LoadFile(config_path.string())["auto_buff"]["detect"]["color"].as<std::string>("red"),
                          YAML::LoadFile(config_path.string())["auto_buff"]["detect"])
{
}

TraditionalDetector::TraditionalDetector(const std::filesystem::path &camera_config, std::string color,
                                         const YAML::Node &settings)
    : save_masks_(settings["save_masks"].as<bool>(false))
{
    set_color(color);
    detect_debug_.set_enabled(settings["debug"].as<bool>(false));
    // 与 mas_vision 一样，由使用参数的模块直接读取自己的标定。
    auto yaml            = YAML::LoadFile(camera_config.string());
    auto calibration     = yaml["calibration"] ? yaml["calibration"] : yaml;
    const auto intrinsic = calibration["camera_matrix"].as<std::array<double, 9>>();
    const auto distortion =
        calibration[yaml["calibration"] ? "distort_coeffs" : "distortion"].as<std::array<double, 5>>();
    calibration_size_ = {calibration["image_width"].as<int>(0), calibration["image_height"].as<int>(0)};
    // 兼容主工程不含宽高的标定；只在明确记录标定尺寸时校验输入，不自动缩放。
    if ((calibration_size_ != cv::Size{} && calibration_size_.empty()) || intrinsic[0] <= 0 || intrinsic[4] <= 0 ||
        std::abs(intrinsic[8] - 1) > 1e-8 ||
        std::abs(intrinsic[1]) + std::abs(intrinsic[3]) + std::abs(intrinsic[6]) + std::abs(intrinsic[7]) > 1e-8 ||
        !std::ranges::all_of(intrinsic, [](double v) { return std::isfinite(v); }) ||
        !std::ranges::all_of(distortion, [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("Invalid traditional camera calibration");
    auto &config            = detector_.config;
    config.intrinsic        = cv::Mat(3, 3, CV_64F, const_cast<double *>(intrinsic.data())).clone();
    config.distortion       = cv::Mat(1, 5, CV_64F, const_cast<double *>(distortion.data())).clone();
    config.min_distance     = settings["min_distance"].as<double>(config.min_distance);
    config.max_distance     = settings["max_distance"].as<double>(config.max_distance);
    config.max_perspective  = settings["max_perspective"].as<double>(config.max_perspective);
    config.active_threshold = settings["active_threshold"].as<double>(config.active_threshold);
    config.match_threshold  = settings["match_threshold"].as<double>(config.match_threshold);
    if (!std::isfinite(config.min_distance) || !std::isfinite(config.max_distance) || config.min_distance <= 0 ||
        config.max_distance <= config.min_distance || !std::isfinite(config.max_perspective) ||
        config.max_perspective < 0 || config.max_perspective >= 90 || !std::isfinite(config.active_threshold) ||
        config.active_threshold <= 0 || config.active_threshold >= 1 || !std::isfinite(config.match_threshold) ||
        config.match_threshold <= 0)
        throw std::runtime_error("Invalid RMCS detector distance, perspective or score thresholds");
}

Detection TraditionalDetector::detect_impl(const cv::Mat &image, double)
{
    if (image.empty() || image.type() != CV_8UC3)
        throw std::runtime_error("RMCS traditional detector needs a BGR image");
    if (calibration_size_ != cv::Size{} && image.size() != calibration_size_)
        throw std::runtime_error("Traditional input resolution does not match its camera calibration");
    Detection result;
    const auto color       = color_;
    detector_.config.color = color;
    const auto elements    = detector_.detect(image);
    // R 评分仅供候选排序，不提前淘汰几何上可能正确的低分 R。
    for (const auto &icon : elements.icons)
        result.markers.push_back(Marker{{icon.center.x, icon.center.y}, color_, icon.score, true});
    for (const auto &target : elements.bullseyes)
    {
        Board board;
        board.center         = {target.center.x, target.center.y};
        board.color          = color_;
        board.appearance     = target.active ? Appearance::single_ring : Appearance::full_mark;
        board.source         = "rmcs_traditional";
        board.pvd            = target.score;
        board.features_valid = !target.active;
        for (int i = 0; i < 4; ++i)
        {
            cv::Point2d point{target.corners[i].x, target.corners[i].y};
            board.radius = std::max(board.radius, cv::norm(point - board.center));
            // 上游圆环的 corners 仅用于估计半径，不是真实臂端点，不能传给 PnP。
            if (!target.active)
                board.corners[i] = point;
            board.features_valid &=
                point.x >= 1 && point.y >= 1 && point.x < image.cols - 1 && point.y < image.rows - 1;
        }
        result.boards.push_back(board);
    }
    if (save_masks_)
        auto_buff::extract_channel(image, color, result.mask);
    return result;
}

} // namespace auto_buff
