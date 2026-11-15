#include <filesystem>
#include "armor_detector.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "armor_types.hpp"
#include "mas_log.hpp"
#include "yaml-cpp/yaml.h"

namespace auto_aim
{
namespace
{
// 单像素颜色差异门限
constexpr int TARGET_COLOR_MARGIN = 20;

enum class ColorMatch
{
    OPPOSITE,           // 我方
    NEUTRAL,            // 中性
    TARGET,             // 敌方
};

/**
 * @brief 根据灯条颜色差异和有效采样点数量判断装甲板颜色
 * @param diff_sum 灯条颜色差异总和
 * @param valid_count 有效采样点数量
 * @return ColorMatch 装甲板颜色匹配结果
 */
constexpr ColorMatch classifyTargetColor(int diff_sum, int valid_count)
{
    if (valid_count <= 0) return ColorMatch::NEUTRAL;
    if (diff_sum < -TARGET_COLOR_MARGIN * valid_count) return ColorMatch::OPPOSITE;
    if (diff_sum > TARGET_COLOR_MARGIN * valid_count) return ColorMatch::TARGET;
    return ColorMatch::NEUTRAL;
}

} // namespace

ArmorDetector::ArmorDetector(const std::string &config_path)
{
    // 禁用 OpenCV 多线程，避免在处理小任务时产生过多的线程切换和 yielding 开销
    cv::setNumThreads(1);

    // yaml 读取参数
    try
    {
        // 检查文件是否存在
        std::ifstream file(config_path);
        if (!file.good())
        {
            MAS_LOG_ERROR("Config file not found: {}, using default values", config_path.c_str());
            return;
        }

        YAML::Node config = YAML::LoadFile(config_path);
        if (config["auto_aim"] && config["auto_aim"]["armor_detector"])
        {
            YAML::Node armor_detector = config["auto_aim"]["armor_detector"];
            detect_debug_.set_enabled(armor_detector["debug"].as<bool>(false));
            // 读取二值化参数
            binary_thres_ = armor_detector["binary_thres"].as<int>(90);
            // 读取颜色参数
            std::string color_str = armor_detector["detect_color"].as<std::string>("RED");
            if (color_str == "blue" || color_str == "BLUE")
            {
                detect_color_ = EnemyColor::BLUE;
            }
            else
            {
                detect_color_ = EnemyColor::RED;
            }
            // 读取参数
            YAML::Node lights_params = armor_detector["lights_params"];
            min_lightbar_ratio_      = lights_params["min_lightbar_ratio"].as<double>(0.1);
            max_lightbar_ratio_      = lights_params["max_lightbar_ratio"].as<double>(0.5);
            min_lightbar_length_     = lights_params["min_lightbar_length"].as<double>(8.0);
            max_angle_error_         = lights_params["max_angle_error"].as<double>(20.0) / 57.3; // rad
            max_lightbar_area_       = lights_params["max_lightbar_area"].as<double>(10000.0);

            YAML::Node armors_params = armor_detector["armors_params"];
            min_armor_ratio_         = armors_params["min_armor_ratio"].as<double>(0.7);
            max_armor_ratio_         = armors_params["max_armor_ratio"].as<double>(0.8);
            max_side_ratio_          = armors_params["max_side_ratio"].as<double>(3.2);
            max_rectangular_error_   = armors_params["max_rectangular_error"].as<double>(3.2) / 57.3; // rad

            // 读取数字识别参数
            if (armor_detector["number_params"])
            {
                YAML::Node  number_params = armor_detector["number_params"];
                std::string model_path    = number_params["model_path"].as<std::string>("../mas_auto_aim_armor/armor_detector/model/lenet.onnx");
                std::string label_path    = number_params["label_path"].as<std::string>("../mas_auto_aim_armor/armor_detector/model/label.txt");
                double      confidence    = number_params["classifier_threshold"].as<double>(0.7);

                std::vector<std::string> ignore_classes;
                // 读取忽略类别
                if (number_params["ignore_classes"])
                {
                    ignore_classes.clear();
                    for (const auto &cls : number_params["ignore_classes"])
                    {
                        ignore_classes.push_back(cls.as<std::string>());
                    }
                }
                // 创建数字识别器
                classifier = std::make_unique<NumberClassifier>(
                    (std::filesystem::path(config_path).parent_path() / model_path).string(),
                    (std::filesystem::path(config_path).parent_path() / label_path).string(), confidence,
                    ignore_classes);
            }
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_WARN("Failed to load config, using defaults: {}", e.what());
    }
}

std::vector<Armor> ArmorDetector::ArmorDetect(const cv::Mat &bgr_img) noexcept
{
    if (bgr_img.empty() || bgr_img.cols <= 0 || bgr_img.rows <= 0)
    {
        MAS_LOG_WARN("Invalid input image");
        return armors_;
    }

    if (bgr_img.channels() == 3)
    {
        // 使用 create 复用已分配的内存
        gray_.create(bgr_img.size(), CV_8UC1);
        cv::cvtColor(bgr_img, gray_, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray_ = bgr_img;
    }
    // 使用 create 复用已分配的内存
    binary_.create(gray_.size(), CV_8UC1);
    cv::threshold(gray_, binary_, binary_thres_, 255, cv::THRESH_BINARY);

    // 查找灯条
    lights_ = findLights(binary_, bgr_img);

    // 每根来源灯条在配板前只搜索一次端点，共享候选复用精修结果。
    light_corner_corrector_.correctCorners(lights_, gray_);

    // 查找装甲板
    armors_ = findArmors(lights_, bgr_img);

    detect_debug_.draw(bgr_img, armors_, lights_);
    return armors_;
}

std::vector<LightBar> ArmorDetector::findLights(const cv::Mat &bin_img, const cv::Mat &src_img) noexcept
{
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> lightbars;
    lightbars.reserve(20);

    for (const auto &contour : contours)
    {
        // 初步过滤
        if (contour.size() < 4)
        {
            continue;
        }
        // 计算轮廓面积
        double contour_area = cv::contourArea(contour);

        // 面积过滤
        if (contour_area > max_lightbar_area_)
        {
            continue;
        }

        // 旋转矩形
        auto r_rect = cv::minAreaRect(contour);

        // 初步创建灯条
        auto lightbar = LightBar(r_rect);

        // 根据几何条件过滤
        if (lightbar.angle_error > max_angle_error_)
        {
            continue;
        }
        if (lightbar.ratio < min_lightbar_ratio_ || lightbar.ratio > max_lightbar_ratio_)
        {
            continue;
        }
        if (lightbar.length < min_lightbar_length_)
        {
            continue;
        }

        // 颜色识别：在灯条区域内采样，避免单条中心线被反光或运动模糊误判
        if (!src_img.empty() && src_img.channels() == 3)
        {
            const int      cols      = src_img.cols;
            const int      rows      = src_img.rows;
            const uint8_t *pixel_ptr = src_img.data;
            const size_t   stride    = src_img.step;
            const bool     is_red    = (detect_color_ == EnemyColor::RED);

            constexpr int  N_LENGTH_SAMPLES = 10;
            constexpr int  N_WIDTH_SAMPLES  = 5;
            cv::Point2f    axis             = lightbar.bottom - lightbar.top;
            const float    axis_length      = cv::norm(axis);

            if (axis_length < 1e-3f)
            {
                continue;
            }

            axis *= 1.0f / axis_length;
            const cv::Point2f perpendicular(-axis.y, axis.x); // axis 的垂直方向单位向量
            int               diff_sum    = 0;
            int               valid_count = 0;

            for (int i = 1; i < N_LENGTH_SAMPLES; i++)
            {
                const float length_ratio = static_cast<float>(i) / N_LENGTH_SAMPLES;
                const cv::Point2f center = lightbar.top + (lightbar.bottom - lightbar.top) * length_ratio;
                for (int j = 0; j < N_WIDTH_SAMPLES; ++j)
                {
                    // 覆盖灯条核心及两侧光晕，避免只采到过曝白芯。
                    const float width_ratio = (static_cast<float>(j) - 2.0f) / 2.0f;
                    const cv::Point2f pt = center + perpendicular * static_cast<float>(lightbar.width * width_ratio * 0.75);
                    const int px = static_cast<int>(pt.x);
                    const int py = static_cast<int>(pt.y);
                    if (px < 0 || px >= cols || py < 0 || py >= rows) continue;

                    const uint8_t *pixel = pixel_ptr + py * stride + px * 3;
                    const int brightness = std::max({static_cast<int>(pixel[0]), static_cast<int>(pixel[1]), static_cast<int>(pixel[2])});
                    const int color_diff = is_red ? (pixel[2] - pixel[0]) : (pixel[0] - pixel[2]);
                    // 暗部噪声和白色饱和点都不携带可靠红蓝信息。
                    if (brightness < 40 || (color_diff >= -TARGET_COLOR_MARGIN && color_diff <= TARGET_COLOR_MARGIN)) continue;
                    diff_sum += color_diff;
                    valid_count++;
                }
            }

            const ColorMatch color_match = classifyTargetColor(diff_sum, valid_count);
            if (color_match == ColorMatch::OPPOSITE)
            {
                continue;
            }
            
            lightbar.target_color_confirmed = color_match == ColorMatch::TARGET;
            lightbar.color = is_red ? EnemyColor::RED : EnemyColor::BLUE;
        }

        lightbars.emplace_back(lightbar);
    }

    // 排序
    std::sort(lightbars.begin(), lightbars.end(), [](const LightBar &a, const LightBar &b) { return a.center.x < b.center.x; });

    return lightbars;
}

std::vector<Armor> ArmorDetector::findArmors(const std::vector<LightBar> &lights, const cv::Mat &src_img) noexcept
{
    std::vector<Armor> armors;
    std::vector<Armor> candidates; // 预筛选的候选装甲板

    // 几何特征筛选，收集候选装甲板
    for (auto left = lights.begin(); left != lights.end(); left++)
    {
        for (auto right = left + 1; right != lights.end(); right++)
        {
            if (left->color != right->color) continue;
            // 中性灯条可以补全一根过曝灯条，但两根都无法确认颜色时不能组成装甲板。
            if (!left->target_color_confirmed && !right->target_color_confirmed)
            {
                continue;
            }
            auto armor = Armor(*left, *right);

            const bool contain_light = containLight(left - lights.begin(), right - lights.begin(), lights);
            const bool reject_ratio  = armor.ratio < min_armor_ratio_ || armor.ratio > max_armor_ratio_;
            const bool reject_side_ratio = armor.side_ratio > max_side_ratio_;
            const bool reject_rectangular_error = armor.rectangular_error > max_rectangular_error_;
            // 几何特征判断
            if (contain_light || reject_ratio || reject_side_ratio || reject_rectangular_error)
                continue;

            // 只保留小装甲板，沿用精修前宽高比上限。
            if (armor.init_ratio > 3.0)
            {
                continue;
            }

            candidates.emplace_back(armor);
        }
    }

    for (auto &armor : candidates)
    {
        // 数字识别
        const auto reject_reason = classifier->classify(src_img, armor);

        if (reject_reason != NumberClassifier::RejectReason::PASS) continue;

        armors.emplace_back(armor);
    }

    return armors;
}

bool ArmorDetector::containLight(const int i, const int j, const std::vector<LightBar> &lights) noexcept
{
    const LightBar &light_1       = lights[i], light_2 = lights[j];
    double          avg_length    = (light_1.length + light_2.length) / 2.0;
    double          avg_width     = (light_1.width + light_2.width) / 2.0;

    const std::vector<cv::Point2f> armor_polygon{light_1.top, light_2.top, light_2.bottom, light_1.bottom};
    // 仅检查这两个灯条之间的灯条
    for (int k = i + 1; k < j; k++)
    {
        const LightBar &test_light = lights[k];

        // 防止数字干扰
        if (test_light.width > 2 * avg_width)
        {
            continue;
        }
        // 防止红点准星或弹丸干扰
        if (test_light.length < 0.5 * avg_length)
        {
            continue;
        }

        if (cv::pointPolygonTest(armor_polygon, test_light.top, false) >= 0 ||
            cv::pointPolygonTest(armor_polygon, test_light.bottom, false) >= 0 ||
            cv::pointPolygonTest(armor_polygon, test_light.center, false) >= 0)
        {
            return true;
        }
    }
    return false;
}

} // namespace auto_aim
