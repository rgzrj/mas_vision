#include "auto_aim/armor_detector/armor_detector.hpp"
#include "tests/common/test.hpp"

#include <fstream>
#include <cmath>
#include <yaml-cpp/yaml.h>

int main()
{
    return tests::run("armor_detector_test", []
    {
        using tests::require;
        // 包括左右倾斜、两灯条角度不同，以及整块装甲板滚转的情况。
        struct Angles { float roll, left, right; double deviation; };
        for (const auto &angles : {Angles{0, 0, 0, 0}, {0, 20, 20, 20}, {0, -20, -20, 20},
                                   {0, 10, 30, 30}, {15, 35, 35, 20}, {-15, -35, -35, 20}})
        {
            const double roll = angles.roll * CV_PI / 180.0;
            const cv::Point2f left_center(190, 240);
            const auto right_center = left_center + cv::Point2f(float(100 * std::cos(roll)), float(100 * std::sin(roll)));
            auto_aim::Armor armor(auto_aim::LightBar(cv::RotatedRect(left_center, {8, 50}, angles.left)),
                                  auto_aim::LightBar(cv::RotatedRect(right_center, {8, 50}, angles.right)));
            require(std::abs(armor.rectangular_error - angles.deviation * CV_PI / 180.0) < 1e-5,
                    "Rectangular error is not the angular deviation in radians");
        }
        const std::filesystem::path root(BUFF_PROJECT_DIR);
        tests::TemporaryDirectory temporary;
        const auto path = temporary.path / "auto_aim.yaml";
        auto config = YAML::LoadFile((root / "config/auto_aim.yaml").string());
        auto parameters = config["auto_aim"]["armor_detector"];
        parameters["debug"] = false;
        for (const auto *key : {"model_path", "label_path"})
            parameters["number_params"][key] =
                std::filesystem::absolute(root / "config" /
                                          parameters["number_params"][key].as<std::string>()).string();

        const auto require_failure = [&]
        {
            bool rejected = false;
            try
            {
                auto_aim::ArmorDetector detector(path.string());
            }
            catch (const std::exception &)
            {
                rejected = true;
            }
            require(rejected, "Invalid detector initialization was silently accepted");
        };

        // 缺文件、损坏 YAML、缺配置段都必须在构造时失败。
        require_failure();
        for (const auto *text : {"auto_aim: [", "{}", "auto_aim: {}", "auto_aim: {armor_detector: []}"})
        {
            std::ofstream(path) << text;
            require_failure();
        }
        for (const auto *group : {"lights_params", "armors_params", "number_params"})
        {
            auto invalid = YAML::Clone(config);
            invalid["auto_aim"]["armor_detector"].remove(group);
            std::ofstream(path) << invalid;
            require_failure();
        }

        // 模型和标签既不能缺字段，也不能指向不存在的文件。
        for (const auto *key : {"model_path", "label_path"})
        {
            auto invalid = YAML::Clone(config);
            auto numbers = invalid["auto_aim"]["armor_detector"]["number_params"];
            numbers.remove(key);
            std::ofstream(path) << invalid;
            require_failure();
            numbers[key] = (temporary.path / "missing").string();
            std::ofstream(path) << invalid;
            require_failure();
        }
        const auto empty_labels = temporary.path / "empty_labels.txt";
        std::ofstream(empty_labels).close();
        auto invalid = YAML::Clone(config);
        invalid["auto_aim"]["armor_detector"]["number_params"]["label_path"] = empty_labels.string();
        std::ofstream(path) << invalid;
        require_failure();

        // 只放宽数字类别筛选，几何检测、精修与分类都执行生产实现。
        parameters["number_params"]["classifier_threshold"] = 0;
        parameters["number_params"]["ignore_classes"] = YAML::Node(YAML::NodeType::Sequence);
        std::ofstream(path) << config;
        auto_aim::ArmorDetector detector(path.string());
        require(detector.ArmorDetect({}).empty(), "Initial empty input returned a detection");
        const cv::Mat black = cv::Mat::zeros(480, 640, CV_8UC3);
        auto sample = black.clone();
        cv::rectangle(sample, {190, 200, 8, 50}, {0, 100, 255}, cv::FILLED);
        cv::rectangle(sample, {290, 200, 8, 50}, {0, 100, 255}, cv::FILLED);
        require(detector.ArmorDetect(sample).size() == 1, "Synthetic armor was not detected");
        require(detector.ArmorDetect({}).empty(), "Empty input reused the previous armor");
        require(detector.ArmorDetect(sample).size() == 1, "Detector did not recover after empty input");
        require(detector.ArmorDetect(black).empty(), "Blank image retained the previous armor");

        // 判色重构前后都执行这些输入，覆盖中性补全、异色拒绝、边界及非连续图像。
        const cv::Scalar red(0, 140, 255), blue(255, 140, 0), white(255, 255, 255);
        const auto color_sample = [&](cv::Scalar left, cv::Scalar right, int x = 190)
        {
            auto image = black.clone();
            cv::rectangle(image, {x, 200, 8, 50}, left, cv::FILLED);
            cv::rectangle(image, {x + 100, 200, 8, 50}, right, cv::FILLED);
            return image;
        };
        for (auto color : {auto_aim::EnemyColor::RED, auto_aim::EnemyColor::BLUE})
        {
            detector.ArmorDetector_Set_Color(color);
            const auto target = color == auto_aim::EnemyColor::RED ? red : blue;
            const auto opposite = color == auto_aim::EnemyColor::RED ? blue : red;
            require(detector.ArmorDetect(color_sample(target, target)).size() == 1, "Target-color armor rejected");
            require(detector.ArmorDetect(color_sample(opposite, opposite)).empty(), "Opposite-color armor accepted");
            require(detector.ArmorDetect(color_sample(target, white)).size() == 1, "Neutral light failed to complete armor");
            require(detector.ArmorDetect(color_sample(white, white)).empty(), "Two neutral lights formed armor");
            require(detector.ArmorDetect(color_sample(target, opposite)).empty(), "Mixed-color lights formed armor");
            require(detector.ArmorDetect(color_sample(target, target, 0)).size() == 1, "Image-edge armor rejected");
            cv::Mat padded(500, 680, CV_8UC3, cv::Scalar());
            auto roi = padded(cv::Rect(10, 10, 640, 480));
            color_sample(target, white).copyTo(roi);
            require(!roi.isContinuous() && detector.ArmorDetect(roi).size() == 1, "Color sampling ignored image stride");
        }
        detector.ArmorDetector_Set_Color(auto_aim::EnemyColor::RED);

        const auto tilted_image = [&](float degrees)
        {
            auto image = black.clone();
            for (float x : {190.f, 290.f})
            {
                cv::Point2f vertices[4];
                cv::RotatedRect({x, 240}, {8, 50}, degrees).points(vertices);
                std::vector<cv::Point> polygon;
                for (const auto &vertex : vertices)
                    polygon.emplace_back(cvRound(vertex.x), cvRound(vertex.y));
                cv::fillConvexPoly(image, polygon, {0, 100, 255});
            }
            return image;
        };
        parameters["armors_params"]["max_rectangular_error"] = 25;
        std::ofstream(path) << config;
        auto_aim::ArmorDetector angular_detector(path.string());
        require(angular_detector.ArmorDetect(tilted_image(20)).size() == 1, "20 degree armor failed a 25 degree limit");
        require(angular_detector.ArmorDetect(tilted_image(30)).empty(), "30 degree armor passed a 25 degree limit");
        parameters["armors_params"]["max_rectangular_error"] = 35;
        std::ofstream(path) << config;
        auto_aim::ArmorDetector relaxed_detector(path.string());
        require(relaxed_detector.ArmorDetect(tilted_image(30)).size() == 1, "Configured angle limit was not applied");
    });
}
