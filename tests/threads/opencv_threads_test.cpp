#include "tests/common/detection.hpp"
#include "auto_aim/armor_detector/armor_detector.hpp"
#include "auto_buff/buff_detect/yolo/yolo.hpp"
#include "tests/common/test.hpp"

#include <fstream>
#include <yaml-cpp/yaml.h>

int main()
{
    return tests::run("opencv_threads_test", []
    {
        using tests::require;
        const std::filesystem::path root(BUFF_PROJECT_DIR);
        tests::TemporaryDirectory temporary;
        const auto aim_path = temporary.path / "auto_aim.yaml";
        const auto buff_path = temporary.path / "auto_buff.yaml";

        auto aim = YAML::LoadFile((root / "config/auto_aim.yaml").string());
        aim["auto_aim"]["armor_detector"]["debug"] = false;
        for (const auto *key : {"model_path", "label_path"})
            aim["auto_aim"]["armor_detector"]["number_params"][key] =
                std::filesystem::absolute(root / "config" /
                                          aim["auto_aim"]["armor_detector"]["number_params"][key].as<std::string>()).string();
        std::ofstream(aim_path) << aim;
        auto buff = YAML::LoadFile((root / "config/auto_buff.yaml").string());
        auto detect = buff["auto_buff"]["detect"]["yolo"];
        detect["model"] = std::filesystem::absolute(root / "config" / detect["model"].as<std::string>()).string();
        buff["auto_buff"]["detect"]["backend"] = "yolo";
        detect["device"] = "CPU";
        buff["auto_buff"]["detect"]["common"]["debug"] = false;
        detect["tracy_debug"] = false;
        detect["threads"] = 2;
        std::ofstream(buff_path) << buff;

        // 与程序入口相同，统一设置一次；OpenVINO的threads=2不应影响它。
        cv::setNumThreads(1);
        const auto require_unchanged = [&]
        {
            require(cv::getNumThreads() == 1, "Detector changed process-wide OpenCV threads");
        };
        require_unchanged();
        {
            auto_aim::ArmorDetector armor(aim_path.string());
            require_unchanged();
            auto_buff::YoloDetector yolo(buff, buff_path.parent_path());
            require_unchanged();
            const cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
            armor.ArmorDetect(image);
            tests::detect(yolo, image, 0);
            armor.ArmorDetector_Set_Color(auto_aim::BLUE);
            yolo.BuffDetector_Set_Color(auto_buff::EnemyColor::BLUE);
            require_unchanged();
        }
        {
            auto_buff::YoloDetector yolo(buff, buff_path.parent_path());
            require_unchanged();
            auto_aim::ArmorDetector armor(aim_path.string());
            require_unchanged();
        }

    });
}
