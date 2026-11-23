#include "tests/common/detection.hpp"
#include "auto_buff/buff_detect/traditional/traditional.hpp"
#include "auto_buff/buff_detect/traditional/rmcs/compat/thinning.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "auto_buff/buff_detect/traditional/process.hpp"

#include <opencv2/imgproc.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include "tests/common/test.hpp"

namespace {
using namespace auto_buff;
namespace fs = std::filesystem;
using tests::require;

// Independent Zhang-Suen definition checks the bundled OpenCV lookup-table port.
cv::Mat reference_thinning(const cv::Mat &input) {
    cv::Mat image = input / 255;
    bool changed;
    do {
        changed = false;
        for (int pass = 0; pass < 2; ++pass) {
            cv::Mat remove = cv::Mat::zeros(image.size(), CV_8U);
            for (int y = 1; y < image.rows - 1; ++y)
                for (int x = 1; x < image.cols - 1; ++x) {
                    if (!image.at<uchar>(y, x))
                        continue;
                    std::array<int, 8> p{image.at<uchar>(y - 1, x), image.at<uchar>(y - 1, x + 1),
                                         image.at<uchar>(y, x + 1), image.at<uchar>(y + 1, x + 1),
                                         image.at<uchar>(y + 1, x), image.at<uchar>(y + 1, x - 1),
                                         image.at<uchar>(y, x - 1), image.at<uchar>(y - 1, x - 1)};
                    int neighbors = 0, transitions = 0;
                    for (int k = 0; k < 8; ++k) {
                        neighbors += p[k];
                        transitions += p[k] == 0 && p[(k + 1) % 8] == 1;
                    }
                    bool triples = pass == 0 ? p[0] * p[2] * p[4] == 0 && p[2] * p[4] * p[6] == 0
                                             : p[0] * p[2] * p[6] == 0 && p[0] * p[4] * p[6] == 0;
                    if (neighbors >= 2 && neighbors <= 6 && transitions == 1 && triples)
                        remove.at<uchar>(y, x) = 1;
                }
            changed |= cv::countNonZero(remove) != 0;
            image.setTo(0, remove);
        }
    } while (changed);
    return image * 255;
}

void write_camera(const fs::path &path, cv::Size size) {
    std::ofstream(path) << "image_width: " << size.width << "\nimage_height: " << size.height
                        << "\ncamera_matrix: [1000,0," << size.width / 2. << ",0,1000," << size.height / 2.
                        << ",0,0,1]\ndistortion: [0,0,0,0,0]\n";
}
} // namespace

void check_traditional() {
    cv::RNG random(42);
    for (int i = 0; i < 12; ++i) {
        cv::Mat binary(32, 40, CV_8U);
        random.fill(binary, cv::RNG::UNIFORM, 0, 256);
        cv::threshold(binary, binary, 80, 255, cv::THRESH_BINARY);
        cv::Mat actual;
        cv::rmcs_compat::thinning(binary, actual, cv::rmcs_compat::THINNING_ZHANGSUEN);
        require(cv::countNonZero(actual != reference_thinning(binary)) == 0,
                "Bundled OpenCV skeleton differs from Zhang-Suen definition");
    }
    auto camera = fs::temp_directory_path() /
                  ("rmcs_detect_" +
                   std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
    write_camera(camera, {640, 480});
    auto_buff::TraditionalDetector detector(tests::traditional_config("red", YAML::Load("save_masks: true")), YAML::LoadFile(camera.string()));
    cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC3);
    auto empty = tests::detect(detector, blank, 0);
    require(empty.markers.empty() && empty.boards.empty() && empty.mask.size() == blank.size(),
            "Blank input or full-frame mask contract failed");
    cv::Mat ring = blank.clone();
    cv::circle(ring, {320, 240}, 60, {0, 0, 220}, 3, cv::LINE_AA);
    // 主工程 YAML 未记录图像宽高；省略它们不能改变传统检测结果。
    const auto host_camera = camera.parent_path() / (camera.stem().string() + "_host.yaml");
    auto host_yaml         = YAML::LoadFile(camera.string());
    host_yaml.remove("image_width");
    host_yaml.remove("image_height");
    std::ofstream(host_camera) << host_yaml;
    auto_buff::TraditionalDetector host_detector(tests::traditional_config(), YAML::LoadFile(host_camera.string()));
    require(tests::detect(host_detector, ring, 0).boards.size() == 1,
            "Host calibration without image dimensions cannot detect a ring");
    fs::remove(host_camera);
    auto active = tests::detect(detector, ring, 0);
    require(active.boards.size() == 1 && active.boards[0].appearance == auto_buff::Appearance::single_ring &&
                !active.boards[0].features_valid,
            "Single ring must be active with no physical arm endpoints");
    for (auto corner : active.boards[0].corners)
        require(corner == cv::Point2d{}, "Upstream ring radius placeholders leaked as arm measurements");
    // 同一检测器切色，模拟电控覆盖本地红色后再恢复配置。
    cv::Mat blue_ring = blank.clone();
    cv::circle(blue_ring, {320, 240}, 60, {220, 0, 0}, 3, cv::LINE_AA);
    require(tests::detect(detector, blue_ring, 0).boards.empty(), "Red detector accepted a blue ring");
    detector.BuffDetector_Set_Color(auto_buff::EnemyColor::BLUE);
    auto blue = tests::detect(detector, blue_ring, 0);
    require(blue.boards.size() == 1 && blue.boards[0].color == auto_buff::EnemyColor::BLUE &&
                tests::detect(detector, ring, 0).boards.empty(),
            "Runtime blue selection failed");
    detector.BuffDetector_Set_Color(auto_buff::EnemyColor::RED);
    require(tests::detect(detector, ring, 0).boards.size() == 1 && tests::detect(detector, blue_ring, 0).boards.empty(),
            "Local red selection was not restored");
    cv::Mat disk = ring.clone();
    cv::circle(disk, {320, 240}, 33, {0, 0, 220}, 3, cv::LINE_AA);
    std::array<cv::Point, 4> tips{{{320, 170}, {250, 240}, {320, 310}, {390, 240}}};
    for (auto tip : tips)
        cv::line(disk, {320, 240}, tip, {0, 0, 255}, 5, cv::LINE_AA);
    auto pending = tests::detect(detector, disk, .1);
    require(pending.boards.size() == 1 && pending.boards[0].appearance == auto_buff::Appearance::full_mark &&
                pending.boards[0].features_valid,
            "Original RMCS four-arm extraction failed");
    require(cv::norm(pending.boards[0].center - cv::Point2d(320, 240)) < 1,
            "Original contour center changed");
    for (auto tip : tips) {
        double best = 1e9;
        for (auto actual : pending.boards[0].corners)
            best = std::min(best, cv::norm(actual - cv::Point2d(tip)));
        require(best < 8, "Original RMCS endpoint differs from the physical synthetic tip");
    }
    require(tests::detect(detector, disk, .1).boards.size() == pending.boards.size(),
            "Stateless RMCS gained temporal confirmation or timestamp reset rules");
    bool rejected = false;
    try {
        tests::detect(detector, cv::Mat::zeros(100, 100, CV_8UC3), 0);
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    require(rejected, "Mismatched calibration resolution was accepted");

    auto photo = tool::read_image(tool::from_utf8(BUFF_PROJECT_DIR) / "tests/data/img/input.png");
    write_camera(camera, photo.size());
    const auto calibration = YAML::LoadFile(camera.string());
    const auto intrinsic = calibration["camera_matrix"].as<std::array<double, 9>>();
    const auto distortion = calibration["distortion"].as<std::array<double, 5>>();
    for (auto color : {auto_buff::EnemyColor::RED, auto_buff::EnemyColor::BLUE}) {
        std::string name = color == auto_buff::EnemyColor::RED ? "red" : "blue";
        auto_buff::RuneDetector original;
        original.config.intrinsic = cv::Mat(3, 3, CV_64F, const_cast<double *>(intrinsic.data())).clone();
        original.config.distortion = cv::Mat(1, 5, CV_64F, const_cast<double *>(distortion.data())).clone();
        original.config.color = color;
        auto expected = original.detect(photo);
        auto_buff::TraditionalDetector adapted(tests::traditional_config(name, YAML::Load("save_masks: true")), calibration);
        auto actual = tests::detect(adapted, photo, 0);
        require(actual.boards.size() == expected.bullseyes.size(),
                "Adapter changed upstream detection count");
        require(actual.markers.size() == expected.icons.size(), "Adapter discarded upstream R candidates");
        for (std::size_t i = 0; i < expected.icons.size(); ++i) {
            require(cv::norm(actual.markers[i].center -
                             cv::Point2d(expected.icons[i].center.x, expected.icons[i].center.y)) < 1e-9 &&
                        actual.markers[i].score == expected.icons[i].score && actual.markers[i].color == color &&
                        actual.markers[i].observed,
                    "Adapter changed R candidate position/score/color");
        }
        for (std::size_t i = 0; i < expected.bullseyes.size(); ++i) {
            const auto &target = expected.bullseyes[i];
            require(cv::norm(actual.boards[i].center - cv::Point2d(target.center.x, target.center.y)) <
                            1e-9 &&
                        actual.boards[i].pvd == target.score && actual.boards[i].color == color,
                    "Adapter changed original centers/PVD/color");
        }
        cv::Mat mask;
        auto_buff::extract_channel(photo, color, mask);
        require(cv::countNonZero(mask != actual.mask) == 0,
                "Diagnostic mask differs from upstream color extraction");
        std::cout << "Photo " << name << ": R=" << actual.markers.size() << " boards=" << actual.boards.size()
                  << " (matches upstream)\n";
    }
    fs::remove(camera);
    std::cout << "RMCS detector: skeleton equivalence, physical tips, active-ring state, calibration and "
                 "source parity passed.\n";
}
