#include "auto_buff/buff_detect/yolo/yolo.hpp"
#include "auto_buff/buff_detect/buff_detector.hpp"
#include "tests/common/test.hpp"
#include "tests/common/detection.hpp"

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>
#include <openvino/opsets/opset8.hpp>
#include <openvino/pass/serialize.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>

namespace
{
// 输出保留真实五点协议，分数随输入R通道变化，用于识别串帧和输入覆盖。
void save_model(const std::filesystem::path &path)
{
    using namespace ov::opset8;
    constexpr std::size_t count = 6300;
    const ov::Shape shape{1, 18, count};
    std::vector<float> output(18 * count), weight(output.size());
    output[0] = .6f;
    weight[0] = .3f;
    const std::array<cv::Point2f, 5> points{{{100, 50}, {50, 100}, {300, 300}, {150, 100}, {100, 150}}};
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        output[(3 + i * 3) * count] = points[i].x;
        output[(4 + i * 3) * count] = points[i].y;
        output[(5 + i * 3) * count] = .95f;
    }

    auto input = std::make_shared<Parameter>(ov::element::f32, ov::Shape{1, 3, 480, 640});
    auto red   = std::make_shared<Gather>(input, Constant::create(ov::element::i64, ov::Shape{1}, {0}),
                                          Constant::create(ov::element::i64, ov::Shape{}, {1}));
    auto mean =
        std::make_shared<ReduceMean>(red, Constant::create(ov::element::i64, ov::Shape{4}, {0, 1, 2, 3}), false);
    auto value =
        std::make_shared<Add>(Constant::create(ov::element::f32, shape, output),
                              std::make_shared<Multiply>(mean, Constant::create(ov::element::f32, shape, weight)));
    ov::serialize(std::make_shared<ov::Model>(ov::OutputVector{value}, ov::ParameterVector{input}), path.string());
}

void check(const auto_buff::BuffDetector::Detection &result, auto_buff::EnemyColor color, auto_buff::RuneMode mode,
           double score)
{
    tests::require(result.mode == mode && result.boards.size() == 1 && result.boards[0].color == color,
                   "Frame color/mode was replaced by later settings");
    tests::require(result.raw_poses.size() == 1 && std::abs(result.raw_poses[0].score - score) < .02,
                   "Result order or input tensor changed");
    tests::require(cv::norm(result.boards[0].center - cv::Point2d(100, 100)) < 1e-4,
                   "Shared preprocessing/decoding changed point geometry");
}
} // namespace

TEST_MAIN
{
    (void)argc;
    (void)argv;
    return tests::run(
        "yolo_execution_test",
        []
        {
            using namespace auto_buff;
            cv::setNumThreads(1);
            tests::TemporaryDirectory directory;
            save_model(directory.path / "pose.xml");
            const auto path      = directory.path / "detect.yaml";
            const auto configure = [&](const std::string &device, bool reuse)
            {
                auto config   = YAML::Load("auto_buff: {detect: {backend: yolo, common: {color: red}, yolo: {}}}");
                auto yolo     = config["auto_buff"]["detect"]["yolo"];
                yolo["model"] = "pose.xml";
                yolo["feature_points"] = "none";
                yolo["device"]         = device;
                yolo["threads"]        = device == "GPU" ? "ignored" : "1";
                yolo["reuse_buffers"]  = reuse;
                std::ofstream(path) << config;
            };
            const auto collect = [](BuffDetector &detector)
            {
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
                while (std::chrono::steady_clock::now() < deadline)
                {
                    if (auto result = detector.collect())
                        return std::move(*result);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                throw std::runtime_error("Detection did not finish");
            };
            const cv::Mat red(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
            const cv::Mat blue(480, 640, CV_8UC3, cv::Scalar(255, 0, 0));
            tests::require(!BuffDetector::create(path), "Missing configuration produced a detector");

            for (bool reuse : {false, true})
            {
                configure("CPU", reuse);
                auto detector = BuffDetector::create(path);
                tests::require(bool(detector), "CPU creation failed without traditional configuration");
                std::filesystem::remove(path); // 创建完成后不再读取业务文件。
                tests::require(!detector->collect(), "Empty detector returned a frame");
                tests::require(detector->submit(red, 1, RuneMode::small), "CPU rejected its first frame");
                tests::require(!detector->submit({}, 9), "Full CPU slot consumed a new input");
                detector->BuffDetector_Set_Color(EnemyColor::BLUE);
                auto first = collect(*detector);
                check(first, EnemyColor::RED, RuneMode::small, .9);
                tests::require(first.image.data == red.data && first.seconds == 1,
                               "Frame image or exposure timestamp changed");
                tests::require(detector->submit(blue, 2, RuneMode::big), "Collect did not free the CPU slot");
                check(collect(*detector), EnemyColor::BLUE, RuneMode::big, .6);
                tests::require(!detector->collect(), "Collected frame was returned twice");

                // 空观测是一次完成检测，不等同于尚无完成结果。
                detector->BuffDetector_Set_Color(EnemyColor::RED);
                tests::require(detector->submit(blue, 3), "CPU slot was not released");
                auto empty = collect(*detector);
                tests::require(empty.boards.empty() && empty.seconds == 3, "Empty observations lost the frame");
            }

            configure("AUTO", false);
            tests::require(!BuffDetector::create(path), "Invalid device was accepted");
            configure("CPU", false);
            auto invalid                                      = YAML::LoadFile(path.string());
            invalid["auto_buff"]["detect"]["yolo"]["threads"] = 0;
            std::ofstream(path) << invalid;
            tests::require(!BuffDetector::create(path), "Invalid CPU threads were accepted");

            // 传统后端不解析未启用的 YOLO 段；同步计算也必须遵守满槽拒收。
            std::ofstream(directory.path / "hikcamera.yaml")
                << "camera_matrix: [900,0,320,0,900,240,0,0,1]\ndistortion: [0,0,0,0,0]\n";
            std::ofstream(path) << tests::traditional_config();
            auto traditional = BuffDetector::create(path);
            tests::require(bool(traditional), "Traditional creation failed without YOLO configuration");
            const cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC3);
            tests::require(traditional->submit(blank, 4), "Traditional frame rejected");
            tests::require(!traditional->submit({}, 5), "Full traditional slot consumed input");
            tests::require(collect(*traditional).seconds == 4 && !traditional->collect(),
                           "Traditional slot contract changed");

            ov::Core core;
            const auto devices = core.get_available_devices();
            if (!std::ranges::any_of(devices, [](const auto &device) { return device.starts_with("GPU"); }))
            {
                std::cout << "GPU checks skipped: no GPU runtime device\n";
                return;
            }

            configure("GPU", false);
            auto detector = BuffDetector::create(path);
            tests::require(bool(detector), "GPU initialization failed");
            tests::require(detector->submit(red, 5, RuneMode::small), "First GPU frame rejected");
            detector->BuffDetector_Set_Color(EnemyColor::BLUE);
            tests::require(detector->submit(blue, 6, RuneMode::big), "Second GPU frame rejected");
            detector->BuffDetector_Set_Color(EnemyColor::RED);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            tests::require(!detector->submit({}, 7), "Completed but uncollected request freed a slot");

            auto first = collect(*detector);
            check(first, EnemyColor::RED, RuneMode::small, .9);
            tests::require(first.seconds == 5 && first.image.data == red.data, "GPU frame metadata mismatch");
            tests::require(detector->submit(red, 7), "GPU collect did not release a slot");
            auto second = collect(*detector);
            check(second, EnemyColor::BLUE, RuneMode::big, .6);
            tests::require(second.seconds == 6 && second.image.data == blue.data, "GPU FIFO order changed");
            check(collect(*detector), EnemyColor::RED, RuneMode::unknown, .9);
            tests::require(!detector->collect(), "GPU returned a frame twice");
            tests::require(detector->submit(blue, 8), "GPU reuse failed"); // 析构时仍有在途请求。
        });
}
