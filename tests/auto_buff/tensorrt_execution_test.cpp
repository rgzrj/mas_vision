#include "auto_buff/buff_detect/yolo/inference/inference.hpp"
#include "auto_buff/buff_detect/yolo/observation/pose_output.hpp"
#include "tests/common/test.hpp"

#include <yaml-cpp/yaml.h>
#include <opencv2/core/utility.hpp>
#include <algorithm>
#include <cmath>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

using auto_buff::EnemyColor;
using auto_buff::Inference;
using auto_buff::RuneMode;

static_assert(std::is_same_v<decltype(std::declval<Inference &>().front()), const Inference::Slot *>);

namespace
{
const Inference::Slot &wait_front(Inference &inference)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (const auto *frame = inference.front())
            return *frame;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error("TensorRT request did not complete within 10 seconds");
}
} // namespace

TEST_MAIN
{
    if (argc != 2)
    {
        std::cout << "tensorrt_execution_test <target-built.engine>\n";
        return 77; // CTest 不假定存在与当前设备匹配的 engine。
    }

    return tests::run(
        "tensorrt_execution_test",
        [&]
        {
            using tests::require;
            auto settings     = YAML::Load("device: GPU\nreuse_buffers: true");
            settings["model"] = std::filesystem::absolute(tests::argument_path(argv[1])).string();
            cv::setNumThreads(1);
            const cv::Mat red(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
            const cv::Mat blue(600, 800, CV_8UC3, cv::Scalar(255, 0, 0));

            {
                Inference inference(settings, {});
                require(!inference.front(), "Empty queue exposed a result");
                require(inference.submit(red, 1.0, RuneMode::small, EnemyColor::RED), "First submit failed");
                require(inference.submit(blue, 2.0, RuneMode::big, EnemyColor::BLUE), "Second submit failed");
                require(!inference.submit({}, 3.0, RuneMode::unknown, EnemyColor::RED), "Full queue accepted input");

                const auto &first = wait_front(inference);
                require(first.seconds == 1.0 && first.mode == RuneMode::small && first.color == EnemyColor::RED &&
                            first.image.data == red.data,
                        "First result lost its frame context");
                require(first.output.size() == auto_buff::kPoseChannels * 6300 &&
                            std::ranges::all_of(first.output, [](float x) { return std::isfinite(x); }),
                        "TensorRT returned invalid output");
                require(!inference.submit({}, 3.0, RuneMode::unknown, EnemyColor::RED),
                        "Completed but uncollected slot was reused");
                inference.pop();

                const auto &second = wait_front(inference);
                require(second.seconds == 2.0 && second.mode == RuneMode::big && second.color == EnemyColor::BLUE &&
                            second.image.data == blue.data && second.scale == 0.8,
                        "Second result lost its context or letterbox scale");
                const std::vector<float> saved(second.output.begin(), second.output.end());
                require(inference.submit(red, 3.0, RuneMode::small, EnemyColor::RED), "Released slot was not reused");
                require(std::ranges::equal(saved, second.output), "Slot reuse overwrote another result");
                inference.pop();
                require(wait_front(inference).seconds == 3.0, "Ring wrap changed FIFO order");
                inference.pop();
                require(!inference.front(), "Drained queue retained a result");

                require(inference.submit(red, 4.0, RuneMode::small, EnemyColor::RED), "Teardown submit failed");
                require(inference.submit(blue, 5.0, RuneMode::big, EnemyColor::BLUE), "Teardown second submit failed");
                // 带两个在途请求销毁；后端须等待拷贝与推理，再释放资源。
            }
            require(cv::getNumThreads() == 1, "TensorRT changed process-wide OpenCV threads");
        });
}
