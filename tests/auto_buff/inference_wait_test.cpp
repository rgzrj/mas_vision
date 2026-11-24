#include "auto_buff/buff_detect/yolo/inference/inference.hpp"
#include "tests/common/test.hpp"

#include <yaml-cpp/yaml.h>

namespace
{
std::array<bool, 2> completed{};
bool expired     = false;
bool start_error = false;
bool query_error = false;

template <class Work> void must_fail(Work work, const std::string &message)
{
    try
    {
        work();
    }
    catch (const std::runtime_error &error)
    {
        tests::require(std::string(error.what()).find(message) != std::string::npos, "Unexpected inference failure");
        return;
    }
    throw std::runtime_error("Inference failure was not reported");
}
} // namespace

namespace auto_buff
{
// 替代厂商运行时，直接验证生产 inference.cpp 的槽位与超时逻辑。
struct Inference::Runtime
{
    std::array<std::vector<float>, 2> inputs{std::vector<float>(3 * 480 * 640), std::vector<float>(3 * 480 * 640)};
    std::array<float, 1> result{};
};

Inference::Inference(const YAML::Node &, const std::filesystem::path &) : runtime_(std::make_unique<Runtime>())
{
    capacity_ = 2;
}
Inference::~Inference() = default;

float *Inference::input(std::size_t index)
{
    return runtime_->inputs[index].data();
}
void Inference::start(std::size_t index)
{
    if (start_error)
        throw std::runtime_error("Simulated start failure");
    if (expired)
        slots_[index].submitted -= std::chrono::seconds(6);
}
bool Inference::ready(std::size_t index)
{
    if (query_error)
        throw std::runtime_error("Simulated query failure");
    return completed[index];
}
std::span<const float> Inference::output(std::size_t)
{
    return runtime_->result;
}
} // namespace auto_buff

int main()
{
    return tests::run(
        "inference_wait_test",
        []
        {
            using namespace auto_buff;
            const cv::Mat image(480, 640, CV_8UC3, cv::Scalar());
            const YAML::Node settings;
            Inference inference(settings, {});
            tests::require(!inference.front(), "Empty inference returned a slot");
            tests::require(inference.submit(image, 1, RuneMode::small, EnemyColor::RED), "First slot rejected");
            tests::require(inference.submit(image, 2, RuneMode::big, EnemyColor::BLUE), "Second slot rejected");
            tests::require(!inference.submit({}, 3, RuneMode::unknown, EnemyColor::RED), "Full ring consumed input");
            completed[1] = true;
            tests::require(!inference.front(), "A later completion bypassed the FIFO head");
            completed[0] = true;
            tests::require(inference.front()->seconds == 1, "First frame identity changed");
            inference.pop();
            tests::require(inference.front()->seconds == 2, "Second frame identity changed");
            inference.pop();

            completed = {};
            expired   = true;
            tests::require(inference.submit(image, 3, RuneMode::small, EnemyColor::RED), "Slot reuse failed");
            must_fail([&] { inference.front(); }, "timed out");
            completed.fill(true);
            must_fail([&] { inference.front(); }, "unavailable");
            must_fail([&] { inference.submit(image, 4, RuneMode::small, EnemyColor::RED); }, "unavailable");

            expired = false;
            Inference failed_start(settings, {});
            start_error = true;
            must_fail([&] { failed_start.submit(image, 1, RuneMode::small, EnemyColor::RED); }, "start failure");
            start_error = false;
            must_fail([&] { failed_start.front(); }, "unavailable");

            Inference failed_query(settings, {});
            failed_query.submit(image, 1, RuneMode::small, EnemyColor::RED);
            query_error = true;
            must_fail([&] { failed_query.front(); }, "query failure");
            query_error = false;
            must_fail([&] { failed_query.front(); }, "unavailable");
        });
}
