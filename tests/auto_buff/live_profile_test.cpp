#include "tools/live_profile_support.hpp"
#include "tests/common/test.hpp"
#include <openvino/opsets/opset8.hpp>

TEST_MAIN
{
    (void)argc;
    (void)argv;
    return tests::run(
        "live_profile_test",
        []
        {
            using Clock = tool::ProfileClock;
            tool::FramePacer pacer(120);
            const auto now  = Clock::time_point{} + std::chrono::seconds(1);
            const auto step = std::chrono::ceil<Clock::duration>(std::chrono::duration<double>(1. / 120));
            tests::require(pacer.due(now), "First frame was delayed");
            pacer.accepted(now);
            tests::require(!pacer.due(now + step - Clock::duration(1)) && pacer.due(now + step), "Invalid FPS limit");
            pacer.accepted(now + std::chrono::seconds(1));
            tests::require(!pacer.due(now + std::chrono::seconds(1)), "Late submission allowed a catch-up burst");

            // 两个在途输入互不覆盖，BGR必须按RGB平面写入，并保留居中的114填充。
            ov::Tensor first(ov::element::f32, {1, 3, 4, 4}), second(ov::element::f32, {1, 3, 4, 4});
            cv::Mat blue(2, 4, CV_8UC3, cv::Scalar(255, 0, 0));
            const auto box = tool::prepare_profile_input(blue, first);
            tool::prepare_profile_input(cv::Mat(4, 4, CV_8UC3, cv::Scalar(0, 0, 255)), second);
            tests::require(box.scale == 1 && box.pad == cv::Point(0, 1), "Letterbox geometry changed");
            const auto *a = first.data<const float>();
            const auto *b = second.data<const float>();
            tests::require(std::abs(a[0] - 114.f / 255) < 1e-6f && a[4] == 0 && a[32 + 4] == 1,
                           "Padding or RGB layout changed");
            tests::require(b[0] == 1 && b[32] == 0 && a[32 + 4] == 1, "In-flight inputs share writable memory");

            // 用真实CPU运行时验证两个异步请求的输入/输出隔离；不需要相机或GPU。
            auto input = std::make_shared<ov::opset8::Parameter>(ov::element::f32, ov::Shape{1, 3, 4, 4});
            auto twice = std::make_shared<ov::opset8::Multiply>(
                input, ov::opset8::Constant::create(ov::element::f32, ov::Shape{}, {2.f}));
            auto model = std::make_shared<ov::Model>(ov::OutputVector{twice}, ov::ParameterVector{input});
            ov::Core core;
            auto network = core.compile_model(model, "CPU", ov::num_streams(1), ov::inference_num_threads(1));
            auto one = network.create_infer_request(), two = network.create_infer_request();
            one.set_input_tensor(first);
            two.set_input_tensor(second);
            one.start_async();
            two.start_async();
            two.wait();
            one.wait();
            tests::require(one.get_output_tensor().data<const float>()[32 + 4] == 2 &&
                               two.get_output_tensor().data<const float>()[0] == 2 &&
                               one.get_output_tensor().data<const float>()[4] == 0,
                           "Concurrent requests changed another frame's result");
        });
}
