#include "yolo.hpp"

#include "pose_output.hpp"
#include "rm_utils/yolo_timing.hpp"

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace auto_buff
{
/**
 * @brief 读取检测参数，检查模型接口，并创建可复用的 CPU 推理请求。
 * @param config_path 配置文件路径；检测参数与调试显示开关在 auto_buff.detect 节点读取。
 * @note 深大与自训练模型共用三类五点协议，不再接受旧 disk/r/ring 模型。
 */
YoloDetector::YoloDetector(const std::filesystem::path &config_path)
{
    try
    {
        const auto config     = YAML::LoadFile(config_path.string());
        const auto detect     = config["auto_buff"]["detect"];
        const auto model_path = (config_path.parent_path() / detect["model"].as<std::string>()).string();

        tracy_debug_ = detect["tracy_debug"].as<bool>(false);
        detect_debug_.set_enabled(detect["debug"].as<bool>(false));
        reuse_buffers_ = detect["reuse_buffers"].as<bool>(false);

        confidence_       = detect["confidence"].as<float>(.25f);
        point_confidence_ = detect["point_confidence"].as<float>(.8f);
        nms_distance_     = detect["nms_distance"].as<float>(30.f);
        feature_points_   = detect["feature_points"].as<std::string>("rmcs_pixels");

        // 缺省颜色由配置决定，运行后仍可通过 set_color 接收电控模式切换。
        set_color(detect["color"].as<std::string>("red"));

        if (feature_points_ != "none" && feature_points_ != "rmcs_pixels")
            throw std::runtime_error("feature_points must be none or rmcs_pixels");

        const int threads        = detect["threads"].as<int>(4);
        const int opencv_threads = detect["opencv_threads"].as<int>(threads);
        if (threads < 1 || opencv_threads < 1)
            throw std::runtime_error("detect.threads and detect.opencv_threads must be positive");
        // OpenCV 是进程级线程设置，只在构造时设置；缺省沿用原行为。
        cv::setNumThreads(opencv_threads);

        // 与MAS一样直接加载配置指定的模型；相对路径以配置所在目录为基准。
        const auto model = core_.read_model(model_path);

        // 此处检查网络的输入输出类型；INT8 量化模型仍可使用 FP32 输入输出。
        if (model->inputs().size() != 1 || model->outputs().size() != 1 ||
            model->input().get_element_type() != ov::element::f32 ||
            model->output().get_element_type() != ov::element::f32 ||
            model->input().get_shape() != ov::Shape{1, 3, kInputHeight, kInputWidth})
            throw std::runtime_error("Expected FP32 pose I/O and input [1,3,480,640]");

        // 3 个状态分数 + 5×(x,y,score)，沿用深大原模型的无框输出。
        if (model->output().get_shape() != ov::Shape{1, 18, 6300})
            throw std::runtime_error("Expected SZU-compatible pose output [1,18,6300]");

        // 单流优先降低单帧延迟；请求与输入张量只创建一次，逐帧复用。
        network_ = core_.compile_model(model, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
                                       ov::num_streams(1), ov::inference_num_threads(threads));
        request_ = network_.create_infer_request();
        std::cout << "YOLO model: " << model_path << " (FP32 I/O [1,3,480,640] -> [1,18,6300])\n";
        std::cout << "YOLO CPU: threads=" << threads << ", opencv_threads=" << opencv_threads
                  << ", streams=1, reuse_buffers=" << reuse_buffers_ << '\n';
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        throw; // 加载失败须交给调用方处理，不能留下没有有效推理请求的检测器。
    }
}

/**
 * @brief 单帧执行预处理、推理、解码、原图判色及可选像素精修。
 * @param image 原始分辨率的 BGR 图像，CV_8UC3；输出点和框均还原到该图坐标。
 * @return 本帧全部有效 R 候选和符板观测；无有效目标时返回空容器。
 * @note 曝光时间由应用连同结果快照提交给调试线程；检测器不维护时序模型。
 */
Detection YoloDetector::detect_impl(const cv::Mat &image, double)
{
    if (image.empty() || image.type() != CV_8UC3)
        throw std::runtime_error("YOLO needs a BGR image");

    rm_utils::yolo_timing = {};
    // Tracy 与测试程序读取同一线程的分段耗时，不改变算法分支。
    const bool profiling = tracy_debug_ && tracy::IsProfilerStarted();
    auto &timing         = rm_utils::yolo_timing;
    timing.enabled       = profiling;
    TracyCZoneN(yolo_zone, "YOLO", profiling);
    rm_utils::TimedZone yolo_timer(timing.yolo_ms, yolo_zone, timing.enabled);
    using Clock      = std::chrono::steady_clock;
    const auto tick  = [&] { return timing.enabled ? Clock::now() : Clock::time_point{}; };
    const auto start = tick();

    // 等比例 letterbox；RGB/NCHW/255，点位在解码时还原到原图坐标。
    const double scale = std::min(double(kInputWidth) / image.cols, double(kInputHeight) / image.rows);
    const cv::Size size(cvRound(image.cols * scale), cvRound(image.rows * scale));
    const cv::Point pad((kInputWidth - size.width) / 2, (kInputHeight - size.height) / 2);

    if (!reuse_buffers_)
    {
        input_.release();
        floating_.release();
    }
    input_.create(kInputHeight, kInputWidth, CV_8UC3);
    input_.setTo(cv::Scalar(114, 114, 114));
    cv::resize(image, input_(cv::Rect(pad, size)), size);
    input_.convertTo(floating_, CV_32F, 1. / 255.);

    // 直接复用 CPU 输入张量的内存，避免每帧重新创建推理请求。
    auto tensor      = request_.get_input_tensor();
    auto *data       = tensor.data<float>();
    const auto plane = kInputWidth * kInputHeight;

    // split 源顺序 B/G/R，目的内存反向排列得到 RGB 三个连续平面，按通道排列，每个通道的像素连续存放。
    std::array<cv::Mat, 3> channels{cv::Mat(kInputHeight, kInputWidth, CV_32F, data + 2 * plane),
                                    cv::Mat(kInputHeight, kInputWidth, CV_32F, data + plane),
                                    cv::Mat(kInputHeight, kInputWidth, CV_32F, data)};
    cv::split(floating_, channels.data());
    const auto prepared = tick();
    // 同步推理完成后读取输出；解码包含坐标还原、点置信度筛选和重复目标处理。
    request_.infer();
    const auto inferred = tick();

    // 解码输出，按符板中心距离抑制重复候选；R 不参与中心计算。
    const auto output = request_.get_output_tensor();
    const auto shape  = output.get_shape();
    auto objects = decode_pose_output({output.data<const float>(), output.get_size()}, shape[2], image.size(), scale,
                                      pad, confidence_, point_confidence_, nms_distance_);
    if (timing.enabled)
    {
        const auto decoded   = Clock::now();
        timing.preprocess_ms = std::chrono::duration<double, std::milli>(prepared - start).count();
        timing.infer_ms      = std::chrono::duration<double, std::milli>(inferred - prepared).count();
        timing.decode_ms     = std::chrono::duration<double, std::milli>(decoded - inferred).count();
    }

    // YOLO 阶段止于解码；不包含后续判色、几何组织和精修。
    yolo_timer.finish();

    // 原始第 2 点是 R；四端点转为公共 BUFF，后续继续使用 RMCS 的位姿与跟踪。
    auto result = make_pose_detection(objects, image, color_, feature_points_ == "rmcs_pixels", profiling);

    // 原始五点按值保留
    result.raw_poses = std::move(objects);

    return result;
}

} // namespace auto_buff
