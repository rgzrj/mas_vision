#include "rm_utils/yolo_timing.hpp"
#include "auto_buff/buff_detect/yolo/yolo.hpp"
#include "rm_utils/io/media_reader.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <tracy/Tracy.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <thread>
#include <vector>

namespace
{
// 使用最近秩 P95；预热样本不进入 CSV 和统计。
void statistics(std::ostream &out, std::vector<double> values)
{
    std::sort(values.begin(), values.end());
    double sum = 0;
    for (double value : values)
        sum += value;
    out << "{\"mean_ms\":" << sum / values.size()
        << ",\"p95_ms\":" << values[std::size_t(std::ceil(values.size() * .95)) - 1] << ",\"max_ms\":" << values.back()
        << '}';
}
} // namespace

int main(int argc, char *argv[])
{
    try
    {
        // 在 main 返回之前结束 Tracy 客户端工作线程。
        struct Shutdown
        {
            ~Shutdown()
            {
                if (tracy::IsProfilerStarted())
                    tracy::ShutdownProfiler();
            }
        } shutdown;
        if (argc < 2 || std::string(argv[1]) == "--help")
        {
            std::cout << "yolo_profile config.yaml [--frames 300] [--warmup 20] [--wait 0] [--output directory]\n"
                         "Runs each full detect once with pixel refinement enabled; image inputs repeat.\n"
                         "--wait: seconds to wait for a Tracy Viewer/Capture connection before processing.\n";
            return 0;
        }
        const auto config_path = std::filesystem::absolute(tool::from_utf8(argv[1]));
        const auto config      = YAML::LoadFile(config_path.string());
        int frames = 300, warmup = 20, wait = 0;
        const auto stamp =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        auto output = std::filesystem::path(BUFF_PROJECT_DIR) / "output/profiling" / std::to_string(stamp);
        for (int i = 2; i < argc; i += 2)
        {
            if (i + 1 >= argc)
                throw std::runtime_error("Missing option value");
            const auto key   = std::string(argv[i]);
            const auto value = std::string(argv[i + 1]);
            if (key == "--frames")
                frames = std::stoi(value);
            else if (key == "--warmup")
                warmup = std::stoi(value);
            else if (key == "--wait")
                wait = std::stoi(value);
            else if (key == "--output")
                output = tool::from_utf8(argv[i + 1]);
            else
                throw std::runtime_error("Unknown option: " + key);
        }
        if (frames <= 0 || warmup < 0 || wait < 0)
            throw std::runtime_error("frames must be positive; warmup/wait must be nonnegative");
        const auto detect = config["auto_buff"]["detect"]["yolo"];
        if (!detect["tracy_debug"].as<bool>(false))
            throw std::runtime_error("Set detect.tracy_debug=true to collect YOLO/refinement timings");
        if (detect["feature_points"].as<std::string>("rmcs_pixels") != "rmcs_pixels")
            throw std::runtime_error(
                "Set detect.feature_points=rmcs_pixels; this tool measures the full refined pipeline");
        const auto resolve = [&](const std::string &value)
        { return std::filesystem::absolute(config_path.parent_path() / tool::from_utf8(value)).lexically_normal(); };
        cv::setNumThreads(1);
        auto_buff::YoloDetector detector(config, config_path.parent_path());
        detector.disable_debug(); // 测速不构建图元，也不调用显示编码。
        rm_utils::start_tracy();  // YAML 已校验开启；测试入口管理客户端生命周期。
        tool::MediaReader reader(resolve(config["input"]["path"].as<std::string>()),
                                 config["input"]["type"].as<std::string>("video"));
        std::filesystem::create_directories(output);
        std::ofstream csv(output / "frames.csv");
        if (!csv)
            throw std::runtime_error("Cannot create profiling CSV");
        csv << std::setprecision(10) << "frame,exposure_seconds,yolo_ms,refine_ms,refine_calls,detect_total_ms\n";
        tracy::SetThreadName("YOLO profile");
        if (wait)
        {
            std::cout << "Waiting for Tracy connection (" << wait << " s)..." << std::endl;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(wait);
            while (!TracyIsConnected && std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (!TracyIsConnected)
                throw std::runtime_error("No Tracy connection before --wait deadline");
        }
        std::vector<double> yolo, refine, total;
        std::uint64_t calls = 0;
        tool::Frame frame;
        bool have_image = false;
        for (int i = 0; int(yolo.size()) < frames; ++i)
        {
            if ((!reader.is_image() || !have_image) && !reader.next(frame))
                break;
            have_image       = true;
            const auto start = std::chrono::steady_clock::now();
            // 直接测生产检测接口，窗口已显式关闭。
            if (!detector.submit(frame.image, frame.seconds))
                throw std::runtime_error("Previous frame was not collected");
            while (!detector.collect())
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            const double elapsed =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            const auto timing = rm_utils::yolo_timing;
            FrameMark;
            if (i < warmup)
                continue;
            TracyPlot("YOLO ms", timing.yolo_ms);
            TracyPlot("PixelRefine ms per frame", timing.refine_ms);
            TracyPlot("PixelRefine calls per frame", double(timing.refine_calls));
            csv << frame.index << ',' << frame.seconds << ',' << timing.yolo_ms << ',' << timing.refine_ms << ','
                << timing.refine_calls << ',' << elapsed << '\n';
            yolo.push_back(timing.yolo_ms);
            refine.push_back(timing.refine_ms);
            total.push_back(elapsed);
            calls += timing.refine_calls;
        }
        if (yolo.empty())
            throw std::runtime_error("No measured frames; input may be shorter than warmup");
        std::ofstream summary(output / "summary.json");
        summary << std::setprecision(10) << "{\"frames\":" << yolo.size() << ",\"warmup\":" << warmup
                << ",\"threads\":" << detect["threads"].as<int>(4) << ",\"refine_calls\":" << calls << ",\"yolo\":";
        statistics(summary, yolo);
        summary << ",\"refine\":";
        statistics(summary, refine);
        summary << ",\"detect_total\":";
        statistics(summary, total);
        summary << "}\n";
        csv.close();
        summary.close();
        if (!csv || !summary)
            throw std::runtime_error("Cannot finish profiling reports");
        std::ofstream(output / "source_config.yaml") << config;
        std::cout << "Measured " << yolo.size() << " frames; refinement calls=" << calls
                  << "\nOutput: " << tool::utf8(std::filesystem::absolute(output)) << std::endl;
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "yolo_profile: " << error.what() << '\n';
        return 1;
    }
}
