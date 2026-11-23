#include "record.hpp"
#include "session.hpp"
#include "rm_utils/io/paired_recorder.hpp"
#include "hardware/hikcamera/hikcamera.hpp"
#include "hardware/serial/user_serial.hpp"
#include "mas_log.hpp"
#include "rm_utils/display/display.hpp"
#include <atomic>
#include <thread>
#include <cmath>
extern std::atomic<bool> g_shutdown;
namespace app
{
int run_record(const std::filesystem::path &path)
try
{
    const auto config   = YAML::LoadFile(path.string());
    const auto settings = config["recording"];
    const auto resolve  = [&](const std::string &p)
    { return std::filesystem::absolute(path.parent_path() / tool::from_utf8(p)); };
    const auto duration = config["runtime"]["max_runtime_s"].as<double>(0);
    const int capacity  = settings["queue_size"].as<int>(32);
    if (!std::isfinite(duration) || duration < 0 || capacity <= 0)
        throw std::runtime_error("Invalid recording duration/queue size");
    const auto storage = settings["storage"].as<std::string>("video");
    record::PairedRecorder recorder(resolve(settings["output_dir"].as<std::string>("../output/record")),
                                    settings["fps"].as<double>(80), storage, std::size_t(capacity));
    third_party::MasLog::init(tool::utf8(recorder.directory() / "app.log"));
    save_session_configs(path.parent_path(), recorder.directory());
    YAML::Node manifest;
    const auto pipeline = settings["pipeline"].as<std::string>("buff");
    if (pipeline != "buff" && pipeline != "armor")
        throw std::runtime_error("recording.pipeline must be armor or buff");
    manifest["pipeline"]            = pipeline;
    manifest["input"]["type"]       = "paired";
    manifest["input"]["path"]       = storage == "video" ? "raw.avi" : "images";
    manifest["input"]["frames_csv"] = "frames.csv";
    manifest["algorithm_config"]    = pipeline == "buff" ? "auto_buff.yaml" : "auto_aim.yaml";
    manifest["output"]["directory"] =
        tool::utf8(std::filesystem::relative(resolve("../output/replay"), recorder.directory()));
    manifest["output"]["save_annotated"] = true;
    manifest["playback"]["speed"]        = 1.0;
    std::ofstream file(recorder.directory() / "replay.yaml");
    file << manifest;
    file.close();
    if (!file)
        throw std::runtime_error("Cannot save replay manifest");
    auto &serial = UserSerial::getInstance(tool::utf8(path.parent_path() / "serial_config.yaml"));
    hardware::HikCamera camera(tool::utf8(path.parent_path() / "hikcamera.yaml"));
    if (!camera.openCamera())
        MAS_LOG_WARN("Waiting for camera reconnect");
    const bool show = config["display"]["enabled"].as<bool>(true);
    auto &display   = rm_utils::Display::getInstance();
    if (show)
        display.start(config["display"], path, recorder.directory());
    // 独立采集只启动接收任务，不周期发送视觉控制包，不链接实机线程。
    std::jthread receiver(
        [&]
        {
            while (!g_shutdown)
            {
                serial.receiveSerial();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    struct Stop
    {
        ~Stop()
        {
            g_shutdown = true;
        }
    } stop;
    const auto started = std::chrono::steady_clock::now();
    MAS_LOG_INFO("Recording: {}", tool::utf8(recorder.directory()));
    while (!g_shutdown)
    {
        recorder.check();
        serial.pollPose();
        if (duration > 0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >= duration)
            break;
        auto captured = camera.getImage();
        if (show)
            rm_utils::Display::getInstance().poll();
        if (captured.frame.empty())
            continue;
        tool::RecordedFrame frame;
        frame.image = captured.frame;
        frame.timestamp_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(captured.timestamp.time_since_epoch()).count();
        frame.t_recv_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(captured.t_recv.time_since_epoch()).count();
        frame.mode = serial.getVisionMode();
        frame.pose = serial.awaitPose(captured.timestamp);
        recorder.push(frame);
        if (show)
        {
            auto &display = rm_utils::Display::getInstance();
            display.set_time(frame.timestamp_ns * 1e-9);
            display.display_add("record", captured.frame);
            display.end_frame();
            display.poll();
        }
    }
    g_shutdown = true;
    receiver.join();
    camera.closeCamera();
    display.shutdown();
    recorder.close();
    serial.closeSerial();
    MAS_LOG_INFO("Session saved: {}", tool::utf8(recorder.directory() / "replay.yaml"));
    return 0;
}
catch (...)
{
    rm_utils::Display::getInstance().shutdown();
    throw;
}
} // namespace app
