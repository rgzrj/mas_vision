#include "live_profile_support.hpp"
#include "auto_buff/buff_detect/yolo/observation/pose_output.hpp"
#include "auto_buff/buff_track/track/buff_tracker.hpp"
#include "auto_buff/buff_shoot/rune_shooter.hpp"
#include "hardware/hikcamera/hikcamera.hpp"
#include "hardware/serial/user_serial.hpp"
#include "rm_utils/io/media_recorder.hpp"
#include "rm_utils/io/runtime_path.hpp"
#include "threads/frame_pool.hpp"
#include "mas_log.hpp"

#include <atomic>
#include <csignal>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <thread>

std::atomic<bool> g_shutdown{false};

namespace
{
using Clock = tool::ProfileClock;

void stop_signal(int)
{
    g_shutdown = true;
}

double milliseconds(Clock::time_point from, Clock::time_point to)
{
    return std::chrono::duration<double, std::milli>(to - from).count();
}

// 请求按提交顺序出队；图像、曝光、姿态和模式快照始终跟随同一个槽位。
struct PendingFrame
{
    ov::InferRequest request;
    CameraFrame camera;
    VisionState state;
    auto_buff::EnemyColor color = auto_buff::EnemyColor::UNKNOWN;
    std::uint64_t index        = 0;
    std::optional<Eigen::Quaterniond> pose;
    tool::Letterbox box{};
    Clock::time_point submitted{};
    double prepare_ms = 0;
    bool busy         = false;

    ~PendingFrame()
    {
        // 异常退出时也先结束GPU访问，再释放本帧内存；正常路径在出队时传播异常。
        if (busy)
            try
            {
                request.wait();
            }
            catch (...)
            {
            }
    }
};

void sensor_value(std::ostream &out, double seconds, const std::string &name, const std::filesystem::path &path)
{
    std::ifstream input(path);
    std::string value;
    if (input >> value)
        out << seconds << ',' << name << ',' << value << '\n';
}

// 只读内核公开项；温度保留type，节流计数保留路径，缺失项不伪造为零。
void sample_machine(std::ostream &out, double seconds)
{
    std::error_code error;
    for (const auto &zone : std::filesystem::directory_iterator("/sys/class/thermal", error))
    {
        std::ifstream type(zone.path() / "type");
        std::string name;
        if (type >> name)
            sensor_value(out, seconds, "temperature_mC/" + zone.path().filename().string() + "/" + name,
                         zone.path() / "temp");
    }
    error.clear();
    for (const auto &cpu : std::filesystem::directory_iterator("/sys/devices/system/cpu", error))
    {
        const auto name = cpu.path().filename().string();
        if (!name.starts_with("cpu"))
            continue;
        for (const auto *count : {"core_throttle_count", "package_throttle_count"})
            sensor_value(out, seconds, name + "/" + count, cpu.path() / "thermal_throttle" / count);
        sensor_value(out, seconds, name + "/cpu_freq_kHz", cpu.path() / "cpufreq/scaling_cur_freq");
    }
    error.clear();
    for (const auto &card : std::filesystem::directory_iterator("/sys/class/drm", error))
        sensor_value(out, seconds, card.path().filename().string() + "/gpu_freq_MHz", card.path() / "gt_cur_freq_mhz");
    error.clear();
    for (const auto &zone : std::filesystem::directory_iterator("/sys/class/powercap", error))
    {
        const auto name = zone.path().filename().string();
        sensor_value(out, seconds, name + "/energy_uJ", zone.path() / "energy_uj");
        sensor_value(out, seconds, name + "/energy_range_uJ", zone.path() / "max_energy_range_uj");
    }
}

int run(int argc, char **argv)
{
    if (argc == 2 && std::string(argv[1]) == "--help")
    {
        fmt::print("gpu_live_profile [--seconds 900] [--fps 120] [--streams 2] [--no-display]\n"
                   "Uses config/auto_buff.yaml and HikCamera; receives pose/mode, never sends vision commands.\n");
        return 0;
    }
    double duration = 900, fps = 120;
    int streams     = 2;
    bool no_display = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string option = argv[i];
        if (option == "--no-display")
            no_display = true;
        else if (i + 1 == argc)
            throw std::invalid_argument("Missing option value");
        else if (option == "--seconds")
            duration = std::stod(argv[++i]);
        else if (option == "--fps")
            fps = std::stod(argv[++i]);
        else if (option == "--streams")
            streams = std::stoi(argv[++i]);
        else
            throw std::invalid_argument("Unknown option: " + option);
    }
    if (!std::isfinite(duration) || duration <= 0 || (streams != 1 && streams != 2))
        throw std::invalid_argument("seconds must be positive; streams must be 1 or 2");
    tool::FramePacer pacer(fps);

    std::filesystem::current_path(rm_utils::project_root());
    cv::setNumThreads(1);
    const std::filesystem::path config_path = "config/auto_buff.yaml";
    const auto config                       = YAML::LoadFile(config_path.string());
    const auto detect                       = config["auto_buff"]["detect"]["yolo"];
    const auto refine                       = detect["feature_points"].as<std::string>("rmcs_pixels");
    const double confidence                 = detect["confidence"].as<double>(.25);
    const double point_confidence           = detect["point_confidence"].as<double>(.8);
    const double nms_distance               = detect["nms_distance"].as<double>(30);
    if (config["auto_buff"]["detect"]["backend"].as<std::string>() != "yolo" || (refine != "none" && refine != "rmcs_pixels"))
        throw std::invalid_argument("Select backend=yolo and feature_points=none/rmcs_pixels");
    const auto color_name = config["auto_buff"]["detect"]["common"]["color"].as<std::string>();
    if (color_name != "red" && color_name != "blue")
        throw std::invalid_argument("Detection color must be red or blue");
    const auto color = color_name == "red" ? auto_buff::EnemyColor::RED : auto_buff::EnemyColor::BLUE;
    const double max_age = config["runtime"]["max_frame_age_ms"].as<double>(100);
    if (!std::isfinite(max_age) || max_age <= 0 || max_age > 1000)
        throw std::invalid_argument("max_frame_age_ms must be in (0,1000]");

    const auto output = tool::create_session_directory("output/profiling/gpu_live");
    third_party::MasLog::init(tool::utf8(output / "app.log"));
    for (const auto *name : {"auto_buff.yaml", "hikcamera.yaml"})
        std::filesystem::copy_file(config_path.parent_path() / name, output / name);
    const auto model_path = config_path.parent_path() / detect["model"].as<std::string>();
    ov::Core core;
    const auto model = core.read_model(model_path.string());
    if (model->inputs().size() != 1 || model->outputs().size() != 1 ||
        model->input().get_element_type() != ov::element::f32 ||
        model->output().get_element_type() != ov::element::f32 ||
        model->input().get_shape() != ov::Shape{1, 3, 480, 640} ||
        model->output().get_shape() != ov::Shape{1, 18, 6300})
        throw std::runtime_error("Expected SZU FP32 I/O [1,3,480,640] -> [1,18,6300]");
    auto network = core.compile_model(model, "GPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
                                      ov::num_streams(streams));
    std::array<PendingFrame, 2> slots;
    for (int i = 0; i < streams; ++i)
    {
        slots[i].request = network.create_infer_request();
        std::fill_n(slots[i].request.get_input_tensor().data<float>(), 3 * 480 * 640, 0.f);
    }
    // 编译和预热不进入实机测量；启动相机后才计运行时长。
    for (int warmup = 0; warmup < 16; ++warmup)
    {
        for (int i = 0; i < streams; ++i)
        {
            slots[i].request.start_async();
            slots[i].busy = true;
        }
        for (int i = 0; i < streams; ++i)
        {
            slots[i].request.wait();
            slots[i].busy = false;
        }
    }
    MAS_LOG_INFO("GPU live profile: model={}, device={}, streams={}, target_fps={}, duration_s={}", model_path.string(),
                 core.get_property("GPU", ov::device::full_name),
                 static_cast<int>(network.get_property(ov::num_streams)), fps, duration);
    MAS_LOG_INFO("Output: {}; serial receive only; submit_to_collect_ms includes ordered waiting", tool::utf8(output));

    auto_buff::RmcsTracker tracker(config_path, "config/hikcamera.yaml");
    auto_buff::RuneShooter shooter(config_path);
    auto_buff::DetectDebug detect_debug(!no_display && config["auto_buff"]["detect"]["common"]["debug"].as<bool>(false));
    auto &display        = rm_utils::Display::getInstance();
    const unsigned views = no_display ? 0 : auto_buff::BuffDebug::configure_layout(config_path);
    if (no_display)
    {
        tracker.disable_debug();
        shooter.disable_debug();
    }
    else
    {
        display.start(config_path, output);
        display.select_layout("buff");
    }

    std::ofstream frames(output / "frames.csv"), health(output / "health.csv"), machine(output / "machine.csv");
    if (!frames || !health || !machine)
        throw std::runtime_error("Cannot create profiling CSV");
    frames << std::setprecision(12)
           << "frame,elapsed_s,epoch,mode,prepare_ms,submit_to_collect_ms,postprocess_ms,track_ms,shoot_ms,process_ms,"
              "age_ms,pose,boards,initialized,status\n";
    health << "elapsed_s,mode,camera_delivered_fps,submitted_fps,processed_fps,tracked_fps,initialized_fps,cpu_percent,"
              "queue_full_total\n";
    machine << "elapsed_s,metric,value\n";

    hardware::HikCamera camera("config/hikcamera.yaml");
    if (!camera.openCamera())
        throw std::runtime_error("Cannot open HikCamera for profiling");
    auto &serial = UserSerial::getInstance();
    rigtorp::SPSCQueue<CameraFrame> camera_queue(2);
    std::atomic<std::uint64_t> received{0}, queue_full{0};
    std::exception_ptr camera_error, serial_error;
    std::jthread capture(
        [&](std::stop_token stop)
        {
            try
            {
                while (!stop.stop_requested() && !g_shutdown)
                {
                    auto frame = camera.getImage();
                    if (frame.frame.empty())
                        continue;
                    ++received;
                    if (!camera_queue.try_push(std::move(frame)))
                        ++queue_full;
                }
            }
            catch (...)
            {
                camera_error = std::current_exception();
                g_shutdown   = true;
            }
        });
    std::jthread receiver(
        [&](std::stop_token stop)
        {
            // 析构请求停止时解除receiveSerial中的重连等待。
            std::stop_callback on_stop(stop, [] { g_shutdown = true; });
            try
            {
                while (!stop.stop_requested() && !g_shutdown)
                {
                    serial.receiveSerial();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            catch (...)
            {
                serial_error = std::current_exception();
                g_shutdown   = true;
            }
        });

    std::deque<int> pending;
    std::optional<std::uint64_t> epoch;
    std::uint64_t submitted = 0, processed = 0, tracked = 0, initialized_frames = 0;
    std::uint64_t completed = 0;
    std::vector<double> ages;
    std::uint64_t last_received = 0, last_submitted = 0, last_processed = 0, last_tracked = 0;
    std::uint64_t last_initialized = 0;
    VisionState previous;
    auto selected_color    = color;
    const auto started     = Clock::now();
    auto last_report       = started;
    auto last_cpu          = std::clock();
    const auto cpu_started = last_cpu;
    sample_machine(machine, 0);
    while (!g_shutdown && milliseconds(started, Clock::now()) < duration * 1000)
    {
        display.poll();
        serial.pollPose();
        const auto current = serial.getVisionState();
        if (current.mode != previous.mode)
        {
            if (current.mode == AUTO_AIM_RED)
                selected_color = color;
            else if (current.mode >= SMALL_RUNE_RED && ((current.mode & 1) != (previous.mode & 1)))
                selected_color = (current.mode & 1) ? auto_buff::EnemyColor::BLUE : auto_buff::EnemyColor::RED;
            previous = current;
        }
        if (!pending.empty() && slots[pending.front()].request.wait_for(std::chrono::milliseconds(0)))
        {
            auto &slot = slots[pending.front()];
            slot.request.wait();
            const auto collected = Clock::now();
            const bool stale =
                current.epoch != slot.state.epoch || milliseconds(slot.camera.timestamp, collected) > max_age;
            std::string status    = stale ? "stale" : "pose_unavailable";
            std::size_t boards    = 0;
            bool initialized      = false;
            double postprocess_ms = 0, track_ms = 0, shoot_ms = 0;
            if (!stale)
            {
                if (epoch != slot.state.epoch)
                {
                    tracker.reset();
                    shooter.reset();
                    epoch = slot.state.epoch;
                }
                const auto timestamp = slot.camera.timestamp;
                const double seconds = std::chrono::duration<double>(timestamp.time_since_epoch()).count();
                if (views)
                    display.set_time(seconds);
                const auto tensor = slot.request.get_output_tensor();
                auto poses        = auto_buff::decode_pose_output({tensor.data<const float>(), tensor.get_size()}, 6300,
                                                                  slot.camera.frame.size(), slot.box.scale, slot.box.pad,
                                                                  confidence, point_confidence, nms_distance);
                auto observation =
                    auto_buff::make_pose_detection(poses, slot.camera.frame, slot.color, refine == "rmcs_pixels");
                observation.raw_poses = std::move(poses);
                const auto mode =
                    slot.state.mode >= BIG_RUNE_RED ? auto_buff::RuneMode::big : auto_buff::RuneMode::small;
                auto_buff::filter_rune_observations(observation, mode, slot.color);
                detect_debug.draw(observation, slot.camera.frame);
                boards              = observation.boards.size();
                const auto detected = Clock::now();
                postprocess_ms      = milliseconds(collected, detected);
                if (slot.pose)
                {
                    tracker.set_imu_orientation(*slot.pose);
                    const auto tracking   = tracker.update(observation, seconds, slot.camera.frame);
                    const auto tracked_at = Clock::now();
                    if (tracking.initialized)
                        shooter.update({Clock::now()});
                    shooter.shoot(tracking, slot.camera.frame);
                    track_ms    = milliseconds(detected, tracked_at);
                    shoot_ms    = milliseconds(tracked_at, Clock::now());
                    initialized = tracking.initialized;
                    status      = tracking.status;
                }
                else
                {
                    tracker.reset();
                    shooter.reset();
                    auto_buff::BuffDebug::unavailable(slot.camera.frame, views, "Pose unavailable");
                }
                if (views)
                    display.end_frame();
            }
            const auto finished = Clock::now();
            const bool valid    = !stale && milliseconds(slot.camera.timestamp, finished) <= max_age &&
                               serial.getVisionState().epoch == slot.state.epoch;
            if (valid)
            {
                ++processed;
                tracked += bool(slot.pose);
                initialized_frames += initialized;
                ages.push_back(milliseconds(slot.camera.timestamp, finished));
            }
            else
                status = "stale";
            frames << slot.index << ',' << milliseconds(started, finished) / 1000 << ',' << slot.state.epoch << ','
                   << int(slot.state.mode) << ',' << slot.prepare_ms << ',' << milliseconds(slot.submitted, collected)
                   << ',' << postprocess_ms << ',' << track_ms << ',' << shoot_ms << ','
                   << milliseconds(collected, finished) << ',' << milliseconds(slot.camera.timestamp, finished) << ','
                   << bool(slot.pose) << ',' << boards << ',' << initialized << ',' << status << '\n';
            slot.busy   = false;
            slot.camera = {};
            pending.pop_front();
            ++completed;
        }

        auto now = Clock::now();
        if (pending.size() < std::size_t(streams) && pacer.due(now))
        {
            CameraFrame frame;
            const auto state = current;
            if (state.mode >= SMALL_RUNE_RED && threads::take_latest_frame(camera_queue, frame) &&
                !frame.frame.empty() && frame.timestamp <= now && milliseconds(frame.timestamp, now) <= max_age)
            {
                // 在取新帧时计周期，不能在预处理结束后再额外等8.33ms。
                pacer.accepted(now);
                int index = 0;
                while (slots[index].busy)
                    ++index;
                auto &slot               = slots[index];
                slot.camera              = std::move(frame);
                slot.state               = state;
                slot.color               = selected_color;
                slot.index               = submitted + 1;
                slot.pose                = serial.awaitPose(slot.camera.timestamp);
                const auto prepare_start = Clock::now();
                slot.box        = tool::prepare_profile_input(slot.camera.frame, slot.request.get_input_tensor());
                slot.prepare_ms = milliseconds(prepare_start, Clock::now());
                slot.submitted  = Clock::now();
                slot.request.start_async();
                slot.busy = true;
                pending.push_back(index);
                ++submitted;
            }
        }
        now = Clock::now();
        if (milliseconds(last_report, now) >= 1000)
        {
            const double interval    = milliseconds(last_report, now) / 1000;
            const double elapsed     = milliseconds(started, now) / 1000;
            const auto cpu           = std::clock();
            const double cpu_percent = double(cpu - last_cpu) / CLOCKS_PER_SEC / interval * 100;
            const auto camera_frames = received.load();
            health << elapsed << ',' << int(current.mode) << ',' << (camera_frames - last_received) / interval << ','
                   << (submitted - last_submitted) / interval << ',' << (processed - last_processed) / interval << ','
                   << (tracked - last_tracked) / interval << ',' << (initialized_frames - last_initialized) / interval
                   << ',' << cpu_percent << ',' << queue_full.load() << '\n';
            sample_machine(machine, elapsed);
            MAS_LOG_INFO("GPU live: processed={:.1f} fps, tracked={:.1f} fps, initialized={:.1f} fps, "
                         "camera_delivered={:.1f} fps, CPU={:.1f}%, mode={}",
                         (processed - last_processed) / interval, (tracked - last_tracked) / interval,
                         (initialized_frames - last_initialized) / interval, (camera_frames - last_received) / interval,
                         cpu_percent, int(current.mode));
            last_report      = now;
            last_cpu         = cpu;
            last_received    = camera_frames;
            last_submitted   = submitted;
            last_processed   = processed;
            last_tracked     = tracked;
            last_initialized = initialized_frames;
            if (!frames || !health || !machine)
                throw std::runtime_error("Cannot write profiling CSV");
        }
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    const double elapsed     = milliseconds(started, Clock::now()) / 1000;
    const double cpu_percent = double(std::clock() - cpu_started) / CLOCKS_PER_SEC / elapsed * 100;
    const auto delivered     = received.load();
    sample_machine(machine, elapsed);
    g_shutdown = true;
    capture.request_stop();
    receiver.request_stop();
    capture.join();
    receiver.join();
    for (auto &slot : slots)
        if (slot.busy)
            slot.request.wait();
    serial.closeSerial();
    if (camera_error)
        std::rethrow_exception(camera_error);
    if (serial_error)
        std::rethrow_exception(serial_error);

    std::ofstream summary(output / "summary.json");
    summary << std::setprecision(12) << "{\n  \"elapsed_s\": " << elapsed << ",\n  \"target_fps\": " << fps
            << ",\n  \"streams\": " << streams << ",\n  \"views\": " << views << ",\n  \"submitted\": " << submitted
            << ",\n  \"completed\": " << completed << ",\n  \"processed\": " << processed
            << ",\n  \"tracked\": " << tracked << ",\n  \"initialized\": " << initialized_frames
            << ",\n  \"stale\": " << completed - processed << ",\n  \"inflight_at_stop\": " << pending.size()
            << ",\n  \"camera_delivered_fps\": " << delivered / elapsed
            << ",\n  \"processed_fps\": " << processed / elapsed << ",\n  \"tracked_fps\": " << tracked / elapsed
            << ",\n  \"initialized_fps\": " << initialized_frames / elapsed << ",\n  \"cpu_percent\": " << cpu_percent
            << ",\n  \"valid_age_ms\": ";
    if (ages.empty())
        summary << "null";
    else
    {
        std::sort(ages.begin(), ages.end());
        const auto percentile = [&](double fraction)
        { return ages[std::size_t(std::ceil(ages.size() * fraction)) - 1]; };
        summary << "{\"p50\":" << percentile(.5) << ",\"p95\":" << percentile(.95) << ",\"p99\":" << percentile(.99)
                << ",\"max\":" << ages.back() << '}';
    }
    summary << "\n}\n";
    summary.close();
    frames.close();
    health.close();
    machine.close();
    if (!frames || !health || !machine || !summary)
        throw std::runtime_error("Cannot finish profiling CSV");
    display.shutdown();
    MAS_LOG_INFO("GPU live finished: submitted={}, processed={}, tracked={}, output={}", submitted, processed, tracked,
                 tool::utf8(output));
    return processed ? 0 : 1;
}
} // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, stop_signal);
    std::signal(SIGTERM, stop_signal);
    try
    {
        return run(argc, argv);
    }
    catch (const std::exception &error)
    {
        g_shutdown = true;
        rm_utils::Display::getInstance().shutdown();
        MAS_LOG_ERROR("GPU live profiling failed: {}", error.what());
        return 1;
    }
}
