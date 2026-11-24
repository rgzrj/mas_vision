#include "replay.hpp"
#include "armor_process.hpp"
#include "buff_process.hpp"
#include "rm_utils/io/paired_player.hpp"
#include "rm_utils/io/media_recorder.hpp"
#include "rm_utils/io/playback_control.hpp"
#include "mas_log.hpp"
#include "rm_utils/display/display.hpp"
#include "auto_aim/debug/armor_debug.hpp"
#include "auto_aim/debug/visualize/track_debug.hpp"
#include "auto_aim/debug/visualize/shoot_debug.hpp"
#include "auto_aim/debug/visualize/detect_debug.hpp"
#include "auto_buff/debug/buff_debug.hpp"
#include "auto_buff/debug/visualize/detect_debug.hpp"
#include "auto_buff/debug/visualize/track_debug.hpp"
#include "auto_buff/debug/visualize/shoot_debug.hpp"
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <cmath>
#include <fstream>
#include <thread>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
extern std::atomic<bool> g_shutdown;
namespace app
{
namespace
{
class TerminalInput
{
public:
    TerminalInput()
    {
        if (!isatty(STDIN_FILENO))
            return;
        if (tcgetattr(STDIN_FILENO, &saved_) != 0)
            throw std::runtime_error("Cannot read terminal settings");
        auto mode = saved_;
        mode.c_lflag &= ~(ICANON | ECHO);
        // poll 后也不能阻塞读取：Ctrl+C 可能在两者之间清空终端输入队列。
        mode.c_cc[VMIN]  = 0;
        mode.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &mode) != 0)
            throw std::runtime_error("Cannot enable playback keys");
        enabled_ = true;
    }
    ~TerminalInput()
    {
        if (enabled_)
            tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
    void poll(tool::PlaybackControl &control)
    {
        if (!enabled_)
            return;
        pollfd input{STDIN_FILENO, POLLIN, 0};
        while (::poll(&input, 1, 0) > 0 && (input.revents & POLLIN))
        {
            unsigned char key = 0;
            if (::read(STDIN_FILENO, &key, 1) != 1)
                break;
            control.key(key);
        }
    }

private:
    termios saved_{};
    bool enabled_ = false;
};
} // namespace
int run_replay(const std::filesystem::path &path, bool check_only)
try
{
    const auto config   = YAML::LoadFile(path.string());
    const bool legacy   = bool(config["replay"]);
    const auto settings = legacy ? config["replay"] : config["input"];
    const auto pipeline = config["pipeline"].as<std::string>("buff");
    if (pipeline != "armor" && pipeline != "buff")
        throw std::runtime_error("pipeline must be armor or buff");
    const auto resolve = [&](const std::string &p)
    { return std::filesystem::absolute(path.parent_path() / tool::from_utf8(p)); };
    // 普通回放先打开会话日志，再加载媒体和算法；配对检查仍不创建输出目录。
    std::filesystem::path output;
    if (!check_only)
    {
        const auto output_settings = legacy ? settings : config["output"];
        output = tool::create_session_directory(
            resolve(output_settings[legacy ? "output_dir" : "directory"].as<std::string>("../output/replay")));
        third_party::MasLog::init(tool::utf8(output / "app.log"));
        MAS_LOG_INFO("Replay config: {}", tool::utf8(path));
    }

    const bool paired = legacy || settings["type"].as<std::string>("paired") == "paired";
    if (!paired && settings["type"].as<std::string>() != "media")
        throw std::runtime_error("input.type must be paired or media");
    std::unique_ptr<replay::Player> player;
    std::unique_ptr<tool::MediaReader> reader;
    if (paired)
        player = std::make_unique<replay::Player>(resolve(settings[legacy ? "media" : "path"].as<std::string>()),
                                                  resolve(settings["frames_csv"].as<std::string>()));
    else
        reader = std::make_unique<tool::MediaReader>(resolve(settings["path"].as<std::string>()),
                                                     settings["media_type"].as<std::string>("video"));
    const auto next = [&](tool::RecordedFrame &frame)
    {
        if (player)
            return player->next(frame);
        tool::Frame media;
        if (!reader->next(media))
            return false;
        frame              = {};
        frame.image        = media.image;
        frame.index        = media.index;
        frame.timestamp_ns = std::int64_t(media.seconds * 1e9);
        return true;
    };
    if (check_only)
    {
        if (!paired)
            throw std::runtime_error("Pair validation requires paired input");
        tool::RecordedFrame frame;
        std::size_t frames = 0;
        while (!g_shutdown && next(frame))
            ++frames;
        MAS_LOG_INFO("Pairing check {}: {} frames", g_shutdown ? "interrupted" : "passed", frames);
        return g_shutdown ? 130 : 0;
    }
    const auto algorithm_path = resolve(
        legacy ? settings["buff_yaml"].as<std::string>("auto_buff.yaml")
               : config["algorithm_config"].as<std::string>(pipeline == "armor" ? "auto_aim.yaml" : "auto_buff.yaml"));
    const auto parameters = YAML::LoadFile(algorithm_path.string());
    std::unique_ptr<ArmorModules> armor;
    std::unique_ptr<BuffModules> buff;
    cv::setNumThreads(1);
    if (pipeline == "armor")
        armor = std::make_unique<ArmorModules>(algorithm_path);
    else
        buff = std::make_unique<BuffModules>(algorithm_path);
    const double speed = legacy ? settings["speed"].as<double>(1) : config["playback"]["speed"].as<double>(1);
    if (!std::isfinite(speed) || speed < 0)
        throw std::runtime_error("Playback speed must be finite and nonnegative");
    const bool show       = legacy ? settings["display"].as<bool>(true) : config["playback"]["display"].as<bool>(true);
    auto &display         = rm_utils::Display::getInstance();
    auto debug_parameters = YAML::Clone(parameters);
    if (!show)
        for (const auto *name : (pipeline == "armor" ? std::array{"armor_detector", "armor_track", "armor_shoot"}
                                                     : std::array{"detect", "track", "shoot"}))
        {
            auto module = debug_parameters[pipeline == "armor" ? "auto_aim" : "auto_buff"][name];
            if (pipeline == "buff" && std::string(name) == "detect")
                module["common"]["debug"] = false;
            else
                module["debug"] = false;
        }
    if (!show && armor)
    {
        // 原始pose/track开关也受回放的display选项约束，Plotter开关保持独立。
        debug_parameters["auto_aim"]["armor_track"]["armor_track_debug"] = false;
        debug_parameters["auto_aim"]["armor_track"]["armor_pose_debug"]  = false;
        armor->detector.disable_debug();
        armor->tracker.disable_debug();
        armor->shooter.disable_debug();
    }
    if (!show && buff)
    {
        buff->detector->disable_debug();
        buff->tracker.disable_debug();
        buff->shooter.disable_debug();
    }
    const auto armor_views = armor ? auto_aim::ArmorDebug::enabled_views(debug_parameters) : 0;
    const auto buff_views  = buff ? auto_buff::BuffDebug::enabled_views(debug_parameters) : 0;
    if (armor_views || buff_views)
    {
        display.start(parameters["display"] ? parameters["display"] : config["display"], algorithm_path, output);
        if (armor)
            auto_aim::ArmorDebug::configure_layout(armor_views, false);
        else
            auto_buff::BuffDebug::configure_layout(buff_views, false);
    }
    tool::PlaybackControl control;
    TerminalInput keyboard;
    const auto poll = [&]
    {
        keyboard.poll(control);
        if (show)
            display.poll(&control);
    };
    const auto local_color =
        pipeline == "buff"
            ? parameters["auto_buff"]["detect"]["common"]["color"].as<std::string>("red")
            : (parameters["auto_aim"]["armor_detector"]["detect_color"].as<std::string>("RED") == "BLUE" ? "blue"
                                                                                                         : "red");
    int last_mode       = 0;
    std::int64_t frames = 0, tracked = 0, missing_pose = 0, inactive = 0, previous_ns = -1;
    auto previous_start = std::chrono::steady_clock::now();
    const auto reset    = [&]
    {
        if (armor)
            armor->reset();
        else
        {
            buff->reset();
        }
    };
    tool::RecordedFrame frame;
    MAS_LOG_INFO("Replay pipeline: {}; input: {}; output: {}", pipeline, paired ? "paired" : "media", tool::utf8(output));
    MAS_LOG_INFO("Replay keys: space pause/resume, n single step, q quit");
    while (!g_shutdown)
    {
        poll();
        if (control.quit)
            break;
        const bool stepping = control.paused && control.take_step();
        if (control.paused && !stepping)
        {
            previous_start = std::chrono::steady_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        if (!next(frame))
            break;
        if (previous_ns >= 0 && speed > 0 && !stepping)
        {
            const auto due =
                previous_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                     std::chrono::duration<double>((frame.timestamp_ns - previous_ns) * 1e-9 / speed));
            while (!g_shutdown && !control.quit)
            {
                poll();
                if (control.paused)
                {
                    if (control.take_step())
                        break;
                }
                else if (std::chrono::steady_clock::now() >= due)
                    break;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
        if (g_shutdown || control.quit)
            break;
        previous_start = std::chrono::steady_clock::now();
        previous_ns    = frame.timestamp_ns;
        if (paired && frame.mode != last_mode)
        {
            reset();
            last_mode = frame.mode;
        }
        const auto color = paired ? replay_color(frame.mode, local_color) : local_color;
        const auto exposure =
            std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::nanoseconds(frame.timestamp_ns)));
        const bool active =
            !paired || (armor ? (frame.mode == 0 || frame.mode == 1) : (frame.mode >= 2 && frame.mode <= 5));
        if (!active)
        {
            reset();
            ++inactive;
        }
        if (paired && !frame.pose)
            ++missing_pose;
        if (armor)
        {
            armor->detector.ArmorDetector_Set_Color(color == "blue" ? auto_aim::BLUE : auto_aim::RED);
            std::unique_lock display_frame(display.frame_mutex(), std::defer_lock);
            if (armor_views)
            {
                display_frame.lock();
                display.set_time(frame.timestamp_ns * 1e-9);
            }
            ArmorResult result;
            if (active)
                result = process_armor_frame(*armor, frame.image, exposure, frame.pose, paired, false);
            else
                result.status = "mode_inactive";
            tracked += result.target_ready;
            if (!active || result.status == "waiting_for_pose")
            {
                auto_aim::ArmorDebug::unavailable(frame.image, armor_views,
                                                  active ? "Pose unavailable" : "mode_inactive", !active);
                if (armor_views & 2)
                    auto_aim::TrackDebug::clear();
                if (armor_views & 4)
                    auto_aim::ShootDebug::clear();
                if (!active && (armor_views & 1))
                    auto_aim::DetectDebug::clear();
            }
            if (armor_views)
                display.end_frame();
        }

        else
        {
            std::unique_lock display_frame(display.frame_mutex(), std::defer_lock);
            if (buff_views)
            {
                display_frame.lock();
                display.set_time(frame.timestamp_ns * 1e-9);
            }
            BuffFrame input;
            input.image              = frame.image;
            input.exposure_timestamp = exposure;
            input.imu_orientation    = frame.pose;
            input.color              = color;
            input.mode               = !paired ? auto_buff::RuneMode::unknown
                                               : (frame.mode <= 3 ? auto_buff::RuneMode::small : auto_buff::RuneMode::big);
            input.require_pose       = paired;
            auto_buff::TrackResult tracking;
            if (active)
                tracking = process_buff_frame(*buff, input);
            else
                tracking.status = "mode_inactive";
            tracked += tracking.initialized;
            if (!active || tracking.status == "waiting_for_pose")
            {
                auto_buff::BuffDebug::unavailable(frame.image, buff_views,
                                                  active ? "Pose unavailable" : "mode_inactive", !active);
                if (buff_views & 2)
                    auto_buff::TrackDebug::clear();
                if (buff_views & 4)
                    auto_buff::ShootDebug::clear();
                if (!active && (buff_views & 1))
                    auto_buff::DetectDebug::clear();
            }
            if (buff_views)
                display.end_frame();
        }
        ++frames;
    }
    display.shutdown();
    YAML::Node summary;
    summary["frames"]               = frames;
    summary["source_frames"]        = player ? player->frames().size() : reader->frame_count();
    summary["missing_pose_frames"]  = missing_pose;
    summary["mode_inactive_frames"] = inactive;
    summary["tracked_frames"]       = tracked;
    summary["stopped_by_user"]      = bool(g_shutdown || control.quit);
    summary["complete"]             = !g_shutdown && !control.quit;
    std::ofstream result(output / "summary.yaml");
    result << summary;
    result.close();
    if (!result)
        throw std::runtime_error("Cannot finish replay reports");
    MAS_LOG_INFO("Replay finished: {} frames; tracked={}", frames, tracked);
    return 0;
}
catch (...)
{
    rm_utils::Display::getInstance().shutdown();
    throw;
}
} // namespace app
