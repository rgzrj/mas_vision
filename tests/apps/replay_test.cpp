#include "tests/common/detection.hpp"
#include "mas_log.hpp"
#include "rm_utils/io/frame_metadata.hpp"
#include "apps/replay/replay.hpp"
#include "apps/record/session.hpp"
#include "rm_utils/io/paired_recorder.hpp"
#include "rm_utils/io/paired_player.hpp"
#include "apps/replay/buff_process.hpp"
#include "tests/common/synthetic_rune.hpp"
#include "tests/common/test.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <sstream>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <atomic>

std::atomic<bool> g_shutdown{false};

namespace
{
namespace fs = std::filesystem;
using namespace auto_buff;

void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

template <class F> void must_fail(F operation, const char *message)
{
    bool failed = false;
    try
    {
        operation();
    }
    catch (const std::exception &)
    {
        failed = true;
    }
    require(failed, message);
}

std::string contents(const fs::path &path)
{
    std::ifstream stream(path);
    require(bool(stream), "Cannot read test output");
    return {std::istreambuf_iterator<char>(stream), {}};
}

fs::path session(const fs::path &root, const char *storage)
{
    record::PairedRecorder recorder(root, 80, storage, 16);
    const auto directory         = recorder.directory();
    const std::int64_t offsets[] = {0, 12000000, 43000000, 51000000};
    for (int i = 0; i < 4; ++i)
    {
        tool::RecordedFrame frame;
        frame.image        = cv::Mat(64, 96, CV_8UC3, cv::Scalar(10 + i * 10, 30, 60));
        frame.timestamp_ns = 5000000000LL + offsets[i];
        frame.t_recv_ns    = frame.timestamp_ns + 3000000;
        if (i != 1)
            frame.pose = Eigen::Quaterniond(2, .1 * i, -.3, .7);
        frame.mode = i == 0 ? 0 : i == 1 ? 2 : i == 2 ? 1 : 3;
        recorder.push(frame);
        frame.image.setTo(cv::Scalar::all(255)); // 提交后改源缓冲，录制内容不能随之变化。
    }
    recorder.close();
    require(!fs::exists(directory / "timestamps.csv"), "Paired recording wrote a redundant timestamp file");
    const auto summary = YAML::LoadFile((directory / "recording_summary.yaml").string());
    require(summary["complete"].as<bool>() && summary["written_frames"].as<int>() == 4,
            "Close did not drain accepted frames");
    require(summary["received_frames"].as<int>() == 4 && summary["accepted_frames"].as<int>() == 4 &&
                summary["peak_queue"].as<int>() >= 1 &&
                std::abs(summary["input_fps"].as<double>() - 3e9 / offsets[3]) < 1e-6 &&
                std::abs(summary["recorded_input_fps"].as<double>() - 3e9 / offsets[3]) < 1e-6,
            "Recording statistics used nominal container fps or lost frame counts");
    replay::Player player(directory / (std::string(storage) == "video" ? "raw.avi" : "images"),
                          directory / "frames.csv");
    tool::RecordedFrame frame;
    for (int i = 0; i < 4; ++i)
    {
        require(player.next(frame), "Missing paired frame");
        require(frame.index == i && frame.timestamp_ns == 5000000000LL + offsets[i] &&
                    frame.t_recv_ns - frame.timestamp_ns == 3000000,
                "Frame pairing or timestamps changed");
        require(bool(frame.pose) == (i != 1), "Missing pose became an invented quaternion");
        require(frame.mode == (i == 0 ? 0 : i == 1 ? 2 : i == 2 ? 1 : 3), "Recorded mode changed");
        if (frame.pose)
            require(std::abs(frame.pose->norm() - 1) < 1e-12 &&
                        frame.pose->angularDistance(Eigen::Quaterniond(2, .1 * i, -.3, .7).normalized()) < 1e-12,
                    "Quaternion direction or normalization changed");
        require(std::abs(int(frame.image.at<cv::Vec3b>(10, 10)[0]) - (10 + i * 10)) <= 4,
                "Image buffer was reused or reordered");
    }
    require(!player.next(frame), "Unexpected extra frame");
    return directory;
}

fs::path replay_output(const fs::path &root)
{
    for (const auto &entry : fs::directory_iterator(root))
        if (entry.is_directory())
            return entry.path();
    throw std::runtime_error("Replay produced no output");
}

// 流水线边界在应用层验证；各算法测试只直接调用对应模块。
using FrameDetector = tests::FrameDetector;

void check_pipeline(const fs::path &root)
{
    const auto camera = root / "hikcamera.yaml";
    std::ofstream(camera) << "calibration: {image_width: 960, image_height: 540, camera_matrix: "
                             "[900,0,480,0,900,270,0,0,1], distort_coeffs: [0,0,0,0,0]}\n"
                             "handeye_calibration:\n  R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0]\n"
                             "  t_camera2gimbal: [0.1,0.02,0.03]\n  R_gimbal2imubody: [0,1,0,-1,0,0,0,0,1]\n";
    const auto config = root / "pipeline_buff.yaml";
    std::ofstream(config) << "auto_buff: {detect: {backend: traditional, common: {color: red}}, track: {}, shoot: {}}\n";
    app::BuffModules modules(config);
    auto replacement = std::make_unique<FrameDetector>();
    auto &detector   = *replacement;
    modules.detector = std::move(replacement);
    const Eigen::Quaterniond basis(Eigen::AngleAxisd(-std::numbers::pi / 2, Eigen::Vector3d::UnitZ()));
    app::BuffFrame frame;
    frame.image = cv::Mat::zeros(540, 960, CV_8UC3);
    for (int i = 0; i < 4; ++i)
    {
        const double time = i / 50.;
        const Eigen::Quaterniond gimbal(Eigen::AngleAxisd(.12 * std::sin(time), Eigen::Vector3d::UnitZ()));
        detector.observation =
            tests::synthetic(time, tests::intrinsics, gimbal, gimbal * Eigen::Vector3d(.1, .02, .03));
        frame.exposure_timestamp = tests::stamp(time);
        frame.imu_orientation    = basis * gimbal * basis.conjugate();
        const auto output        = app::process_buff_frame(modules, frame);
        require(output.initialized && std::abs(output.state.x - 5) < .05,
                "Pipeline did not apply the paired camera pose");
    }
    frame.require_pose = true;
    frame.imu_orientation.reset();
    const auto missing = app::process_buff_frame(modules, frame);
    require(!missing.initialized && missing.status == "waiting_for_pose",
            "Paired missing-pose input reused a fixed camera transform");
    detector.observation.boards.resize(1);
    auto ring           = detector.observation.boards.front();
    ring.appearance     = Appearance::single_ring;
    ring.features_valid = false;
    detector.observation.boards.push_back(ring);
    LightArm lamp;
    lamp.color      = auto_buff::EnemyColor::RED;
    lamp.pose.state = 2;
    detector.observation.light_arms.push_back(lamp);
    detector.observation.raw_poses.push_back(lamp.pose);
    frame.mode     = RuneMode::big;
    const auto big = tests::detect(detector, frame.image, 1, frame.mode);
    require(big.mode == RuneMode::big && big.boards.size() == 1 && big.light_arms.size() == 1 &&
                big.raw_poses.size() == 1,
            "Paired big mode did not reach the common observation adapter");
    frame.mode       = RuneMode::small;
    const auto small = tests::detect(detector, frame.image, 1, frame.mode);
    require(small.boards.size() == 2 && small.light_arms.empty() && small.raw_poses.size() == 1,
            "Paired small mode lost a pending/ring or erased raw model diagnostics");
    frame.mode  = RuneMode::unknown;
    frame.color = "blue";
    detector.BuffDetector_Set_Color(auto_buff::EnemyColor::BLUE);
    const auto blue = tests::detect(detector, frame.image, 1, frame.mode);
    require(blue.boards.empty() && blue.light_arms.empty(), "Per-frame color filtering omitted the light-arm branch");

    struct WaitingDetector : FrameDetector
    {
        std::optional<Detection> collect_impl() override { return std::nullopt; }
    };
    modules.detector = std::make_unique<WaitingDetector>();
    g_shutdown = true;
    const auto interrupted = app::process_buff_frame(modules, frame);
    g_shutdown = false;
    require(!interrupted.initialized && interrupted.status == "interrupted",
            "Replay ignored shutdown while waiting for a detection");
}

void check_mode_color_policy()
{
    for (const std::string initial : {"red", "blue"})
        for (int mode : {0, 1, 2, 3, 4, 5, 5, 4, 2, 3, 2, 0, 2, 1, 2})
        {
            const auto expected = mode % 2 == 0 ? initial : std::string("blue");
            require(app::replay_color(mode, initial) == expected,
                    "Replay mode color differs from the live YAML color policy");
        }
}

void check_mode_colors(const fs::path &root)
{
    cv::Mat image = cv::Mat::zeros(540, 960, CV_8UC3);
    for (int side = 0; side < 2; ++side)
    {
        const cv::Point center(side ? 720 : 240, 270);
        const cv::Scalar color = side ? cv::Scalar(220, 0, 0) : cv::Scalar(0, 0, 220);
        cv::circle(image, center, 60, color, 3, cv::LINE_AA);
        cv::circle(image, center, 33, color, 3, cv::LINE_AA);
        for (auto delta : {cv::Point(0, -70), cv::Point(-70, 0), cv::Point(0, 70), cv::Point(70, 0)})
            cv::line(image, center, center + delta, color, 5, cv::LINE_AA);
    }
    record::PairedRecorder recorder(root / "mode_colors", 50, "images", 16);
    const auto recording = recorder.directory();
    const int modes[]    = {4, 2, 3, 5, 4, 2, 5, 2, 0, 2, 1, 2};
    for (int i = 0; i < 12; ++i)
    {
        tool::RecordedFrame frame;
        frame.image        = image;
        frame.timestamp_ns = 5000000000LL + i * 20000000LL;
        frame.t_recv_ns    = frame.timestamp_ns;
        frame.pose         = Eigen::Quaterniond::Identity();
        frame.mode         = modes[i];
        recorder.push(frame);
    }
    recorder.close();
    for (const std::string initial : {"red", "blue"})
    {
        YAML::Node buff;
        buff["auto_buff"]["detect"]["backend"] = "traditional";
        buff["auto_buff"]["detect"]["common"]["color"]   = initial;
        buff["auto_buff"]["shoot"]["debug"]    = false;
        std::ofstream(root / "mode_buff.yaml") << buff;
        const app::BuffModules modules(root / "mode_buff.yaml");
        require(bool(modules.detector), "Complete pipeline was not created without enabled switches");
        YAML::Node config;
        config["replay"]["media"]      = tool::utf8(recording / "images");
        config["replay"]["frames_csv"] = tool::utf8(recording / "frames.csv");
        config["replay"]["buff_yaml"]  = "mode_buff.yaml";
        config["replay"]["output_dir"] = "mode_output_" + initial;
        config["replay"]["speed"]      = 0;
        config["replay"]["display"]    = false;
        const auto yaml                = root / "mode_replay.yaml";
        std::ofstream(yaml) << config;
        require(app::run_replay(yaml) == 0, "Color-mode replay failed");
        const auto output  = replay_output(root / ("mode_output_" + initial));
        const auto summary = YAML::LoadFile((output / "summary.yaml").string());
        require(summary["mode_inactive_frames"].as<int>() == 2,
                "Armor modes 0/1 were not counted as inactive rune frames");
        require(summary["frames"].as<int>() == 12 && !fs::exists(output / "detections.jsonl") &&
                    !fs::exists(output / "annotated.avi") && !fs::exists(output / "annotated.mp4"),
                "Replay lost frames or still exported reports/videos");
    }
}

void check_media()
{
    // 使用独立临时目录验证录制、视频回读和配置相对路径。
    tests::TemporaryDirectory directory;
    const auto temporary = directory.path;
    cv::Mat blank(540, 960, CV_8UC3, cv::Scalar(0));
    tool::MediaRecorder recorder(temporary / tool::from_utf8("完整视频"), 25, true, true);
    const auto session_name = tool::utf8(recorder.directory().filename());
    require(session_name.size() >= 15 && session_name[2] == '.' && session_name[5] == '.' && session_name[8] == '_' &&
                std::ranges::all_of(session_name.substr(9, 6), [](char c) { return c >= '0' && c <= '9'; }),
            "Output directory must use YY.MM.DD_HHMMSS naming");
    for (int i = 0; i < 4; ++i)
    {
        cv::Mat f = blank.clone();
        cv::circle(f, {100 + i * 20, 100}, 10, {0, 0, 255}, -1);
        recorder.write({f, i, i / 25.}, f);
    }
    recorder.close();
    require(std::filesystem::is_regular_file(recorder.directory() / "timestamps.csv"),
            "Plain media recording lost its timestamp sidecar");
    tool::MediaReader replay(recorder.annotated_path(), "video");
    require(std::abs(replay.fps() - 25) < .01, "Recorded FPS changed");
    tool::Frame frame;
    int count = 0;
    while (replay.next(frame))
    {
        require(frame.image.size() == blank.size(), "Recording cropped the frame");
        require(std::abs(frame.seconds - count / 25.) < .002, "Replay timestamps changed");
        ++count;
    }
    require(count == 4, "Recording skipped frames");
    cv::VideoCapture raw(tool::utf8(recorder.raw_path()));
    require(raw.isOpened() && int(raw.get(cv::CAP_PROP_FRAME_COUNT)) == 4, "Raw recording failed");
    tool::MediaRecorder avi(temporary / "mjpeg_compat", 25, false, true, "avi");
    for (int i = 0; i < 4; ++i)
        avi.write({blank, i, i / 25.}, blank);
    avi.close();
    require(avi.annotated_path().extension() == ".avi", "Forced AVI format was ignored");
    cv::VideoCapture mjpeg(tool::utf8(avi.annotated_path()), cv::CAP_OPENCV_MJPEG);
    require(mjpeg.isOpened() && std::abs(mjpeg.get(cv::CAP_PROP_FPS) - 25) < .01,
            "Built-in MJPEG recording could not reopen");
    int avi_frames = 0;
    cv::Mat avi_frame;
    while (mjpeg.read(avi_frame))
    {
        require(avi_frame.size() == blank.size(), "AVI changed frame size");
        ++avi_frames;
    }
    require(avi_frames == 4, "AVI skipped frames");
    std::cout << "Recording: Unicode directory, all four full frames, "
                 "raw/annotated output and timestamps passed.\n";
    std::cout << "Check artifacts: " << tool::utf8(temporary) << '\n';
}

} // namespace

int main()
{
    const auto root = fs::temp_directory_path() /
                      tool::from_utf8("mas_buff_成对检查_" +
                                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try
    {
        fs::create_directory(root);
        check_mode_color_policy();
        check_pipeline(root);
        check_mode_colors(root);
        check_media();
        const auto images = session(root / "image_sessions", "images");
        const auto video  = session(root / "video_sessions", "video");
        // 旧版八列 CSV 仍可读；新版的无姿态帧不会被当作单位姿态。
        const auto legacy = root / "legacy.csv";
        std::ofstream(legacy) << "frame_index,timestamp_ns,t_recv_ns,q_w,q_x,q_y,q_z,mode\n0,1,2,1,0,0,0,2\n";
        require(tool::read_metadata(legacy).front().pose.has_value(), "Cannot read mas_vision legacy metadata");
        std::ofstream(legacy) << "frame_index,timestamp_ns,t_recv_ns,q_w,q_x,q_y,q_z,mode\n0,1,2,0,0,0,0,2\n";
        must_fail([&] { tool::read_metadata(legacy); }, "Invalid valid-pose quaternion accepted");
        std::ofstream(legacy)
            << "frame_index,timestamp_ns,t_recv_ns,q_w,q_x,q_y,q_z,mode,pose_valid\n0,1,2,0,0,0,0,-1,0\n";
        require(!tool::read_metadata(legacy).front().pose, "Cannot read missing-pose frames from live recording");
        std::ofstream(legacy)
            << "frame_index,timestamp_ns,t_recv_ns,q_w,q_x,q_y,q_z,mode\n0,1,2,1,0,0,0,2\n1,1,3,1,0,0,0,2\n";
        must_fail([&] { tool::read_metadata(legacy); }, "Repeated exposure timestamp accepted");

        {
            record::PairedRecorder failed(root / "failed_session", 80, "images", 2);
            failed.push(tool::RecordedFrame{}); // 编码失败必须传回入口，不能留下 complete=true。
            must_fail([&] { failed.close(); }, "Background write failure was swallowed");
            require(!YAML::LoadFile((failed.directory() / "recording_summary.yaml").string())["complete"].as<bool>(),
                    "Failed recording reported success");
        }

        auto csv             = contents(video / "frames.csv");
        const auto last_line = csv.rfind('\n', csv.size() - 2);
        std::ofstream(root / "short.csv") << csv.substr(0, last_line + 1);
        must_fail(
            [&]
            {
                replay::Player short_csv(video / "raw.avi", root / "short.csv");
                tool::RecordedFrame frame;
                while (short_csv.next(frame))
                {
                }
            },
            "Video/CSV count mismatch ignored");
        fs::rename(images / "images/3.png", images / "images/missing.png");
        must_fail(
            [&]
            {
                replay::Player missing(images / "images", images / "frames.csv");
                tool::RecordedFrame frame;
                while (missing.next(frame))
                {
                }
            },
            "Missing indexed image ignored");
        fs::rename(images / "images/missing.png", images / "images/3.png");

        // 完整回放入口：覆盖动态手眼、无姿态帧、mode=0/1、颜色切换和变速。
        std::ofstream(root / "hikcamera.yaml")
            << "calibration: {image_width: 96, image_height: 64, camera_matrix: [100,0,48,0,100,32,0,0,1], "
               "distort_coeffs: [0,0,0,0,0]}\nhandeye_calibration:\n"
               "  R_camera2gimbal: [1,0,0,0,1,0,0,0,1]\n  t_camera2gimbal: [0,0,0]\n"
               "  R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]\n";
        std::ofstream(root / "buff.yaml") << "auto_buff:\n  detect: {backend: traditional, common: {color: red}}\n"
                                             "  track: {}\n  shoot: {debug: false}\n";
        std::ofstream(root / "serial_config.yaml") << "port: test\n";
        const auto parameters = YAML::LoadFile((root / "buff.yaml").string());
        fs::create_directory(root / "snapshot");
        app::save_recording_snapshot(root / "buff.yaml", parameters, root / "snapshot", true);
        // 两个入口共用的快照必须可独立搬移，且不能改写调用者的原配置。
        require(parameters["auto_buff"]["detect"]["common"]["color"].as<std::string>() == "red" && !parameters["input"] &&
                    fs::exists(root / "snapshot/hikcamera.yaml") && fs::exists(root / "snapshot/serial_config.yaml"),
                "Snapshot mutated source parameters");
        std::ofstream(root / "weights.onnx") << "model snapshot fixture";
        auto yolo                              = YAML::Clone(parameters);
        yolo["auto_buff"]["detect"]["backend"] = "yolo";
        yolo["auto_buff"]["detect"]["yolo"]["model"]   = "weights.onnx";
        for (bool copy_model : {false, true})
        {
            const auto output = root / (copy_model ? "copied_model" : "external_model");
            fs::create_directory(output);
            app::save_recording_snapshot(root / "buff.yaml", yolo, output, copy_model);
            const auto saved = YAML::LoadFile((output / "auto_buff.yaml").string());
            const auto model = tool::from_utf8(saved["auto_buff"]["detect"]["yolo"]["model"].as<std::string>());
            require(contents(output / model) == contents(root / "weights.onnx"), "Snapshot lost the selected model");
            require(fs::exists(output / "model.onnx") == copy_model && model.is_absolute() != copy_model,
                    "Model copy policy changed");
        }
        std::ofstream(root / "weights.xml") << "IR graph snapshot fixture";
        std::ofstream(root / "weights.bin") << "IR weights snapshot fixture";
        yolo["auto_buff"]["detect"]["yolo"]["model"] = "weights.xml";
        const auto ir_output                 = root / "copied_ir";
        fs::create_directory(ir_output);
        app::save_recording_snapshot(root / "buff.yaml", yolo, ir_output, true);
        require(contents(ir_output / "model.xml") == contents(root / "weights.xml") &&
                    contents(ir_output / "model.bin") == contents(root / "weights.bin"),
                "Snapshot must preserve both OpenVINO IR files");
        std::ofstream(root / "weights.engine") << "TensorRT engine snapshot fixture";
        yolo["auto_buff"]["detect"]["yolo"]["model"] = "weights.engine";
        const auto engine_output = root / "copied_engine";
        app::save_recording_snapshot(root / "buff.yaml", yolo, engine_output, true);
        const auto engine_config = YAML::LoadFile((engine_output / "auto_buff.yaml").string());
        require(engine_config["auto_buff"]["detect"]["yolo"]["model"].as<std::string>() == "model.engine" &&
                    contents(engine_output / "model.engine") == contents(root / "weights.engine"),
                "Snapshot renamed or corrupted the TensorRT engine");
        fs::remove(root / "hikcamera.yaml"); // 回放不能再依赖原标定文件。
        {
            YAML::Node config;
            config["replay"]["media"]      = tool::utf8(images / "images");
            config["replay"]["frames_csv"] = tool::utf8(images / "frames.csv");
            config["replay"]["buff_yaml"]  = "missing_model_and_config.yaml";
            config["replay"]["output_dir"] = "check_must_not_create_output";
            const auto path                = root / "check.yaml";
            std::ofstream(path) << config;
            g_shutdown = true;
            require(app::run_replay(path, true) == 130 && !fs::exists(root / "check_must_not_create_output"),
                    "Interrupted pair check reported success or created output");
            g_shutdown = false;
            require(app::run_replay(path, true) == 0 && !fs::exists(root / "check_must_not_create_output"),
                    "Pair check tried to load algorithms or create output");
            fs::copy_file(images / "images/0.png", images / "images/999.png");
            must_fail([&] { app::run_replay(path, true); }, "Pair check accepted an extra image");
            fs::remove(images / "images/999.png");
        }
        std::int64_t first_tracked = -1;
        for (int speed : {0, 2})
        {
            YAML::Node config;
            auto replay          = config["replay"];
            replay["media"]      = tool::utf8(images / "images");
            replay["frames_csv"] = tool::utf8(images / "frames.csv");
            replay["buff_yaml"]  = "snapshot/auto_buff.yaml";
            replay["output_dir"] = "out_" + std::to_string(speed);
            replay["speed"]      = speed;
            replay["display"]    = false;
            const auto path      = root / "replay.yaml";
            std::ofstream(path) << config;
            require(app::run_replay(path) == 0, "Replay entry failed");
            const auto output  = replay_output(root / ("out_" + std::to_string(speed)));
            // 初始化摘要必须先于最终统计写入本次会话，不能停留在默认终端 logger。
            third_party::MasLog::shutdown();
            std::string logs;
            for (const auto &file : fs::directory_iterator(output))
                if (file.path().filename().string().starts_with("app_") && file.path().extension() == ".log")
                    logs += contents(file.path());
            const auto loaded = logs.find("Buff detector: backend=traditional");
            const auto finished = logs.find("Replay finished:");
            require(loaded != std::string::npos && finished != std::string::npos && loaded < finished,
                    "Replay initialization or final statistics missing from session log");

            const auto summary = YAML::LoadFile((output / "summary.yaml").string());
            require(summary["complete"].as<bool>() && summary["frames"].as<int>() == 4 &&
                        summary["missing_pose_frames"].as<int>() == 1 && summary["mode_inactive_frames"].as<int>() == 2,
                    "Replay lost mode or missing-pose semantics");
            require(!fs::exists(output / "detections.jsonl") && !fs::exists(output / "performance.csv"),
                    "Replay retained removed diagnostic exports");
            if (speed == 0)
                first_tracked = summary["tracked_frames"].as<std::int64_t>();
            else
                require(first_tracked == summary["tracked_frames"].as<std::int64_t>(),
                        "Playback speed changed tracked frame count");
        }
        fs::remove_all(root);
        std::cout << "Paired recording and replay checks passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << "\nTest files: " << tool::utf8(root) << '\n';
        return 1;
    }
}
