#include "media_recorder.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace tool {
std::filesystem::path create_session_directory(const std::filesystem::path &root)
{
    std::filesystem::create_directories(root);
    const auto timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm local{};
    if (!localtime_r(&timestamp, &local))
        throw std::runtime_error("Cannot format recording date");
    std::ostringstream prefix;
    prefix << std::put_time(&local, "%y.%m.%d_%H%M%S");
    // 同一秒内多次启动时追加序号，避免覆盖已有会话。
    for (int n = 0;; ++n)
    {
        const auto directory = root / (prefix.str() + (n == 0 ? "" : "_" + std::to_string(n)));
        if (std::filesystem::create_directory(directory))
            return directory;
    }
}

MediaRecorder::MediaRecorder(const std::filesystem::path &root, double fps, bool raw, bool annotated, std::string format,
                   bool save_timestamps)
    : format_(std::move(format)), fps_(fps), save_raw_(raw), save_annotated_(annotated)
{
    if (format_ != "auto" && format_ != "avi")
        throw std::runtime_error("record.format must be auto or avi");
    directory_ = create_session_directory(root);
    // 成对录制已有 frames.csv；只有普通媒体处理需要单独保存时间表。
    if (save_timestamps)
    {
        timestamps_.open(directory_ / "timestamps.csv");
        if (!timestamps_)
            throw std::runtime_error("Cannot create timestamp sidecar");
        timestamps_ << "frame,media_seconds\n" << std::setprecision(12);
    }
}

void MediaRecorder::open_video(cv::VideoWriter &writer, std::filesystem::path &output, const char *stem)
{
    const auto mp4 = directory_ / (std::string(stem) + ".mp4");
    if (format_ == "auto")
    {
        try
        {
            if (writer.open(utf8(mp4), cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps_, size_))
            {
                output = mp4;
                std::cout << "Recording: " << utf8(output) << " (" << writer.getBackendName() << ")\n";
                return;
            }
        }
        catch (const cv::Exception &error)
        {
            std::cerr << "MP4 writer: " << error.what() << '\n';
        }
        writer.release();
        std::cerr << "MP4 encoder unavailable; trying AVI/MJPEG.\n";
    }
    const auto avi = directory_ / (std::string(stem) + ".avi");
    // OpenCV 内置 MJPEG 编码器不依赖 FFmpeg 后端，用于 MP4 不可用时回退。
    for (int backend : {int(cv::CAP_OPENCV_MJPEG), int(cv::CAP_ANY)})
    {
        try
        {
            if (writer.open(utf8(avi), backend, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_, size_))
            {
                output = avi;
                std::cout << "Recording: " << utf8(output) << " (" << writer.getBackendName() << ")\n";
                return;
            }
        }
        catch (const cv::Exception &error)
        {
            std::cerr << "AVI writer: " << error.what() << '\n';
        }
        writer.release();
    }
    throw std::runtime_error("Cannot create MP4 or AVI video in " + utf8(directory_) +
                             "; check OpenCV video backends and output permissions");
}

void MediaRecorder::write(const Frame &frame, const cv::Mat &annotated)
{
    if (frame.image.empty() || annotated.size() != frame.image.size())
        throw std::runtime_error("MediaRecorder requires full-size nonempty frames");
    if (frames_ && frame.image.size() != size_)
        throw std::runtime_error("Video frame size changed");
    // 首帧确定输出分辨率；后续帧必须保持尺寸一致。
    size_ = frame.image.size();
    if (fps_ == 0)
    {
        if (save_raw_)
            write_image(directory_ / "raw.png", frame.image);
        if (save_annotated_)
            write_image(directory_ / "annotated.png", annotated);
    }
    else
    {
        auto append = [&](cv::VideoWriter &writer, std::filesystem::path &output, const char *stem,
                          const cv::Mat &image) {
            if (!writer.isOpened())
                open_video(writer, output, stem);
            writer.write(image);
        };
        if (save_raw_)
            append(raw_writer_, raw_path_, "raw", frame.image);
        if (save_annotated_)
            append(annotated_writer_, annotated_path_, "annotated_full", annotated);
    }
    if (timestamps_.is_open())
    {
        timestamps_ << frame.index << ',' << frame.seconds << '\n';
        if (!timestamps_)
            throw std::runtime_error("Timestamp sidecar write failed");
    }
    ++frames_;
}

void MediaRecorder::close()
{
    raw_writer_.release();
    annotated_writer_.release();
    if (timestamps_.is_open())
        timestamps_.flush();
}
} // namespace tool
