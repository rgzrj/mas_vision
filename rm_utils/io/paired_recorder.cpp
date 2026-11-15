#include "paired_recorder.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <chrono>

namespace record {

PairedRecorder::PairedRecorder(const std::filesystem::path &root, double fps, const std::string &storage,
                               std::size_t capacity)
    : output_(root, fps, storage == "video", false, "avi", false), images_(storage == "images"), capacity_(capacity)
{
    if ((storage != "video" && storage != "images") || !std::isfinite(fps) || fps <= 0 || capacity == 0)
        throw std::runtime_error("Invalid recording storage, fps or queue_size");
    if (images_)
        std::filesystem::create_directory(directory() / "images");
    metadata_.open(directory() / "frames.csv");
    tool::write_metadata_header(metadata_);
    if (!metadata_)
        throw std::runtime_error("Cannot create frames.csv");
    writer_ = std::thread(&PairedRecorder::write_loop, this);
}

PairedRecorder::~PairedRecorder()
{
    try
    {
        close();
    }
    catch (...)
    {} // 正常入口显式 close，析构仍确保编码线程退出。
}

void PairedRecorder::check()
{
    std::lock_guard lock(mutex_);
    if (error_)
        std::rethrow_exception(error_);
}

void PairedRecorder::push(const tool::RecordedFrame &frame)
{
    auto copy  = frame;
    copy.image = frame.image.clone(); // 相机缓冲可能复用，不能只保留 Mat 的浅拷贝。
    std::lock_guard lock(mutex_);
    if (error_)
        std::rethrow_exception(error_);
    if (stopping_)
        throw std::runtime_error("Recording is already closed");
    if (received_++ == 0)
        first_input_ns_ = frame.timestamp_ns;
    last_input_ns_ = frame.timestamp_ns;
    if (queue_.size() >= capacity_)
    {
        error_ = std::make_exception_ptr(std::runtime_error("Recording queue full; accepted frames will be drained"));
        std::rethrow_exception(error_);
    }
    copy.index = accepted_++;
    queue_.push_back(std::move(copy));
    peak_queue_ = std::max(peak_queue_, queue_.size());
    ready_.notify_one();
}

void PairedRecorder::write_loop()
{
    try
    {
        while (true)
        {
            tool::RecordedFrame frame;
            {
                std::unique_lock lock(mutex_);
                ready_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
                if (queue_.empty())
                    break;
                frame = std::move(queue_.front());
                queue_.pop_front();
            }
            const auto image_start = std::chrono::steady_clock::now();
            if (images_)
                tool::write_image(directory() / "images" / (std::to_string(frame.index) + ".png"), frame.image);
            output_.write({frame.image, frame.index, frame.timestamp_ns * 1e-9}, frame.image);
            const auto metadata_start = std::chrono::steady_clock::now();
            const double image_ms = std::chrono::duration<double, std::milli>(metadata_start - image_start).count();
            // 仅在对应图像写入后追加 CSV，录像和元数据始终采用同一顺序。
            tool::write_frame_metadata(metadata_, frame);
            if (!metadata_)
                throw std::runtime_error("frames.csv write failed");
            image_write_ms_ += image_ms;
            max_image_write_ms_ = std::max(max_image_write_ms_, image_ms);
            metadata_write_ms_ += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - metadata_start).count();
            if (written_++ == 0)
                first_written_ns_ = frame.timestamp_ns;
            last_written_ns_ = frame.timestamp_ns;
        }
    }
    catch (...)
    {
        std::lock_guard lock(mutex_);
        error_ = std::current_exception();
    }
}

void PairedRecorder::close()
{
    if (!writer_.joinable())
        return;
    {
        std::lock_guard lock(mutex_);
        stopping_ = true;
    }
    ready_.notify_one();
    writer_.join();
    try
    {
        output_.close();
        metadata_.close();
        if (!metadata_)
            throw std::runtime_error("Cannot finish frames.csv");
    }
    catch (...)
    {
        if (!error_)
            error_ = std::current_exception();
    }
    YAML::Node summary;
    summary["accepted_frames"] = accepted_;
    summary["written_frames"]  = written_;
    // 输入是提交给写入器的帧，不冒充传感器原始采集帧数；时间取曝光时间而非容器 FPS。
    summary["received_frames"] = received_;
    summary["peak_queue"] = peak_queue_;
    summary["input_fps"] = received_ > 1 && last_input_ns_ > first_input_ns_
                               ? (received_ - 1) * 1e9 / (last_input_ns_ - first_input_ns_) : 0.;
    summary["recorded_input_fps"] = written_ > 1 && last_written_ns_ > first_written_ns_
                                        ? (written_ - 1) * 1e9 / (last_written_ns_ - first_written_ns_) : 0.;
    summary["mean_image_write_ms"] = written_ ? image_write_ms_ / written_ : 0.;
    summary["max_image_write_ms"] = max_image_write_ms_;
    summary["mean_metadata_write_ms"] = written_ ? metadata_write_ms_ / written_ : 0.;
    summary["complete"]        = !error_ && accepted_ == written_;
    if (error_)
    {
        try
        {
            std::rethrow_exception(error_);
        }
        catch (const std::exception &error)
        {
            summary["error"] = error.what();
        }
    }
    std::ofstream report(directory() / "recording_summary.yaml");
    report << summary;
    report.close();
    if (!report)
        throw std::runtime_error("Cannot save recording summary");
    check();
}

} // namespace record
