#pragma once
#include "rm_utils/io/media_reader.hpp"
#include <fstream>

namespace tool {
// 创建带时间戳的独立输出目录；同一秒重复启动时追加序号。
std::filesystem::path create_session_directory(const std::filesystem::path &root);

// 同步媒体写入器：维护会话目录，按需输出原图、叠图及媒体时间表。
class MediaRecorder
{
public:
    /**
     * @brief 创建独立会话目录，视频编码器在首帧到来时打开。
     * @param fps 源帧率；为 0 时保存单张 PNG，其他情况写视频。
     * @param save_timestamps 是否写 timestamps.csv；成对录制另有 frames.csv。
     */
    MediaRecorder(const std::filesystem::path &directory, double fps, bool raw, bool annotated, std::string format = "auto",
             bool save_timestamps = true);
    /**
     * @brief 按输入顺序写入完整尺寸图像和可选时间表；不裁剪或缩放。
     */
    void write(const Frame &, const cv::Mat &annotated);
    // 释放编码器并刷新时间表；调用后可读取最终输出路径及帧数。
    void close();

    const std::filesystem::path &directory() const { return directory_; }

    std::int64_t frames() const { return frames_; }

    const std::filesystem::path &raw_path() const { return raw_path_; }

    const std::filesystem::path &annotated_path() const { return annotated_path_; }

private:
    std::filesystem::path directory_;
    std::filesystem::path raw_path_, annotated_path_;
    std::string format_;
    void open_video(cv::VideoWriter &, std::filesystem::path &, const char *stem);
    cv::VideoWriter raw_writer_, annotated_writer_;
    cv::Size size_;
    double fps_;
    bool save_raw_, save_annotated_;
    std::int64_t frames_ = 0;
    std::ofstream timestamps_;
};
} // namespace tool
