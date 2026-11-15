#pragma once

#include "rm_utils/io/paired_frame.hpp"
#include "rm_utils/io/media_recorder.hpp"
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <thread>

namespace record {

// 沿用 mas_vision 的成对录制：采集端只提交图像副本，编码和 CSV 由同一线程按序写入。
class PairedRecorder
{
public:
    /**
     * @brief 创建录像/图片与姿态成对写入器，并启动后台编码线程。
     * @param storage video 写 raw.avi；images 按连续帧序号写 PNG。
     * @param capacity 待编码队列容量；写盘落后时报告队列满，不静默丢帧。
     */
    PairedRecorder(const std::filesystem::path &root, double fps, const std::string &storage, std::size_t capacity);
    ~PairedRecorder();
    /**
     * @brief 复制图像并将其与曝光时间、姿态一起提交，写盘由后台线程完成。
     */
    void push(const tool::RecordedFrame &frame);
    // 将后台写入错误重新抛给采集线程，避免采集继续而文件已停止增长。
    void check();
    void close(); // 停止接收，排空队列后返回；写入失败时抛出异常。

    const std::filesystem::path &directory() const { return output_.directory(); }

private:
    void write_loop();

    tool::MediaRecorder output_;
    bool images_;
    std::size_t capacity_;
    std::ofstream metadata_;
    std::deque<tool::RecordedFrame> queue_;
    std::mutex mutex_;
    std::condition_variable ready_;
    bool stopping_ = false;
    std::exception_ptr error_;
    // accepted_ 是已入队数量，written_ 是图像与元数据均写入的数量。
    std::int64_t accepted_ = 0, written_ = 0;
    std::int64_t received_ = 0, first_input_ns_ = 0, last_input_ns_ = 0;
    std::int64_t first_written_ns_ = 0, last_written_ns_ = 0;
    std::size_t peak_queue_ = 0;
    double image_write_ms_ = 0, max_image_write_ms_ = 0, metadata_write_ms_ = 0;
    std::thread writer_;
};

} // namespace record
