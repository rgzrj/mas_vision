#pragma once

#include "rm_utils/io/paired_frame.hpp"
#include <filesystem>
#include <opencv2/videoio.hpp>
#include <vector>

namespace replay {

/**
 * @brief 解析 frames.csv，校验连续索引、单调时间及姿态有效性。
 * @return 按录制顺序排列的元数据，图像在 Player::next 中读取。
 */
std::vector<tool::RecordedFrame> read_metadata(const std::filesystem::path &csv);

// media 可以是视频，也可以是 record 生成的 images 目录；索引始终以 CSV 为准。
class Player
{
public:
    /**
     * @brief 打开媒体和配对元数据；media 为视频文件或 images 目录。
     */
    Player(const std::filesystem::path &media, const std::filesystem::path &csv);
    /**
     * @brief 读取一组图像与对应元数据，正常结束返回 false；数量不匹配时抛出异常。
     */
    bool next(tool::RecordedFrame &frame);

    const std::vector<tool::RecordedFrame> &frames() const { return frames_; }

    double fps() const { return capture_.isOpened() ? capture_.get(cv::CAP_PROP_FPS) : 0; }

private:
    std::filesystem::path media_;
    std::vector<tool::RecordedFrame> frames_;
    cv::VideoCapture capture_;
    std::size_t next_ = 0;
    bool images_;
};

} // namespace replay
