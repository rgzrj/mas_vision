#include "paired_player.hpp"
#include "frame_metadata.hpp"
#include "media_reader.hpp"
#include <stdexcept>

namespace replay
{
Player::Player(const std::filesystem::path &media, const std::filesystem::path &csv)
    : media_(media), frames_(tool::read_metadata(csv)), images_(std::filesystem::is_directory(media))
{
    if (images_)
    {
        std::size_t count = 0;
        for (const auto &file : std::filesystem::directory_iterator(media))
            count += file.is_regular_file() && file.path().extension() == ".png";
        if (count != frames_.size())
            throw std::runtime_error("Image count differs from CSV");
    }
    else if (!capture_.open(tool::utf8(media)))
        throw std::runtime_error("Cannot open video: " + tool::utf8(media));
}

bool Player::next(tool::RecordedFrame &frame)
{
    if (next_ == frames_.size())
    {
        cv::Mat extra;
        if (!images_ && capture_.read(extra))
            throw std::runtime_error("Video has more frames than CSV");
        return false;
    }
    // CSV 决定本帧身份和时钟，解码器的播放时间不参与算法计算。
    frame = frames_[next_];
    if (images_)
        frame.image = tool::read_image(media_ / (std::to_string(frame.index) + ".png"));
    else if (!capture_.read(frame.image) || frame.image.empty())
        throw std::runtime_error("Video ended before CSV frame " + std::to_string(next_));
    ++next_;
    return true;
}

} // namespace replay
