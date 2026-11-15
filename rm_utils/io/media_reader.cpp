#include "rm_utils/io/media_reader.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace tool {
std::string utf8(const std::filesystem::path &path)
{
    auto text = path.u8string();
    return {reinterpret_cast<const char *>(text.data()), text.size()};
}

std::filesystem::path from_utf8(const std::string &text)
{
    return std::filesystem::path(std::u8string(text.begin(), text.end()));
}

cv::Mat read_image(const std::filesystem::path &path)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
        throw std::runtime_error("Cannot read image: " + utf8(path));
    std::vector<uchar> bytes((std::istreambuf_iterator<char>(stream)), {});
    cv::Mat image = cv::imdecode(bytes, cv::IMREAD_COLOR);
    if (image.empty())
        throw std::runtime_error("Image decode failed: " + utf8(path));
    return image;
}

void write_image(const std::filesystem::path &path, const cv::Mat &image)
{
    std::vector<uchar> bytes;
    if (!cv::imencode(".png", image, bytes))
        throw std::runtime_error("PNG encode failed");
    std::ofstream stream(path, std::ios::binary);
    stream.write(reinterpret_cast<const char *>(bytes.data()), std::streamsize(bytes.size()));
    if (!stream)
        throw std::runtime_error("Image write failed: " + utf8(path));
}

MediaReader::MediaReader(const std::filesystem::path &path, const std::string &type) : is_image_(type == "image")
{
    if (type != "image" && type != "video")
        throw std::runtime_error("input.type must be image or video");
    if (!std::filesystem::is_regular_file(path))
        throw std::runtime_error("Input file does not exist: " + utf8(path));
    if (is_image_)
    {
        image_ = read_image(path);
        count_ = 1;
    }
    else
    {
        if (!capture_.open(utf8(path)))
            throw std::runtime_error("Cannot open video: " + utf8(path));
        fps_ = capture_.get(cv::CAP_PROP_FPS);
        if (!std::isfinite(fps_) || fps_ <= 0)
            throw std::runtime_error("Video has no valid source FPS");
        count_ = std::int64_t(capture_.get(cv::CAP_PROP_FRAME_COUNT));
    }
}

bool MediaReader::next(Frame &frame)
{
    if (is_image_)
    {
        if (index_)
            return false;
        frame = {image_, index_++, 0};
        return true;
    }
    if (!capture_.read(frame.image))
    {
        if (count_ > 0 && index_ < count_)
            throw std::runtime_error("Video ended before its declared frame count: " + std::to_string(index_) + "/" +
                                     std::to_string(count_));
        return false;
    }
    double timestamp = capture_.get(cv::CAP_PROP_POS_MSEC) / 1000;
    // 部分解码后端不提供有效 PTS，此时按恒定帧率用帧序号 / FPS 恢复媒体时间。
    if (!std::isfinite(timestamp) || timestamp < 0 || (index_ > 0 && timestamp <= last_seconds_))
        timestamp = index_ / fps_;
    if (index_ > 0 && timestamp <= last_seconds_)
        throw std::runtime_error("Video timestamps are not increasing");
    frame.index   = index_++;
    frame.seconds = last_seconds_ = timestamp;
    return true;
}
} // namespace tool
