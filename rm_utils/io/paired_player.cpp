#include "paired_player.hpp"
#include "rm_utils/io/media_reader.hpp"
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace replay {
namespace {

// frames.csv 只含数字和固定列名，不使用路径或带引号的文本字段。
std::vector<std::string> fields(std::string line)
{
    if (!line.empty() && line.back() == '\r')
        line.pop_back();
    std::istringstream stream(line);
    std::vector<std::string> result;
    std::string field;
    while (std::getline(stream, field, ','))
        result.push_back(field);
    return result;
}

std::int64_t integer(const std::string &text)
{
    std::size_t end;
    const auto value = std::stoll(text, &end);
    if (end != text.size())
        throw std::runtime_error("Expected integer: " + text);
    return value;
}

double number(const std::string &text)
{
    std::size_t end;
    const auto value = std::stod(text, &end);
    if (end != text.size() || !std::isfinite(value))
        throw std::runtime_error("Invalid number: " + text);
    return value;
}

} // namespace

std::vector<tool::RecordedFrame> read_metadata(const std::filesystem::path &path)
{
    std::ifstream stream(path);
    std::string line;
    if (!std::getline(stream, line))
        throw std::runtime_error("Cannot read frames.csv: " + tool::utf8(path));
    if (line.compare(0, 3, "\xef\xbb\xbf") == 0)
        line.erase(0, 3);
    // 按列名访问，允许附加诊断列；必要列缺失或重复时直接报告。
    const auto headers = fields(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < headers.size(); ++i)
        if (!columns.emplace(headers[i], i).second)
            throw std::runtime_error("Duplicate CSV column");
    for (const char *key : {"frame_index", "timestamp_ns", "t_recv_ns", "q_w", "q_x", "q_y", "q_z", "mode"})
        if (!columns.contains(key))
            throw std::runtime_error(std::string("Missing CSV column: ") + key);
    std::vector<tool::RecordedFrame> frames;
    std::size_t line_number = 1;
    while (std::getline(stream, line))
    {
        ++line_number;
        try
        {
            const auto values = fields(line);
            if (values.size() != headers.size())
                throw std::runtime_error("CSV column count mismatch");
            const auto value = [&](const char *key) -> const std::string & {
                return values[columns.at(key)];
            };
            tool::RecordedFrame frame;
            frame.index        = integer(value("frame_index"));
            frame.timestamp_ns = integer(value("timestamp_ns"));
            frame.t_recv_ns    = integer(value("t_recv_ns"));
            const auto mode    = integer(value("mode"));
            // 兼容已有 run 记录：无姿态时旧版本将 mode 写成 -1。
            const auto valid = columns.contains("pose_valid") ? integer(value("pose_valid")) : 1;
            if ((valid != 0 && valid != 1) || mode < -1 || mode > 5 || (valid && mode < 0))
                throw std::runtime_error("Invalid mode or pose_valid");
            frame.mode = static_cast<int>(mode);
            const Eigen::Quaterniond q(number(value("q_w")), number(value("q_x")), number(value("q_y")),
                                       number(value("q_z")));
            if (valid)
            {
                if (!std::isfinite(q.norm()) || q.norm() <= 1e-6)
                    throw std::runtime_error("Invalid quaternion");
                frame.pose = q.normalized();
            }
            if (frame.index != static_cast<std::int64_t>(frames.size()) || frame.timestamp_ns < 0 ||
                frame.t_recv_ns < frame.timestamp_ns ||
                (!frames.empty() && frame.timestamp_ns <= frames.back().timestamp_ns))
                throw std::runtime_error("Invalid frame index or timestamp order");
            frames.push_back(std::move(frame));
        }
        catch (const std::exception &error)
        {
            throw std::runtime_error(tool::utf8(path) + ":" + std::to_string(line_number) + ": " + error.what());
        }
    }
    if (stream.bad() || frames.empty())
        throw std::runtime_error("Empty or unreadable frames.csv");
    return frames;
}

Player::Player(const std::filesystem::path &media, const std::filesystem::path &csv)
    : media_(media), frames_(read_metadata(csv)), images_(std::filesystem::is_directory(media))
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
