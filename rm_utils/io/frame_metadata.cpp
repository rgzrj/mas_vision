#include "frame_metadata.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace tool
{
namespace
{
enum class Field
{
    index,
    exposure,
    received,
    qw,
    qx,
    qy,
    qz,
    mode,
    pose_valid
};

struct Column
{
    Field field;
    const char *name;
    bool required;
};

// 表头、写入顺序和必填列共用一份定义；旧记录允许没有 pose_valid。
constexpr std::array columns{
    Column{Field::index, "frame_index", true},
    Column{Field::exposure, "timestamp_ns", true},
    Column{Field::received, "t_recv_ns", true},
    Column{Field::qw, "q_w", true},
    Column{Field::qx, "q_x", true},
    Column{Field::qy, "q_y", true},
    Column{Field::qz, "q_z", true},
    Column{Field::mode, "mode", true},
    Column{Field::pose_valid, "pose_valid", false},
};

// 此格式只含数字和列名，不支持带引号的文本字段。
std::vector<std::string> split_fields(std::string line)
{
    if (!line.empty() && line.back() == '\r')
        line.pop_back();

    std::istringstream stream(line);
    std::vector<std::string> result;
    std::string field;
    while (std::getline(stream, field, ','))
        result.push_back(field);

    if (!line.empty() && line.back() == ',')
        result.emplace_back();

    return result;
}

std::int64_t integer(const std::string &text)
{
    if (text.empty())
        throw std::runtime_error("Expected integer");

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

void write_metadata_header(std::ostream &out)
{
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto &column : columns)
        out << (column.field == columns.front().field ? "" : ",") << column.name;
    out << '\n';
}

void write_frame_metadata(std::ostream &out, const RecordedFrame &frame)
{
    const auto q = frame.pose.value_or(Eigen::Quaterniond(0, 0, 0, 0));

    for (const auto &column : columns)
    {
        if (column.field != columns.front().field)
            out << ',';

        switch (column.field)
        {
        case Field::index:
            out << frame.index;
            break;
        case Field::exposure:
            out << frame.timestamp_ns;
            break;
        case Field::received:
            out << frame.t_recv_ns;
            break;
        case Field::qw:
            out << q.w();
            break;
        case Field::qx:
            out << q.x();
            break;
        case Field::qy:
            out << q.y();
            break;
        case Field::qz:
            out << q.z();
            break;
        case Field::mode:
            out << frame.mode;
            break;
        case Field::pose_valid:
            out << int(frame.pose.has_value());
            break;
        }
    }
    out << '\n';
}

std::vector<RecordedFrame> read_metadata(const std::filesystem::path &path)
{
    const auto utf8 = path.u8string();
    const std::string filename(utf8.begin(), utf8.end());
    std::ifstream stream(path);
    std::string line;
    std::size_t line_number = 1;

    try
    {
        if (!std::getline(stream, line))
            throw std::runtime_error("Cannot read frames.csv");
        if (line.compare(0, 3, "\xef\xbb\xbf") == 0)
            line.erase(0, 3);

        const auto headers = split_fields(line);
        std::map<std::string, std::size_t> positions;
        for (std::size_t i = 0; i < headers.size(); ++i)
            if (!positions.emplace(headers[i], i).second)
                throw std::runtime_error("Duplicate CSV column: " + headers[i]);

        std::map<Field, std::size_t> indices;
        for (const auto &column : columns)
        {
            const auto found = positions.find(column.name);
            if (found != positions.end())
                indices.emplace(column.field, found->second);
            else if (column.required)
                throw std::runtime_error(std::string("Missing CSV column: ") + column.name);
        }

        std::vector<RecordedFrame> frames;
        while (std::getline(stream, line))
        {
            ++line_number;
            const auto values = split_fields(line);
            if (values.size() != headers.size())
                throw std::runtime_error("CSV column count mismatch");

            const auto value = [&](Field field) -> const std::string & { return values[indices.at(field)]; };
            RecordedFrame frame;
            frame.index        = integer(value(Field::index));
            frame.timestamp_ns = integer(value(Field::exposure));
            frame.t_recv_ns    = integer(value(Field::received));

            const auto mode  = integer(value(Field::mode));
            const auto valid = indices.contains(Field::pose_valid) ? integer(value(Field::pose_valid)) : 1;
            // 旧实机记录允许用 mode=-1 表示无姿态帧。
            if ((valid != 0 && valid != 1) || mode < -1 || mode > 5 || (valid && mode < 0))
                throw std::runtime_error("Invalid mode or pose_valid");
            frame.mode = static_cast<int>(mode);

            const Eigen::Quaterniond q(number(value(Field::qw)), number(value(Field::qx)), number(value(Field::qy)),
                                       number(value(Field::qz)));
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

        if (stream.bad() || frames.empty())
            throw std::runtime_error("Empty or unreadable frames.csv");

        return frames;
    }
    catch (const std::exception &error)
    {
        throw std::runtime_error(filename + ":" + std::to_string(line_number) + ": " + error.what());
    }
}
} // namespace tool
