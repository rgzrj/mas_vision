#pragma once
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <filesystem>
#include <string>

namespace tool {
// 文件系统路径与 UTF-8 字符串互转，供 OpenCV 和配置中的中文路径使用。
std::string utf8(const std::filesystem::path &path);
std::filesystem::path from_utf8(const std::string &text);
/**
 * @brief 通过文件流读取并解码为 BGR 图像，支持中文文件路径。
 */
cv::Mat read_image(const std::filesystem::path &path);
/**
 * @brief 将图像编码为 PNG 后写入指定路径，失败时抛出异常。
 */
void write_image(const std::filesystem::path &path, const cv::Mat &image);

// 普通图片/视频输入；seconds 为媒体时间，不是算法处理耗时。
struct Frame
{
    cv::Mat image;
    std::int64_t index = 0;
    double seconds     = 0;
};

// 统一图片与视频读取；图片只返回一帧，视频按源顺序读取。
class MediaReader
{
public:
    /**
     * @brief 打开输入媒体；type 为 image 或 video，视频须有有效源帧率。
     */
    MediaReader(const std::filesystem::path &path, const std::string &type);
    /**
     * @brief 读取下一帧并填写索引和秒级时间戳，正常读完返回 false。
     * @note 视频优先采用解码时间戳，后端不提供时按帧序号/源帧率回退。
     */
    bool next(Frame &frame);

    bool is_image() const { return is_image_; }

    double fps() const { return fps_; }

    std::int64_t frame_count() const { return count_; }

private:
    bool is_image_;
    cv::Mat image_;
    cv::VideoCapture capture_;
    double fps_          = 0;
    double last_seconds_ = -1;
    std::int64_t index_ = 0, count_ = 0;
};
} // namespace tool
