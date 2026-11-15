#pragma once

#include "../display.hpp"

namespace rm_utils
{

// 独立的Rerun后端，负责SDK、Viewer、图像、曲线和RRD；SDL不使用本类。
// 由 Display 串行调用；不创建第二个调试工作线程。
class RerunDisplay
{
public:
    RerunDisplay(bool spawn, const std::string &file, const std::string &address,
                 const std::string &image_encoding, const std::string &hud_encoding);
    ~RerunDisplay();
    void flush();
    // 不解释业务名称；调用方负责布局内容，views只提供对应状态区的容量。
    void layout(const std::vector<DisplayView> &views, std::span<const unsigned char> blueprint);
    void set_time(double seconds);
    void add(const std::string &name, const cv::Mat &image, const std::vector<DisplayText> &texts,
             const std::vector<DisplayPoint> &points, const std::vector<DisplayLine> &lines);
    void scalar(const std::string &path, double value);
    void points(const std::string &path, const std::vector<cv::Point2d> &points,
                const std::vector<std::string> &labels);
    void point3d(const std::string &path, double x, double y, double z);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rm_utils
