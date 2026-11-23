#pragma once

#include "../display.hpp"

namespace rm_utils
{

/**
 * @brief Rerun 后端，负责 Viewer、图像、曲线和 RRD。
 * @note 由 Display 串行调用，不另建调试工作线程。
 */
class RerunDisplay
{
  public:
    RerunDisplay(bool spawn, const std::string &file, const std::string &address, const std::string &image_encoding,
                 const std::string &hud_encoding);
    ~RerunDisplay();

    /**
     * @brief 等待 SDK 刷新，输出本次提交统计。
     */
    void flush();

    /**
     * @brief 应用调用方布局和 HUD 高度，不解释业务模块名称。
     */
    void layout(const std::vector<DisplayView> &views, std::span<const unsigned char> blueprint);

    /**
     * @brief 设置曝光/媒体时间，开始新帧时允许重新记录底图。
     */
    void set_time(double seconds);

    /**
     * @brief 依次提交底图、图元和状态面板。
     */
    void add(const std::string &name, const cv::Mat &image, const std::vector<DisplayText> &texts,
             const std::vector<DisplayPoint> &points, const std::vector<DisplayLine> &lines);

    /**
     * @brief 记录曲线；非有限值清除当前路径，保留历史。
     */
    void scalar(const std::string &path, double value);
    void points(const std::string &path, const std::vector<cv::Point2d> &points,
                const std::vector<std::string> &labels);
    void point3d(const std::string &path, double x, double y, double z);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rm_utils
