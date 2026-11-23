#pragma once

#include <opencv2/core.hpp>
#include <cstdint>
#include <functional>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace tool
{
struct PlaybackControl;
}
namespace YAML
{
class Node;
}

namespace rm_utils
{

// 三个模块共用算法线程统计的处理吞吐；不使用显示采样率或单模块耗时倒数。
std::string processing_fps_text(double fps);

struct DisplayColor
{
    std::uint8_t r = 255, g = 255, b = 255, a = 255;
};

// 图元使用原图像素坐标，颜色顺序为 RGBA。
struct DisplayText
{
    std::string content;
    int x = 0, y = 0, size = 24;
    DisplayColor color{};
    int thickness      = 1;
    bool image_overlay = false; // true 随图像缩放；false 交给后端布局。
};

struct DisplayPoint
{
    int x = 0, y = 0;
    DisplayColor color{};
    int size = 2;
};

struct DisplayLine
{
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0, thickness = 2;
    DisplayColor color{};
};

// 布局由业务方描述；显示底座不识别 detect/track/shoot 等具体模块。
struct DisplayView
{
    std::string name;
    int hud_height = 0; // Rerun状态区的像素高度；0按实际文字范围分配。SDL忽略此项。
};

/**
 * @brief SDL/Rerun 公共显示入口，业务模块通过 debug 开关控制提交。
 * @note 主线程负责 start/poll/shutdown；算法线程按完整帧串行提交。
 *       图像采样与 Viewer 暂停不影响算法、采集和串口。
 */
class Display
{
  public:
    static constexpr double kImageFps    = 60; // Rerun 图像提交上限。
    static constexpr double kSdlImageFps = 30; // SDL 图像提交上限。

    static Display &getInstance();
    Display(const Display &)            = delete;
    Display &operator=(const Display &) = delete;
    ~Display();

    // 会话：后端互斥，只有 Rerun 使用录制、连接及编码参数。
    void start(bool spawn = true, const std::string &file = {}, const std::string &address = {},
               const std::string &backend = "rerun", const std::string &image_encoding = "jpeg",
               const std::string &hud_encoding = "png");

    /**
     * @brief 实机只保存配置；选中非空布局后，poll 在主线程创建后端。
     */
    void start(const std::filesystem::path &config_path, const std::filesystem::path &output = {});

    /**
     * @brief 回放/采集立即创建会话；配置中的资源路径相对于 config_path。
     */
    void start(const YAML::Node &settings, const std::filesystem::path &config_path,
               const std::filesystem::path &output = {});

    /**
     * @brief 停止提交者后调用，刷新并关闭当前后端。
     */
    void shutdown() noexcept;

    /**
     * @brief 处理窗口事件；playback 用于回放，on_key 用于标定，关闭窗口传入 ESC。
     */
    void poll(tool::PlaybackControl *playback = nullptr, const std::function<void(int)> &on_key = {});

    // 布局：业务描述视图，router 仅选择名称；blueprint 在调用时复制或使用。
    void register_layout(const std::string &name, const std::vector<DisplayView> &views,
                         std::span<const unsigned char> blueprint = {});
    void select_layout(const std::string &name);
    void layout(const std::vector<DisplayView> &views, std::span<const unsigned char> blueprint = {});

    // 帧提交：持 frame_mutex → set_time → 查询采样并提交 → end_frame。
    std::mutex &frame_mutex()
    {
        return frame_mutex_;
    }

    /**
     * @brief 设置曝光/媒体时间，单位秒；多个领域须使用同帧时间。
     */
    void set_time(double seconds);

    /**
     * @brief 提交前查询是否需要复制图像，不推进显示时钟。
     */
    bool image_due(double seconds, std::optional<double> last_image_time) const;
    /**
     * @brief set_time 后查询本帧是否需要整理图像和图元。
     */
    bool image_due() const;
    /**
     * @brief 只有 Rerun 接收数值，SDL 可跳过曲线整理。
     */
    bool values_enabled() const;
    /**
     * @brief SDL 发布最新一组图像；Rerun 无需此操作。
     */
    void end_frame();

    /**
     * @brief 提交图像和图元；调用方不得覆盖仍由后端持有的图像存储。
     * @note SDL 按 width 等比显示，height 仅保留兼容；Rerun 使用原图。
     */
    void display_add(const std::string &name, const cv::Mat &image, const std::vector<DisplayText> &texts = {},
                     const std::vector<DisplayPoint> &points = {}, const std::vector<DisplayLine> &lines = {},
                     int width = 720, int height = 540);

    /**
     * @brief Rerun 数值不受图像采样限制；NaN/Inf 清除当前值，保留历史。
     */
    void scalar(const std::string &path, double value);
    void points2d(const std::string &path, const std::vector<cv::Point2d> &points,
                  const std::vector<std::string> &labels = {});
    void point3d(const std::string &path, double x, double y, double z);

  private:
    Display();
    void update_layout(); // 同步布局变更，后端仍只由主线程创建。

    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::mutex frame_mutex_;
};

} // namespace rm_utils
