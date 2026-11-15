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

namespace tool { struct PlaybackControl; }
namespace YAML
{
class Node;
}

namespace rm_utils {

// 三个模块共用算法线程统计的处理吞吐；不使用显示采样率或单模块耗时倒数。
std::string processing_fps_text(double fps);

struct DisplayColor
{
    std::uint8_t r = 255, g = 255, b = 255, a = 255;
};

// 复用原 Display 图元；坐标始终为原图像素，颜色顺序为 RGBA。
struct DisplayText
{
    std::string content;
    int x = 0, y = 0, size = 24;
    DisplayColor color{};
    int thickness = 1;
    bool image_overlay = false; // 状态面板直接画在图像坐标中，缩放时一同缩放；点旁标签保留原行为。
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
 * @brief SDL/Rerun 调试出口，保留各模块现有的 display_add 调用。
 * @note 应用创建会话，各模块仍由自己的 debug 控制提交；关闭时不编码图像。
 *       Viewer 的暂停只控制观看，不阻塞采集、串口或算法线程。
 */
class Display
{
  public:
    // 只限制图像提交；算法与数值曲线继续逐帧运行。
    static constexpr double kImageFps = 60;
    static constexpr double kSdlImageFps = 30;
    static Display &getInstance();
    Display(const Display &) = delete;
    Display &operator=(const Display &) = delete;
    ~Display();

    // 两个后端互斥：sdl只显示本机画面；rerun管理Viewer、连接和RRD。
    // spawn/file/address/image_encoding/hud_encoding仅供rerun使用，sdl忽略。
    // start/shutdown/poll由主线程调用；图元和end_frame由同一算法/预览生产者调用。
    void start(bool spawn = true, const std::string &file = {}, const std::string &address = {},
               const std::string &backend = "rerun", const std::string &image_encoding = "jpeg",
               const std::string &hud_encoding = "png");
    // 实机只准备公共display配置；模块注册并选择非空布局后，poll在主线程创建后端。
    void start(const std::filesystem::path &config_path, const std::filesystem::path &output = {});
    // 独立回放/采集已自行选择视图，传入配置节点时立即创建会话。
    void start(const YAML::Node &settings, const std::filesystem::path &config_path,
               const std::filesystem::path &output = {});
    void shutdown() noexcept;
    // 多个领域的调试工作线程按完整帧串行提交，避免时间线和图元交叉。
    std::mutex &frame_mutex()
    {
        return frame_mutex_;
    }
    // 提交端在复制图像前询问；采样策略由所选后端统一决定，不推进显示线程时间线。
    bool image_due(double seconds, std::optional<double> last_image_time) const;
    // set_time之后查询本帧是否需要图像；在组织文字、点线和投影前调用。
    bool image_due() const;
    // 只有Rerun记录数值；SDL调用方直接跳过曲线数据的整理。
    bool values_enabled() const;

    // SDL：一帧所有模块提交后发布任务，主线程取最新一组再做实际绘制。
    // Rerun 后端下两者为空操作。
    void end_frame();
    // 仅离线回放传入控制对象；实机仍只处理窗口事件，不响应暂停/单步。
    // on_key由主线程接收按键字符；关闭窗口时收到27（ESC），供标定工具结束采集。
    void poll(tool::PlaybackControl *playback = nullptr, const std::function<void(int)> &on_key = {});

    // 创建会话后、提交首帧前调用。空列表表示无窗口；不调用则按提交名称预览。
    // blueprint是调用方提供的Rerun布局字节；函数内同步读取，不保留span。
    void layout(const std::vector<DisplayView> &views, std::span<const unsigned char> blueprint = {});
    // 领域注册自己的布局，router只选择名称；后台不执行SDL窗口操作。
    void register_layout(const std::string &name, const std::vector<DisplayView> &views,
                         std::span<const unsigned char> blueprint = {});
    void select_layout(const std::string &name);

    // 必须使用当前帧曝光/媒体时间；SDK 时间线属于调用线程，不改动算法时间戳。
    void set_time(double seconds);
    // SDL按width等比缩放，高度由原图比例决定；height仅保留MAS调用兼容，当前不参与计算。
    // Rerun记录原图，忽略显示尺寸。SDL只读共享image存储，直到主线程显示完成。
    // 调用方不能原地覆盖待显示存储；海康相机池已用cv::Mat引用计数保护复用。
    void display_add(const std::string &name, const cv::Mat &image,
                     const std::vector<DisplayText> &texts = {},
                     const std::vector<DisplayPoint> &points = {},
                     const std::vector<DisplayLine> &lines = {}, int width = 720, int height = 540);

    // 数值跟随提交快照；Rerun 图像和二维图元最高 kImageFps，实机允许跳过旧快照。
    void scalar(const std::string &path, double value);
    void points2d(const std::string &path, const std::vector<cv::Point2d> &points,
                  const std::vector<std::string> &labels = {});
    void point3d(const std::string &path, double x, double y, double z);

  private:
    Display();
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::mutex frame_mutex_;
};

} // namespace rm_utils
