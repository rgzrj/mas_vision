#pragma once

#include "../display.hpp"
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;

namespace rm_utils
{

/**
 * @brief SDL 独立窗口后端，提交只读图像和图元，主线程负责绘制。
 * @note begin_frame/add/points/end_frame 由 Display 串行调用。
 */
class SdlDisplay
{
  public:
    SdlDisplay();
    ~SdlDisplay();

    /**
     * @brief 设置可见视图顺序，不创建或操作窗口。
     */
    void layout(const std::vector<DisplayView> &views);

    /**
     * @brief 开始整理本帧图像和图元。
     */
    void begin_frame();
    void add(const std::string &name, const cv::Mat &image, const std::vector<DisplayText> &texts,
             const std::vector<DisplayPoint> &points, const std::vector<DisplayLine> &lines, int width);
    void points(const std::string &path, const std::vector<cv::Point2d> &points,
                const std::vector<std::string> &labels);

    /**
     * @brief 发布完整帧，覆盖尚未显示的旧任务。
     */
    void end_frame();

    /**
     * @brief 主线程处理事件并显示最新帧；关闭单窗口不停止业务。
     */
    void poll(tool::PlaybackControl *playback = nullptr, const std::function<void(int)> &on_key = {});

  private:
    struct Panel
    {
        cv::Mat image; // 持有存储引用直到显示完成；相机池依据引用计数避免覆盖。
        std::vector<DisplayText> texts;
        std::vector<DisplayPoint> points;
        std::vector<DisplayLine> lines;
        int width = 720;
    };

    struct CachedText
    {
        DisplayText style;
        SDL_Texture *texture = nullptr;
        cv::Size size;
        int baseline = 0, padding = 0;
    };

    struct Window
    {
        SDL_Window *window     = nullptr;
        SDL_Renderer *renderer = nullptr;
        SDL_Texture *texture   = nullptr;
        cv::Size texture_size;
        std::uint32_t texture_format = 0;
        std::vector<CachedText> text_cache; // 只缓存最近50种文字；动态数字不会无限增长。
    };

    struct UploadBuffer
    {
        cv::Mat source; // 仅在本次poll中标识底图；下一次poll先释放，不能跨帧复用内容。
        cv::Mat pixels; // 保留已分配的BGRA存储，同源窗口共享一次转换。
    };

    // 主线程持有窗口、纹理和像素转换缓冲。
    std::vector<UploadBuffer> upload_buffers_;
    std::unordered_map<std::string, Window> windows_; // 仅主线程操作系统窗口与纹理。
    std::atomic<bool> closed_{false};                 // SDL_QUIT；关闭单个窗口不关闭整个显示后端。

    // 布局和待显示任务由 mutex_ 保护。
    bool layout_set_ = false;
    std::vector<std::string> names_;

    // 当前帧由单个提交者整理；发布后交给主线程。
    std::unordered_map<std::string, Panel> panels_;
    std::unordered_map<std::string, std::vector<DisplayPoint>> raw_points_;
    std::unordered_map<std::string, std::vector<DisplayText>> raw_labels_;

    std::mutex mutex_;
    std::unordered_map<std::string, Panel> pending_; // 仅保留最新一组任务；主线程取走后释放锁再绘制。
    std::unordered_set<std::string> closed_names_;   // mutex_ 保护，避免工作线程重绘已关闭窗口。

    void handle_events(tool::PlaybackControl *playback, const std::function<void(int)> &on_key);
    void prepare_window(Window &view, const std::string &name, const Panel &panel);

    bool hidden(const std::string &name);

    const CachedText &text_texture(Window &view, const DisplayText &text);
    const cv::Mat &bgra_image(const cv::Mat &image);

    void render(Window &view, const Panel &panel);
    void render_geometry(Window &view, const Panel &panel);
    void render_text(Window &view, const Panel &panel);
};

} // namespace rm_utils
