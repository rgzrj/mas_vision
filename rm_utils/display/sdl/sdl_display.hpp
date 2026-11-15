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

// 提交线程只生成显示尺寸的BGR图；所有SDL窗口、事件和纹理操作均由主线程执行。
class SdlDisplay
{
  public:
    SdlDisplay();
    ~SdlDisplay();
    // 线程安全的布局描述；不操作SDL窗口，不设置则接受任意预览名称。
    void layout(const std::vector<DisplayView> &views);
    // 以下四个提交接口由同一个诊断/预览生产者调用，Display串行保护。
    void begin_frame();
    void add(const std::string &name, const cv::Mat &image, const std::vector<DisplayText> &texts,
             const std::vector<DisplayPoint> &points, const std::vector<DisplayLine> &lines, int width);
    void points(const std::string &path, const std::vector<cv::Point2d> &points,
                const std::vector<std::string> &labels);
    void end_frame();
    // 仅主线程调用：消费最新一组图像并处理事件；关闭单窗口不停止业务任务。
    void poll(tool::PlaybackControl *playback = nullptr, const std::function<void(int)> &on_key = {});

  private:
    struct Window
    {
        SDL_Window *window = nullptr;
        SDL_Renderer *renderer = nullptr;
        SDL_Texture *texture = nullptr;
        cv::Size texture_size;
    };
    std::unordered_map<std::string, Window> windows_; // 仅主线程操作系统窗口与纹理。
    std::atomic<bool> closed_{false}; // SDL_QUIT；关闭单个窗口不关闭整个显示后端。
    bool layout_set_ = false;
    std::vector<std::string> names_;
    std::unordered_map<std::string, cv::Mat> panels_;
    std::unordered_map<std::string, std::vector<DisplayPoint>> raw_points_;
    std::unordered_map<std::string, std::vector<DisplayText>> raw_labels_;
    std::mutex mutex_;
    std::unordered_map<std::string, cv::Mat> pending_; // 仅保留最新一组，分别呈现至各窗口。
    std::unordered_set<std::string> closed_names_; // mutex_ 保护，避免工作线程重绘已关闭窗口。
    bool hidden(const std::string &name);
};

} // namespace rm_utils
