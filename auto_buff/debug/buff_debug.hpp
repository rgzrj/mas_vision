#pragma once

#include "rm_utils/display/display.hpp"

#include <chrono>
#include <filesystem>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

namespace YAML
{
class Node;
}

namespace auto_buff
{
// 管理打符调试窗口布局和实机单帧提交。
class BuffDebug
{
public:
    explicit BuffDebug(const std::filesystem::path &config);

    // 锁随调用方的帧作用域释放，保证同一帧的绘制使用同一个时间戳。
    std::unique_lock<std::mutex> begin_frame(std::chrono::steady_clock::time_point timestamp) const;
    void pose_unavailable(const cv::Mat &image) const;
    void end_frame() const;

    /**
     * @brief 读取各模块的图像调试开关
     * @param config 打符配置，包含 auto_buff 节点
     * @return 窗口位掩码，bit 0～2 分别对应 detect、track、shoot；0 表示全部关闭
     */
    static unsigned enabled_views(const YAML::Node &config);

    /**
     * @brief 从配置文件读取调试开关，并注册实机窗口布局
     * @param config_path 打符配置文件路径
     * @return 已启用的窗口位掩码，供当前帧的显示控制使用
     */
    static unsigned configure_layout(const std::filesystem::path &config_path);

    /**
     * @brief 根据已启用的窗口设置布局
     * @param views enabled_views 返回的窗口位掩码
     * @param realtime true 注册实机布局，false 直接设置独立回放布局
     */
    static void configure_layout(unsigned views, bool realtime);

    /**
     * @brief 缺少姿态或模式未启用时，清除上一帧叠图并显示当前状态
     * @param image 当前帧 BGR 图像
     * @param views 已启用的窗口位掩码
     * @param status 待显示的状态文字
     * @param include_detect true 同时清理检测窗口，false 保留本帧检测结果
     * @details 只更新状态文字；曲线由对应调试子类清理，不运行算法
     */
    static void unavailable(const cv::Mat &image, unsigned views, const std::string &status,
                            bool include_detect = false);

private:
    unsigned views_;
};

// 各绘制子类共用窗口开关、FPS 统计和图元提交。
class BuffDebugBase
{
public:
    bool enabled() const noexcept
    {
        return enabled_;
    }
    void set_enabled(bool enabled) noexcept
    {
        enabled_ = enabled;
    }

protected:
    /**
     * @brief 初始化当前模块的调试开关与窗口名
     */
    BuffDebugBase(bool enabled, const std::string &window_name);

    /**
     * @brief 更新本实例的处理帧率；首个统计周期不足一秒时返回 -1
     */
    double update_fps();

    /**
     * @brief 将子类生成的文字、点和线提交到本实例的 Display 窗口
     * @note 本方法只提交图元，不修改文字；Display持有只读图像引用直到显示完成
     */
    void show(const cv::Mat &image, std::vector<rm_utils::DisplayText> &texts,
              const std::vector<rm_utils::DisplayPoint> &points, const std::vector<rm_utils::DisplayLine> &lines) const;

private:
    bool enabled_ = false;    ///< YAML 控制的模块图像调试开关
    std::string window_name_; ///< 本模块对应的 Display 窗口

    // 每个调试实例独立统计算法调用频率，首个完整周期前 FPS 为 -1。
    std::chrono::steady_clock::time_point fps_start_{};
    unsigned fps_frames_ = 0;
    double fps_          = -1;
};

} // namespace auto_buff
