#ifndef _ARMOR_TRACK_H_
#define _ARMOR_TRACK_H_

#include "armor_track/armor_pose.hpp"
#include "armor_track/armor_target.hpp"
#include "plotter.hpp"
#include "auto_aim/debug/visualize/track_debug.hpp"
#include "auto_aim/debug/visualize/pose_debug.hpp"

#include <chrono>
#include <Eigen/Dense>
#include <map>
#include <optional>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

namespace auto_aim
{

class ArmorTrack
{
  public:
    ArmorTrack(const std::string &track_config_path = "config/auto_aim.yaml",
               const std::string &pose_config_path = "config/hikcamera.yaml");
    ~ArmorTrack();
    /**
     * @brief 获取位姿解算器
     * @return 位姿解算器引用
     */
    ArmorPose &armor_pose() noexcept { return armor_pose_; }
    /**
     * @brief 主跟踪函数
     * @param armors 检测到的装甲板列表
     * @param t 当前时间戳
     * @param bgr_img 图像，用尺寸确定中心优先级
     * @return 当前锁定目标，tracking/temp_lost 状态下有值，否则 nullopt
     */
    std::optional<Target> track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t,
                                const cv::Mat &bgr_img) noexcept;

    const ArmorPose &armor_pose() const noexcept
    {
        return armor_pose_;
    }
    const std::optional<Target> &target_state() const noexcept
    {
        return target_;
    }
    void reset();
    // 只影响图像显示，不影响Plotter和手眼采样。
    void disable_debug() noexcept
    {
        track_debug_.set_enabled(false);
        pose_debug_.set_enabled(false);
    }
    std::string state() const { return state_; }

  private:
    TrackDebug track_debug_;
    PoseDebug pose_debug_;

    // 装甲板姿态解算器
    ArmorPose armor_pose_;
    // 当前跟踪目标
    std::optional<Target> target_;
    // 状态机标志，取值为 "lost"、"tracking"、"temp_lost"
    std::string state_ = "lost";

    int min_detect_count_            = 5;                                  // 最小检测次数
    int detect_count_                = 0;                                  // 当前检测次数
    int temp_lost_count_             = 0;                                  // 当前临时丢失次数
    int max_temp_lost_count_         = 25;                                 // 普通目标最大丢失次数
    int outpost_max_temp_lost_count_ = 75;                                 // 前哨站最大丢失次数
    std::chrono::steady_clock::time_point last_timestamp_;                 // 上一帧时间戳

    /**
     * @brief 状态机处理
     * @param found 是否找到目标
     */
    void state_machine(bool found);

    /**
     * @brief 设置新目标
     * @param armor 初始装甲板
     * @param t 时间戳
     * @return 是否成功
     */
    bool set_target(const Armor &armor, std::chrono::steady_clock::time_point t);

    /**
     * @brief 更新目标
     * @param armors 检测到的装甲板列表
     * @param t 时间戳
     * @return 是否成功
     */
    bool update_target(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t);

    // 手眼标定的 UDP 采集独立于图像 debug。
    bool plotter_enable_ = false;
    rm_utils::Plotter plotter_;
    void plotHandeye(const std::vector<Armor> &armors);
};

} // namespace auto_aim
#endif // _ARMOR_TRACK_H_
