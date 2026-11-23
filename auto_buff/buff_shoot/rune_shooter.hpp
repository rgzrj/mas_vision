#pragma once

#include "auto_buff/buff_track/buff_state.hpp"
#include "rm_utils/pimpl.hpp"

#include <opencv2/core/mat.hpp>
#include <optional>
#include <filesystem>

namespace auto_buff
{
struct TrackResult;
}

namespace auto_buff
{

// 仅保留 RMCS FireController 的能量机关路径；装甲板选板逻辑不属于本工程。
class RuneShooter
{
    RMCS_PIMPL_DEFINITION(RuneShooter)

public:
    struct Config
    {
        double bullet_speed        = 22.5;  // m/s
        double gravity             = 9.81;  // 重力加速度，单位：m/s^2
        double resistance          = 0.014; // 二次空气阻力系数，单位：1/m
        double fire_delay_time     = 0.080; // 统一发弹延迟，单位：s
        double offset_yaw          = 0.0;   // degree
        double offset_pitch        = 0.0;   // degree
        double max_pitch           = 80.0;  //degree
        double rune_idle_duration  = 0.4;   // s
        double rune_shoot_duration = 0.2;   // s
    };

    struct State
    {
        rm_utils::Timestamp timestamp{}; // 当前解算时刻；回放使用记录时间，实机使用当前 steady_clock。
    };

    struct Aimed
    {
        double aim_yaw         = 0.0; // 目标绝对 yaw，rad，左为正；不是相对当前云台角的增量。
        double aim_pitch       = 0.0; // 目标绝对 pitch，rad，下为正。
        double fly_time        = 0.0; // 弹道解算得到的飞行时长，秒。
        double prediction_time = 0.0; // 飞行 + 发令延迟 + 从观测到当前解算的时长，秒。

        bool should_shoot = false; // 已选中待击板且处于 shoot 窗口的计算结果，由应用提交电控。
        bool idle         = true;  // 当前是否处于 RMCS idle 阶段。

        auto_buff::AimPoint target; // 含角度偏置的单位瞄准方向及方向角速度/角加速度前馈。
        Eigen::Vector3d center;    // 外推时刻的符中心，世界系米。
        Eigen::Vector3d attack;    // 弹道迭代选中的击打点；无待击板时退回符中心。
    };

    /**
     * @brief 创建 RMCS 打符火控，读取弹速、发令延迟、角度偏置和发弹节奏。
     * @param config 偏置输入为度，其余时间参数为秒，弹速为 m/s。
     */
    explicit RuneShooter(const Config &config);
    /// 实机/回放入口：模块内部读取 shoot 参数。
    explicit RuneShooter(const std::filesystem::path &config_path);

    /// 更新解算时钟；应在同一次 aim 调用之前设置。
    auto update(State state) -> void;
    /**
     * @brief 外推待击板并迭代飞行时间，输出目标角和发射建议。
     * @param track 曝光时刻的世界系跟踪状态；内部使用副本外推，不改变 EKF。
     * @param track_timestamp 此状态对应的曝光时刻，用于补偿已过去的处理时间。
     * @return 弹道解；不可达时为空，未预热或没有待击板时只瞄符中心并禁止发弹。
     * @note 此模块只计算，不访问串口或设备。
     */
    auto aim(const RuneState &track, rm_utils::Timestamp track_timestamp) -> std::optional<Aimed>;
    /**
     * @brief 根据跟踪结果解算或重置火控；开启调试时在模块内部提交结果。
     * @param track 本帧跟踪结果；未初始化时重置发弹周期。
     */
    auto shoot(const auto_buff::TrackResult &track, const cv::Mat &image = {}) -> std::optional<Aimed>;

    void disable_debug() noexcept;
    /// 清空 idle/shoot 周期；模型丢失、换模式或重新初始化时调用。
    auto reset() noexcept -> void;
};

} // namespace auto_buff
