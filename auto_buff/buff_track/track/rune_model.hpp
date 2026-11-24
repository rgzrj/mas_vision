#pragma once

#include "auto_buff/buff_track/buff_state.hpp"

#include <Eigen/Geometry>
#include <memory>
#include <span>

namespace auto_buff
{

// RMCS 六维 EKF 模型：持有滤波、板位有效期及拟合上下文，不依赖检测器或串口。
class RuneModel
{
public:
    /**
     * @brief 创建尚未初始化的模型，并设置过程噪声和初始协方差。
     * @param config RMCS 原版模型及初始化门限。
     */
    explicit RuneModel(const RuneConfig &config) noexcept;
    ~RuneModel() noexcept;

    RuneModel(const RuneModel &)            = delete;
    RuneModel &operator=(const RuneModel &) = delete;

    /// 设置原图内参（行主序 3×3）和五项畸变，不改变当前滤波状态。
    auto update_camera(const cv::Mat &matrix, const cv::Mat &distortion) noexcept -> void;
    /// 更新相机到世界的变换；动态相机在对应帧预测/校正前调用。
    auto update_transform(const Eigen::Isometry3d &transform) noexcept -> void;

    /**
     * @brief 由 R 和完整待击板求初始位姿，建立 EKF 与五板投影。
     * @param icons 本帧全部 R 候选。
     * @param bullseyes 已由适配层验证的符板观测。
     * @param timestamp 本帧曝光时刻，也是该模型预热计时的起点。
     * @return 几何初始化成功返回 true；失败时由外层继续等待观测。
     */
    auto init(std::span<const auto_buff::RuneIcon> icons, std::span<const auto_buff::RuneBullseye> bullseyes,
              rm_utils::Timestamp timestamp) noexcept -> bool;
    /**
     * @brief 按恒角速度推进 EKF 状态和协方差，并刷新投影及正弦相位。
     * @param dt 距上一帧的时间间隔，单位秒。
     * @param now 当前曝光时刻，用于板位有效期与后续校正。
     */
    auto predict(double dt, rm_utils::Timestamp now) noexcept -> void;
    /**
     * @brief 用马氏距离及匈牙利匹配关联本帧观测，再逐点校正 EKF。
     * @param icons 本帧 R 候选，只能与 R 特征关联。
     * @param bullseyes 待击板或打击后圆环，只能与五个符板中心关联。
     * @return 至少一个待击板完成校正时为 true；仅 R 或圆环校正仍返回 false。
     * @note 返回值用于外层丢失计时；空观测只清空匹配，不产生拟合样本。
     */
    [[nodiscard]] auto correct(std::span<const auto_buff::RuneIcon> icons,
                               std::span<const auto_buff::RuneBullseye> bullseyes) noexcept -> bool;

    /// 检查状态、协方差、空间范围及符面朝向是否发散。
    auto diverged() const -> bool;

    /// 导出状态副本，角速度优先取正弦拟合、其次恒速拟合、最后 EKF。
    auto state() const noexcept -> RuneState;
    /// 原始 EKF 后验及 P 对角项，只读导出，不使用运动拟合结果替代。
    auto ekf_snapshot() const noexcept -> RuneEkfSnapshot;
    /// 返回模型持有的本帧关联观测和校正前投影，不复制缓存。
    auto addition() const -> const RuneAddition &;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace auto_buff
