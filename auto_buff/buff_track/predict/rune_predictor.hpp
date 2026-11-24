#pragma once

#include "auto_buff/buff_track/buff_state.hpp"
#include "rm_utils/algorithm/buff/rune_energy_fitter.hpp"

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <vector>

namespace auto_buff
{

// 只保存拟合样本窗口；恒速/正弦的数值拟合位于 rm_utils/algorithm。
class RunePredictor
{
public:
    /// 清空拟合窗口和正弦优先时限；调用者另行清空 RunePrediction 结果。
    void reset();
    /**
     * @brief 根据真实符板校正样本更新运动拟合，并选择恒速或正弦模型。
     * @param pending_count 本帧校正的待击板数；多个待击板触发三秒正弦优先窗口。
     * @param active_count 本帧校正的打击后圆环数；纯预测或仅 R 校正不采样。
     * @param stamp 当前曝光时刻，用于维护优先窗口。
     * @param elapsed_seconds 从模型初始化到本帧的时长，单位秒。
     * @param rotation_angle EKF 当前连续转角，单位弧度。
     * @param prediction 接收拟合参数及采样计数；不是另一个 EKF 状态。
     */
    void correct(int pending_count, int active_count, rm_utils::Timestamp stamp, double elapsed_seconds,
                 double rotation_angle, RunePrediction &prediction);

private:
    rm_utils::RuneEnergyFitter fitter_;
    rm_utils::Timestamp force_sine_until_{}; // 在此时刻之前优先采用可用的正弦解。
};

/**
 * @brief 在状态副本上外推，再将五个符板中心投影回原图。
 * @param state 当前世界系状态，调用后保持不变。
 * @param seconds 向未来外推的时长，单位秒。
 * @param intrinsic 原图内参矩阵。
 * @param distortion 畸变系数。
 * @param transform 当前相机到世界的变换，平移单位米。
 * @return 可投影的符板中心，feature_id 为 1～5；不包含 R。
 */
auto predict_rune_features(const RuneState &state, double seconds, const cv::Mat &intrinsic, const cv::Mat &distortion,
                           const Eigen::Isometry3d &transform) -> std::vector<RuneAddition::Tracked>;

} // namespace auto_buff
