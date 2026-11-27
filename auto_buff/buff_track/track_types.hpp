#pragma once

#include "rm_utils/clock.hpp"
#include "auto_buff/buff_types.hpp"

#include <Eigen/Core>
#include <array>
#include <span>
#include <vector>

namespace auto_buff {

// track 持有当前滤波状态；pose 返回初始化解，predict 只外推状态副本。
// 保留 RMCS 原字段，便于与上游对照公式、拟合结果和调试画面。
struct RuneState
{
    // RMCS 世界坐标，单位米；旋转角为 rad，角速度为 rad/s。
    double x;
    double y;
    double z;

    rm_utils::Timestamp start_timestamp; // 几何初始化时刻，用于原版三秒/六秒预热。

    double rotation_speed; // 当前采用的角速度，来自拟合或 EKF，单位 rad/s。
    double rotation_angle; // 连续转角；拟合使用连续值，不在每帧折回 [-pi,pi]。

    double face_yaw = 0; // 符面法线在世界水平面中的偏航角，单位 rad。

    std::array<bool, 5> inactive; // 沿用上游命名：true 表示当前仍可击打的符板。

    bool use_prediction_speed = false; // 恒速拟合是否被选中；为 false 时还需看 sine_valid。
    double prediction_cost    = 0.0;   // 当前被选中模型的加权转角残差平方均值。

    double sine_C     = 0.0;   // 转角拟合常数，单位 rad；保留以对照原版拟合结果。
    double sine_v     = 0.0;   // 正弦角速度的基值，单位 rad/s。
    double sine_a     = 0.0;   // 角速度振幅，单位 rad/s。
    double sine_omega = 0.0;   // 正弦频率参数，单位 rad/s。
    double sine_phase = 0.0;   // 当前状态时刻的相位 omega*t+phi，单位 rad，不是初相 phi。
    double sine_t     = 0.0;   // 当前拟合时间，相对于模型初始化，单位秒。
    bool sine_valid   = false; // 是否使用正弦积分进行未来外推。

    std::size_t update_count = 0; // 有符板/圆环完成校正的帧数，不含纯预测或仅 R 校正。

    /**
     * @brief 按恒速或正弦积分推进当前状态，供预测器在状态副本上调用。
     * @param seconds 向未来推进的时长，单位秒；不修改 EKF 协方差。
     */
    auto transition(double seconds) -> void;

    /// 返回符中心的世界坐标，单位米，供弹道解算使用。
    auto center_in_world() const -> Eigen::Vector3d;
    /**
     * @brief 取得当前首个有效待击板的空间瞄准点及方向前馈。
     * @return 未预热完成或无待击板时为空；保留 RMCS 的固定板位遍历顺序。
     */
    auto get_aimpoints() const -> auto_buff::AimPoints;
};

// 原始 EKF 后验快照，独立于 RuneState 中可能已替换为拟合值的角速度。
// 顺序：[世界 x,y,z (m), 角速度 (rad/s), 转角 (rad), 符面偏航 (rad)]。
struct RuneEkfSnapshot
{
    std::array<double, 6> state{};
    std::array<double, 6> variance{}; // 同序的 P 对角项，单位为对应状态单位的平方。
};

// 初始化几何解由 pose 返回，不持有滤波器或生命周期。
struct RunePose
{
    Eigen::Vector3d center_in_world; // 符中心世界坐标，单位米。
    double face_yaw_in_world;        // 符面法线的世界系偏航，单位 rad。
    double rotation_angle;       // 作为种子的符板转角，单位 rad。
    cv::Point2d icon_pixel;          // 被选中 R 的原图像素坐标。
    cv::Point2d seed_pixel;          // 被选中待击板的圆心，用于初始 EKF 校正。
};

// 拟合参数由 track 的 Context 持有；预测器只保存真实校正样本窗口。
struct RunePrediction
{
    double prediction_speed   = 0.0;   // 线性转角拟合的斜率，单位 rad/s。
    bool use_prediction_speed = false; // 是否采用恒速拟合；与 sine_valid 联合决定模型。
    double prediction_cost    = 0.0;   // 恒速模型的加权转角残差平方均值。

    double sine_C     = 0.0;   // 转角拟合常数，rad。
    double sine_v     = 0.0;   // 角速度基值，rad/s。
    double sine_a     = 0.0;   // 角速度振幅，rad/s。
    double sine_omega = 0.0;   // 正弦频率参数，rad/s。
    double sine_phase = 0.0;   // 当前曝光时刻的相位；predict 会随时间推进。
    double sine_t     = 0.0;   // 相对于模型初始化的时间，单位秒。
    bool sine_valid   = false; // 当前是否选择了可用的正弦拟合。
    double sine_cost  = 0.0;   // 正弦模型的加权转角残差平方均值。

    std::size_t update_count = 0; // 有符板校正的帧数，用于诊断，不含纯预测。
};

// EKF 过程噪声按 [x,y,z,角速度,转角,符面偏航] 排列；保持 RMCS 原字段和默认值。
struct RuneConfig
{
    // EKF过程噪声Q矩阵对角元，单位：
    // x/y/z：m²
    double noise_x = 1e-5;
    double noise_y = 1e-5;
    double noise_z = 1e-5;

    double noise_rotation_angle = 1e-3; // theta（自转角度）过程噪声，rad²
    double noise_rotation_speed = 1e-0; // w（自转角速度）过程噪声，(rad/s)²
    double noise_face_yaw       = 1e-5; // psi（符面偏航角）过程噪声，rad²

    double noise_observation = 20.0; // 观测噪声R对角元，像素²；二维像素(u,v)观测协方差
    
    double gate_threshold = 13.816; // 匈牙利关联使用的平方马氏距离门限。

    double init_seed_mean_error = 10.0; // PnP 五个种子点的重投影 RMS 上限，像素。
    double init_seed_max_error  = 20.0; // 任一种子点的重投影误差上限，像素。
    double init_center_gate     = 30.0; // 布局验证中的符板中心误差门限，像素。
    double init_pitch_bound     = 20.0; // 初始化允许的符面法线俯仰绝对值，度。

    double diverge_face_angle = 45.0; // 符面法线与世界原点到符中心方向的水平夹角上限，度。
};

struct RuneAssociation
{
    struct Tracked
    {
        int feature_id; // 0 为 R，1～5 为五个板位；不是检测数组下标。
        cv::Point2d point;  // 原图像素坐标。
    };

    std::vector<Tracked> matched;   // 本帧被接受的观测。
    std::vector<Tracked> predicted; // 本轮关联前各特征的投影位置。
};

} // namespace auto_buff
