#pragma once
#include "auto_buff/buff_detect/detection.hpp"
#include "auto_buff/debug/visualize/track_debug.hpp"
#include "rune_model.hpp"
#include "../pose/rune_camera.hpp"
#include <Eigen/Geometry>
#include <filesystem>
#include <memory>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace auto_buff
{
// 对外的单帧跟踪结果。像素特征均使用原图坐标，空间状态沿用 RMCS 前/左/上世界系。
struct TrackResult
{
    bool initialized = false;        // 当前是否持有有效模型；不等于已经允许发射。
    bool observed    = false;        // 本帧是否初始化成功或有观测通过关联和校正。
    bool ready       = false;        // 预热完成且存在仍有效的待击板。
    rm_utils::Timestamp timestamp{}; // 当前状态对应的曝光/媒体时刻。
    CampColor color = CampColor::UNKNOWN;
    std::string status = "waiting_for_full_mark";
    std::vector<Board> geometry_boards; // pose 用本帧灯臂观测反解的板心，单独诊断。
    auto_buff::RuneModel::State state{};
    RuneEkfSnapshot ekf;                                  // 当前后验；initialized=false 时不可作为有效状态使用。
    std::optional<Eigen::Quaterniond> gimbal_orientation; // 本帧输入的云台到世界姿态，仅供显示。
    std::vector<auto_buff::RuneModel::Addition::Tracked> predicted; // 本轮校正前的特征投影。
    std::vector<auto_buff::RuneModel::Addition::Tracked> matched;   // 本帧通过关联的实际像素观测。
    std::vector<auto_buff::RuneModel::Addition::Tracked> future;    // 指定预测时长后的五个符板中心。
};

// 当前拟合公式的显示文本。
std::string rune_motion_text(const TrackResult &track);

// 跟踪子系统的应用入口：组织观测和生命周期，内部复用 RMCS pose / EKF /
// predict。
class RmcsTracker
{
public:
    /**
     * @brief 创建位姿相机并读取 RMCS 跟踪参数；相机 YAML 由 pose 内部加载。
     * @param camera_config 支持平铺或 calibration 节点的相机 YAML。
     * @param track_config track 配置节点，含丢失时长、预测时长及原版模型门限。
     */
    RmcsTracker(const std::filesystem::path &camera_config, const YAML::Node &track_config);
    /// 实机/回放入口：读取 track 参数，将相机配置路径交给 pose 内部加载。
    RmcsTracker(const std::filesystem::path &config_path, const std::filesystem::path &camera_config);
    /**
     * @brief 转换公共观测并执行初始化，或 predict → correct → 丢失检查。
     * @param detection 本帧全部候选；初始位姿需要 R 和完整待击板的真实端点。
     * @param media_seconds 曝光/媒体时间，单位秒；倒退或长间隔会重新初始化。
     * @param image 同帧原图；使用 image.size() 核对标定尺寸，开启 debug 时同时用于绘制。
     * @return 当前状态、关联观测和未来投影；初始化失败时 initialized=false。
     */
    // 完成计算后用同帧原图绘制，图像尺寸直接用于标定一致性检查。
    TrackResult update(const Detection &detection, double media_seconds, const cv::Mat &image);
    void disable_debug() noexcept
    {
        track_debug_.set_enabled(false);
    }
    /// 清空模型、颜色和计时；相机参数及手眼标定继续保留。
    void reset();
    /**
     * @brief 用曝光时刻的 IMU 姿态更新相机到世界的变换。
     * @param orientation mas_vision 串口 IMU 本体基下的姿态，内部归一化并应用基变换。
     * @note 须配置完整手眼及 R_gimbal2imubody，并在对应帧的 update 前调用。
     */
    void set_imu_orientation(const Eigen::Quaterniond &orientation);

private:
    TrackResult update_model(const Detection &detection, double media_seconds, cv::Size image_size);
    TrackDebug track_debug_;
    RuneCamera camera_; // 位姿模块内部读取并持有相机标定，跟踪不再解析相机 YAML。
    std::optional<Eigen::Quaterniond> frame_gimbal_orientation_; // 随下一次 update 消费，避免显示上一帧姿态。
    auto_buff::RuneModel::Config config_;
    std::unique_ptr<auto_buff::RuneModel> model_;
    double last_time_ = -1, corrected_time_ = -1; // 上轮时间 / 最近待击板校正时间，单位秒。
    double timeout_ = 1.5, horizon_ = .1;         // 丢失重置门限 / 未来投影时长，单位秒。
    CampColor color_ = CampColor::UNKNOWN;
};
} // namespace auto_buff
