#include "buff_tracker.hpp"
#include "rune_model.hpp"
#include "mas_log.hpp"
#include "../predict/rune_predictor.hpp"
#include "../pose/big_activated.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace auto_buff
{
RmcsTracker::RmcsTracker(const std::filesystem::path &config_path, const std::filesystem::path &camera_config)
    : RmcsTracker(camera_config, YAML::LoadFile(config_path.string())["auto_buff"]["track"])
{
}

RmcsTracker::RmcsTracker(const std::filesystem::path &camera_config, const YAML::Node &settings)
    : camera_(camera_config)
{
    track_debug_.set_enabled(settings["debug"].as<bool>(false));
    // 丢失计时和显示预测时长分开配置，均使用媒体/曝光时间，而非循环运行耗时。
    timeout_ = settings["lost_timeout_s"].as<double>(1.5);
    horizon_ = settings["prediction_seconds"].as<double>(.1);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(horizon_) || horizon_ < 0)
        throw std::runtime_error("Invalid track timeout/prediction horizon");
    // 配置字段和默认值沿用 RMCS，宏仅用于同名字段赋值，不增加额外配置层。
#define READ_RMCS(name) config_.name = settings[#name].as<double>(config_.name)
    READ_RMCS(noise_x);
    READ_RMCS(noise_y);
    READ_RMCS(noise_z);
    READ_RMCS(noise_rotation_angle);
    READ_RMCS(noise_rotation_speed);
    READ_RMCS(noise_face_yaw);
    READ_RMCS(noise_observation);
    READ_RMCS(gate_threshold);
    READ_RMCS(init_seed_mean_error);
    READ_RMCS(init_seed_max_error);
    READ_RMCS(init_center_gate);
    READ_RMCS(init_pitch_bound);
    READ_RMCS(diverge_face_angle);
#undef READ_RMCS
    for (double value : {config_.noise_x, config_.noise_y, config_.noise_z, config_.noise_rotation_angle,
                         config_.noise_rotation_speed, config_.noise_face_yaw})
        if (!std::isfinite(value) || value < 0)
            throw std::runtime_error("RMCS process noise must be finite and nonnegative");
    for (double value :
         {config_.noise_observation, config_.gate_threshold, config_.init_seed_mean_error, config_.init_seed_max_error,
          config_.init_center_gate, config_.init_pitch_bound, config_.diverge_face_angle})
        if (!std::isfinite(value) || value <= 0)
            throw std::runtime_error("RMCS observation noise and gates must be finite and positive");

    MAS_LOG_INFO("Buff tracker ready: camera={}, lost_timeout={}s, prediction={}s", camera_config.string(), timeout_,
                 horizon_);
}

RmcsTracker::~RmcsTracker() = default;

void RmcsTracker::set_imu_orientation(const Eigen::Quaterniond &orientation)
{
    frame_gimbal_orientation_ = camera_.set_imu_orientation(orientation);
}

void RmcsTracker::reset()
{
    if (model_)
        MAS_LOG_DEBUG("Buff model reset");

    model_.reset();
    last_time_      = -1;
    corrected_time_ = -1;
    color_          = EnemyColor::UNKNOWN;
}

TrackResult RmcsTracker::update_model(const BuffDetector::Detection &detection, double seconds, cv::Size size)
{
    if (!std::isfinite(seconds) || seconds < 0)
        throw std::runtime_error("Invalid media timestamp");
    camera_.check_image_size(size);
    // 时间倒退或中断过久时旧模型已不适用；短时漏检仍走下面的 predict 分支。
    if (last_time_ >= 0 && (seconds <= last_time_ || seconds - last_time_ > timeout_))
    {
        MAS_LOG_DEBUG("Buff reset: media time discontinuity, previous={}, current={}", last_time_, seconds);
        reset();
    }
    // 输入已按指定红/蓝筛选；分数只用于选择 R 颜色，不淘汰同色位置候选。
    const RuneIcon *color_reference = nullptr;
    for (const auto &marker : detection.markers)
    {
        if (marker.observed && marker.color != EnemyColor::UNKNOWN &&
            (!color_reference || marker.score > color_reference->score))
            color_reference = &marker;
    }
    if (color_reference && color_ != EnemyColor::UNKNOWN && color_ != color_reference->color)
    {
        MAS_LOG_DEBUG("Buff reset: target color changed");
        reset();
    }
    if (color_reference)
        color_ = color_reference->color;
    // 无色 R 不决定目标颜色。YOLO 已按电控/配置筛板，初始化时使用首个有效板的颜色；
    // 模型建立后保持选色，避免同帧红蓝共存时每帧切换。传统有色 R 的选择逻辑不变。
    if (color_ == EnemyColor::UNKNOWN)
    {
        for (const auto &board : detection.boards)
            if ((board.color == EnemyColor::RED || board.color == EnemyColor::BLUE) &&
                board.appearance == Appearance::full_mark && board.features_valid)
            {
                color_ = board.color;
                break;
            }
    }
    const auto stamp =
        rm_utils::Timestamp{} + std::chrono::duration_cast<rm_utils::Duration>(std::chrono::duration<double>(seconds));
    rm_utils::playback_timestamp = stamp;
    // 公共 BuffDetector::Detection 只在适配层转换一次；模型只消费 R 和符板的几何观测。
    std::vector<auto_buff::RuneIcon> icons;
    std::vector<auto_buff::RuneBullseye> bullseyes;
    for (const auto &marker : detection.markers)
    {
        if (marker.observed && (marker.color == EnemyColor::UNKNOWN || marker.color == color_))
            icons.push_back(marker);
    }
    for (const auto &b : detection.boards)
    {
        if (b.appearance == Appearance::unknown || (b.color != EnemyColor::RED && b.color != EnemyColor::BLUE) ||
            (color_ != EnemyColor::UNKNOWN && b.color != color_))
            continue;
        // 初始化必须有完整物理端点；已初始化后缺点板仍可提供中心观测。
        if (b.appearance == Appearance::full_mark && !b.features_valid && !model_)
            continue;
        auto_buff::RuneBullseye converted{};
        converted.center = cv::Point2d(b.center);
        for (int i = 0; i < 4; ++i)
            converted.corners[i] = cv::Point2d(b.corners[i]);
        converted.active = b.appearance == Appearance::single_ring || b.appearance == Appearance::big_activated;
        converted.score  = 1.; // 原版跟踪不消费此字段；不把 PVD 或 YOLO 分数混作概率。
        bullseyes.push_back(converted);
    }
    TrackResult result;
    if (!detection.light_arms.empty())
    {
        std::vector<LightArm> arms;
        for (const auto &arm : detection.light_arms)
            if (arm.color == color_)
                arms.push_back(arm);
        result.geometry_boards = reconstruct_big_activated(arms, camera_.intrinsic(), camera_.distortion(),
                                                           camera_.transform(), config_, icons, bullseyes);
        for (const auto &board : result.geometry_boards)
        {
            auto_buff::RuneBullseye converted{};
            converted.center = cv::Point2d(board.center);
            converted.active = true;
            converted.score  = 1.;
            bullseyes.push_back(converted);
        }
    }
    result.gimbal_orientation = frame_gimbal_orientation_;
    frame_gimbal_orientation_.reset();
    bool initialized_now = false;
    // 未初始化时只尝试几何建模；成功后的后续帧才执行 EKF 预测和观测校正。
    if (!model_)
    {
        auto model = std::make_unique<auto_buff::RuneModel>(config_);
        model->update_camera(camera_.intrinsic(), camera_.distortion());
        model->update_transform(camera_.transform());
        if (model->init(icons, bullseyes, stamp))
        {
            model_          = std::move(model);
            corrected_time_ = seconds;
            initialized_now = true;

            const auto now = std::chrono::steady_clock::now();
            if (init_log_time_ == std::chrono::steady_clock::time_point{} ||
                now - init_log_time_ >= std::chrono::seconds(5))
            {
                const auto state = model_->state();
                MAS_LOG_INFO("Buff model initialized: center=[{:.3f},{:.3f},{:.3f}]m, angle={:.3f}rad", state.x,
                             state.y, state.z, state.rotation_angle);
                init_log_time_ = now;
            }
        }
    }
    else
    {
        model_->update_transform(camera_.transform());
        // 遮挡期间仍推进时间和协方差；空 correct 只刷新投影，不产生拟合样本。
        model_->predict(seconds - last_time_, stamp);
        bool pending_corrected = model_->correct(icons, bullseyes);
        // 原版返回“待击板已校正”；仅 R 或已激活圆环校正不刷新丢失计时。
        if (pending_corrected)
            corrected_time_ = seconds;
        const bool diverged = model_->diverged();
        if (diverged || seconds - corrected_time_ > timeout_)
        {
            const auto now = std::chrono::steady_clock::now();
            if (!diverged && (lost_log_time_ == std::chrono::steady_clock::time_point{} ||
                              now - lost_log_time_ >= std::chrono::seconds(5)))
            {
                MAS_LOG_WARN("Buff model lost: no pending-board correction for {:.3f}s (limit={}s)",
                             seconds - corrected_time_, timeout_);
                lost_log_time_ = now;
            }
            reset();
            result.status = "lost_or_diverged";
        }
    }
    last_time_ = seconds;
    if (!model_)
    {
        return result;
    }
    result.color       = color_;
    result.initialized = true;
    result.timestamp   = stamp;
    result.state       = model_->state();
    result.ekf         = model_->ekf_snapshot();
    // 空观测分支已清空 tracked，不会把上一帧的匹配当作本帧新观测。
    result.matched   = model_->addition().tracked;
    result.observed  = initialized_now || !result.matched.empty();
    result.predicted = model_->addition().predicted;
    // 上游 converge 恒为 true；实际可用条件由瞄准点预热和待击板有效期决定。
    result.ready  = !result.state.get_aimpoints().empty();
    result.status = result.observed ? (result.ready ? "tracking" : "warming_up") : "predicting";
    // 未来投影只修改状态副本，直接复用当前相机参数。
    result.future = auto_buff::predict_rune_features(result.state, horizon_, camera_.intrinsic(), camera_.distortion(),
                                                     camera_.transform());
    return result;
}

TrackResult RmcsTracker::update(const BuffDetector::Detection &detection, double seconds, const cv::Mat &image)
{
    auto result = update_model(detection, seconds, image.size());
    track_debug_.draw(result, image);
    return result;
}

} // namespace auto_buff
