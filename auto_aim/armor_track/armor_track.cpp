#include "armor_track.hpp"

#include "mas_log.hpp"
#include "math_tools.hpp"
#include "yaml-cpp/yaml.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <opencv2/core/mat.hpp>
#include <utility>

namespace auto_aim
{
ArmorTrack::ArmorTrack(const std::string &track_config_path, const std::string &pose_config_path)
    : armor_pose_(pose_config_path), last_timestamp_(std::chrono::steady_clock::now())
{
    try
    {
        // 加载YAML配置文件
        const YAML::Node config = YAML::LoadFile(track_config_path);
        const YAML::Node track  = config["auto_aim"]["armor_track"];

        track_debug_.set_enabled(track["armor_track_debug"].as<bool>(track["debug"].as<bool>(false)));
        pose_debug_.set_enabled(track_debug_.enabled() && track["armor_pose_debug"].as<bool>(false));
        
        plotter_enable_              = track["plotter_enable"].as<bool>(false);
        min_detect_count_            = track["min_detect_count"].as<int>(5);
        max_temp_lost_count_         = track["max_temp_lost_count"].as<int>(25);
        outpost_max_temp_lost_count_ = track["outpost_max_temp_lost_count"].as<int>(75);

        MAS_LOG_INFO("armor_track yaml loaded successfully");
    }
    catch (const std::exception &e)
    {
        MAS_LOG_WARN("Failed to load config, using defaults: {}", e.what());
    }
}

ArmorTrack::~ArmorTrack() = default;

void ArmorTrack::reset()
{
    target_.reset();
    state_           = "lost";
    detect_count_    = 0;
    temp_lost_count_ = 0;

    last_timestamp_ = {};
}

std::optional<Target> ArmorTrack::track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t,
                                       const cv::Mat &bgr_img) noexcept
{
    const auto dt   = rm_utils::delta_time(t, last_timestamp_);
    last_timestamp_ = t;

    if (state_ != "lost" && dt > 0.1)
    {
        state_           = "lost";
        detect_count_    = 0;
        temp_lost_count_ = 0;
        target_.reset();
    }

    if (!armors.empty())
    {
        const cv::Point2f              img_center(bgr_img.cols / 2.0f, bgr_img.rows / 2.0f);
        const std::vector<std::string> priority{"1", "2", "3", "4", "5", "sentry", "outpost", "base"};

        // 获取装甲板优先级分数（越小优先级越高）
        const auto rank = [&](const Armor &armor) {
            const auto it = std::find(priority.begin(), priority.end(), armor.number);
            return it == priority.end() ? 100 : static_cast<int>(std::distance(priority.begin(), it));
        };
        std::sort(armors.begin(), armors.end(), [&](const Armor &lhs, const Armor &rhs) {
            const int lhs_rank = rank(lhs);
            const int rhs_rank = rank(rhs);
            if (lhs_rank != rhs_rank) return lhs_rank < rhs_rank;
            return cv::norm(lhs.center - img_center) < cv::norm(rhs.center - img_center);
        });
    }

    // 状态机处理
    bool found = false;
    if (state_ == "lost")
    {
        if (!armors.empty() && armor_pose_.GetArmorPose(armors.front()))
            found = set_target(armors.front(), t);
    }
    else
    {
        // 非丢失状态下，更新已有目标
        found = update_target(armors, t);
    }

    state_machine(found);

    // 发散检测：目标参数异常时重置
    if (state_ != "lost" && target_.has_value() && target_->diverged())
    {
        MAS_LOG_INFO("Target diverged!");
        state_        = "lost";
        target_.reset();
    }

    plotHandeye(armors);

    track_debug_.draw(bgr_img, armors, *this);
    pose_debug_.draw(bgr_img, armors, armor_pose_);

    if ((state_ == "tracking" || state_ == "temp_lost") && target_.has_value())
        return target_;
    return std::nullopt;
}

void ArmorTrack::state_machine(bool found)
{
    if (state_ == "lost")
    {
        // 丢失状态：找到目标后进入检测状态
        if (!found) return;
        state_ = "detecting";
        detect_count_ = 1;
    }
    else if (state_ == "detecting")
    {
        // 检测状态：连续检测到足够次数后进入跟踪状态
        if (found)
        {
            detect_count_++;
            if (detect_count_ >= min_detect_count_) state_ = "tracking";
        }
        else
        {
            // 检测失败，回到丢失状态
            detect_count_ = 0;
            state_        = "lost";
            target_.reset();
        }
    }
    else if (state_ == "tracking")
    {
        // 跟踪状态：丢失目标后进入临时丢失状态
        if (found) return;
        temp_lost_count_ = 1;
        state_           = "temp_lost";
    }
    else if (state_ == "temp_lost")
    {
        // 临时丢失状态：重新找到目标则恢复跟踪，否则计数
        if (found)
        {
            state_ = "tracking";
            temp_lost_count_ = 0;
        }
        else
        {
            temp_lost_count_++;
            // 前哨站允许更长的丢失时间
            int max_lost = (target_.has_value() && target_->name == "outpost") ? outpost_max_temp_lost_count_ : max_temp_lost_count_;
            if (temp_lost_count_ > max_lost)
            {
                state_ = "lost";
                target_.reset();
            }
        }
    }
}

bool ArmorTrack::set_target(const Armor &armor, std::chrono::steady_clock::time_point t)
{
    double          radius;    // 旋转半径
    int             armor_num; // 装甲板数量
    Eigen::VectorXd P0_dig;    // 初始协方差矩阵对角线

    if (armor.number == "outpost")
    {
        // 前哨站：3块装甲板，半径0.2765m
        radius    = 0.2765;
        armor_num = 3;
        P0_dig    = Eigen::VectorXd{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    }
    else if (armor.number == "base")
    {
        // 基地：3块装甲板，半径0.3205m
        radius    = 0.3205;
        armor_num = 3;
        P0_dig    = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    }
    else
    {
        // 普通车辆：4块装甲板，半径0.2m
        radius    = 0.2;
        armor_num = 4;
        P0_dig    = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    }

    // 使用 Target 构造函数创建目标
    target_ = Target(armor, t, radius, radius, 0.0, armor_num, P0_dig);
    return true;
}

bool ArmorTrack::update_target(std::vector<Armor> &armors,
                               std::chrono::steady_clock::time_point t)
{
    if (!target_.has_value())
        return false;

    // EKF预测
    target_->predict(t);

    if (std::none_of(armors.begin(), armors.end(), [&](const Armor &armor) { return armor.number == target_->name; }))
        return false;

    Target trial    = *target_;
    trial.is_switch = false;
    
    int successful_updates = 0;
    for (Armor &armor : armors)
    {
        if (armor.number != trial.name) continue;
        if (!armor_pose_.GetArmorPose(armor)) continue;

        if (trial.update(armor))
            ++successful_updates;
    }
    if (successful_updates == 0)
        return false;
    target_ = std::move(trial);
    return true;
}

void ArmorTrack::plotHandeye(const std::vector<Armor> &armors)
{
    if (!plotter_enable_)
        return;

    nlohmann::json data;
    // 手眼标定只记录原始角点和同帧云台姿态，不使用当前相机外参或 EKF 结果。
    // 字段每帧固定输出，避免 plot_monitor 按字段集合拆成多个 CSV。
    const bool handeye_valid =
        armors.size() == 1 && armors.front().points.size() == 4;
    data["handeye"]["valid"]     = handeye_valid;
    data["handeye"]["armor_big"] = false; // 固定小装甲板，保留接收端字段兼容。
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2f point    = handeye_valid ? armors.front().points[i] : cv::Point2f{};
        const std::string name     = "p" + std::to_string(i);
        data["handeye"][name]["x"] = point.x;
        data["handeye"][name]["y"] = point.y;
    }

    const Eigen::Matrix3d R_gimbal2world = armor_pose_.Get_R_gimbal2world();
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            data["handeye"]["R_gimbal2world"]["r" + std::to_string(row) + std::to_string(col)] =
                R_gimbal2world(row, col);

    plotter_.plot(data);
}

} // namespace auto_aim
