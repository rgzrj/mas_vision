#include "armor_pose.hpp"

#include <cmath>
#include <exception>
#include <fstream>
#include <limits>

#include "armor_types.hpp"
#include "mas_log.hpp"
#include "math_tools.hpp"
#include "yaml-cpp/yaml.h"

namespace auto_aim
{
ArmorPose::ArmorPose(const std::string &config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
    // yaml 读取参数
    try
    {
        // 检查文件是否存在
        std::ifstream file(config_path);
        if (!file.good())
        {
            MAS_LOG_ERROR("Config file not found: {}, using default values", config_path.c_str());
            return;
        }

        YAML::Node config = YAML::LoadFile(config_path);
        // 读取参数
        auto R_gimbal2imubody_data = config["handeye_calibration"]["R_gimbal2imubody"].as<std::vector<double>>();
        auto R_camera2gimbal_data  = config["handeye_calibration"]["R_camera2gimbal"].as<std::vector<double>>();
        auto t_camera2gimbal_data  = config["handeye_calibration"]["t_camera2gimbal"].as<std::vector<double>>();
        R_gimbal2imubody_          = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
        R_camera2gimbal_           = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
        t_camera2gimbal_           = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

        auto                                         camera_matrix_data  = config["calibration"]["camera_matrix"].as<std::vector<double>>();
        auto                                         distort_coeffs_data = config["calibration"]["distort_coeffs"].as<std::vector<double>>();
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
        Eigen::Matrix<double, 1, 5>                  distort_coeffs(distort_coeffs_data.data());
        cv::eigen2cv(camera_matrix, camera_matrix_);
        cv::eigen2cv(distort_coeffs, distort_coeffs_);

        MAS_LOG_INFO("armor_pose yaml loaded successfully");
    }
    catch (const std::exception &e)
    {
        MAS_LOG_WARN("Failed to load config, using defaults: {}", e.what());
    }
}

ArmorPose::~ArmorPose() = default;


void ArmorPose::set_R_gimbal2world(const Eigen::Quaterniond &q)
{
    Eigen::Matrix3d R_imubody2world = q.toRotationMatrix();
    R_gimbal2world_                 = R_gimbal2imubody_.transpose() * R_imubody2world * R_gimbal2imubody_;
}

bool ArmorPose::GetArmorPose(Armor &armor) const
{
    const auto &object_points = SMALL_ARMOR_POINTS;

    // 使用 SOLVEPNP求解
    cv::Vec3d rvec, tvec;
    if (!cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE) ||
        !std::isfinite(tvec[0]) || !std::isfinite(tvec[1]) || !std::isfinite(tvec[2]) || tvec[2] <= 0.0)
    {
        return false;
    }

    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    armor.xyz_in_world  = R_gimbal2world_ * armor.xyz_in_gimbal;

    if (!armor.xyz_in_gimbal.allFinite() || !armor.xyz_in_world.allFinite()) return false;

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);

    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world  = R_gimbal2world_ * R_armor2gimbal;
    armor.ypr_in_gimbal            = rm_utils::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world             = rm_utils::eulers(R_armor2world, 2, 1, 0);
    armor.ypd_in_world             = rm_utils::xyz2ypd(armor.xyz_in_world);

    if (!armor.ypr_in_gimbal.allFinite() || !armor.ypr_in_world.allFinite() || !armor.ypd_in_world.allFinite()) return false;

    optimize_yaw(armor);
    return armor.ypr_in_world.allFinite();
}

void ArmorPose::optimize_yaw(Armor &armor) const
{
    std::vector<cv::Point2f> normalized_points;
    try
    {
        cv::undistortPoints(armor.points, normalized_points, camera_matrix_, distort_coeffs_);
    }
    catch (const cv::Exception &)
    {
        return;
    }
    if (normalized_points.size() != armor.points.size()) return;

    const double center = rm_utils::eulers(R_gimbal2world_, 2, 1, 0)[0];    //yaw 搜索区间的中心角
    const double pitch = armor.number == "outpost" ? -15.0 * CV_PI / 180.0
                                                    : 15.0 * CV_PI / 180.0;
    const auto rotation = [pitch](double yaw) {
        const double sin_yaw = std::sin(yaw);
        const double cos_yaw = std::cos(yaw);
        const double sin_pitch = std::sin(pitch);
        const double cos_pitch = std::cos(pitch);
        return Eigen::Matrix3d{
            {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
            {sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch},
            {-sin_pitch, 0.0, cos_pitch}};
    };
    double best_yaw = armor.ypr_in_world[0];
    double best_rms = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_t = Eigen::Vector3d::Zero();
    // 搜索函数，搜索给定范围内的yaw角度，找到最优解(角度制)
    const auto search = [&](double begin_deg, double end_deg, double step_deg) {
        for (double offset = begin_deg; offset <= end_deg; offset += step_deg)
        {
            const double yaw = rm_utils::limit_rad(center + offset * CV_PI / 180.0);
            const Eigen::Matrix3d R_armor2world = rotation(yaw);
            Eigen::Vector3d t_camera;
            double rms = 0.0;
            if (solve_translation(R_armor2world, normalized_points, t_camera, rms) &&
                rms < best_rms)
            {
                best_rms = rms;
                best_yaw = yaw;
                best_t   = t_camera;
            }
        }
    };

    search(-70.0, 70.0, 5.0);
    if (!std::isfinite(best_rms)) return;
    const double best_offset = rm_utils::limit_rad(best_yaw - center) * 180.0 / CV_PI;
    search(best_offset - 5.0, best_offset + 5.0, 0.5);

    const Eigen::Matrix3d R_armor2world = rotation(best_yaw);
    armor.xyz_in_gimbal = R_camera2gimbal_ * best_t + t_camera2gimbal_;
    armor.xyz_in_world  = R_gimbal2world_ * armor.xyz_in_gimbal;
    armor.ypr_in_world  = rm_utils::eulers(R_armor2world, 2, 1, 0);
    armor.ypr_in_gimbal = rm_utils::eulers(R_gimbal2world_.transpose() * R_armor2world, 2, 1, 0);
    armor.ypd_in_world  = rm_utils::xyz2ypd(armor.xyz_in_world);
}

bool ArmorPose::solve_translation(const Eigen::Matrix3d &R_armor2world,
                                  const std::vector<cv::Point2f> &normalized_points,
                                  Eigen::Vector3d &t_camera, double &rms) const
{
    const auto &object_points = SMALL_ARMOR_POINTS;
    if (normalized_points.size() != object_points.size()) return false;

    const Eigen::Matrix3d R_armor2camera =
        R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
    // 构建线性方程组 Ax = b，x为相机坐标系下的平移向量 t_camera
    Eigen::Matrix<double, 8, 3> A;
    Eigen::Matrix<double, 8, 1> b;
    for (std::size_t i = 0; i < object_points.size(); ++i)
    {
        const Eigen::Vector3d point = R_armor2camera * Eigen::Vector3d(
            object_points[i].x, object_points[i].y, object_points[i].z);
        const double u = normalized_points[i].x;
        const double v = normalized_points[i].y;
        A.row(static_cast<Eigen::Index>(2 * i)) << 1.0, 0.0, -u;
        A.row(static_cast<Eigen::Index>(2 * i + 1)) << 0.0, 1.0, -v;
        b[static_cast<Eigen::Index>(2 * i)] = u * point.z() - point.x();
        b[static_cast<Eigen::Index>(2 * i + 1)] = v * point.z() - point.y();
    }

    const Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 8, 3>> qr(A);
    if (qr.rank() < 3) return false;
    t_camera = qr.solve(b);
    if (!t_camera.allFinite()) return false;

    double squared_error = 0.0;
    for (std::size_t i = 0; i < object_points.size(); ++i)
    {
        const Eigen::Vector3d point = R_armor2camera * Eigen::Vector3d(
            object_points[i].x, object_points[i].y, object_points[i].z) + t_camera;
        if (!point.allFinite() || point.z() <= 1e-4) return false;
        const double du = point.x() / point.z() - normalized_points[i].x;
        const double dv = point.y() / point.z() - normalized_points[i].y;
        squared_error += du * du + dv * dv;
    }
    const double focal = 0.5 * (camera_matrix_.at<double>(0, 0) + camera_matrix_.at<double>(1, 1));
    rms = std::sqrt(squared_error / object_points.size()) * focal;
    return std::isfinite(rms);
}

std::vector<cv::Point2f> ArmorPose::reproject_armor(const Eigen::Vector3d &xyz_in_world, double yaw, std::string name) const
{
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto pitch     = (name == "outpost") ? -15.0 * CV_PI / 180.0 
                                            : 15.0 * CV_PI / 180.0;
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // clang-format off
    const Eigen::Matrix3d R_armor2world {
        {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
        {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
        {         -sin_pitch,        0,           cos_pitch}
    };
    // clang-format on

    // get R_armor2camera t_armor2camera
    const Eigen::Vector3d &t_armor2world  = xyz_in_world;
    Eigen::Matrix3d        R_armor2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
    Eigen::Vector3d        t_armor2camera = R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    // get rvec tvec
    cv::Vec3d rvec;
    cv::Mat   R_armor2camera_cv;
    cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, rvec);
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    // reproject
    std::vector<cv::Point2f> image_points;
    const auto              &object_points = SMALL_ARMOR_POINTS;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
    return image_points;
}

std::vector<cv::Point2f> ArmorPose::world2pixel(const std::vector<cv::Point3f> &worldPoints) const
{
    Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
    Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

    cv::Mat rvec;
    cv::Mat tvec;
    cv::eigen2cv(R_world2camera, rvec);
    cv::eigen2cv(t_world2camera, tvec);

    std::vector<cv::Point3f> valid_world_points;
    for (const auto &world_point : worldPoints)
    {
        Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
        Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

        if (camera_point.z() > 0)
        {
            valid_world_points.push_back(world_point);
        }
    }
    // 如果没有有效点，返回空vector
    if (valid_world_points.empty())
    {
        return std::vector<cv::Point2f>();
    }
    std::vector<cv::Point2f> pixelPoints;
    cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
    return pixelPoints;
}

} // namespace auto_aim
