#include "calibrate_handeye.hpp"
#include "rm_utils/display/display.hpp"
#include <yaml-cpp/yaml.h>
#include "hardware/hikcamera/hikcamera.hpp"
#include "hardware/serial/user_serial.hpp"
#include "mas_log.hpp"
#include "math_tools.hpp"
#include <atomic>
#include <thread>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
extern std::atomic<bool> g_shutdown;
namespace calibration
{
constexpr const char *CONFIG_PATH = "config/hikcamera.yaml";
// 生成标定板3D点
std::vector<cv::Point3f> centers_3d(const cv::Size &pattern_size, float center_distance_mm)
{
    std::vector<cv::Point3f> centers_3d;

    for (int i = 0; i < pattern_size.height; i++)
    {
        for (int j = 0; j < pattern_size.width; j++)
        {
            float x = 0;
            float y = (-j + 0.5 * pattern_size.width) * center_distance_mm;
            float z = (-i + 0.5 * pattern_size.height) * center_distance_mm;
            centers_3d.push_back({x, y, z});
        }
    }

    return centers_3d;
}

int calibrate_handeye_main()
{
    MAS_LOG_INFO("HandEye Calibration Start");
    const auto config = YAML::LoadFile(CONFIG_PATH);
    const cv::Size pattern_size(config["calibration"]["pattern_cols"].as<int>(),
                                config["calibration"]["pattern_rows"].as<int>());
    const float center_distance_mm = config["calibration"]["center_distance_mm"].as<float>();
    auto camera_values             = config["calibration"]["camera_matrix"].as<std::vector<double>>();
    auto distortion_values         = config["calibration"]["distort_coeffs"].as<std::vector<double>>();
    auto rotation_values           = config["handeye_calibration"]["R_gimbal2imubody"].as<std::vector<double>>();
    if (camera_values.size() != 9 || rotation_values.size() != 9)
        throw std::runtime_error("Calibration rotations and camera matrix must contain 9 values");
    const cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F, camera_values.data()).clone();
    const cv::Mat distort_coeffs(distortion_values);
    const Eigen::Matrix3d R_gimbal2imubody =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation_values.data());
    auto &user_serial = UserSerial::getInstance("config/serial_config.yaml");
    hardware::HikCamera camera(CONFIG_PATH);
    if (!camera.openCamera())
        throw std::runtime_error("Cannot open camera for hand-eye calibration");
    YAML::Node settings;
    settings["backend"] = "sdl";
    auto &display       = rm_utils::Display::getInstance();
    display.start(settings, CONFIG_PATH);
    // 标定只接收姿态；主线程是唯一姿态消费者，接收任务自动停止并join。
    std::jthread receiver(
        [&](std::stop_token stop)
        {
            while (!stop.stop_requested() && !g_shutdown)
            {
                user_serial.receiveSerial();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    bool save_requested = false, clear_requested = false, quit_requested = false;
    const auto key = [&](int value)
    {
        if (value == 's')
            save_requested = true;
        if (value == 'c')
            clear_requested = true;
        if (value == 'q' || value == 27)
            quit_requested = true;
    };
    std::vector<cv::Mat> rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list;
    while (!g_shutdown && !quit_requested)
    {
        display.poll(nullptr, key);
        user_serial.pollPose();
        if (g_shutdown || quit_requested)
            break;
        auto frame = camera.getImage();
        if (frame.frame.empty())
            continue;
        cv::Mat image                  = frame.frame.clone();
        const auto pose                = user_serial.awaitPose(frame.timestamp);
        Eigen::Matrix3d R_gimbal2world = Eigen::Matrix3d::Identity();
        if (pose)
            R_gimbal2world = R_gimbal2imubody.transpose() * pose->toRotationMatrix() * R_gimbal2imubody;
        const Eigen::Vector3d ypr = rm_utils::eulers(R_gimbal2world, 2, 1, 0) * 57.3;
        std::vector<cv::Point2f> centers_2d;
        const bool board_detected = cv::findCirclesGrid(image, pattern_size, centers_2d);
        if (clear_requested)
        {
            clear_requested = false;
            rvecs.clear();
            tvecs.clear();
            R_world2gimbal_list.clear();
            t_world2gimbal_list.clear();
            MAS_LOG_INFO("Cleared all calibration data");
        }
        if (save_requested)
        {
            save_requested = false;
            cv::Mat rvec, tvec;
            if (board_detected && pose &&
                cv::solvePnP(centers_3d(pattern_size, center_distance_mm), centers_2d, camera_matrix, distort_coeffs,
                             rvec, tvec, false, cv::SOLVEPNP_IPPE))
            {
                cv::Mat rotation;
                cv::eigen2cv(Eigen::Matrix3d(R_gimbal2world.transpose()), rotation);
                rvecs.push_back(rvec);
                tvecs.push_back(tvec);
                R_world2gimbal_list.push_back(rotation);
                t_world2gimbal_list.push_back(cv::Mat::zeros(3, 1, CV_64F));
                MAS_LOG_INFO("Saved hand-eye sample {}", rvecs.size());
            }
            else
                MAS_LOG_WARN("Cannot save: board, paired pose or PnP unavailable");
        }
        if (board_detected)
            cv::drawChessboardCorners(image, pattern_size, centers_2d, true);
        display.set_time(std::chrono::duration<double>(frame.timestamp.time_since_epoch()).count());
        display.display_add(
            "HandEye Calibration", image,
            {{fmt::format("Data count: {}", rvecs.size()), 20, 40, 28, {0, 255, 0, 255}, 1, true},
             {pose ? fmt::format("YPR: {:.1f}, {:.1f}, {:.1f}", ypr[0], ypr[1], ypr[2]) : "Pose: unavailable",
              20,
              80,
              24,
              {255, 220, 0, 255},
              1,
              true},
             {board_detected ? "Board detected" : "No board detected", 20, 120, 24, {0, 255, 0, 255}, 1, true},
             {"s:save  c:clear  q/ESC:finish", 20, 160, 24, {0, 255, 0, 255}, 1, true}});
        display.end_frame();
        display.poll(nullptr, key);
    }
    receiver.request_stop();
    receiver.join();
    camera.closeCamera();
    user_serial.closeSerial();

    display.shutdown();

    // 执行手眼标定
    if (rvecs.size() < 3)
    {
        MAS_LOG_ERROR("Calibration failed, need at least 3 data, got {}", rvecs.size());
        return -1;
    }

    cv::Mat R_gimbal2camera, t_gimbal2camera;
    cv::Mat R_world2board, t_world2board;

    cv::calibrateRobotWorldHandEye(rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, R_world2board, t_world2board,
                                   R_gimbal2camera, t_gimbal2camera);

    t_gimbal2camera /= 1e3; // mm to m
    t_world2board /= 1e3;   // mm to m

    // 计算R_camera2gimbal和t_camera2gimbal
    cv::Mat R_camera2gimbal, t_camera2gimbal;
    cv::Mat R_board2world, t_board2world;
    cv::transpose(R_gimbal2camera, R_camera2gimbal);
    cv::transpose(R_world2board, R_board2world);
    t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera;
    t_board2world   = -R_board2world * t_world2board;

    // 计算相机同理想情况的偏角
    Eigen::Matrix3d R_camera2gimbal_eigen;
    cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
    Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
    Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
    Eigen::Vector3d camera_ypr     = rm_utils::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

    // 计算标定板到世界坐标系原点的水平距离
    auto x        = t_board2world.at<double>(0);
    auto y        = t_board2world.at<double>(1);
    auto distance = std::sqrt(x * x + y * y);

    // 计算标定板同竖直摆放时的偏角
    Eigen::Matrix3d R_board2world_eigen;
    cv::cv2eigen(R_board2world, R_board2world_eigen);
    Eigen::Vector3d board_ypr = rm_utils::eulers(R_board2world_eigen, 2, 1, 0) * 57.3;

    // 保存结果
    YAML::Node config_out = YAML::LoadFile(CONFIG_PATH);

    std::vector<double> R_camera2gimbal_data(R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
    std::vector<double> t_camera2gimbal_data(t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

    config_out["handeye_calibration"]["R_camera2gimbal"] = R_camera2gimbal_data;
    config_out["handeye_calibration"]["t_camera2gimbal"] = t_camera2gimbal_data;
    config_out["handeye_calibration"]["R_camera2gimbal"].SetStyle(YAML::EmitterStyle::Flow);
    config_out["handeye_calibration"]["t_camera2gimbal"].SetStyle(YAML::EmitterStyle::Flow);

    std::ofstream fout(CONFIG_PATH);
    fout << config_out;
    fout.close();
    if (!fout)
        throw std::runtime_error("Cannot save hand-eye calibration");

    MAS_LOG_INFO("Saved calibration result to {}", CONFIG_PATH);
    MAS_LOG_INFO("相机同理想情况的偏角: yaw={:.2f} pitch={:.2f} roll={:.2f} degree", camera_ypr[0], camera_ypr[1],
                 camera_ypr[2]);
    MAS_LOG_INFO("标定板到世界坐标系原点的水平距离: {:.2f} m", distance);
    MAS_LOG_INFO("标定板同竖直摆放时的偏角: yaw={:.2f} pitch={:.2f} roll={:.2f} degree", board_ypr[0], board_ypr[1],
                 board_ypr[2]);

    return 0;
}

} // namespace calibration
