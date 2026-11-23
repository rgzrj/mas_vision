#include "calibrate_handeye.hpp"

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "hardware/hikcamera/hikcamera.hpp"
#include "rm_utils/display/display.hpp"
#include "mas_log.hpp"
#include "math_tools.hpp"
#include "hardware/serial/user_serial.hpp"

extern std::atomic<bool> g_shutdown;

namespace calibration
{

// YAML配置文件路径
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

// 按键与采集循环都在主线程执行，无需互斥锁。
struct CalibrationState
{
    bool save_requested  = false;
    bool clear_requested = false;
    bool quit_requested  = false;
};

int calibrate_handeye_main()
{
    MAS_LOG_INFO("HandEye Calibration Start");

    // 读取配置
    YAML::Node config = YAML::LoadFile(CONFIG_PATH);

    auto pattern_cols       = config["calibration"]["pattern_cols"].as<int>();
    auto pattern_rows       = config["calibration"]["pattern_rows"].as<int>();
    auto center_distance_mm = config["calibration"]["center_distance_mm"].as<float>();
    cv::Size pattern_size(pattern_cols, pattern_rows);

    auto camera_matrix_data  = config["calibration"]["camera_matrix"].as<std::vector<double>>();
    auto distort_coeffs_data = config["calibration"]["distort_coeffs"].as<std::vector<double>>();
    if (camera_matrix_data.size() != 9)
        throw std::runtime_error("Camera matrix must contain 9 values");

    cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();
    cv::Mat distort_coeffs(distort_coeffs_data);

    auto R_gimbal2imubody_data = config["handeye_calibration"]["R_gimbal2imubody"].as<std::vector<double>>();
    if (R_gimbal2imubody_data.size() != 9)
        throw std::runtime_error("R_gimbal2imubody must contain 9 values");

    Eigen::Matrix3d R_gimbal2imubody = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());

    // 初始化相机、串口和显示
    auto &user_serial = UserSerial::getInstance("config/serial_config.yaml");
    hardware::HikCamera camera(CONFIG_PATH);
    if (!camera.openCamera())
        throw std::runtime_error("Cannot open camera for hand-eye calibration");

    auto &display       = rm_utils::Display::getInstance();
    display.start("sdl", CONFIG_PATH);

    CalibrationState calib_state;
    const auto on_key = [&](int key)
    {
        if (key == 's')
            calib_state.save_requested = true;
        if (key == 'c')
            calib_state.clear_requested = true;
        if (key == 'q' || key == 27)
            calib_state.quit_requested = true;
    };

    // 标定只接收姿态，不发送视觉控制包。
    std::jthread receiver(
        [&](std::stop_token stop)
        {
            // receiveSerial 重连等待读取 g_shutdown，退出通知必须先于 join。
            std::stop_callback on_stop(stop, [] { g_shutdown = true; });

            while (!stop.stop_requested() && !g_shutdown)
            {
                user_serial.receiveSerial();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });

    // 标定数据存储
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;

    MAS_LOG_INFO("HandEye Calibration Started");

    while (!g_shutdown.load())
    {
        // 处理窗口事件，再获取图像与曝光时刻配对的姿态。
        display.poll(nullptr, on_key);
        user_serial.pollPose();
        if (g_shutdown || calib_state.quit_requested)
            break;

        auto frame = camera.getImage();
        if (frame.frame.empty())
            continue;

        cv::Mat image   = frame.frame.clone();
        const auto pose = user_serial.awaitPose(frame.timestamp);

        // 计算云台到世界的旋转
        Eigen::Matrix3d R_imubody2imuabs = Eigen::Matrix3d::Identity();
        if (pose)
            R_imubody2imuabs = pose->toRotationMatrix();

        Eigen::Matrix3d R_gimbal2world = R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
        Eigen::Vector3d ypr            = rm_utils::eulers(R_gimbal2world, 2, 1, 0) * 57.3;

        // 检测标定板
        std::vector<cv::Point2f> centers_2d;
        bool board_detected = cv::findCirclesGrid(image, pattern_size, centers_2d);

        // 处理保存请求
        const bool should_save     = calib_state.save_requested;
        calib_state.save_requested = false;

        if (should_save && board_detected && pose)
        {
            // PnP求解标定板位姿
            cv::Mat rvec, tvec;
            auto centers_3d_ = centers_3d(pattern_size, center_distance_mm);
            if (cv::solvePnP(centers_3d_, centers_2d, camera_matrix, distort_coeffs, rvec, tvec, false,
                             cv::SOLVEPNP_IPPE))
            {
                // 计算R_world2gimbal
                Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();
                cv::Mat R_world2gimbal_cv;
                cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);
                cv::Mat t_world2gimbal = (cv::Mat_<double>(3, 1) << 0, 0, 0);

                // 记录数据
                rvecs.emplace_back(rvec);
                tvecs.emplace_back(tvec);
                R_world2gimbal_list.emplace_back(R_world2gimbal_cv);
                t_world2gimbal_list.emplace_back(t_world2gimbal);
                MAS_LOG_INFO("Saved hand-eye sample {}", rvecs.size());
            }
            else
            {
                MAS_LOG_WARN("Cannot save: PnP failed");
            }
        }
        else if (should_save)
        {
            MAS_LOG_WARN("Cannot save: board or paired pose unavailable");
        }

        // 处理清空请求
        const bool should_clear     = calib_state.clear_requested;
        calib_state.clear_requested = false;

        if (should_clear)
        {
            rvecs.clear();
            tvecs.clear();
            R_world2gimbal_list.clear();
            t_world2gimbal_list.clear();
            MAS_LOG_INFO("Cleared all calibration data");
        }

        // 准备显示文本
        std::vector<rm_utils::DisplayText> texts;
        texts.push_back({fmt::format("Data count: {}", rvecs.size()), 20, 40, 28, {0, 255, 0, 255}, 1, true});
        if (pose)
        {
            const auto orientation = fmt::format("YPR: {:.1f}, {:.1f}, {:.1f}", ypr[0], ypr[1], ypr[2]);
            texts.push_back({orientation, 20, 80, 24, {0, 0, 255, 255}, 1, true});
        }
        else
        {
            texts.push_back({"Pose: unavailable", 20, 80, 24, {0, 0, 255, 255}, 1, true});
        }

        if (board_detected)
        {
            texts.push_back({"Board detected", 20, 120, 24, {0, 255, 0, 255}, 1, true});
        }
        else
        {
            texts.push_back({"No board detected", 20, 120, 24, {0, 0, 255, 255}, 1, true});
        }

        texts.push_back({"'s':save 'c':clear 'q':quit&calibrate", 20, 160, 24, {0, 255, 0, 255}, 1, true});

        // 在图像上绘制标定板角点
        if (board_detected)
        {
            cv::drawChessboardCorners(image, pattern_size, centers_2d, board_detected);
        }

        // 发送到显示
        display.set_time(std::chrono::duration<double>(frame.timestamp.time_since_epoch()).count());
        display.display_add("HandEye Calibration", image, texts, {}, {}, 720, 540);
        display.end_frame();
        display.poll(nullptr, on_key);
    }

    // 先停止接收任务，再关闭设备和显示。
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
