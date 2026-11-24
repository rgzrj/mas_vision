#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <opencv2/core.hpp>
#include <thread>

#include "BS_thread_pool.hpp"
#include "mas_log.hpp"

#include "rm_utils/display/display.hpp"
#include "apps/calibration/calibrate_camera.hpp"
#include "apps/calibration/calibrate_handeye.hpp"
#include "rm_utils/io/runtime_path.hpp"
#include "rm_utils/io/media_recorder.hpp"
#include "threads/hikcamera_thread/hikcamera_thread.hpp"
#include "threads/serial_thread/serial_thread.hpp"
#include "threads/vision_router/vision_router.hpp"
#include "threads/vision_router/vision_command.hpp"
#include "threads/armor_thread/armor_thread.hpp"
#include "threads/buff_thread/buff_thread.hpp"

// 全局原子变量用于控制所有线程的生命周期
std::atomic<bool> g_shutdown{false};

// 信号处理函数
void signal_handler(int) { g_shutdown = true; }


int main(int argc, char **argv)
{
    // 信号、日志和标定命令。
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try
    {
        std::filesystem::current_path(rm_utils::project_root());
        if (argc > 1)
        {
            third_party::MasLog::init("logs/mas_vision.log");
            const std::string command = argv[1];
            if (command == "calibrate_camera")
            {
                return calibrate_camera_main();
            }
            else if (command == "calibrate_handeye")
            {
                return calibration::calibrate_handeye_main();
            }
            else
            {
                if (command != "--help")
                    fmt::print("未知命令: {}\n", command);
                fmt::print("相机校准 : ./mas_vision calibrate_camera\n"
                           "手眼标定 : ./mas_vision calibrate_handeye\n"
                           "录制 : ./mas_vision_record\n"
                           "回放 : ./mas_vision_replay\n");
                return command == "--help" ? 0 : 1;
            }
        }
        const auto output = tool::create_session_directory("output/live");
        third_party::MasLog::init(tool::utf8(output / "app.log"));
        MAS_LOG_INFO("Mas Vision System Start");

        // OpenCV进程内统一单线程；OpenVINO推理线程仍由配置单独控制。
        cv::setNumThreads(1);

        // 显示模块。
        const std::filesystem::path aim_config_path  = "config/auto_aim.yaml";
        const std::filesystem::path buff_config_path = "config/auto_buff.yaml";

        auto &display     = rm_utils::Display::getInstance();
        display.start(buff_config_path, output);

        // 队列与线程池。相机只有router一个消费者，算法各自消费工作队列。
        size_t camera_buffer_size = 2;
        auto camera_buffer        = std::make_shared<rigtorp::SPSCQueue<CameraFrame>>(camera_buffer_size);
        threads::VisionQueue armor_queue(2), buff_queue(2);
        threads::VisionCommand commands;
        BS::thread_pool pool(5);

        try
        {
            // 任务异常先通知所有线程退出，再交给 future 传回主线程。
            const auto submit_task = [&pool](auto task)
            {
                return pool.submit_task([task]
                {
                    try
                    {
                        task();
                    }
                    catch (...)
                    {
                        g_shutdown = true;
                        throw;
                    }
                });
            };
            // 五个任务只启动一次，切模式由router处理。
            auto camera_future = submit_task(
                [&] { threads::hikcamera_thread_func("config/hikcamera.yaml", camera_buffer_size, camera_buffer); });
            auto serial_future = submit_task([&] { threads::serial_thread_func(commands); });
            auto router_future = submit_task(
                [&] { threads::vision_router_func(commands, *camera_buffer, armor_queue, buff_queue); });
            auto armor_future  = submit_task([&] { threads::armor_thread_func(commands, armor_queue, aim_config_path); });
            auto buff_future   = submit_task([&] { threads::buff_thread_func(commands, buff_queue, buff_config_path); });

            MAS_LOG_INFO("Thread pool created with {} threads; output: {}", pool.get_thread_count(),
                         tool::utf8(output));
            // 主线程处理SDL事件，收到退出通知后读取任务结果。
            while (!g_shutdown)
            {
                display.poll();
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            // 6. 等待五个任务退出，再关闭显示。
            camera_future.get();
            serial_future.get();
            router_future.get();
            armor_future.get();
            buff_future.get();
        }
        catch (...)
        {
            // 投递任务或SDL处理失败时，先通知退出，再让线程池析构等待。
            g_shutdown = true;
            throw;
        }
        display.shutdown();
        MAS_LOG_INFO("Mas Vision System Stopped");
        return 0;
    }
    catch (const std::exception &e)
    {
        g_shutdown = true;
        rm_utils::Display::getInstance().shutdown();
        MAS_LOG_ERROR("Vision stopped: {}", e.what());
        return 1;
    }
}
