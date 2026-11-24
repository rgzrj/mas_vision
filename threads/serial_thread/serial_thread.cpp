#include "serial_thread.hpp"
#include "threads/vision_router/vision_command.hpp"
#include "mas_log.hpp"
#include "hardware/serial/user_serial.hpp"
#include <thread>

namespace threads
{
void serial_thread_func(VisionCommand &commands)
{
    MAS_LOG_INFO("Serial processing thread started");

    UserSerial &serial = UserSerial::getInstance();

    auto       last_send_time = std::chrono::steady_clock::now();
    const auto send_interval  = std::chrono::milliseconds(4);

    while (!g_shutdown)
    {
        // 先接收并同步模式版本，避免重连后发送旧指令。
        serial.receiveSerial();
        commands.update(serial.getVisionState());

        auto current_time = std::chrono::steady_clock::now();
        // 发送数据,固定250hz
        if (current_time - last_send_time >= send_interval)
        {
            serial.sendPacketToSerial(commands.packet(current_time, serial.isQuaternionValid()));
            last_send_time = current_time;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    serial.closeSerial();
    MAS_LOG_INFO("Serial processing thread exited");
}
} // namespace threads
