#include "hardware/serial/user_serial.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "rm_utils/io/media_recorder.hpp"
#include "rm_utils/io/runtime_path.hpp"
#include "mas_log.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
#include <chrono>
#include <atomic>
#include <cmath>
#include <csignal>
#include <thread>

std::atomic<bool> g_shutdown{false};
namespace
{
volatile std::sig_atomic_t interrupted = 0;
void stop_signal(int)
{
    interrupted = 1;
}
} // namespace

int main(int argc, char *argv[])
{
    try
    {
        using tool::utf8;
        const auto argument = argc > 1 ? std::string(argv[1]) : "--help";
        if (argument == "--help")
        {
            std::cout << "serial_test serial_config.yaml [seconds=10]\n"
                         "Receives and prints mode/quaternion; does not send aiming or fire commands.\n";
            return 0;
        }
        const double seconds = argc > 2 ? std::stod(std::string(argv[2])) : 10;
        if (!std::isfinite(seconds) || seconds < 0)
            throw std::runtime_error("Duration must be >= 0");
        const auto output = tool::create_session_directory(rm_utils::project_root() / "output/profiling");
        third_party::MasLog::init(tool::utf8(output / "serial_test.log"));
        std::cout << "Output: " << tool::utf8(output) << '\n';
        auto &serial = UserSerial::getInstance(tool::utf8(std::filesystem::absolute(tool::from_utf8(argv[1]))));
        if (!serial.isConnectedStatus())
            throw std::runtime_error("Cannot open serial device");
        interrupted = 0;
        std::signal(SIGINT, stop_signal);
        const auto start = std::chrono::steady_clock::now();
        auto next_print  = start;
        while (
            !interrupted &&
            (seconds == 0 || std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds))
        {
            serial.receiveSerial();
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_print)
            {
                std::cout << "mode=" << int(serial.getVisionMode()) << " age_ms=" << serial.quaternionAgeMs();
                if (serial.isQuaternionValid())
                {
                    const auto q = serial.q(now);
                    std::cout << " q_wxyz=" << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z();
                }
                std::cout << '\n';
                next_print = now + std::chrono::milliseconds(100);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        g_shutdown          = true;
        const bool received = serial.hasReceivedData();
        serial.closeSerial();
        if (!received)
            throw std::runtime_error("No valid pose packets received");
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "serial_test: " << error.what() << '\n';
        return 1;
    }
}
