#include "hardware/serial/user_serial.hpp"
#include "hardware/serial/crc16.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "rm_utils/io/media_recorder.hpp"
#include "rm_utils/io/runtime_path.hpp"
#include "mas_log.hpp"
#include "tests/common/test.hpp"
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

TEST_MAIN
{
    return tests::run(
        "serial_test",
        [&]
        {
            static_assert(sizeof(SendPacket) == 10);
#ifndef SENTRY
            static_assert(sizeof(ReceivePacket) == 17);
#endif
            using tool::utf8;
            const auto argument = argc > 1 ? tool::utf8(tests::argument_path(argv[1])) : "--help";
            if (argument == "--self-test")
            {
                constexpr std::uint8_t bytes[]{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
                tests::require(hardware::checksum(bytes, sizeof(bytes)) == 0x6f91,
                               "Protocol CRC16 reference vector changed");
                std::cout << "Packet layout and CRC16 checked without opening a device\n";
                return;
            }
            if (argument == "--help")
            {
                std::cout << "serial_test serial_config.yaml [seconds=10]\nserial_test --self-test\n"
                             "Receives and prints mode/quaternion; does not send aiming or fire commands.\n";
                return;
            }
            const double seconds = argc > 2 ? std::stod(tool::utf8(tests::argument_path(argv[2]))) : 10;
            tests::require(std::isfinite(seconds) && seconds >= 0, "Duration must be >= 0");
            const auto output = tool::create_session_directory(rm_utils::project_root() / "output/profiling");
            third_party::MasLog::init(tool::utf8(output / "serial_test.log"));
            std::cout << "Output: " << tool::utf8(output) << '\n';
            auto &serial =
                UserSerial::getInstance(tool::utf8(std::filesystem::absolute(tests::argument_path(argv[1]))));
            tests::require(serial.isConnectedStatus(), "Cannot open serial device");
            interrupted = 0;
            std::signal(SIGINT, stop_signal);
            const auto start = std::chrono::steady_clock::now();
            auto next_print  = start;
            while (!interrupted &&
                   (seconds == 0 ||
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds))
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
            tests::require(received, "No valid pose packets received");
        });
}
