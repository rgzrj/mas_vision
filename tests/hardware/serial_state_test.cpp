#include "hardware/serial/user_serial.hpp"
#include "hardware/serial/crc16.hpp"
#include <cstring>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <thread>
#include <numbers>
#include <stdexcept>
std::atomic<bool> g_shutdown{false};
static void require(bool ok, const char *message)
{
    if (!ok)
        throw std::runtime_error(message);
}
static void feed(UserSerial &serial, std::uint8_t mode,
                 const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity())
{
    ReceivePacket packet{};
    packet.mode = mode;
    packet.q[0] = orientation.w();
    packet.q[1] = orientation.x();
    packet.q[2] = orientation.y();
    packet.q[3] = orientation.z();
    std::vector<std::uint8_t> bytes(1 + sizeof(packet) + 2);
    bytes[0] = 0x5a;
    std::memcpy(bytes.data() + 1, &packet, sizeof(packet));
    const auto crc          = hardware::checksum(bytes.data(), bytes.size() - 2);
    bytes[bytes.size() - 2] = std::uint8_t(crc);
    bytes.back()            = std::uint8_t(crc >> 8);
    serial::Serial::incoming.insert(serial::Serial::incoming.end(), bytes.begin(), bytes.end());
    serial.receiveSerial();
}
int main()
{
    try
    {
        const auto config = std::filesystem::temp_directory_path() / "mas_vision_serial_state_test.yaml";
        std::ofstream(config) << "serial: {port: mock, data_timeout_ms: 1000}\n";
        auto &serial = UserSerial::getInstance(config.string());
        // 通过生产接收解析与公开姿态查询验证插值，不访问内部容器。
        using namespace std::chrono_literals;
        const auto before_first = std::chrono::steady_clock::now();
        feed(serial, 0);
        const auto left_time = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(10ms);
        const auto right_time = std::chrono::steady_clock::now();
        const Eigen::Quaterniond rotated(Eigen::AngleAxisd(std::numbers::pi / 2, Eigen::Vector3d::UnitZ()));
        feed(serial, 0, rotated);
        const auto after_second = std::chrono::steady_clock::now();
        require(!serial.poseAt(before_first - 1ms) && !serial.poseAt(after_second + 1ms),
                "Pose query silently clamped outside history");
        const auto left = serial.poseAt(left_time);
        const auto right = serial.poseAt(right_time);
        const auto midpoint = serial.poseAt(left_time + (right_time - left_time) / 2);
        require(left && right && midpoint && midpoint->angularDistance(left->slerp(.5, *right)) < 1e-6,
                "UserSerial pose interpolation changed");
        require(serial.q(before_first - 1ms).angularDistance(rotated) < 1e-6,
                "Legacy q() no longer falls back to the latest pose");
        std::this_thread::sleep_for(1100ms);
        feed(serial, 0);
        require(!serial.poseAt(after_second + 500ms), "Pose interpolation bridged an excessive sample gap");

        for (int i = 0; i < 5100; ++i)
            feed(serial, 0);
        feed(serial, 4);
        require(serial.getVisionMode() == BIG_RUNE_RED && serial.isQuaternionValid(),
                "Full pose queue blocked mode or freshness update");
        const auto state = serial.getVisionState();
        serial.sendPacketToSerial({true, .1f, .2f, 1});
        require(serial::Serial::sent.size() == 13 && serial::Serial::sent[1] == 1, "13-byte found packet changed");
        const auto crc = hardware::checksum(serial::Serial::sent.data(), serial::Serial::sent.size() - 2);
        require(serial::Serial::sent[11] == std::uint8_t(crc) && serial::Serial::sent[12] == std::uint8_t(crc >> 8),
                "Send packet checksum changed");
        serial.sendPacketToSerial({false, .1f, .2f, 0});
        require(serial::Serial::sent[1] == 0 && serial::Serial::sent[10] == 0,
                "Inactive packet flags changed");

        feed(serial, 4);
        require(serial.getVisionState().epoch == state.epoch, "Repeated mode changed its epoch");
        feed(serial, 2);
        feed(serial, 4);
        const auto switched = serial.getVisionState();
        require(switched.mode == state.mode && switched.epoch == state.epoch + 2,
                "A->B->A lost a mode transition");
        serial.pollPose();
        require(!serial.poseAt(left_time), "Bounded pose history retained samples beyond its capacity");
        serial.reconnect();
        const auto reconnected = serial.getVisionState();
        require(reconnected.mode == switched.mode && reconnected.epoch == switched.epoch + 1,
                "Reconnect did not invalidate the previous mode version");
        feed(serial, 2);
        require(!serial.poseAt(right_time), "Reconnect retained a pose from the previous session");
        feed(serial, 3);
        require(serial.getVisionMode() == SMALL_RUNE_BLUE, "Pose consumer failed to recover after a full queue");
        serial.closeSerial();
        std::filesystem::remove(config);
        std::cout << "Production serial parser: full queue, mode versions, reconnect and packet encoding passed\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
