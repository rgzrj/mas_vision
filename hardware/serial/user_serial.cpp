#include "user_serial.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <fstream>
#include <iterator>
#include <stdio.h>
#include <string.h>

#include "crc16.hpp"
#include "mas_log.hpp"
#include "serial_types.hpp"
#include <thread>
#include <yaml-cpp/yaml.h>

extern std::atomic<bool> g_shutdown;

constexpr uint8_t RX_HEADER = 0x5A;
constexpr uint8_t TX_HEADER = 0xA5;
constexpr size_t  RX_FRAME_SIZE = 1 + sizeof(ReceivePacket) + 2;
constexpr size_t  TX_FRAME_SIZE = 1 + sizeof(SendPacket) + 2;

UserSerial::UserSerial(const std::string &config_path)
{
    try
    {
        // 检查文件是否存在
        std::ifstream file(config_path);
        if (!file.good())
        {
            MAS_LOG_WARN("Config file not found: {}, using default values", config_path.c_str());
        }

        // 加载YAML文件
        YAML::Node config = YAML::LoadFile(config_path);

        if (!config["serial"])
        {
            MAS_LOG_WARN("No 'serial' section in config file");
        }

        if (config["serial"])
        {
            if (config["serial"]["debug"]) debug_ = config["serial"]["debug"].as<bool>(false);
            if (config["serial"]["port"]) port_ = config["serial"]["port"].as<std::string>();
            if (config["serial"]["baudrate"]) baudrate_ = config["serial"]["baudrate"].as<uint32_t>(115200);
            if (config["serial"]["timeout"]) timeout_ = config["serial"]["timeout"].as<int>(2);
            if (config["serial"]["data_timeout_ms"]) data_timeout_ms_ = config["serial"]["data_timeout_ms"].as<int>(100);
            if (config["serial"]["bytesize"])
            {
                int bs = config["serial"]["bytesize"].as<int>(8);
                switch (bs)
                {
                case 5:
                    bytesize_ = serial::fivebits;
                    break;
                case 6:
                    bytesize_ = serial::sixbits;
                    break;
                case 7:
                    bytesize_ = serial::sevenbits;
                    break;
                case 8:
                    bytesize_ = serial::eightbits;
                    break;
                default:
                    bytesize_ = serial::eightbits;
                    break;
                }
            }
            if (config["serial"]["parity"])
            {
                std::string p = config["serial"]["parity"].as<std::string>("none");
                if (p == "none")
                    parity_ = serial::parity_none;
                else if (p == "odd")
                    parity_ = serial::parity_odd;
                else if (p == "even")
                    parity_ = serial::parity_even;
                else
                    parity_ = serial::parity_none;
            }
            if (config["serial"]["stopbits"])
            {
                int sb = config["serial"]["stopbits"].as<int>(1);
                if (sb == 1)
                    stopbits_ = serial::stopbits_one;
                else if (sb == 2)
                    stopbits_ = serial::stopbits_two;
                else
                    stopbits_ = serial::stopbits_one;
            }
            if (config["serial"]["flowcontrol"])
            {
                std::string fc = config["serial"]["flowcontrol"].as<std::string>("none");
                if (fc == "none")
                    flowcontrol_ = serial::flowcontrol_none;
                else if (fc == "software")
                    flowcontrol_ = serial::flowcontrol_software;
                else if (fc == "hardware")
                    flowcontrol_ = serial::flowcontrol_hardware;
                else
                    flowcontrol_ = serial::flowcontrol_none;
            }
        }

        MAS_LOG_INFO("Config loaded successfully from: {}", config_path.c_str());
    }
    catch (const YAML::Exception &e)
    {
        MAS_LOG_ERROR("YAML parsing error: {}", e.what());
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Error loading config: {}", e.what());
    }

    recv_buffer_.reserve(1024);
    temp_read_buf_.resize(256);
    has_received_data_.store(false);
    // 初始化滑动窗口
    // 初始时间戳设为 epoch，这样第一次循环一定会尝试去取数据

    try
    {
        serial_port =
            std::make_unique<serial::Serial>(port_, baudrate_, serial::Timeout::simpleTimeout(timeout_), bytesize_, parity_, stopbits_, flowcontrol_);
        if (serial_port && serial_port->isOpen())
        {
            isConnected = true;
        }
        else
        {
            isConnected = false;
            return;
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception while opening serial port: {}", e.what());
        isConnected = false;
        return;
    }
}

UserSerial::~UserSerial() {}

void UserSerial::closeSerial()
{
    if (serial_port)
    {
        if (serial_port->isOpen())
        {
            serial_port->close();
        }
        serial_port.reset();
    }
    isConnected = false;
    MAS_LOG_INFO("Serial port closed");
}

void UserSerial::sendVision(float yaw, float pitch, uint8_t found, uint8_t fire)
{
    submitVision(getVisionState(), std::chrono::steady_clock::now(), std::chrono::milliseconds(100),
                 {bool(found), yaw, pitch, fire});
}

bool UserSerial::submitVision(VisionState state, std::chrono::steady_clock::time_point exposure,
                              std::chrono::steady_clock::duration max_age, SendPacket packet)
{
    if (!isQuaternionValid())
        packet.found = false;
    return commands_.submit(state, exposure, max_age, packet);
}

void UserSerial::disableVision()
{
    commands_.clear();
}

void UserSerial::sendPacketToSerial()
{
    while (!g_shutdown && (!isConnected || !serial_port || !serial_port->isOpen()))
    {
        reconnect();
        if (!isConnected || !serial_port || !serial_port->isOpen())
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (g_shutdown)
        return;

    try
    {
        if (!isQuaternionValid())
            commands_.clear();
        const auto packet = commands_.packet(std::chrono::steady_clock::now());

        std::array<uint8_t, TX_FRAME_SIZE> frame{};
        frame[0] = TX_HEADER;
        std::memcpy(frame.data() + 1, &packet, sizeof(packet));
        const uint16_t crc       = hardware::checksum(frame.data(), TX_FRAME_SIZE - 2);
        frame[TX_FRAME_SIZE - 2] = static_cast<uint8_t>(crc & 0xFF);
        frame[TX_FRAME_SIZE - 1] = static_cast<uint8_t>(crc >> 8);
        serial_port->write(frame.data(), frame.size());
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception while sending data: {}", e.what());
        isConnected = false;
    }
}

void UserSerial::receiveSerial()
{
    while (!g_shutdown && (!isConnected || !serial_port || !serial_port->isOpen()))
    {
        reconnect();
    }
    if (g_shutdown) return;

    try
    {
        size_t bytes_available = serial_port->available();
        if (bytes_available > 0)
        {
            size_t bytes_to_read = std::min(bytes_available, temp_read_buf_.size());
            size_t bytes_read    = serial_port->read(temp_read_buf_.data(), bytes_to_read);

            if (bytes_read > 0)
            {
                recv_buffer_.insert(recv_buffer_.end(), temp_read_buf_.begin(), temp_read_buf_.begin() + bytes_read);

                if (debug_)
                {
                    std::string hex_str;
                    for (size_t i = 0; i < bytes_read; ++i)
                    {
                        char buf[4];
                        snprintf(buf, sizeof(buf), "%02X ", temp_read_buf_[i]);
                        hex_str += buf;
                    }
                    MAS_LOG_INFO("Received {} bytes (HEX): {}", bytes_read, hex_str);
                }
            }

            while (recv_buffer_.size() >= RX_FRAME_SIZE)
            {
                auto header_it = std::find(recv_buffer_.begin(), recv_buffer_.end(), RX_HEADER);
                if (header_it == recv_buffer_.end())
                {
                    recv_buffer_.clear();
                    break;
                }
                if (header_it != recv_buffer_.begin())
                {
                    recv_buffer_.erase(recv_buffer_.begin(), header_it);
                    header_it = recv_buffer_.begin();
                }

                if (recv_buffer_.size() < RX_FRAME_SIZE)
                {
                    break;
                }

                const uint16_t expected_crc = static_cast<uint16_t>(
                    recv_buffer_[RX_FRAME_SIZE - 2] |
                    (static_cast<uint16_t>(recv_buffer_[RX_FRAME_SIZE - 1]) << 8));
                if (hardware::checksum(recv_buffer_.data(), RX_FRAME_SIZE - 2) != expected_crc)
                {
                    recv_buffer_.erase(recv_buffer_.begin());
                    continue;
                }

                ReceivePacket packet;
                std::memcpy(&packet, recv_buffer_.data() + 1, sizeof(packet));

                Eigen::Quaterniond q(packet.q[0], packet.q[1], packet.q[2], packet.q[3]);
                const double norm = q.norm();
                if (std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()) && norm > 1e-6)
                {
                    q.normalize();
                    const auto received_at = std::chrono::steady_clock::now();
                    // 队列满只丢姿态样本，不阻断模式和通信状态。
                    (void)data_queue_.try_push({q, received_at});
                    switch (packet.mode)
                    {
                    case 0:
                        commands_.set_mode(VisionMode::AUTO_AIM_RED);
                        break;
                    case 1:
                        commands_.set_mode(VisionMode::AUTO_AIM_BLUE);
                        break;
                    case 2:
                        commands_.set_mode(VisionMode::SMALL_RUNE_RED);
                        break;
                    case 3:
                        commands_.set_mode(VisionMode::SMALL_RUNE_BLUE);
                        break;
                    case 4:
                        commands_.set_mode(VisionMode::BIG_RUNE_RED);
                        break;
                    case 5:
                        commands_.set_mode(VisionMode::BIG_RUNE_BLUE);
                        break;
                    default:
                        commands_.set_mode(VisionMode::AUTO_AIM_RED);
                        MAS_LOG_WARN("[UserSerial] Invalid mode: {}", packet.mode);
                        break;
                    }

#ifdef SENTRY
                        {
                            std::lock_guard<std::mutex> lock(referee_mutex_);
                            referee_data_ = packet.referee_data;
                        }
#endif
                        const auto received_ms = std::chrono::duration_cast<std::chrono::milliseconds>(received_at.time_since_epoch()).count();
                        last_data_receive_ms_.store(received_ms);
                        has_received_data_.store(true);
                }
                recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + RX_FRAME_SIZE);
            }
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception while receiving data: {}", e.what());
        isConnected = false;
    }
}

void UserSerial::reconnect()
{
    if (g_shutdown)
    {
        return;
    }
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reconnect_time_).count() < 5000)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return;
    }
    last_reconnect_time_ = now;

    MAS_LOG_INFO("Attempting to reconnect to serial port...");

    if (serial_port)
    {
        if (serial_port->isOpen())
        {
            serial_port->close();
        }
        serial_port.reset();
    }
    isConnected = false;

    commands_.new_session();
    disableVision();
    pose_cutoff_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());
    recv_buffer_.clear();

    // data_queue_ 是单生产者/单消费者队列，重连线程不能从生产者侧 pop。上层靠标志位拒绝使用旧数据
    has_received_data_.store(false);
    last_data_receive_ms_.store(0);

    try
    {
        serial_port =
            std::make_shared<serial::Serial>(port_, baudrate_, serial::Timeout::simpleTimeout(timeout_), bytesize_, parity_, stopbits_, flowcontrol_);
        if (serial_port && serial_port->isOpen())
        {
            isConnected = true;
            MAS_LOG_INFO("Reconnected to {}", port_.c_str());
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception while reconnecting to {}: {}", port_.c_str(), e.what());
        isConnected = false;
    }
}

bool UserSerial::isQuaternionValid() const
{
    const auto age_ms = quaternionAgeMs();
    return age_ms >= 0 && age_ms <= data_timeout_ms_;
}

int64_t UserSerial::quaternionAgeMs() const
{
    if (!has_received_data_.load()) return -1;

    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    return now_ms - last_data_receive_ms_.load();
}

void UserSerial::pollPose()
{
    const auto cutoff = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(pose_cutoff_ns_.load()));
    while (!pose_history_.empty() && pose_history_.front().timestamp < cutoff)
        pose_history_.pop_front();
    while (auto *sample = data_queue_.front())
    {
        if (sample->timestamp >= cutoff &&
            (pose_history_.empty() || sample->timestamp > pose_history_.back().timestamp))
        {
            pose_history_.push_back(*sample);
            while (pose_history_.size() > 5000 ||
                   (!pose_history_.empty() &&
                    sample->timestamp - pose_history_.front().timestamp > std::chrono::seconds(2)))
                pose_history_.pop_front();
        }
        data_queue_.pop();
    }
}

std::optional<Eigen::Quaterniond> UserSerial::poseAt(std::chrono::steady_clock::time_point t)
{
    pollPose();
    if (!isQuaternionValid())
        return {};
    return interpolate_pose(t, std::chrono::milliseconds(data_timeout_ms_));
}

std::optional<Eigen::Quaterniond> UserSerial::awaitPose(std::chrono::steady_clock::time_point t)
{
    auto pose           = poseAt(t);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5);
    // 仅等待尚未到达的后点；早于历史窗口的帧不能靠等待修复。
    while (!pose && isQuaternionValid() &&
           (pose_history_.empty() ? std::chrono::steady_clock::time_point{} : pose_history_.back().timestamp) < t &&
           std::chrono::steady_clock::now() < deadline && !g_shutdown)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        pose = poseAt(t);
    }
    return pose;
}

Eigen::Quaterniond UserSerial::q(std::chrono::steady_clock::time_point t)
{
    // 兼容已有独立串口工具；生产链路使用 poseAt 的显式有效性。
    pollPose();
    return interpolate_pose(t).value_or(pose_history_.empty() ? Eigen::Quaterniond::Identity()
                                                              : pose_history_.back().quaternion);
}

std::optional<Eigen::Quaterniond> UserSerial::interpolate_pose(std::chrono::steady_clock::time_point time,
                                                               std::chrono::steady_clock::duration max_gap) const
{
    if (pose_history_.empty() || time < pose_history_.front().timestamp || time > pose_history_.back().timestamp)
        return {};
    const auto right = std::lower_bound(pose_history_.begin(), pose_history_.end(), time,
                                        [](const auto &p, auto t) { return p.timestamp < t; });
    if (right->timestamp == time)
        return right->quaternion;
    const auto left = std::prev(right);
    if (right->timestamp - left->timestamp > max_gap)
        return {};
    const double interval = std::chrono::duration<double>(right->timestamp - left->timestamp).count();
    const double ratio    = std::chrono::duration<double>(time - left->timestamp).count() / interval;
    return left->quaternion.slerp(ratio, right->quaternion).normalized();
}
