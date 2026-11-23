#include "threads/vision_router/vision_router.hpp"
#include "auto_aim/debug/armor_debug.hpp"
#include "auto_buff/debug/buff_debug.hpp"
#include "hardware/serial/user_serial.hpp"
#include "hardware/serial/crc16.hpp"
#include "tests/common/test.hpp"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <cstring>
#include <fstream>
#include <thread>

std::atomic<bool> g_shutdown{false};

namespace
{
// 仅替换串口传输，模式解析、router及配置读取均执行生产代码。
void feed(UserSerial &serial, std::uint8_t mode)
{
    ReceivePacket packet{};
    packet.mode = mode;
    packet.q[0] = 1;
    std::vector<std::uint8_t> bytes(1 + sizeof(packet) + 2);
    bytes[0] = 0x5a;
    std::memcpy(bytes.data() + 1, &packet, sizeof(packet));
    const auto crc          = hardware::checksum(bytes.data(), bytes.size() - 2);
    bytes[bytes.size() - 2] = std::uint8_t(crc);
    bytes.back()            = std::uint8_t(crc >> 8);
    serial::Serial::incoming.insert(serial::Serial::incoming.end(), bytes.begin(), bytes.end());
    serial.receiveSerial();
}
struct RestoreDirectory
{
    std::filesystem::path path = std::filesystem::current_path();
    ~RestoreDirectory()
    {
        std::filesystem::current_path(path);
    }
};
} // namespace

int main()
{
    return tests::run("router_runtime_test",
                      []
                      {
                          using tests::require;
                          tests::TemporaryDirectory directory;
                          RestoreDirectory restore;
                          std::filesystem::create_directory(directory.path / "config");
                          std::filesystem::current_path(directory.path);
                          const std::filesystem::path aim = "config/auto_aim.yaml", buff = "config/auto_buff.yaml";
                          std::ofstream(aim)
                              << "auto_aim: {armor_detector: {debug: false, detect_color: RED}, "
                                 "armor_track: {armor_track_debug: false}, armor_shoot: {debug: false}}\n";
                          std::ofstream(buff)
                              << "auto_buff: {detect: {common: {debug: false, color: blue}}, track: {debug: false}, "
                                 "shoot: {debug: false}}\nruntime: {max_frame_age_ms: 17.5, max_runtime_s: 0.03}\n";
                          std::ofstream("config/serial_config.yaml") << "serial: {port: mock, data_timeout_ms: 1000}\n";
                          const auto armor_views = auto_aim::ArmorDebug::enabled_views(YAML::LoadFile(aim.string()));
                          const auto buff_views  = auto_buff::BuffDebug::enabled_views(YAML::LoadFile(buff.string()));
                          require(!armor_views && !buff_views, "Path constructors ignored YAML debug switches");
                          auto &serial = UserSerial::getInstance();
                          rigtorp::SPSCQueue<CameraFrame> camera(2);
                          threads::VisionQueue armor(2), rune(2);
                          for (const std::string color : {"red", "blue"})
                          {
                              auto aim_config = YAML::LoadFile(aim.string());
                              aim_config["auto_aim"]["armor_detector"]["detect_color"] =
                                  color == "blue" ? "BLUE" : "RED";
                              std::ofstream(aim) << aim_config;

                              auto buff_config = YAML::LoadFile(buff.string());
                              buff_config["auto_buff"]["detect"]["common"]["color"] = color;
                              std::ofstream(buff) << buff_config;

                              for (const auto mode : {0, 1, 2, 3, 4, 5, 0, 2, 4})
                              {
                                  g_shutdown = false;
                                  feed(serial, mode);
                                  CameraFrame image;
                                  image.frame     = cv::Mat::zeros(2, 2, CV_8UC3);
                                  image.timestamp = std::chrono::steady_clock::now();
                                  require(camera.try_push(std::move(image)), "Camera test queue full");
                                  threads::vision_router_func(camera, armor, rune);
                                  require(g_shutdown, "Runtime deadline did not notify all tasks");
                                  auto &active      = mode <= 1 ? armor : rune;
                                  const auto *frame = active.front();
                                  require(frame != nullptr, "Frame was not routed");
                                  require(frame->state.mode == mode && frame->color == ((mode & 1) ? "blue" : color),
                                          "Mode or YAML color mapping changed");
                                  const auto task = mode <= 1   ? threads::VisionTask::armor
                                                    : mode <= 3 ? threads::VisionTask::small_rune
                                                                : threads::VisionTask::big_rune;
                                  require(frame->task == task, "Wrong pipeline task");
                                  require(frame->max_age == std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                                std::chrono::duration<double, std::milli>(17.5)),
                                          "Frame age setting was not propagated");
                                  active.pop();
                                  require(armor.size() == 0 && rune.size() == 0, "Inactive pipeline received a frame");
                              }
                          }
                          g_shutdown = false;
                          threads::vision_router_func(camera, armor, rune);
                          require(g_shutdown, "No-frame run ignored its deadline");
                          auto config                        = YAML::LoadFile(buff.string());
                          config["runtime"]["max_runtime_s"] = 0;
                          std::ofstream(buff) << config;
                          g_shutdown = false;
                          std::atomic<bool> stopped_early{false};
                          {
                              std::jthread stop(
                                  [&]
                                  {
                                      std::this_thread::sleep_for(std::chrono::milliseconds(20));
                                      stopped_early = g_shutdown.load();
                                      g_shutdown    = true;
                                  });
                              threads::vision_router_func(camera, armor, rune);
                          }
                          require(!stopped_early, "Zero runtime should wait for the shutdown signal");
                          config["runtime"]["max_frame_age_ms"] = 0;
                          std::ofstream(buff) << config;
                          bool rejected = false;
                          g_shutdown    = false;
                          try
                          {
                              threads::vision_router_func(camera, armor, rune);
                          }
                          catch (const std::runtime_error &)
                          {
                              rejected = true;
                          }
                          require(rejected, "Invalid runtime setting was accepted");
                          g_shutdown = true;
                          serial.closeSerial();
                      });
}
