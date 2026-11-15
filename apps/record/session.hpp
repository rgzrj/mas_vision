#pragma once
#include <filesystem>
#include <yaml-cpp/yaml.h>
namespace app
{
// 录制时复制算法配置、内参/手眼、串口配置及模型（XML同时复制BIN），并改写快照内资源路径。
// 不启动相机，不执行算法；回放直接读取这些快照文件。
void save_recording_snapshot(const std::filesystem::path &, const YAML::Node &, const std::filesystem::path &,
                             bool copy_model);
void save_session_configs(const std::filesystem::path &config_dir, const std::filesystem::path &output);
} // namespace app
