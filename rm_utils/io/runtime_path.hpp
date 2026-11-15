#pragma once
#include <filesystem>
#include <cstdlib>

namespace rm_utils
{
// 交付布局为 <项目>/build/mas_buff、<项目>/config；不依赖终端 cwd 或编译机源码路径。
inline std::filesystem::path project_root()
{
    return std::filesystem::canonical("/proc/self/exe").parent_path().parent_path();
}

// 与 cmake/OpenVINO.cmake、environment/setup_linux.sh 使用同一共享安装位置。
inline std::filesystem::path shared_venv()
{
    const char *data = std::getenv("XDG_DATA_HOME");
    if (data && *data)
        return std::filesystem::path(data) / "mas-vision/venv";
    const char *home = std::getenv("HOME");
    return home && *home ? std::filesystem::path(home) / ".local/share/mas-vision/venv"
                         : std::filesystem::path{};
}
}
