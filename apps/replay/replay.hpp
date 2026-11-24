#pragma once
#include <filesystem>
#include <string>
namespace app
{
// 与实机一致，0/2/4 沿用 YAML，1/3/5 使用蓝色。
inline std::string replay_color(int mode, const std::string &local_color)
{
    return mode == 1 || mode == 3 || mode == 5 ? "blue" : local_color;
}

int run_replay(const std::filesystem::path &, bool check_only = false);
}
