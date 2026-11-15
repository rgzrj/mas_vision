#pragma once
#include <filesystem>
namespace app
{
int run_replay(const std::filesystem::path &, bool check_only = false);
}
