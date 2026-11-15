#include <atomic>
#include <csignal>
#include "replay.hpp"
#include "rm_utils/io/runtime_path.hpp"
#include <iostream>
// 全局原子变量控制本程序所有线程的退出。
std::atomic<bool> g_shutdown{false};
void signal_handler(int)
{
    g_shutdown = true;
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    if (argc > 3 || (argc > 1 && std::string(argv[1]) == "--help"))
    {
        std::cout << "mas_vision_replay [replay.yaml] [--validate]\n";
        return argc > 3 ? 1 : 0;
    }
    if (argc == 3 && std::string(argv[2]) != "--validate")
    {
        std::cerr << "Unknown option\n";
        return 1;
    }
    try
    {
        return app::run_replay(
            argc > 1 ? std::filesystem::absolute(argv[1]) : rm_utils::project_root() / "config/replay.yaml", argc == 3);
    }
    catch (const std::exception &e)
    {
        g_shutdown = true;
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
