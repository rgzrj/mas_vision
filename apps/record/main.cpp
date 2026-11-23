#include <atomic>
#include "mas_log.hpp"
#include <csignal>
#include "record.hpp"
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
    if (argc > 2 || (argc > 1 && std::string(argv[1]) == "--help"))
    {
        std::cout << "mas_vision_record [config/record.yaml]\n";
        return argc > 2 ? 1 : 0;
    }
    try
    {
        return app::run_record(argc > 1 ? std::filesystem::absolute(argv[1])
                                        : rm_utils::project_root() / "config/record.yaml");
    }
    catch (const std::exception &e)
    {
        g_shutdown = true;
        MAS_LOG_ERROR("{}", e.what());
        return 1;
    }
}
