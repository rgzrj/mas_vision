#ifndef _PLOTTER_H_
#define _PLOTTER_H_

#include <netinet/in.h> // sockaddr_in

#include <mutex>
#include <chrono>
#include <nlohmann/json.hpp>
#include <string>

namespace rm_utils
{
class Plotter
{
public:
    /**
     * @brief 创建 UDP 调试输出；无效 IPv4 地址抛异常，socket 不可用时停用发送。
     */
    Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);

    ~Plotter();

    /**
     * @brief 发送 JSON；发送失败只限频提示，不中断算法或重试旧数据。
     */
    void plot(const nlohmann::json &json);

private:
    int socket_ = -1;
    sockaddr_in destination_{};
    std::mutex mutex_;
    std::chrono::steady_clock::time_point error_time_{};
};

} // namespace rm_utils

#endif // _PLOTTER_H_
