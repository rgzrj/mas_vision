#include "plotter.hpp"
#include "mas_log.hpp"

#include <arpa/inet.h>  // htons, inet_pton
#include <sys/socket.h> // socket, sendto
#include <unistd.h>     // close
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace rm_utils
{
Plotter::Plotter(std::string host, uint16_t port)
{
    destination_.sin_family      = AF_INET;
    destination_.sin_port        = ::htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &destination_.sin_addr) != 1)
        throw std::invalid_argument("Invalid Plotter IPv4 address: " + host);

    // 地址校验先于资源申请，构造失败时不会遗留 fd。
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0)
        MAS_LOG_WARN("Plotter socket: {}", std::strerror(errno));
}

Plotter::~Plotter()
{
    if (socket_ >= 0)
        ::close(socket_);
}

void Plotter::plot(const nlohmann::json &json)
{
    if (socket_ < 0)
        return;

    const auto data = json.dump();

    std::lock_guard<std::mutex> lock(mutex_);
    const auto sent = ::sendto(socket_, data.data(), data.size(), 0,
                              reinterpret_cast<const sockaddr *>(&destination_), sizeof(destination_));
    if (sent >= 0 && static_cast<std::size_t>(sent) == data.size())
        return;

    const int error = errno;
    const auto now = std::chrono::steady_clock::now();
    if (error_time_ == std::chrono::steady_clock::time_point{} || now - error_time_ >= std::chrono::seconds(5))
    {
        MAS_LOG_WARN("Plotter sendto: written={}, expected={}, error={}", sent, data.size(),
                     sent < 0 ? std::strerror(error) : "Incomplete datagram");
        error_time_ = now;
    }
}

} // namespace rm_utils
