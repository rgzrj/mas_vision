#include "rm_utils/plotter/plotter.hpp"
#include "tests/common/test.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

int main()
{
    return tests::run("plotter_test",
                      []
                      {
                          using tests::require;

                          bool rejected = false;
                          try
                          {
                              rm_utils::Plotter invalid("invalid-address");
                          }
                          catch (const std::invalid_argument &)
                          {
                              rejected = true;
                          }
                          require(rejected, "Invalid Plotter address accepted");

                          const int listener = ::socket(AF_INET, SOCK_DGRAM, 0);
                          require(listener >= 0, "Cannot create UDP listener");
                          struct ListenerCleanup
                          {
                              int fd;
                              ~ListenerCleanup()
                              {
                                  ::close(fd);
                              }
                          } cleanup{listener};

                          sockaddr_in address{};
                          address.sin_family      = AF_INET;
                          address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                          require(::bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0,
                                  "Cannot bind UDP listener");
                          socklen_t size = sizeof(address);
                          require(::getsockname(listener, reinterpret_cast<sockaddr *>(&address), &size) == 0,
                                  "Cannot query UDP port");
                          const timeval timeout{1, 0};
                          require(::setsockopt(listener, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0,
                                  "Cannot set UDP timeout");

                          rm_utils::Plotter plotter("127.0.0.1", ntohs(address.sin_port));
                          const nlohmann::json payload{
                              {"handeye", {{"valid", true}, {"p0", {{"x", 12.5}, {"y", -2.}}}}}};
                          const auto receive = [&]
                          {
                              char buffer[1024];
                              const auto count = ::recv(listener, buffer, sizeof(buffer), 0);
                              require(count > 0, "Missing Plotter datagram");
                              require(nlohmann::json::parse(buffer, buffer + count) == payload, "Plotter JSON changed");
                          };
                          plotter.plot(payload);
                          receive();

                          // 超长 UDP 数据报应失败；后续正常调试数据仍能发送。
                          const nlohmann::json oversized{{"data", std::string(70000, 'x')}};
                          plotter.plot(oversized);
                          plotter.plot(oversized);
                          plotter.plot(payload);
                          receive();
                      });
}
