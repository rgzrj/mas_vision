#include "hardware/serial/serial_types.hpp"
#include "hardware/serial/crc16.hpp"
#include "tests/common/test.hpp"

int main()
{
    return tests::run("serial_protocol_test",
                      []
                      {
                          static_assert(sizeof(SendPacket) == 10);
#ifndef SENTRY
                          static_assert(sizeof(ReceivePacket) == 17);
#endif
                          constexpr std::uint8_t bytes[]{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
                          tests::require(hardware::checksum(bytes, sizeof(bytes)) == 0x6f91,
                                         "Protocol CRC16 reference vector changed");
                      });
}
