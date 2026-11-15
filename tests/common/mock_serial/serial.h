#pragma once
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>
namespace serial
{
enum bytesize_t
{
    fivebits  = 5,
    sixbits   = 6,
    sevenbits = 7,
    eightbits = 8
};
enum parity_t
{
    parity_none,
    parity_odd,
    parity_even
};
enum stopbits_t
{
    stopbits_one,
    stopbits_two
};
enum flowcontrol_t
{
    flowcontrol_none,
    flowcontrol_software,
    flowcontrol_hardware
};
struct Timeout
{
    static Timeout simpleTimeout(uint32_t)
    {
        return {};
    }
};
class Serial
{
public:
    inline static std::vector<uint8_t> incoming;
    inline static std::vector<uint8_t> sent;
    Serial(const std::string &, uint32_t, Timeout, bytesize_t, parity_t, stopbits_t, flowcontrol_t) {}
    bool isOpen() const
    {
        return open_;
    }
    void close()
    {
        open_ = false;
    }
    size_t available() const
    {
        return incoming.size();
    }
    size_t read(uint8_t *p, size_t n)
    {
        n = std::min(n, incoming.size());
        std::copy_n(incoming.begin(), n, p);
        incoming.erase(incoming.begin(), incoming.begin() + n);
        return n;
    }
    size_t write(const uint8_t *p, size_t n)
    {
        sent.assign(p, p + n);
        return n;
    }

private:
    bool open_ = true;
};
} // namespace serial
