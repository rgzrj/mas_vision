#pragma once

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>

// #define SENTRY

// 线包按字节排列，与电控约定的字节布局一致。
#pragma pack(push, 1)

// 接收结构体

struct RefereePacket
{
    uint16_t projectile_allowance_17mm;       // 剩余发弹量
    uint8_t  power_management_shooter_output; // 功率管理 shooter 输出
    uint16_t current_hp;                      // 机器人当前血量
    uint16_t outpost_HP;                      // 前哨站血量
    uint16_t base_HP;                         // 基地血量
    uint8_t  game_progess;
};

#ifdef  SENTRY
struct ReceivePacket
{
    uint8_t mode;
    float   q[4]; // 四元数 wxyz 顺序
    RefereePacket referee_data;
};
#else
struct ReceivePacket
{
    uint8_t mode;
    float   q[4]; // 四元数 wxyz 顺序
};
#endif

// 发送结构体
struct SendPacket
{
    bool    found;
    float   target_yaw;
    float   target_pitch;
    uint8_t fire_advice;
};

#pragma pack(pop)

// 带时间戳的四元数数据
struct QuaternionWithTimestamp
{
    Eigen::Quaterniond                    quaternion;
    std::chrono::steady_clock::time_point timestamp;
};

// 视觉模式枚举
enum VisionMode
{
    AUTO_AIM_RED    = 0,
    AUTO_AIM_BLUE   = 1,
    SMALL_RUNE_RED  = 2,
    SMALL_RUNE_BLUE = 3,
    BIG_RUNE_RED    = 4,
    BIG_RUNE_BLUE   = 5,
};

// 接收模式及其版本，不属于串口线包。
struct VisionState
{
    VisionMode mode     = AUTO_AIM_RED;
    std::uint64_t epoch = 0;
};



