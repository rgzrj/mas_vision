#pragma once

#include "paired_frame.hpp"
#include <filesystem>
#include <iosfwd>
#include <vector>

namespace tool
{
/**
 * @brief 写入 frames.csv 表头，设置四元数的输出精度。
 * @param out 元数据输出流，写入失败由调用方检查。
 */
void write_metadata_header(std::ostream &out);

/**
 * @brief 写入单帧元数据；无姿态时写零占位和 pose_valid=0。
 * @param out 元数据输出流，写入失败由调用方检查。
 * @param frame 本帧时间、姿态及模式，不写图像。
 */
void write_frame_metadata(std::ostream &out, const RecordedFrame &frame);

/**
 * @brief 按列名读取元数据，兼容旧八列记录和附加诊断列。
 * @param path frames.csv 路径。
 * @return 顺序元数据，合法姿态归一化；格式错误抛出带文件和行号的异常。
 */
std::vector<RecordedFrame> read_metadata(const std::filesystem::path &path);
} // namespace tool
