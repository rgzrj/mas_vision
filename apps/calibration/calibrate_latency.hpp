#pragma once

namespace calibration
{
// 软件触发三档曝光测量，达标后写入hikcamera.yaml的transfer_latency_ms。
int calibrate_latency_main();
} // namespace calibration
