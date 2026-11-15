# 装甲板调试

`ArmorDebug` 是公共基类，管理每个实例的开关、窗口名、FPS 统计和图元提交。四个子类负责各阶段的绘制，算法持有对应子类，在内部直接调用 `draw()`。

| 文件 | 持有者 | 作用 |
|---|---|---|
| `armor_debug.hpp/.cpp` | 四个调试子类 | 公共显示操作、窗口布局、无效状态文字 |
| `visualize/detect_debug.hpp/.cpp` | `ArmorDetector` | 灯条矩形与中心、装甲板角点与轮廓、类别分数、几何比例、计数和 FPS |
| `visualize/track_debug.hpp/.cpp` | `ArmorTrack` | 跟踪状态、EKF 参数、各板位投影与编号、选中板、旋转中心和 FPS |
| `visualize/pose_debug.hpp/.cpp` | `ArmorTrack` | 同帧 IMU、首块装甲板位姿、检测轮廓与重投影、平均像素误差 |
| `visualize/shoot_debug.hpp/.cpp` | `ArmorShoot` | 云台角、指令角、角误差、瞄准模式、水平距离、有效标志和板位 ID |
| `blueprints/generate.py` | 布局维护工具 | 生成 Rerun 的 15 种非空窗口组合，普通运行不需要 Python |

## 开关与窗口

配置位于 `config/auto_aim.yaml` 的 `auto_aim` 节点：

| 开关 | 窗口 |
|---|---|
| `armor_detector.debug` | `HikCamera` |
| `armor_track.armor_track_debug` | `HikCamera_track` |
| `armor_track.armor_pose_debug` | `HikCamera_track_pose`，同时要求 track 开启 |
| `armor_shoot.debug` | `HikCamera_shoot` |

子类关闭时，`draw()` 立即返回，不统计 FPS、不准备图元、不复制调试图像。调试层没有后台绘制线程；开启后的绘制与编码耗时计入当前处理线程。

算法只准备文字、点和线，由基类 `show()` 统一提交给公共 `Display`。SDL 的事件处理和实际开窗由主线程 `Display::poll()` 完成，四个窗口各自独立；Rerun 使用已有布局和数值曲线。检测不提供二值图或数字拼接窗口。

## 状态失效与曲线

`ArmorDebug::unavailable()` 只用当前底图和状态文字替换旧叠图。缺姿态时保留检测图；模式未启用时可以一并清理检测图。

曲线由对应子类负责，调用方根据已开启的窗口决定是否清理：

- `DetectDebug::clear()`：装甲板、灯条计数归零。
- `TrackDebug::clear()`：有效标志归零，11 维状态和协方差对角线写入 NaN，避免沿用旧值。
- `ShootDebug::clear()`：yaw、pitch 和开火建议归零。

## 回放与 Plotter

回放调用同一套生产算法。`playback.display=false` 关闭算法的图像调试，Plotter 开关保持独立。回放结果结构位于 `apps/replay/armor_process.hpp`，不进入实机链路；火控直接返回公共 `SendPacket`，串口仍发送包含 found 的 13 字节帧。

`armor_track.plotter_enable` 和 `armor_shoot.plotter_enable` 在生产算法中控制 UDP Plotter，沿用原 MAS 字段。track 的 handeye 数据包含原始四角点与同帧 `R_gimbal2world`，不以 EKF 进入 tracking 为条件；图像 debug 全关仍可采集，缺姿态时不生成标定样本。

接收工具为 `tools/plot_monitor.py`：`--no-gui` 只用标准库记录 CSV，GUI 使用 pyqtgraph/PyQt5。结果保存在 `output/profiling/plotter`，手眼求解入口为主程序 `calibrate_handeye`。
