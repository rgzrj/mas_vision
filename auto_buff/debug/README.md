# 打符调试

`BuffDebug` 管理打符调试窗口布局和实机单帧提交。`BuffDebugBase` 为三个绘制子类提供开关、窗口名、FPS 统计和图元提交。各算法在内部调用对应子类的绘制函数。

| 文件 | 持有位置 | 职责 |
|---|---|---|
| buff_debug.hpp/.cpp | 三个调试子类 | 开关、窗口名、FPS、图元提交、布局与无效状态文字 |
| visualize/detect_debug.hpp/.cpp | Detector | 过滤后的板/R/灯臂、原始网络分数及计数 |
| visualize/track_debug.hpp/.cpp | RmcsTracker | 运动拟合、六维 EKF 与协方差、关联观测和未来投影 |
| visualize/shoot_debug.hpp/.cpp | RuneShooter::Impl | 本帧云台角、目标角、差值、飞行时间与发射建议 |
| blueprints | 布局维护工具 | 三个开关对应的七种 Rerun 布局 |

开关分别为 `auto_buff.detect.common.debug`、`auto_buff.track.debug`、`auto_buff.shoot.debug`。算法在构造时读取配置，显示后端继续由公共 `Display` 根据 `display.backend` 选择 SDL 或 Rerun。SDL 为 detect、track、shoot 三个独立窗口，事件由主线程 `Display::poll()` 处理。

检测公共入口按“后端计算 → 模式/颜色过滤 → draw”执行。跟踪与火控在结果确定后绘制。SDL在采样后持有只读图像引用，主线程负责纹理上传与绘制；图元只在本帧需要显示图像时构建。FPS在图像采样之前累计模块处理调用，不代表窗口刷新率或纯网络推理速度。

实机线程和回放入口设置该帧曝光时间线，锁住整帧显示提交，并在各阶段完成后 `Display::end_frame()`。SDL不创建RRD或整理Rerun数值；Rerun图像采样与逐帧数值记录分开。缺姿态或模式停用时，基类更新状态文字，子类 `clear()` 仅在Rerun会话中清理曲线和空间点；无效数值写 NaN，防止继续显示旧模型。检测原始候选序号只代表当前帧顺序，不是跨帧 ID。

报告、标注视频和性能 CSV 不属于实时调试层。视频及姿态只由 `apps/record` 采集；`apps/replay` 只读取、运行生产算法和显示，输出简要运行统计。测速位于 `tools/buff_profile.cpp` 与 `tools/yolo_profile.cpp`。
