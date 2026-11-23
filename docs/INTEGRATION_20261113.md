# armor/buff接入（2026-11-13）

2026-11-14 更新：main/router 不持有 Debug 对象。armor/buff 的三个算法分别持有调试子类，在内部按 YAML 开关直接绘制；Display 仍由主线程处理窗口事件。实机不依赖回放编排，回放只读取、运行算法和显示，不生成算法 JSONL、标注视频或性能 CSV。测速独立放在 tests，录像独立放在 apps/record。

本轮宿主回归和源码核对保存在 output/buff_structure_20261114；下方保留此前验证记录。宿主测试与 CMake 依赖图检查不能替代 Linux/MVS 实机验收。

## 数据流

相机SPSC → router → armor或buff SPSC → 对应算法线程 → VisionCommand → 串口发送。

相机队列只有router消费；两个工作队列各只有对应算法线程消费。router只投帧，不跨线程pop/reset。每帧携带原始图像、曝光时间、模式、颜色、切换epoch和可选姿态。

mode和通信接收时间不依赖姿态入队成功。串口接收线程在模式变化时增加epoch并使旧命令失效。算法线程在自己的新epoch首帧重置tracker/shooter，模型只加载一次。提交与模式切换共用VisionCommand锁；旧epoch结果不改命令。发送快照按曝光时刻的截止时间清除found/fire，断图不能无限重复旧开火建议。

姿态队列仅由router消费，转为两秒有界历史；按曝光时刻插值，窗口外返回无值；若只是后点尚未到达，最多等待5ms，仍缺失则明确标记。即使无相机帧也持续排出姿态队列。重新连接前的姿态按接收截止时间丢弃。独立采集器是另一个进程，其主线程是该进程唯一姿态消费者。

## 算法与显示

实机armor线程直接创建并跨帧保留ArmorDetector、ArmorTrack、ArmorShoot；buff线程直接持有Detector、RmcsTracker和RuneShooter，配置在模块内部加载。装甲板回放适配ArmorModules/process_armor_frame位于apps/replay/armor_process，实机不依赖它。打符对应适配位于apps/replay/buff_process，仅供离线调用。armor使用MAS原有ExtendedKalmanFilter，buff保留RMCS滤波实现；不替换两者状态模型或弹道方程。装甲板火控的当前时刻由调用方传入，实机使用处理时刻，离线使用记录时刻，避免回放速度改变提前量。

两个领域的图元代码分别集中于 auto_aim/debug/visualize 和 auto_buff/debug/visualize。公共基类管理开关、窗口和 FPS，子类读取本次算法结果；不收集整帧快照、不创建调试工作线程。处理线程按同帧曝光时刻设置显示时间线，并互斥提交整帧图元，避免交叉修改显示上下文。router 统一选择任务、颜色及显示域；SDL 操作窗口仍在主线程，Rerun 共享底图。

track和shoot恢复原MAS Plotter载荷及独立plotter_enable。track每帧输出原始四角点、同帧R_gimbal2world、PnP、状态及模型诊断，不等tracking成功。没有观测时清除本帧有效标志和计数，避免沿用上一帧诊断。GUI开关不控制UDP输出。

不设可视化编译开关，比赛和调试使用同一份程序。各模块debug控制运行时提交，全关时不创建显示会话、不准备调试图元、不复制调试图像；日志照常输出。SDL/Rerun是固定构建依赖。Tracy 分段耗时保留，CSV 测速由 tests 独立工具完成。

主入口直接定义g_shutdown并注册SIGINT/SIGTERM，直接提交五个线程池任务。主循环处理SDL事件和退出标志，收尾逐个等待future；入口异常路径先设置退出标志，再由线程池收尾。calibration的三个入口已加入同一可执行文件，在启动实机任务前分派。图形标定使用当前SDL主线程按键接口，手眼采集只接收串口姿态且拒绝缺失配对姿态的样本；拟合公式沿用原文件。

主线程直接管理Display生命周期，不再使用preview.hpp。配置快照工具移到apps/record/session；record/replay分别保留独立main和CMake入口。record不查OpenVINO或编译算法，replay不查MVS或编译硬件/实机线程。根工程仍能一次生成全部程序。

## 验证范围

本机已通过：

- VisionCommand与UserSerial内部姿态历史回归：切换中的旧结果、A→B→A、重连epoch、断图命令过期、姿态插值越界和间隔过大。
- 生产串口解析器配内存传输：姿态队列满后mode和接收时间仍更新，13字节发送、过期命令清除和旧模式结果拒绝。
- 配对录制/打符回放回归：时间和模式检查、缺失姿态、播放速度、模型与标定快照、Unicode路径及XML/BIN配对。
- 装甲板paired/media回放回归，以及有速度合成目标在回放等待前后得到相同瞄准量。
- 接口检查；使用外部库占位目标配置CMake依赖图，核对三个程序的链接边界。可视化统一编译，实际显示只由YAML控制。
- 26个受保护实现文件哈希未变化，包括RMCS跟踪、火控、符专属数学及明确要求不改的serial/common/src/impl/unix.cc。

验证记录在output/integration_20261113。Windows本机的语法、模拟传输和离线回归不等同于Linux完整编译；CMake依赖图检查也没有编译真实第三方SDK。MVS取图、Linux终端按键、SDL/Rerun上屏、串口硬件、实机帧率和射击精度仍需在Linux设备验收。

## 本轮窗口、Plotter及入口整理的补充验证

记录在output/restore_debug_router_20261113/verification.json：

- 原窗口图元回归通过，检查灯条/四点、位姿重投影、预热期模型、选中板颜色及shoot文本。
- 生产track/shoot载荷回归通过，检查GUI关闭后的原始手眼四点、同帧矩阵、缺观测有效标志及原shoot字段。宿主替换UDP发送端；tests/auto_aim/armor_plotter_test.cpp中的真实Linux UDP用例仍需目标机执行。
- 当前装甲板和打符paired/media回放回归通过；宿主替换终端按键和Display接收后端，算法、配对IO和快照使用生产源码。
- SDL按键回归通过；主入口、router、工作线程及迁移后的record/replay接口检查通过。
- 根构建和两个独立CMake入口的目标依赖检查通过，外部SDK采用占位目标；未执行完整Linux依赖编译。
- OpenVINO发现的九种路径场景通过；恢复的MAS plot_monitor字段展开和双流CSV检查通过，工具输出改到output/profiling/plotter。
- 26个受保护实现哈希仍一致，serial/common/src/impl/unix.cc未修改。
