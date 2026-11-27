> 此文记录合并前的历史结构。2026-11-13当前入口与并发边界见[接入说明](INTEGRATION_20261113.md)和[apps](../apps/README.md)。

# 当前代码逐文件职责说明（2026-11-11）

配套阅读：[结构审查与调用链](CODE_STRUCTURE_AUDIT_20261109.md)。本表说明磁盘上当前文件的实际职责，不代表本轮重构计划。

覆盖当前源码、头文件、构建脚本、运行配置与训练侧代码文件。每个文件单独一行，头文件与实现分开说明。

范围：当前自有运行代码、可选测试、构建/环境、配置、MAS 日志包装、两个单头库入口，以及 rm 中的 Python/启动脚本/参考源码。third_party/rerun/sdk、third_party/tracy、.deps 的外部库内部源码按库说明；build*、output、tests/output、tests/data、数据集图片/标签和依赖归档不是生产源码，不混入逐文件表。

“不进入主流程”不等于无用；历史训练脚本、平台随库文件和供 MAS 合并的公共接口不能据此直接删除。审查未运行训练或访问设备。

## 01 应用入口与流程（apps）（12 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [apps/buff/live.cpp](../apps/buff/live.cpp) | buff_app | 启动海康、串口和打符三个工作任务；主线程轮询显示，回调写JSONL/CSV并提交调试快照，负责停止与收尾；视频采集独立使用record入口。 |
| [apps/buff/live.hpp](../apps/buff/live.hpp) | buff_app | 声明 app::run_live 实机入口。 |
| [apps/buff/preview.hpp](../apps/buff/preview.hpp) | 应用使用的头文件 RAII | 按 YAML 与输出目录配置 Display 并负责关闭；带会话路径/控制台提示，属于应用装配性质的辅助代码。 |
| [apps/buff/read.cpp](../apps/buff/read.cpp) | buff_app | 读取图片/视频，组织离线单帧算法、结果导出、显示与性能日志；无硬件采集。 |
| [apps/buff/read.hpp](../apps/buff/read.hpp) | buff_app | 声明 app::run_read 普通媒体处理及导出入口。 |
| [apps/buff/record.cpp](../apps/buff/record.cpp) | buff_app | 仅采集相机和曝光时刻姿态，交给 PairedRecorder 成对保存，生成回放配置及统计；不运行三阶段算法。 |
| [apps/buff/record.hpp](../apps/buff/record.hpp) | buff_app | 声明 app::run_record 独立采集入口。 |
| [apps/buff/replay.cpp](../apps/buff/replay.cpp) | buff_app | 回放成对图像与 CSV，检查数据一致性，按记录时间驱动算法，处理暂停/继续/单步与导出；check 分支不创建算法。 |
| [apps/buff/replay.hpp](../apps/buff/replay.hpp) | buff_app | 声明 app::run_replay；check_only 控制仅检查配对。 |
| [apps/buff/session.cpp](../apps/buff/session.cpp) | buff_app | 复制本次算法、设备、标定配置，并整理模型和资源引用，生成可回放的会话配置。 |
| [apps/buff/session.hpp](../apps/buff/session.hpp) | buff_app | 声明采集会话的配置快照保存函数。 |
| [apps/main.cpp](../apps/main.cpp) | mas_buff | 定义进程级g_shutdown；解析命令和配置路径，分派run/read/export/record/replay/check并统一报告异常。record仅采集，export负责导出。 |

## 02 算法装配与离线单帧编排（auto_buff 根与 buff_replay）（5 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [auto_buff/buff_modules.cpp](../auto_buff/buff_modules.cpp) | buff_modules | 读取后端选择，将业务和标定配置路径传给各模块构造函数；模块内部解析参数，实例跨帧复用；可启动Tracy。 |
| [auto_buff/buff_modules.hpp](../auto_buff/buff_modules.hpp) | buff_modules | 声明 BuffModules：拥有 detector/tracker/shooter 实例，以及统一创建函数。 |
| [auto_buff/buff_replay/buff_replay.cpp](../auto_buff/buff_replay/buff_replay.cpp) | buff_replay | 普通媒体/成对回放的单帧 detect→筛选→姿态→track→shoot 编排与计时；不打开设备、不发送串口、不显示。实机当前不走此函数。 |
| [auto_buff/buff_replay/buff_replay.hpp](../auto_buff/buff_replay/buff_replay.hpp) | buff_replay | 定义 BuffFrame（图像、时间、可选姿态、颜色和模式）及 process_buff_frame 两个重载。 |
| [auto_buff/buff_type.hpp](../auto_buff/buff_type.hpp) | 跨模块结果协议 | 定义三阶段 BuffResult 与 ProcessingTiming；聚合检测、跟踪、可选瞄准和各阶段耗时，不持有第二套算法状态。 |

## 03 检测（auto_buff/buff_detect）（16 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [auto_buff/buff_detect/detection.cpp](../auto_buff/buff_detect/detection.cpp) | buff_detect | 验证 red/blue 输入；按颜色及小/大符模式筛选观测，保留原始网络候选用于诊断。 |
| [auto_buff/buff_detect/detection.hpp](../auto_buff/buff_detect/detection.hpp) | buff_detect | 定义 Detector 接口、RuneMode、Appearance 及 R/板面/原始五点/灯臂/Detection 观测协议。 |
| [auto_buff/buff_detect/traditional/rmcs/compat/thinning.cpp](../auto_buff/buff_detect/traditional/rmcs/compat/thinning.cpp) | buff_detect | 本地实现骨架细化，避免为这一操作引入完整额外 OpenCV 扩展模块。 |
| [auto_buff/buff_detect/traditional/rmcs/compat/thinning.hpp](../auto_buff/buff_detect/traditional/rmcs/compat/thinning.hpp) | buff_detect | 声明 RMCS 适配需要的二值图像细化接口。 |
| [auto_buff/buff_detect/traditional/rmcs/rune.cpp](../auto_buff/buff_detect/traditional/rmcs/rune.cpp) | buff_detect（RMCS 适配） | 执行色差分割、R骨架识别、符板轮廓与几何筛选，生成传统观测；属于实际传统算法。 |
| [auto_buff/buff_detect/traditional/rmcs/rune.hpp](../auto_buff/buff_detect/traditional/rmcs/rune.hpp) | buff_detect（RMCS 适配） | 声明传统 RuneDetector、参数及轮廓/R骨架检测所需结构。 |
| [auto_buff/buff_detect/traditional/traditional.cpp](../auto_buff/buff_detect/traditional/traditional.cpp) | buff_detect | 加载用于物理尺寸筛选的相机参数，把 RMCS 传统检测输出转为公共 Detection；不调用跟踪。 |
| [auto_buff/buff_detect/traditional/traditional.hpp](../auto_buff/buff_detect/traditional/traditional.hpp) | buff_detect | 声明传统检测适配器 TraditionalDetector。 |
| [auto_buff/buff_detect/yolo/pixels/pixel_geometry.cpp](../auto_buff/buff_detect/yolo/pixels/pixel_geometry.cpp) | buff_detect | 以网络点作搜索先验，提取并筛选真实像素轮廓、测量颜色；颜色判定与按已知颜色提轮廓是不同操作。 |
| [auto_buff/buff_detect/yolo/pixels/pixel_geometry.hpp](../auto_buff/buff_detect/yolo/pixels/pixel_geometry.hpp) | buff_detect | 声明红蓝颜色采样、ROI 色差轮廓、板面轮廓选择和灯臂轮廓提取接口。 |
| [auto_buff/buff_detect/yolo/pixels/pixel_refiner.cpp](../auto_buff/buff_detect/yolo/pixels/pixel_refiner.cpp) | buff_detect | 利用实测轴线/轮廓修正板心和端点，按整组精修或回退；不靠对称关系凭空补缺失端点，不修改网络类别。 |
| [auto_buff/buff_detect/yolo/pixels/pixel_refiner.hpp](../auto_buff/buff_detect/yolo/pixels/pixel_refiner.hpp) | buff_detect | 声明板面与网络五点的像素精修接口及成功/回退约定。 |
| [auto_buff/buff_detect/yolo/observation/pose_output.cpp](../auto_buff/buff_detect/yolo/observation/pose_output.cpp) | buff_detect | 执行类别/点分数门限、原图坐标还原、分组去重及三类观测转换；保留 raw_poses，按需调用像素处理。 |
| [auto_buff/buff_detect/yolo/observation/pose_output.hpp](../auto_buff/buff_detect/yolo/observation/pose_output.hpp) | buff_detect | 声明三类五点输出解码、四端点收集和公共观测生成接口。 |
| [auto_buff/buff_detect/yolo/yolo.cpp](../auto_buff/buff_detect/yolo/yolo.cpp) | buff_detect | 加载 ONNX/IR，配置 CPU 推理；完成 letterbox/RGB/NCHW/归一化、推理、解码与观测适配，记录分阶段耗时。 |
| [auto_buff/buff_detect/yolo/yolo.hpp](../auto_buff/buff_detect/yolo/yolo.hpp) | buff_detect | 声明 OpenVINO YoloDetector、推理资源与可配置项。 |

## 04 跟踪、位姿与运动预测（auto_buff/buff_track）（17 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [auto_buff/buff_track/track_types.hpp](../auto_buff/buff_track/track_types.hpp) | buff_track 公共状态 | 定义 RuneState、初始化位姿、拟合结果、EKF 快照、配置和诊断特征；明确像素/米/弧度与状态顺序。 |
| [auto_buff/buff_track/pose/big_activated.cpp](../auto_buff/buff_track/pose/big_activated.cpp) | buff_track | 结合本帧待击板给出的符面位姿、灯臂模板与实测轮廓恢复板心，并验证几何一致性；不是另一个跟踪器。 |
| [auto_buff/buff_track/pose/big_activated.hpp](../auto_buff/buff_track/pose/big_activated.hpp) | buff_track | 声明大符灯臂到符板中心的几何恢复接口。 |
| [auto_buff/buff_track/pose/rune_camera.cpp](../auto_buff/buff_track/pose/rune_camera.cpp) | buff_track | 加载标定、核对图像尺寸、应用相机/云台/IMU 坐标关系，为位姿与投影提供同一相机几何。 |
| [auto_buff/buff_track/pose/rune_camera.hpp](../auto_buff/buff_track/pose/rune_camera.hpp) | buff_track | 声明相机内参、畸变、手眼与 IMU 姿态变换的持有及查询接口。 |
| [auto_buff/buff_track/pose/rune_pose.cpp](../auto_buff/buff_track/pose/rune_pose.cpp) | buff_track | 组织 PnP 种子并求解，检查重投影、符面姿态与机构布局，输出初始化 RunePose。 |
| [auto_buff/buff_track/pose/rune_pose.hpp](../auto_buff/buff_track/pose/rune_pose.hpp) | buff_track | 声明以真实 R 与待击板端点求初始空间位姿的接口。 |
| [auto_buff/buff_track/pose/rune_projection.cpp](../auto_buff/buff_track/pose/rune_projection.cpp) | buff_track | 将 R 和五个板位从机构/世界系变到相机图像，供关联、校正和初始化检查使用。 |
| [auto_buff/buff_track/pose/rune_projection.hpp](../auto_buff/buff_track/pose/rune_projection.hpp) | buff_track | 声明当前状态的机构特征投影及相关几何计算接口。 |
| [auto_buff/buff_track/pose/szu_light_arm_model.hpp](../auto_buff/buff_track/pose/szu_light_arm_model.hpp) | buff_track | 保存深大灯臂的 56 个三维模板点，供大符灯臂几何恢复使用；属于模型常量。 |
| [auto_buff/buff_track/predict/rune_predictor.cpp](../auto_buff/buff_track/predict/rune_predictor.cpp) | buff_track | 维护真实校正样本，比较恒速与正弦拟合并返回预测参数；不独立持有第二套 EKF。 |
| [auto_buff/buff_track/predict/rune_predictor.hpp](../auto_buff/buff_track/predict/rune_predictor.hpp) | buff_track | 声明转角样本窗口、运动拟合与未来特征预测接口。 |
| [auto_buff/buff_track/predict/rune_state.cpp](../auto_buff/buff_track/predict/rune_state.cpp) | buff_track | 实现 RuneState 恒速/正弦积分、预热后的待击板瞄准点与方向前馈，以及状态副本的未来像素投影。声明分别在 track_types.hpp/rune_predictor.hpp。 |
| [auto_buff/buff_track/track/buff_tracker.cpp](../auto_buff/buff_track/track/buff_tracker.cpp) | buff_track | 将公共检测转为 RMCS 特征，组织大符几何恢复、初始化、预测/校正、超时/重置及未来投影；消费本帧姿态。 |
| [auto_buff/buff_track/track/buff_tracker.hpp](../auto_buff/buff_track/track/buff_tracker.hpp) | buff_track | 声明 RmcsTracker 与 TrackResult；管理每帧观测、生命周期和诊断快照。 |
| [auto_buff/buff_track/track/rune_model.cpp](../auto_buff/buff_track/track/rune_model.cpp) | buff_track | 持有六维状态与协方差，执行预测、投影关联、匈牙利分配、马氏代价和观测校正，维护拟合及发散检查。 |
| [auto_buff/buff_track/track/rune_model.hpp](../auto_buff/buff_track/track/rune_model.hpp) | buff_track | 声明 RMCS RuneModel 模型接口，以 PImpl 隐藏 EKF 实现。 |

## 05 火控（auto_buff/buff_shoot）（2 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [auto_buff/buff_shoot/rune_shooter.cpp](../auto_buff/buff_shoot/rune_shooter.cpp) | buff_shoot | 结合运动预测、弹道飞行时间、发射延迟及偏置迭代求瞄准角，按 idle/shoot 节奏给出 should_shoot；不直接发送协议包。 |
| [auto_buff/buff_shoot/rune_shooter.hpp](../auto_buff/buff_shoot/rune_shooter.hpp) | buff_shoot | 声明 RuneShooter、Aimed、时刻更新和重置接口；PImpl 隐藏计算状态。 |

## 06 业务诊断与可视化（auto_buff/debug）（14 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [auto_buff/debug/blueprints/generate.py](../auto_buff/debug/blueprints/generate.py) | 手工布局生成工具 | 生成 1～7 组合的 Rerun .rbl 布局：各模块图像、按同类分组的曲线；仅改布局时需要 Python，运行程序不需要。 |
| [auto_buff/debug/buff_debug.cpp](../auto_buff/debug/buff_debug.cpp) | buff_debug | 按模块 debug 选择绘制内容；复制一份底图并异步消费，实机覆盖旧待处理帧、离线等待保序；计算生产者 Process FPS。 |
| [auto_buff/debug/buff_debug.hpp](../auto_buff/debug/buff_debug.hpp) | buff_debug | 声明诊断快照、异步工作线程、提交接口和队列/耗时统计。 |
| [auto_buff/debug/performance.hpp](../auto_buff/debug/performance.hpp) | 应用直接包含的头文件实现 | PerformanceLog 按 runtime.profile 写 performance.csv；汇总各阶段和 YOLO 细分耗时、输出开销与结果年龄，不依赖 Viewer 上屏。 |
| [auto_buff/debug/report.cpp](../auto_buff/debug/report.cpp) | buff_debug | 写原始候选、适配后观测、模式、跟踪与瞄准诊断；不决定跟踪关联和开火。 |
| [auto_buff/debug/report.hpp](../auto_buff/debug/report.hpp) | buff_debug | 声明每帧检测/跟踪/火控结果 JSONL 序列化接口。 |
| [auto_buff/debug/visualize/annotation.cpp](../auto_buff/debug/visualize/annotation.cpp) | buff_debug | 在普通图像上合成三阶段结果，服务同步标注视频导出；与实时显示后端分开。 |
| [auto_buff/debug/visualize/annotation.hpp](../auto_buff/debug/visualize/annotation.hpp) | buff_debug | 声明用于导出录像的叠图函数。 |
| [auto_buff/debug/visualize/detect_debug.cpp](../auto_buff/debug/visualize/detect_debug.cpp) | buff_debug | 生成检测状态、颜色、原始/精修点与置信度/数量曲线；不运行检测。 |
| [auto_buff/debug/visualize/detect_debug.hpp](../auto_buff/debug/visualize/detect_debug.hpp) | buff_debug | 声明 DetectDebug 检测绘制接口。 |
| [auto_buff/debug/visualize/shoot_debug.cpp](../auto_buff/debug/visualize/shoot_debug.cpp) | buff_debug | 生成瞄准角、云台角、飞行时间、预测时长及开火建议的文字/曲线/图形。 |
| [auto_buff/debug/visualize/shoot_debug.hpp](../auto_buff/debug/visualize/shoot_debug.hpp) | buff_debug | 声明 ShootDebug 火控绘制接口。 |
| [auto_buff/debug/visualize/track_debug.cpp](../auto_buff/debug/visualize/track_debug.cpp) | buff_debug | 生成当前关联前投影、接受观测、未来投影、EKF/拟合/协方差文本及同类曲线；不修改模型。 |
| [auto_buff/debug/visualize/track_debug.hpp](../auto_buff/debug/visualize/track_debug.hpp) | buff_debug | 声明 TrackDebug 跟踪绘制接口。 |

## 07 实机业务执行与采集任务（threads）（9 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [threads/buff_thread/buff_thread.cpp](../threads/buff_thread/buff_thread.cpp) | buff_threads | 实机模式/颜色与姿态有效性处理、最新帧/帧龄控制、detect→track→shoot、串口目标发布；包含业务策略，不只是线程启动器。 |
| [threads/buff_thread/buff_thread.hpp](../threads/buff_thread/buff_thread.hpp) | buff_threads | 声明 FramePose、SynchronizedResult、实机任务入口及发布回调；同时extern引用由apps/main定义的退出标志。 |
| [threads/frame_pool.hpp](../threads/frame_pool.hpp) | 头文件工具 | take_latest_frame 从单生产者/单消费者相机队列丢弃旧帧、取最新帧，限制实时积压。 |
| [threads/hikcamera_thread/hikcamera_thread.cpp](../threads/hikcamera_thread/hikcamera_thread.cpp) | buff_threads | 初始化并消费海康驱动，将 CameraFrame 推入应用 SPSC 队列，响应退出与设备异常。 |
| [threads/hikcamera_thread/hikcamera_thread.hpp](../threads/hikcamera_thread/hikcamera_thread.hpp) | buff_threads | 声明海康采集任务及退出标志。 |
| [threads/serial_thread/serial_thread.cpp](../threads/serial_thread/serial_thread.cpp) | buff_threads | 引用进程级g_shutdown，周期性发送/接收 UserSerial 数据，结束时关闭串口；此全局还被底层串口业务类引用。 |
| [threads/serial_thread/serial_thread.hpp](../threads/serial_thread/serial_thread.hpp) | buff_threads | 声明串口收发服务任务及退出标志。 |
| [threads/usbcamera_thread/usbcamera_thread.cpp](../threads/usbcamera_thread/usbcamera_thread.cpp) | buff_threads | 消费 USB 相机图像并生成 CameraFrame，推入 SPSC 队列，响应退出。 |
| [threads/usbcamera_thread/usbcamera_thread.hpp](../threads/usbcamera_thread/usbcamera_thread.hpp) | buff_threads | 声明 USB 相机采集任务及退出标志。 |

## 08 硬件与串口协议（hardware）（22 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [hardware/common_def.hpp](../hardware/common_def.hpp) | hardware_common | 定义 CameraFrame（图像、曝光/接收时间）及相机公共基类；不包含打符滤波状态。 |
| [hardware/hikcamera/hikcamera.cpp](../hardware/hikcamera/hikcamera.cpp) | hikcamera | 实现设备枚举、打开/配置、启动/停止及资源释放；负责 MVS 生命周期。 |
| [hardware/hikcamera/hikcamera.hpp](../hardware/hikcamera/hikcamera.hpp) | hikcamera | 声明海康相机设备、SDK 资源、抓帧/守护线程、时间映射、缓冲队列与统计。 |
| [hardware/hikcamera/hikcamera_capture.cpp](../hardware/hikcamera/hikcamera_capture.cpp) | hikcamera | 实现 SDK 抓帧、像素转换、曝光时刻映射、图像排队与丢帧统计。 |
| [hardware/hikcamera/hikcamera_reconnect.cpp](../hardware/hikcamera/hikcamera_reconnect.cpp) | hikcamera | 实现设备失连监测、重连与守护循环，不涉及检测算法。 |
| [hardware/serial/common/include/serial/impl/unix.h](../hardware/serial/common/include/serial/impl/unix.h) | serial Linux 底层 | 声明 POSIX 串口底层实现和超时/文件描述符资源。 |
| [hardware/serial/common/include/serial/impl/win.h](../hardware/serial/common/include/serial/impl/win.h) | 随库保留，不参与 Linux 构建 | 声明 Windows 串口底层实现，当前 Linux 目标不使用。 |
| [hardware/serial/common/include/serial/serial.h](../hardware/serial/common/include/serial/serial.h) | serial 第三方接口 | serial 第三方库的跨平台 Serial/Timeout/PortInfo 公共 API。 |
| [hardware/serial/common/include/serial/v8stdint.h](../hardware/serial/common/include/serial/v8stdint.h) | serial 随库兼容头 | 旧编译器兼容的定宽整数类型头；当前 Linux 主要使用标准整数类型。 |
| [hardware/serial/common/src/impl/list_ports/list_ports_linux.cc](../hardware/serial/common/src/impl/list_ports/list_ports_linux.cc) | serial Linux 底层 | Linux 设备枚举与串口设备信息查询。 |
| [hardware/serial/common/src/impl/list_ports/list_ports_win.cc](../hardware/serial/common/src/impl/list_ports/list_ports_win.cc) | 随库保留，不参与 Linux 构建 | Windows 串口设备枚举，当前 Linux CMake 未加入此文件。 |
| [hardware/serial/common/src/impl/unix.cc](../hardware/serial/common/src/impl/unix.cc) | serial Linux 底层 | 实现 POSIX 串口打开/关闭、termios、读写、等待和控制线操作；本轮只读，按用户要求保持原文件。 |
| [hardware/serial/common/src/impl/win.cc](../hardware/serial/common/src/impl/win.cc) | 随库保留，不参与 Linux 构建 | 实现 Windows 串口系统调用，当前 Linux CMake 未加入此文件。 |
| [hardware/serial/common/src/serial.cc](../hardware/serial/common/src/serial.cc) | serial | 公共 Serial 类到平台实现的转发和共享串口参数操作。 |
| [hardware/serial/crc16.hpp](../hardware/serial/crc16.hpp) | serial 协议 | 提供串口帧 CRC16 计算/校验所用表和函数，不解释业务模式。 |
| [hardware/serial/serial_types.hpp](../hardware/serial/serial_types.hpp) | serial 协议 | 定义收发包、VisionMode 和带时间姿态；区分线上 mode 与枚举语义，发送包包含 found。 |
| [hardware/serial/user_serial.cpp](../hardware/serial/user_serial.cpp) | serial | 组帧/拆帧及 CRC、维护姿态历史，解析电控模式，构造 13 字节视觉发送数据；重连等待引用外部 g_shutdown。 |
| [hardware/serial/user_serial.hpp](../hardware/serial/user_serial.hpp) | serial | 声明 UserSerial 单例、收发数据、姿态历史/插值、新鲜度、模式和目标发布接口。 |
| [hardware/usbcamera/usbcamera.cpp](../hardware/usbcamera/usbcamera.cpp) | usbcamera | 通过 OpenCV VideoCapture 打开设备、设置采集属性并返回图像。 |
| [hardware/usbcamera/usbcamera.hpp](../hardware/usbcamera/usbcamera.hpp) | usbcamera | 声明普通 USB 相机读帧和配置接口。 |
| [hardware/usbcamera/usbcamera_config.cpp](../hardware/usbcamera/usbcamera_config.cpp) | usbcamera | 从 YAML 读取 USB 设备、尺寸、帧率等参数，供实机入口与设备任务使用。 |
| [hardware/usbcamera/usbcamera_config.hpp](../hardware/usbcamera/usbcamera_config.hpp) | usbcamera | 声明 USB 相机配置数据与加载函数。 |

## 09 公共数学、图像与机器人数据（rm_utils）（23 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [rm_utils/algorithm/rune_energy_fitter.cpp](../rm_utils/algorithm/rune_energy_fitter.cpp) | rm_utils | 执行真实时间序列的线性和正弦拟合，返回代价和运动参数，不持有上层跟踪生命周期。 |
| [rm_utils/algorithm/rune_energy_fitter.hpp](../rm_utils/algorithm/rune_energy_fitter.hpp) | rm_utils | 声明打符恒速/正弦转角拟合接口及结果。 |
| [auto_buff/buff_track/pose/rune_pnp.cpp](../auto_buff/buff_track/pose/rune_pnp.cpp) | auto_buff | 用固定能量机关点位求相对相机位姿，并转换 RMCS 坐标约定。 |
| [auto_buff/buff_track/pose/rune_pnp.hpp](../auto_buff/buff_track/pose/rune_pnp.hpp) | auto_buff | 声明打符单板几何的 PnP 结果与求解接口。 |
| [rm_utils/algorithm/conversion.hpp](../rm_utils/algorithm/conversion.hpp) | 头文件公共能力/领域数据 | 头文件提供 OpenCV 右/下/前与 RMCS/ROS 前/左/上位置、旋转的轴变换。 |
| [rm_utils/algorithm/hungarian.hpp](../rm_utils/algorithm/hungarian.hpp) | 头文件公共能力/领域数据 | 头文件实现方阵最小代价匈牙利分配；不决定业务门限。 |
| [rm_utils/algorithm/kalman.hpp](../rm_utils/algorithm/kalman.hpp) | 头文件公共能力/领域数据 | 头文件封装 RMCS 协方差预测及 Joseph 形式校正公式；不是另一套完整 EKF 跟踪器。 |
| [rm_utils/algorithm/mahalanobis.hpp](../rm_utils/algorithm/mahalanobis.hpp) | 头文件公共能力/领域数据 | 头文件计算创新量的平方马氏距离，数值非有限时返回空。 |
| [rm_utils/algorithm/math_tools.cpp](../rm_utils/algorithm/math_tools.cpp) | rm_utils | 实现 MAS 风格公共数学运算；角度范围、坐标变换等由各算法复用。 |
| [rm_utils/algorithm/math_tools.hpp](../rm_utils/algorithm/math_tools.hpp) | rm_utils | 声明角度制转换、limit_rad、欧拉角/四元数/旋转、xyz/ypd 与雅可比等公共数学函数。 |
| [rm_utils/algorithm/orientation.hpp](../rm_utils/algorithm/orientation.hpp) | 头文件公共能力/领域数据 | 头文件实现四元数最短路径/夹紧插值及 IMU 本体系到云台参考基的变换。 |
| [rm_utils/algorithm/reprojection.cpp](../rm_utils/algorithm/reprojection.cpp) | rm_utils | 仅应用内参和畸变投影三维点；不负责世界/相机姿态转换。 |
| [rm_utils/algorithm/reprojection.hpp](../rm_utils/algorithm/reprojection.hpp) | rm_utils | 声明已在 OpenCV 相机坐标系下的点投影接口。 |
| [rm_utils/algorithm/trajectory.cpp](../rm_utils/algorithm/trajectory.cpp) | rm_utils | 实现含阻力参数的弹道求解，供火控使用；不判断当前目标是否允许开火。 |
| [rm_utils/algorithm/trajectory.hpp](../rm_utils/algorithm/trajectory.hpp) | rm_utils | 声明弹道 Trajectory 及飞行时间/俯仰角求解接口。 |
| [rm_utils/clock.hpp](../rm_utils/clock.hpp) | 头文件公共能力/领域数据 | 定义 steady_clock 时间类型与 elapsed_ms，同时提供线程局部回放时刻，供预热/火控统一取时。 |
| [auto_buff/buff_detect/traditional/process.cpp](../auto_buff/buff_detect/traditional/process.cpp) | auto_buff | 实现颜色通道提取/差分，为传统检测提供输入；与 YOLO 局部轮廓策略分工不同。 |
| [auto_buff/buff_detect/traditional/process.hpp](../auto_buff/buff_detect/traditional/process.hpp) | auto_buff | 声明按 CampColor 提取红/蓝通道的公共图像处理接口。 |
| [rm_utils/pimpl.hpp](../rm_utils/pimpl.hpp) | 头文件公共能力/领域数据 | 提供 RMCS PImpl 声明宏，供 RuneModel 和 RuneShooter 隐藏实现及禁止复制。 |
| [auto_buff/buff_types.hpp](../auto_buff/buff_types.hpp) | 打符基础类型 | 颜色枚举、物理尺寸、五点模型、几何观测及带前馈的瞄准点。 |
| [auto_buff/buff_detect/yolo/timing.hpp](../auto_buff/buff_detect/yolo/timing.hpp) | YOLO 诊断 | 定义线程局部 YOLO 分段计时、TimedZone 和 Tracy 生命周期控制。 |

## 10 媒体与配对 IO（rm_utils/io）（11 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [rm_utils/io/media_reader.cpp](../rm_utils/io/media_reader.cpp) | buff_io | 读取图片/视频、给出媒体帧号/时间与帧率，处理资源路径和文件检查。 |
| [rm_utils/io/media_reader.hpp](../rm_utils/io/media_reader.hpp) | buff_io | 声明 Frame、MediaReader 和 UTF-8 路径转换等 IO 辅助。 |
| [rm_utils/io/media_recorder.cpp](../rm_utils/io/media_recorder.cpp) | buff_io | 按调用顺序同步编码 PNG/视频并写 timestamps.csv；维护 MP4/AVI 格式选择和收尾。 |
| [rm_utils/io/media_recorder.hpp](../rm_utils/io/media_recorder.hpp) | buff_io | 声明同步 MediaRecorder、会话目录、原图/叠图/时间表输出。 |
| [rm_utils/io/paired_frame.hpp](../rm_utils/io/paired_frame.hpp) | buff_io 头文件协议 | 定义 RecordedFrame 与 frames.csv 表头/单行写入：曝光/接收纳秒、wxyz、mode、pose_valid。 |
| [rm_utils/io/paired_player.cpp](../rm_utils/io/paired_player.cpp) | buff_io | 校验 CSV 字段、时间/姿态/mode 和帧数，按序配对视频或图片序列；不运行检测/跟踪。 |
| [rm_utils/io/paired_player.hpp](../rm_utils/io/paired_player.hpp) | buff_io | 声明成对 Player、元数据加载及逐帧读取接口。 |
| [rm_utils/io/paired_recorder.cpp](../rm_utils/io/paired_recorder.cpp) | buff_io | 复制入队的图像和姿态，由同一写入线程成对写图像/frames.csv；统计入帧率、保存率、队列峰值和编码耗时。 |
| [rm_utils/io/paired_recorder.hpp](../rm_utils/io/paired_recorder.hpp) | buff_io | 声明带容量限制的后台 PairedRecorder、错误传播与录制统计。 |
| [rm_utils/io/playback_control.hpp](../rm_utils/io/playback_control.hpp) | 头文件控制状态 | 处理空格暂停/继续、n 单步、q/ESC 退出，只控制是否取下一帧，不改变记录时间。 |
| [rm_utils/io/runtime_path.hpp](../rm_utils/io/runtime_path.hpp) | Linux 头文件路径工具 | 从 /proc/self/exe 推导项目根，定位项目/共享用户虚拟环境；包含“项目/build/程序”的交付约定。 |

## 11 显示后端（rm_utils/display）（6 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [rm_utils/display/display.cpp](../rm_utils/display/display.cpp) | buff_display | 按 backend 管理 SDL/Rerun 生命周期、图像采样和提交；SDL 可同时保留 RRD 数值记录。 |
| [rm_utils/display/display.hpp](../rm_utils/display/display.hpp) | buff_display | 声明Display单例门面、通用DisplayView布局和图元、数值记录、时间与主线程轮询接口。 |
| [rm_utils/display/rerun/rerun_display.cpp](../rm_utils/display/rerun/rerun_display.cpp) | buff_display | 启动/连接Viewer、写RRD、底图/HUD及数值/空间数据；消费调用方提供的布局与HUD尺寸，不识别打符模块。 |
| [rm_utils/display/rerun/rerun_display.hpp](../rm_utils/display/rerun/rerun_display.hpp) | buff_display | 声明 Rerun 会话、图像/叠图/曲线记录与布局接口。 |
| [rm_utils/display/sdl/sdl_display.cpp](../rm_utils/display/sdl/sdl_display.cpp) | buff_display | 为 detect/track/shoot 分别创建系统窗口，更新纹理、处理窗口/键盘事件；保留最新显示内容，GUI 操作在主线程。 |
| [rm_utils/display/sdl/sdl_display.hpp](../rm_utils/display/sdl/sdl_display.hpp) | buff_display | 声明 SDL 资源、模块窗口、待显示帧与主线程轮询接口。 |

## 12 构建与环境（22 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [CMakeLists.txt](../CMakeLists.txt) | 根构建入口 | 限定 Linux/C++23/GCC≥13，设置 Release 和选项，导入依赖并显式添加各层；程序统一输出 build 根。 |
| [apps/CMakeLists.txt](../apps/CMakeLists.txt) | buff_app / mas_buff | 构建五个应用流程实现的 buff_app，再用 main.cpp 生成 mas_buff。 |
| [auto_buff/CMakeLists.txt](../auto_buff/CMakeLists.txt) | 算法/编排/诊断目标 | 分别创建三算法库、buff_modules、buff_replay、buff_debug；由诊断目标嵌入打符blueprints。 |
| [auto_buff/buff_detect/CMakeLists.txt](../auto_buff/buff_detect/CMakeLists.txt) | buff_detect | 显式登记传统和 YOLO 检测源码及 OpenVINO/公共工具依赖。 |
| [auto_buff/buff_shoot/CMakeLists.txt](../auto_buff/buff_shoot/CMakeLists.txt) | buff_shoot | 构建火控并依赖 buff_track；不链接串口设备。 |
| [auto_buff/buff_track/CMakeLists.txt](../auto_buff/buff_track/CMakeLists.txt) | buff_track | 显式登记 pose/track/predict 源码，链接公共数学和工具；检测类型依赖还体现在头文件 include。 |
| [build_linux.sh](../build_linux.sh) | 便捷构建脚本 | 校验 Linux 后在项目 build 配置/并行构建；可透传 CMake 参数，不负责安装全部依赖。 |
| [cmake/Dependencies.cmake](../cmake/Dependencies.cmake) | 依赖装配 | 选择本地/系统 yaml-cpp、spdlog、Eigen，并查找 OpenCV、SDL2、OpenVINO 和线程；当前不是按运行后端裁剪。 |
| [cmake/LocalDependencies.cmake](../cmake/LocalDependencies.cmake) | 离线源码依赖 | 为 Rerun/Arrow 及 xsimd/mimalloc/Boost 设置项目内归档路径，减少构建时联网下载。 |
| [cmake/OpenVINO.cmake](../cmake/OpenVINO.cmake) | OpenVINO 查找 | 优先显式目录/已有缓存，再项目和共享虚拟环境，最后系统 SDK；失败给出安装与定位提示。 |
| [environment/setup_linux.sh](../environment/setup_linux.sh) | 一次性环境准备 | 在 XDG_DATA_HOME 或 ~/.local/share/mas-vision/venv 安装固定 OpenVINO，独立于源码解压目录。 |
| [environment/use_local_dependencies.sh](../environment/use_local_dependencies.sh) | 可 source 的环境辅助 | 导出项目内已下载归档的依赖地址；供已有构建使用，不修改算法。 |
| [hardware/CMakeLists.txt](../hardware/CMakeLists.txt) | hardware_common / buff_hardware | 提供硬件公共接口目标，添加USB/串口及必需的海康库，再聚合为 buff_hardware。 |
| [hardware/hikcamera/CMakeLists.txt](../hardware/hikcamera/CMakeLists.txt) | hikcamera | 固定查找真实MVS SDK、检测生命周期API并构建海康；缺失SDK时配置失败。 |
| [hardware/serial/CMakeLists.txt](../hardware/serial/CMakeLists.txt) | serial | 构建业务串口和 POSIX 底层，显式选择 unix.cc/Linux 枚举；不选择 Windows 文件。 |
| [hardware/usbcamera/CMakeLists.txt](../hardware/usbcamera/CMakeLists.txt) | usbcamera | 构建 USB 相机及其 YAML 配置读取实现。 |
| [rm_utils/CMakeLists.txt](../rm_utils/CMakeLists.txt) | rm_utils + 子目标 | 构建公共数学/图像与 buff 专用数学，并分别添加 IO、显示子目标。 |
| [rm_utils/display/CMakeLists.txt](../rm_utils/display/CMakeLists.txt) | buff_display | 构建sdl/与rerun/中的后端及统一Display，只链接图像/SDL/Rerun依赖；不承担业务布局生成或YAML会话配置。 |
| [rm_utils/io/CMakeLists.txt](../rm_utils/io/CMakeLists.txt) | buff_io | 仅构建四种媒体/配对 IO 实现，依赖基础线程/图像/配置能力。 |
| [tests/CMakeLists.txt](../tests/CMakeLists.txt) | 测试登记 | BUILD_TESTING=ON 时构建测试与 yolo_profile，登记 14 项 CTest；部分 Python 检查单独运行。 |
| [third_party/CMakeLists.txt](../third_party/CMakeLists.txt) | buff_utils / Tracy / rerun_sdk | 构建日志包装并公开线程池/队列头，接入 Tracy 客户端与真正的 Rerun SDK。 |
| [threads/CMakeLists.txt](../threads/CMakeLists.txt) | buff_threads | 把相机/串口/实机业务任务聚为一个静态库，依赖硬件与算法装配。 |

## 13 配置（config）（7 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [config/auto_buff.yaml](../config/auto_buff.yaml) | 运行配置，不编译 | 唯一打符业务配置：auto_buff下检测、跟踪、火控及runtime、display。 |
| [config/hikcamera.yaml](../config/hikcamera.yaml) | 运行配置，不编译 | 海康设备与曝光/增益/采集设置，以及图像几何、内参/畸变/手眼等标定数据。 |
| [config/read.yaml](../config/read.yaml) | 运行配置，不编译 | 普通媒体输入与导出参数；算法统一读取auto_buff.yaml。 |
| [config/record.yaml](../config/record.yaml) | 运行配置，不编译 | 独立采集：录像/图片、队列容量、输出和预览；调用处传入固定设备配置路径，不运行算法。 |
| [config/replay.yaml](../config/replay.yaml) | 运行配置，不编译 | 成对回放/配对检查：媒体、frames.csv、算法配置、速度、显示及导出路径。 |
| [config/serial_config.yaml](../config/serial_config.yaml) | 运行配置，不编译 | 串口设备路径、通信与姿态数据相关参数；具体业务包结构由 serial_types.hpp 定义。 |
| [config/usbcamera_config.yaml](../config/usbcamera_config.yaml) | 运行配置，不编译 | USB 相机设备与采集属性。 |

## 14 性能、量化与模型检查工具（tools）（4 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [tools/benchmark_cpu_scheduling.py](../tools/benchmark_cpu_scheduling.py) | 独立 Python 工具 | 使用同一模型按 ANY/PCORE/PCORE/ANY 顺序比较 OpenVINO CPU 推理调度并输出 JSON；不是端到端 FPS。 |
| [tools/compare_rune_quantization.py](../tools/compare_rune_quantization.py) | 独立 Python 工具 | 比较 FP32/INT8 在同一公开画面的分数、点位、候选漂移和推理耗时；无真值标注，不能作为准确率验收。 |
| [tools/quantize_rune_public.py](../tools/quantize_rune_public.py) | 独立 Python 工具 | 从公开视频采样，执行实验性 INT8 PTQ 并导出 XML/BIN；不自动替换实机配置。 |
| [tools/summarize_performance.py](../tools/summarize_performance.py) | 独立 Python 工具 | 读取 performance.csv，去除预热后统计真实处理率及各耗时均值/分位数；不把 Viewer 帧率当算法帧率。 |

## 15 当前 C++ 回归、设备检查与 Python 辅助（tests）（24 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [tests/auto_buff/buff_debug_test.cpp](../tests/auto_buff/buff_debug_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 异步诊断快照所有权、源缓冲复用、离线保序与收尾检查。 |
| [tests/rm_utils/display/display_test.cpp](../tests/rm_utils/display/display_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 显示适配、叠图与 RRD 等诊断输出行为检查。 |
| [tests/apps/playback_control_test.cpp](../tests/apps/playback_control_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 暂停/继续/单步/退出状态机检查。 |
| [tests/apps/replay_test.cpp](../tests/apps/replay_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 成对录制/回放/检查流程、时间与姿态缺失等回归。 |
| [tests/rm_utils/display/sdl_display_test.cpp](../tests/rm_utils/display/sdl_display_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | SDL dummy/software 环境检查三个独立窗口、图像内容、关窗与控制键行为。 |
| [tests/auto_buff/detect_test.cpp](../tests/auto_buff/detect_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 检测回归入口，组织传统检测与当前 YOLO 解码/像素处理检查。 |
| [tests/auto_buff/pose_test.cpp](../tests/auto_buff/pose_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 相机坐标关系、PnP、投影及大符灯臂恢复的合成几何检查。 |
| [tests/auto_buff/predict_test.cpp](../tests/auto_buff/predict_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 恒速/正弦拟合与未来积分/预测行为检查。 |
| [tests/auto_buff/shoot_test.cpp](../tests/auto_buff/shoot_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 瞄准计算、飞行/延迟预测与火控输出行为检查，不发送真实开火命令。 |
| [tests/auto_buff/track_test.cpp](../tests/auto_buff/track_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 跟踪初始化、生命周期、观测校正/缺失与重置行为检查。 |
| [tests/auto_buff/traditional_check.cpp](../tests/auto_buff/traditional_check.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 传统检测的合成灯效、骨架/轮廓与物理几何筛选检查。 |
| [tests/auto_buff/yolo_check.cpp](../tests/auto_buff/yolo_check.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 当前三类五点解码、端点/类别规则、颜色、像素精修与输出报告检查。 |
| [tests/auto_buff/yolo_pose.yaml](../tests/auto_buff/yolo_pose.yaml) | 可选测试/检查，不进入 mas_buff 主流程 | 当前 YOLO 离线示例参数；媒体/模型文件需在本机存在。 |
| [tests/cmake/test_openvino_discovery.py](../tests/cmake/test_openvino_discovery.py) | 可选测试/检查，不进入 mas_buff 主流程 | 独立构造环境场景验证 OpenVINO CMake 发现逻辑；不在当前 CTest 列表自动登记。 |
| [tests/common/synthetic_rune.hpp](../tests/common/synthetic_rune.hpp) | 可选测试/检查，不进入 mas_buff 主流程 | 生成已知相机/空间几何的能量机关合成观测，用于位姿/跟踪回归。 |
| [tests/common/test.hpp](../tests/common/test.hpp) | 可选测试/检查，不进入 mas_buff 主流程 | 测试断言、运行包装、测试资源路径与临时目录等公共辅助。 |
| [tools/serial_test.cpp](../tools/serial_test.cpp) | 手动诊断工具 | 连接真实串口，接收并打印模式与姿态。 |
| [tools/buff_preview.cpp](../tools/buff_preview.cpp) | 手动预览工具 | 读取配置指定的图片／视频并显示生产检测结果。 |
| [tools/camera_test.cpp](../tools/camera_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 相机人工诊断，显式配置后访问真实设备并可显示画面。 |
| [tests/hardware/frame_pool_test.cpp](../tests/hardware/frame_pool_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 验证队列只消费最新帧及空队列等行为。 |
| [tests/hardware/serial_protocol_test.cpp](../tests/hardware/serial_protocol_test.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | 串口协议回归，断言包布局与 CRC16 参考结果。 |
| [tools/benchmark_model.py](../tools/benchmark_model.py) | 可选测试/检查，不进入 mas_buff 主流程 | 调用官方 benchmark_app，分别测延迟、吞吐或逐层统计，保存原命令和日志。 |
| [tools/check_code.py](../tools/check_code.py) | 可选测试/检查，不进入 mas_buff 主流程 | 仅对显式指定的自有 C++ 文件执行格式/clang-tidy/cppcheck，不重写源码。 |
| [tools/inspect_model.py](../tools/inspect_model.py) | 可选测试/检查，不进入 mas_buff 主流程 | 读取 OpenVINO 模型输入输出，保存模型信息；可选启动本地 Netron。 |
| [tools/yolo_profile.cpp](../tools/yolo_profile.cpp) | 可选测试/检查，不进入 mas_buff 主流程 | C++ YOLO 独立性能入口，复用生产检测器输出阶段耗时并可配合 Tracy。 |

## 16 历史测试工具（不能替代当前三类模型验收）（3 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |

## 17 放在 third_party 的日志包装与独立库入口（4 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [third_party/SPSCQueue/SPSCQueue.h](../third_party/SPSCQueue/SPSCQueue.h) | 第三方单头库 | rigtorp 单生产者/单消费者队列实现，用于相机帧传递；不等价于多生产者通用队列。 |
| [third_party/mas_log/mas_log.cpp](../third_party/mas_log/mas_log.cpp) | buff_utils（MAS 包装） | 配置控制台/文件日志 sink、级别与 logger；供硬件、线程和应用复用。 |
| [third_party/mas_log/mas_log.hpp](../third_party/mas_log/mas_log.hpp) | buff_utils（MAS 包装） | 声明 MasLog 单例日志与 MAS_LOG_* 宏；属于项目/MAS 包装，不是 spdlog SDK 本体。 |
| [third_party/thread_pool/BS_thread_pool.hpp](../third_party/thread_pool/BS_thread_pool.hpp) | 第三方单头库 | BS 线程池实现；应用提交相机/串口/业务长期任务。以库入口说明，不逐个内部模板展开。 |

## 18 训练与标注侧 Python（rm，不进入运行构建）（49 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [rm/rune_pose/tools/check_pose.py](../rm/rune_pose/tools/check_pose.py) | 标注/训练工具或归档脚本，不进入 CMake | 校验姿态标签、图像哈希、可见性和数据会话隔离。 |
| [rm/rune_pose/tools/check_training_tools.py](../rm/rune_pose/tools/check_training_tools.py) | 标注/训练工具或归档脚本，不进入 CMake | 不用真实训练验证设备选择、固定接口、数据划分和曲线读取。 |
| [rm/rune_pose/tools/export_pose.py](../rm/rune_pose/tools/export_pose.py) | 标注/训练工具或归档脚本，不进入 CMake | 外部训练后导出 FP32 ONNX、FP16/INT8 OpenVINO，量化只用训练集校准。 |
| [rm/rune_pose/tools/plot_training.py](../rm/rune_pose/tools/plot_training.py) | 标注/训练工具或归档脚本，不进入 CMake | 绘制真实数据分布与训练 results.csv，不编造曲线。 |
| [rm/rune_pose/tools/train_pose.py](../rm/rune_pose/tools/train_pose.py) | 标注/训练工具或归档脚本，不进入 CMake | 旧五类五点 640 输入训练入口，提供 dry-run 训练计划。 |
| [rm/tools/annotate_video_frames.py](../rm/tools/annotate_video_frames.py) | 标注/训练工具或归档脚本，不进入 CMake | 给抽取的视频帧制作/绘制五类五点标注。 |
| [rm/tools/audit_pose_states.py](../rm/tools/audit_pose_states.py) | 标注/训练工具或归档脚本，不进入 CMake | 审查训练样本状态并生成清单/预览，不修改原始图片与标签。 |
| [rm/tools/check_halo_rejection.py](../rm/tools/check_halo_rejection.py) | 标注/训练工具或归档脚本，不进入 CMake | 回归检查红蓝光晕不替代真实灯珠核心。 |
| [rm/tools/inspect_partial.py](../rm/tools/inspect_partial.py) | 标注/训练工具或归档脚本，不进入 CMake | 带原图坐标绘制边缘截断目标，供逐点复核。 |
| [rm/tools/inspect_pose_cores.py](../rm/tools/inspect_pose_cores.py) | 标注/训练工具或归档脚本，不进入 CMake | 检查基于灯珠核心的端点，与旧版标注对照。 |
| [rm/tools/label_small_activated.py](../rm/tools/label_small_activated.py) | 标注/训练工具或归档脚本，不进入 CMake | 测量可见小符已激活圆环灯珠生成草稿，不按完整圆补点。 |
| [rm/tools/match_pose_examples.py](../rm/tools/match_pose_examples.py) | 标注/训练工具或归档脚本，不进入 CMake | 在数据集中定位用户截图对应的实际目标裁剪。 |
| [rm/tools/package_pose.py](../rm/tools/package_pose.py) | 标注/训练工具或归档脚本，不进入 CMake | 打包训练数据并核对 ZIP 条目，不训练模型。 |
| [rm/tools/prepare_pose.py](../rm/tools/prepare_pose.py) | 标注/训练工具或归档脚本，不进入 CMake | 从原图可见灯珠测量五点，准备姿态训练标签，旧框仅作搜索先验。 |
| [rm/tools/prepare_szu.py](../rm/tools/prepare_szu.py) | 标注/训练工具或归档脚本，不进入 CMake | 准备三类五点迁移候选；不把未经审阅的大/小符候选直接当训练真值。 |
| [rm/tools/review_partial.py](../rm/tools/review_partial.py) | 标注/训练工具或归档脚本，不进入 CMake | 对已选边缘目标的真实可见像素作人工窗口辅助测量。 |
| [rm/tools/review_selected_points.py](../rm/tools/review_selected_points.py) | 标注/训练工具或归档脚本，不进入 CMake | 应用人工确认的点位修正，区分灯珠核心与末端灯条。 |
| [rm/tools/verify_pose_revision.py](../rm/tools/verify_pose_revision.py) | 标注/训练工具或归档脚本，不进入 CMake | 核对轴线中心修订并生成前后对照。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/calib_sample.py](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/calib_sample.py) | 标注/训练工具或归档脚本，不进入 CMake | 从训练池按采集组/类别分层抽取量化校准图，固定种子并保存清单，不用验证集校准。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/check_env.py](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/check_env.py) | 标注/训练工具或归档脚本，不进入 CMake | 检查训练/导出环境与真实 GPU 内核执行能力，不以设备可见代替可训练。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/common.py](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/common.py) | 标注/训练工具或归档脚本，不进入 CMake | 旧五类 YOLO11 交付工具共用的路径、letterbox、解码、NMS 和模型签名，不是当前三类 C++ 接口。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/export_fp16.py](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/export_fp16.py) | 标注/训练工具或归档脚本，不进入 CMake | 导出旧五类 640 模型的 FP16 ONNX/OpenVINO，保持约定的输入输出类型。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/export_fp32.py](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/export_fp32.py) | 标注/训练工具或归档脚本，不进入 CMake | 导出旧五类 640 模型的 FP32 ONNX，并记录模型接口签名。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/plot_curves.py](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/plot_curves.py) | 标注/训练工具或归档脚本，不进入 CMake | 读取真实训练 CSV，绘制损失/检测/关键点/学习率曲线。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/quantize_int8.py](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/quantize_int8.py) | 标注/训练工具或归档脚本，不进入 CMake | 对旧五类 FP32 模型执行 NNCF PTQ，导出 rune_pose_best_int8_640.xml/bin 和量化统计。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/smoke_train.py](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/smoke_train.py) | 标注/训练工具或归档脚本，不进入 CMake | 少量轮次试训，记录每轮时间、显存和 GPU 利用率。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/verify_models.py](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/verify_models.py) | 标注/训练工具或归档脚本，不进入 CMake | 比较旧五类 Torch/ONNX/OpenVINO 不同精度原始输出及标注上的关键点/分类指标。 |
| [rm/训练数据/2026_10_31/session/diagnose_evaluation.py](../rm/训练数据/2026_10_31/session/diagnose_evaluation.py) | 标注/训练工具或归档脚本，不进入 CMake | 诊断该归档训练会话的评估结果。 |
| [rm/训练数据/2026_10_31/session/finalize_results.py](../rm/训练数据/2026_10_31/session/finalize_results.py) | 标注/训练工具或归档脚本，不进入 CMake | 整理该归档会话结果与模型/文件哈希清单。 |
| [rm/训练数据/2026_10_31/session/keep_training_awake.py](../rm/训练数据/2026_10_31/session/keep_training_awake.py) | 标注/训练工具或归档脚本，不进入 CMake | 训练期间维持系统唤醒的会话辅助，不参与推理。 |
| [rm/训练数据/2026_10_31/session/preflight.py](../rm/训练数据/2026_10_31/session/preflight.py) | 标注/训练工具或归档脚本，不进入 CMake | 该训练会话开始前的环境/输入预检查。 |
| [rm/训练数据/2026_10_31/source/tools/check.py](../rm/训练数据/2026_10_31/source/tools/check.py) | 标注/训练工具或归档脚本，不进入 CMake | 以合成输入检查预训练骨干与自定义输出接口。 |
| [rm/训练数据/2026_10_31/source/tools/data_trial.py](../rm/训练数据/2026_10_31/source/tools/data_trial.py) | 标注/训练工具或归档脚本，不进入 CMake | 加载已审阅/显式试验标签、letterbox 与训练数据批处理；不同归档版本细节不同。 |
| [rm/训练数据/2026_10_31/source/tools/evaluate.py](../rm/训练数据/2026_10_31/source/tools/evaluate.py) | 标注/训练工具或归档脚本，不进入 CMake | 在原图像素坐标评估网络原始五点并绘制预览，不带精修/track。 |
| [rm/训练数据/2026_10_31/source/tools/evaluate_test.py](../rm/训练数据/2026_10_31/source/tools/evaluate_test.py) | 标注/训练工具或归档脚本，不进入 CMake | 针对解码、匹配与评估指标的合成回归。 |
| [rm/训练数据/2026_10_31/source/tools/export.py](../rm/训练数据/2026_10_31/source/tools/export.py) | 标注/训练工具或归档脚本，不进入 CMake | 导出固定 FP32 ONNX，可选 FP16/INT8；校准使用训练划分。 |
| [rm/训练数据/2026_10_31/source/tools/model_trial.py](../rm/训练数据/2026_10_31/source/tools/model_trial.py) | 标注/训练工具或归档脚本，不进入 CMake | 定义 YOLO26n 骨干和自定义无框五点头/损失；并非深大未公开训练代码的原样复刻。 |
| [rm/训练数据/2026_10_31/source/tools/plot.py](../rm/训练数据/2026_10_31/source/tools/plot.py) | 标注/训练工具或归档脚本，不进入 CMake | 根据真实训练 CSV 生成曲线。 |
| [rm/训练数据/2026_10_31/source/tools/train_trial.py](../rm/训练数据/2026_10_31/source/tools/train_trial.py) | 标注/训练工具或归档脚本，不进入 CMake | 在显式选择的设备训练五点试验模型，未经审阅数据需显式启用。 |
| [rm/训练数据/rune_szu_cuda_20261030/small_activated_review/method_source.py](../rm/训练数据/rune_szu_cuda_20261030/small_activated_review/method_source.py) | 标注/训练工具或归档脚本，不进入 CMake | 归档的小符已激活可见灯珠测量方法，与标注草稿工具对应。 |
| [rm/训练数据/rune_szu_cuda_20261030/tools/check.py](../rm/训练数据/rune_szu_cuda_20261030/tools/check.py) | 标注/训练工具或归档脚本，不进入 CMake | 以合成输入检查预训练骨干与自定义输出接口。 |
| [rm/训练数据/rune_szu_cuda_20261030/tools/data_trial.py](../rm/训练数据/rune_szu_cuda_20261030/tools/data_trial.py) | 标注/训练工具或归档脚本，不进入 CMake | 加载已审阅/显式试验标签、letterbox 与训练数据批处理；不同归档版本细节不同。 |
| [rm/训练数据/rune_szu_cuda_20261030/tools/evaluate.py](../rm/训练数据/rune_szu_cuda_20261030/tools/evaluate.py) | 标注/训练工具或归档脚本，不进入 CMake | 在原图像素坐标评估网络原始五点并绘制预览，不带精修/track。 |
| [rm/训练数据/rune_szu_cuda_20261030/tools/evaluate_test.py](../rm/训练数据/rune_szu_cuda_20261030/tools/evaluate_test.py) | 标注/训练工具或归档脚本，不进入 CMake | 针对解码、匹配与评估指标的合成回归。 |
| [rm/训练数据/rune_szu_cuda_20261030/tools/export.py](../rm/训练数据/rune_szu_cuda_20261030/tools/export.py) | 标注/训练工具或归档脚本，不进入 CMake | 导出固定 FP32 ONNX，可选 FP16/INT8；校准使用训练划分。 |
| [rm/训练数据/rune_szu_cuda_20261030/tools/model_trial.py](../rm/训练数据/rune_szu_cuda_20261030/tools/model_trial.py) | 标注/训练工具或归档脚本，不进入 CMake | 定义 YOLO26n 骨干和自定义无框五点头/损失；并非深大未公开训练代码的原样复刻。 |
| [rm/训练数据/rune_szu_cuda_20261030/tools/plot.py](../rm/训练数据/rune_szu_cuda_20261030/tools/plot.py) | 标注/训练工具或归档脚本，不进入 CMake | 根据真实训练 CSV 生成曲线。 |
| [rm/训练数据/rune_szu_cuda_20261030/tools/train_trial.py](../rm/训练数据/rune_szu_cuda_20261030/tools/train_trial.py) | 标注/训练工具或归档脚本，不进入 CMake | 在显式选择的设备训练五点试验模型，未经审阅数据需显式启用。 |
| [rm/训练数据/rune_szu_cuda_20261030/tools/training_test.py](../rm/训练数据/rune_szu_cuda_20261030/tools/training_test.py) | 标注/训练工具或归档脚本，不进入 CMake | 检查跨尺度监督、仿射标签和模型选择行为。 |

## 19 训练包启动脚本与参考源码（rm，不进入运行构建）（9 个文件）

| 文件 | 所属目标／使用状态 | 实际职责 |
| --- | --- | --- |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/run_all.ps1](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/run_all.ps1) | 归档工具/参考副本 | 旧 Windows 五类训练交付包的检查、试训、训练、导出、量化和验收总脚本。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/setup_env.ps1](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/setup_env.ps1) | 归档工具/参考副本 | 旧 Windows 训练环境准备脚本，不用于当前 Linux 运行环境。 |
| [rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/train.ps1](../rm/交付_rune_pose_yolo11n_5cls_640_20261025/tools/train.ps1) | 归档工具/参考副本 | 旧 Windows 单卡正式训练启动脚本，不参与 mas_buff 构建。 |
| [rm/训练数据/rune_szu_cuda_20261030/reference/pose_output.cpp](../rm/训练数据/rune_szu_cuda_20261030/reference/pose_output.cpp) | 归档工具/参考副本 | 归档时的三类解码/观测转换参考实现；不可当作当前生产版本直接修改。 |
| [rm/训练数据/rune_szu_cuda_20261030/reference/pose_output.hpp](../rm/训练数据/rune_szu_cuda_20261030/reference/pose_output.hpp) | 归档工具/参考副本 | 归档时的三类五点解码接口参考副本；不由当前运行工程 CMake 编译。 |
| [rm/训练数据/rune_szu_cuda_20261030/reference/yolo.cpp](../rm/训练数据/rune_szu_cuda_20261030/reference/yolo.cpp) | 归档工具/参考副本 | 归档时的 OpenVINO 预处理/推理参考实现；与当前 auto_buff 版本分开。 |
| [rm/训练数据/rune_szu_cuda_20261030/reference/yolo.hpp](../rm/训练数据/rune_szu_cuda_20261030/reference/yolo.hpp) | 归档工具/参考副本 | 归档时的 OpenVINO 检测器接口参考副本。 |
| [rm/训练数据/rune_szu_cuda_20261030/start_trial.sh](../rm/训练数据/rune_szu_cuda_20261030/start_trial.sh) | 归档工具/参考副本 | 启动归档 CPU 训练试验，锁定重复启动，保存 PID/日志；会训练模型，不应当作运行工程启动脚本。 |
| [rm/训练数据/rune_szu_cuda_20261030/view_trial.sh](../rm/训练数据/rune_szu_cuda_20261030/view_trial.sh) | 归档工具/参考副本 | 读取该训练包的 latest_trial 路径并跟随训练日志。 |

## 20 非代码资源与第三方边界

| 位置 | 作用与运行关系 |
| --- | --- |
| auto_buff/buff_detect/yolo/model/ | 生产模型及许可证/元信息；model 参数指定 ONNX 或 XML，XML 的 BIN 必须配套。文件名本身不能证明类别协议。 |
| auto_buff/debug/blueprints/1.rbl～7.rbl | 模块 debug 组合对应的七种 Rerun 布局；CMake 转成生成头嵌入程序，不需运行时 Python。 |
| third_party/rerun/sdk/ | 真正的 Rerun C++ SDK，及其 Arrow 构建逻辑；当前构建依赖。业务窗口代码在 rm_utils/display，不在这里。 |
| third_party/tracy/ | Tracy 第三方客户端/分析工具源码；项目构建 TracyClient，运行采集受 tracy_debug 控制。 |
| SDL2 系统包 | 当前以 find_package(SDL2 CONFIG) 查找和链接，没有把 SDL SDK 源码复制进 third_party。 |
| environment/archives/、environment/sources/ | 已下载依赖的源码归档/交付资源；由构建脚本定位，不是业务算法。 |
| .deps/ | 本地第三方依赖、旧 Windows 运行库/试验桥接等；现行 CMake 只使用显式命中的内容，不会递归编译全部。 |
| rm/ 的图片、标签、模型配置及归档 | 训练/标注侧数据；上表列出其中 Python 的用途，但不属于实机分层。 |
| tests/data/ | 测试输入；生产程序仅在配置显式引用时读取。 |
| output/ | 当前实机、采集、离线、回放和测速输出；不进入生产构建。 |
| tests/output/ | 保留的历史实验、运行记录和回退快照；可按确认清单清理，不再作为默认输出。 |
| build/、build-headless/、build-profile/ | 本机生成物和缓存，不能当成当前源码结构，也不能随 Linux 源码包复用旧 CMakeCache。 |
| README*.md、docs/、各模块 README/PORTING | 使用与设计文档，不编译；较早文档的历史记录应与当前实现区分。 |
| .vscode/、.clang-format、.clang-tidy、.gitignore | 编辑器/格式/静态检查/忽略配置，不决定运行时算法职责。 |

## 覆盖核对

本次按明确目录和扩展名枚举文件，检查每个文件都有职责项、链接指向实际文件，并保存当前文件哈希供后续核对；历史结构整理快照可按确认清单清理。核对工具及清单在 tests/output/structure_audit_20261109/；它们只是本次审查产物，不是新增的生产模块。
