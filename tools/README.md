# 诊断与测速工具

常用入口如下，所有命令从仓库根目录运行。

| 工具 | 用途 |
|---|---|
| `plot_monitor.py` | 接收 UDP 调试数据，画曲线并保存 CSV、配置和日志 |
| `check_pairs.py` | 调用生产回放程序检查录制数据配对 |
| `check_code.py` | 检查指定 C++ 文件的格式和静态分析 |
| `inspect_model.py` | 查看模型输入输出，可选打开 Netron |
| `benchmark_model.py` | 调用 OpenVINO 官方 CPU 测速 |
| `compare_inference.py` | 在目标 Linux 上采集环境并对照 CPU/GPU 推理 |
| `benchmark_cpu_scheduling.py` | 对照 CPU 调度配置 |
| `quantize_rune_public.py`、`compare_rune_quantization.py` | 模型量化与同画面对照 |
| `summarize_performance.py` | 汇总整链路 performance.csv |
| `yolo_profile`、`buff_profile` | 检测或整链路耗时分析 |
| `gpu_live_profile` | OpenVINO GPU 双槽实机负载采集 |
| `camera_test`、`serial_test` | 人工检查相机、串口与设备数据 |
| `buff_preview` | 按配置读取图片／视频，显示打符检测结果 |
| `sdl_upload_profile` | SDL 纹理转换、上传与绘制耗时对照 |

单独构建 C++ 工具使用 `-DBUILD_TOOLS=ON`，`BUILD_TESTING` 独立控制模块回归。程序输出到构建目录，原有目标名保持一致。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TOOLS=ON
cmake --build build --target yolo_profile buff_profile sdl_upload_profile -j4
```

## Plotter 调车曲线

先在 `config/auto_aim.yaml` 开启所需的 `armor_track.plotter_enable` / `armor_shoot.plotter_enable`，再启动视觉程序和接收端。

```bash
python3 tools/plot_monitor.py
python3 tools/plot_monitor.py --no-gui
python3 tools/plot_monitor.py --list
python3 tools/plot_monitor.py --view jitter --log output/live/本次会话/app.log
```

默认接收 UDP 9870，C++ 默认发送到 `127.0.0.1`，接收端通常在视觉主机运行。GUI 需要 `pyqtgraph` 和 `PyQt5`，`--no-gui` 和 `--list` 只依赖 Python 标准库。预设视图支持 `spin`、`jitter`、`drift`、`converge`，`--keys` 可指定字段。

记录位于 `output/profiling/plotter/run_时间/`，包含按字段集分流的 CSV、配置快照和 `plot_monitor.log`。`--log` 明确指定配套程序日志，退出时复制为 `base.log`；省略时尝试匹配最近的默认实机或回放日志，自定义回放输出路径需要手动指定。`latest_run.txt` 保存最近记录目录。一个端口只启动一个接收端。

## SDL 上传对照

SDL上传对照是独立测试程序，按30Hz运行，不加载相机、串口或算法：

```bash
cmake -S . -B build -DBUILD_TOOLS=ON
cmake --build build --target sdl_upload_profile -j4
./build/sdl_upload_profile 1280 1024 1 300
./build/sdl_upload_profile 1280 1024 3 300
```

宽、高应填写日志中`SDL source`的原图尺寸。程序按BGR24/BGRA32/BGRA32/BGR24顺序对照，各轮预热30帧；CSV输出转换、全部窗口上传、绘制的平均耗时，总耗时P95、实际刷新率及进程CPU占用。CPU百分比按逻辑CPU累计。此工具只隔离上传路径，不测算法、文字，也不计独立NoMachine进程的CPU；实际整机影响需另行对照。测试时关闭主程序，保持相同远程桌面状态。

C++ 工具由 `tools/CMakeLists.txt` 构建，帮助自检由 `tests/CMakeLists.txt` 登记；Tracy 客户端源码放在 `third_party/tracy`，版本 **0.14.1**，保留官方许可证。统一读取 YAML 的 `auto_buff.detect.yolo.tracy_debug`，默认 false；无需切换编译选项。

## 模型信息与 Netron

Python 工具使用同一个环境安装依赖：

```bash
python -m pip install -r tools/requirements.txt
python tools/inspect_model.py /path/to/model.onnx
python tools/inspect_model.py /path/to/model.onnx --serve
```

第一条检查命令打印并保存输入输出名称、类型、尺寸，不编译或运行模型。`--serve` 在本机 `127.0.0.1:8080` 启动 Netron 并打开浏览器；Ctrl+C 结束。模型不会上传到网站。端口冲突可用 `--port 8081`。

XML 模型需要同目录、同名 BIN。Netron 展示原模型结构；OpenVINO 编译执行时会融合或重排部分层，两者层数不一定一致。

## OpenVINO 官方测速

```bash
python tools/benchmark_model.py /path/to/model.xml --threads 4 --mode latency
python tools/benchmark_model.py /path/to/model.xml --threads 4 --mode throughput
python tools/benchmark_model.py /path/to/model.xml --threads 4 --mode layers
```

`--mode all` 顺序执行上述三种测试，每种测试重新创建独立进程。`--seconds 10` 是每种测量的持续时间，加载、编译另计。`--threads` 对齐配置的 `auto_buff.detect.yolo.threads`，输入保持模型原有空间尺寸，batch=1。

| 模式 | 配置 | 用途 |
| --- | --- | --- |
| latency（默认） | CPU、同步、单流、单请求、指定线程数 | 单次推理延迟 |
| throughput | CPU、异步、AUTO 流数、指定线程数 | 并行处理吞吐；实际流数/请求数见日志 |
| layers | 与 latency 相同，但打开逐层计数 | 分析编译后执行层耗时 |

本机 OpenVINO 2026.4.1 的官方 CLI 要求手动设置线程数/流数时使用 `-hint none`，因此这里显式设置，而不是同时传递性能提示。主程序使用 LATENCY 提示，基准报告中保留实际参数，不能声称两者的编译配置完全相同。逐层统计和吞吐结果不能代替同步延迟。

脚本只调用官方 `benchmark_app` 的 Python 入口，没有另写推理循环。官方默认预热保留，输入由官方工具生成；不包含相机、YOLO 预后处理、精修、显示或录像。查看 `benchmark.log` 的实际精度和 CPU 执行信息，不要仅按文件名判断 FP32/FP16/INT8。

## CPU / GPU 同条件对照

`compare_inference.py` 使用目标 Linux 已有的 OpenVINO 和 NumPy，不安装依赖、不修改业务配置。
先收集驱动、模型 XML/BIN 哈希、实际配置、C++ 运行库路径、供电策略及可读温度：

```bash
python3 tools/compare_inference.py --collect-only
python3 tools/compare_inference.py --device GPU
python3 tools/compare_inference.py --device CPU
```

运行前关闭实机程序，保持相同供电和远程桌面状态。每组预热30次、测10秒、重复3轮，交替测试顺序；
`--device ALL` 从GPU开始比较全部组合。GPU对照单流同步、单流异步、双流双请求；CPU对照2/3/4线程单流同步。
日志打印吞吐量、进程CPU占用、每帧CPU时间和OpenVINO请求P99；`summary.json`保存各轮及环境信息，默认位于`output/profiling/`。

默认固定零输入，仅比较推理；`--image /path/to/photo.png`可使用同一照片，要求当前Python能导入OpenCV。
图片只预处理一次，RGB/NCHW/255、居中letterbox 114；预处理不计入推理测速。
这些结果不包含相机、解码、像素精修、跟踪、SDL，不能代表实机处理帧率，也不能证明检测准确率。
CPU百分比以100%约一个逻辑核计，不代表GPU负载、功耗或硬件寿命；温度仅为各轮开始/结束快照，不是峰值。

## GPU双流实机持续负载

`gpu_live_profile`独立运行海康取图、GPU推理及生产解码/精修/跟踪/火控。每个请求独占输入张量和同帧元数据，
按提交顺序处理结果；在途最多两帧，提交前限制120 FPS，错过的时段不补发。该程序只做独立测速；生产YOLO现已通过`auto_buff.detect.yolo.device`选择CPU同步或GPU双请求异步。
关闭`mas_vision`，使用当前固定设备配置，电控切到打符模式2～5；串口仅接收，不发送目标或开火指令。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DVISION_GPU_PROFILE_ONLY=ON
cmake --build build --target gpu_live_profile live_profile_test yolo_execution_test -j4
./build/live_profile_test
./build/yolo_execution_test
./build/gpu_live_profile --seconds 60 --fps 120 --streams 2 --no-display
```

`VISION_GPU_PROFILE_ONLY=ON`只登记本节三个目标，不要求同步其他回归测试源码。`yolo_execution_test`使用合成模型检查生产CPU/GPU接口、输入隔离、结果顺序和切色；没有GPU时跳过GPU部分。
完整测试时显式改回`-DVISION_GPU_PROFILE_ONLY=OFF`；常规全量测试仍要求对应源码齐全。

先确认短测正常，再以相同供电、散热和远程桌面状态，完成实际打符时长的两分钟测试；热机后也需复测：

```bash
./build/gpu_live_profile --seconds 120 --fps 120 --streams 2 --no-display
```

图像窗口按`config/auto_buff.yaml`各模块debug开关控制；移除`--no-display`即可对照原有SDL/Rerun后端。
SDL仍按30 FPS采样。配置必须为`backend: yolo`，模型直接使用配置指定的XML/ONNX，保持同一模型与精修设置。
测试预处理与生产的`reuse_buffers: false`路径一致；此测试固定使用GPU，`--streams`控制本次测速，生产模式仍由YAML选择。

每次输出在`output/profiling/gpu_live/<时间>/`：

- `frames.csv`：提交序号、模式/epoch、预处理、提交到按序取回、后处理、曝光后帧龄及姿态/跟踪状态。
- `summary.json`：平均有效帧率、CPU占用、过期帧数和有效帧龄P50/P95/P99；停止时仍在途的请求单独计数。
- `health.csv`：每秒相机交付、提交、有效处理、带姿态处理和已初始化跟踪的帧率，以及进程CPU占用。
- `machine.csv`：每秒温度type/原始毫摄氏度、CPU/GPU频率、可读的热节流累计计数及RAPL能量计数。
- `app_*.log`、业务/相机配置快照：本次参数和设备诊断。

`submit_to_collect_ms`包含GPU排队、按序等待及主线程轮询，不是纯内核耗时；曝光时间戳沿用相机估算，
按收帧时刻减去实际曝光的一半。`processed_fps`只计处理结束时仍在帧龄/epoch期限内的帧；`tracked_fps`要求有姿态，
`initialized_fps`还要求跟踪已初始化。没有姿态时仍测检测，但不能作为完整跟踪验收。
相机交付率是`getImage()`实际返回率，不等于传感器标称帧率；推理满额、限速和相机队列丢弃都应分开解释。
传感器不可读时不输出该项，不能当成零；热节流计数要比较前后增量，正常动态调频不等于热降频。
本程序不读取GPU忙碌率或所有功耗限频原因，不能仅凭频率CSV证明完全没有降频。

## Tracy：完整执行一次检测，分别测 YOLO 与精修

正常编译测试工具即可：

```bash
cmake -S . -B build-profile -DCMAKE_BUILD_TYPE=Release -DBUILD_TOOLS=ON
cmake --build build-profile --target yolo_profile -j4
```


在所选 YAML 中开启采集，图像显示开关仍单独控制：

```yaml
auto_buff:
  detect:
    tracy_debug: true
    debug: false
    feature_points: rmcs_pixels
```

先打开同版本 Tracy Profiler 并连接目标，再运行（或先运行命令，15 秒内连接）：

```bash
./build-profile/yolo_profile tests/auto_buff/yolo_pose.yaml --warmup 20 --frames 300 --wait 15
```

不连接 Viewer 也可运行并生成 CSV，此时省略 `--wait`。图片会重复计算同一张，视频按源顺序读取；`--frames` 为预热之后最多测量的帧数，遇到视频结束会提前停止。输入长度不足预热时会报错，不会生成虚假的空统计。

| 指标 | 计时边界 |
| --- | --- |
| `YOLO` / `yolo_ms` | letterbox、类型/通道转换、OpenVINO 同步推理、解码与去重 |
| `PixelRefine` | 原流程内每次 `refine_yolo_pose_pixels()` 调用，失败调用也计入 |
| `refine_ms` | 同一帧所有精修调用耗时之和；未执行时为 0 |
| `refine_calls` | 本帧实际调用精修的次数，不等于最终保留目标数 |
| `detect_total_ms` | 完整 `detect()` 耗时，包含上述两段和判色、几何组织等其他操作 |

每帧只调用一次原有检测器，精修不关闭、不单独重跑。配置必须是 `feature_points: rmcs_pixels`。检测器不直接运行显示；此工具不创建 BuffDebug。公平测速应设 `detect.debug=false`，避免收集不需要的原始五点诊断快照。读图、解码视频、模型初始化、CSV 写入均在检测计时之外；不保存叠图或录像。

Tracy 显示两个区段，以及 YOLO、每帧精修总时间/次数三条曲线。预热区段也会出现在 Tracy 时间线上，但不进入 CSV、统计和三条测量曲线。CSV 的时间使用稳态时钟；Tracy 区段边界略宽，包含少量采集标记开销，数字不必完全相等。这是函数经过时间，不是 CPU 独占忙碌时间，也不是从曝光到电控收到结果的延迟。

Tracy 库随程序链接，客户端由 `yolo_profile` 测试入口启动并在退出时关闭；检测器构造函数不启动客户端。采集需要 `detect.tracy_debug=true` 且客户端已启动。普通主程序即使误开此配置，也不会自行创建采集线程或访问未初始化的客户端；false 时不采集、不读取计时时钟。没有启用调用栈、帧图像、采样或系统上下文切换采集。开关独立于 `detect.debug`，无需切换编译选项。

独立 `yolo_profile` 要求 `tracy_debug: true`，否则提示开启，避免输出全零的假测速报告。普通检测程序可以保持 false 正常工作。

## 报告与回放

默认输出在 `output/profiling/本次时间/`，也可传 `--output 目录`。YOLO 工具使用微秒时间戳作为目录名。

- `model_info.json`：模型 I/O 与 OpenVINO 版本。
- `latency/`、`throughput/`、`layers/`：独立日志、命令记录、官方 CSV 报告。
- `frames.csv`、`summary.json`：逐帧值，以及预热后的平均值、最近秩 P95、最大值。
- `source_config.yaml`：原始配置快照，仅供追溯；相对路径仍按原配置目录解释，不能直接搬动重跑。

如需保存 `.tracy`，使用官方 Capture：

```bash
tracy-capture -a 127.0.0.1 -o yolo.tracy
```

让 Capture 先等待连接，再运行带 `--wait 15` 的 YOLO 工具。结束后用 Profiler 打开记录，或用 `tracy-csvexport yolo.tracy` 导出区段统计。Tracy Viewer/Capture 都属于独立工具，不是主程序依赖。

本机验证所用 Python 为 `D:/YOLOV8/python.exe`，依赖位于 `.deps/openvino_runtime` 和 `.deps/model_tools`；可通过 `PYTHONPATH` 使用。Windows 官方 Tracy 工具已下载到 `.deps/tracy/0.14.1`，其他机器需自行安装同版本。

官方资料：[Tracy](https://github.com/wolfpld/tracy)、[Netron](https://github.com/lutzroeder/netron)、[OpenVINO Benchmark](https://docs.openvino.ai/2026/get-started/learn-openvino/openvino-samples/benchmark-tool.html)。


## 代码规范检查

`check_code.py` 仅检查显式列出的自有 `.cpp/.hpp`，范围为 `apps`、`auto_aim`、`auto_buff`、`hardware`、`threads`、`rm_utils`、`tests`；不改写文件，拒绝这些目录以外的文件及第三方串口库 `hardware/serial/common`。Python 3.10+，只使用标准库。从工程根目录运行；格式检查不需要编译数据库，静态分析需先完成 CMake 配置，使用 `build/compile_commands.json` 中真实的编译参数，也可通过 `--build` 指定构建目录。

```bash
python tools/check_code.py --tool format auto_buff/debug/buff_debug.cpp auto_buff/debug/buff_debug.hpp
python tools/check_code.py --tool format auto_aim/armor_types.hpp hardware/hikcamera/hikcamera.hpp
python tools/check_code.py --tool tidy auto_buff/debug/buff_debug.cpp
python tools/check_code.py --tool cppcheck auto_buff/debug/buff_debug.cpp
# 三项一起执行；缺少工具会明确报错，不静默跳过。
python tools/check_code.py --tool all auto_buff/debug/buff_debug.cpp
```

脚本只从 `PATH` 查找 clang-format、clang-tidy、cppcheck；缺少所选工具会报错。静态分析直接使用编译数据库参数，不额外修改 fmt 的编译宏。cppcheck 需安装支持 C++23 的版本。`.clang-tidy` 先启用核心分析、生命周期与移动语义检查，不做全仓库自动修复。

## 异步显示检查

```bash
./build/buff_debug_test output/profiling/async_debug_checks
rerun rrd compare --unordered --ignore-timeline log_tick output/profiling/async_debug_checks/ownership_reference.rrd output/profiling/async_debug_checks/ownership_reused.rrd
```

Windows 程序为 `build/buff_debug_test.exe`。检查禁用、离线顺序、实机积压覆盖、末帧排空和异常退出。两份 ownership RRD 在提交后分别保留/立即改写源图像与结果，官方比较应一致，验证后台持有独立快照。

`timing.json` 给出 80 帧压力输入的同步绘图耗时和异步提交耗时 P50/P95（最近秩），以及实际绘制/覆盖帧数。异步使用最新帧策略，两者绘制帧数不同；这是调试提交开销检查，不是 YOLO FPS 或目标 CPU 的性能保证。

## 人工设备诊断与检测预览

```bash
cmake -S . -B build -DBUILD_TOOLS=ON
cmake --build build --target camera_test serial_test buff_preview -j4
./build/camera_test config/auto_buff.yaml 10 --display
./build/serial_test config/serial_config.yaml 10
./build/buff_preview tests/auto_buff/yolo_pose.yaml
```

相机诊断需要配置中的 input.type / input.camera_config，串口诊断需要真实串口。`buff_preview` 读取示例配置中的媒体路径。协议 CRC 与包布局由 `serial_protocol_test` 自动检查。
