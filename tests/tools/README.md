# 模型检查与检测测速

工具在此目录，统一由 `tests/CMakeLists.txt` 登记；Tracy 客户端源码放在 `third_party/tracy`，版本 **0.14.1**，保留官方许可证。统一读取 YAML 的 `detect.tracy_debug`，默认 false；无需切换编译选项。

## 1. 模型信息与 Netron

Python 工具使用同一个环境安装依赖：

```bash
python -m pip install -r tests/tools/requirements.txt
python tests/tools/inspect_model.py /path/to/model.onnx
python tests/tools/inspect_model.py /path/to/model.onnx --serve
```

第一条检查命令打印并保存输入输出名称、类型、尺寸，不编译或运行模型。`--serve` 在本机 `127.0.0.1:8080` 启动 Netron 并打开浏览器；Ctrl+C 结束。模型不会上传到网站。端口冲突可用 `--port 8081`。

XML 模型需要同目录、同名 BIN。Netron 展示原模型结构；OpenVINO 编译执行时会融合或重排部分层，两者层数不一定一致。

## 2. OpenVINO 官方测速

```bash
python tests/tools/benchmark_model.py /path/to/model.xml --threads 4 --mode latency
python tests/tools/benchmark_model.py /path/to/model.xml --threads 4 --mode throughput
python tests/tools/benchmark_model.py /path/to/model.xml --threads 4 --mode layers
```

`--mode all` 顺序执行上述三种测试，每种测试重新创建独立进程。`--seconds 10` 是每种测量的持续时间，加载、编译另计。`--threads` 对齐配置的 `detect.threads`，输入保持模型原有空间尺寸，batch=1。

| 模式 | 配置 | 用途 |
| --- | --- | --- |
| latency（默认） | CPU、同步、单流、单请求、指定线程数 | 单次推理延迟 |
| throughput | CPU、异步、AUTO 流数、指定线程数 | 并行处理吞吐；实际流数/请求数见日志 |
| layers | 与 latency 相同，但打开逐层计数 | 分析编译后执行层耗时 |

本机 OpenVINO 2026.4.1 的官方 CLI 要求手动设置线程数/流数时使用 `-hint none`，因此这里显式设置，而不是同时传递性能提示。主程序使用 LATENCY 提示，基准报告中保留实际参数，不能声称两者的编译配置完全相同。逐层统计和吞吐结果不能代替同步延迟。

脚本只调用官方 `benchmark_app` 的 Python 入口，没有另写推理循环。官方默认预热保留，输入由官方工具生成；不包含相机、YOLO 预后处理、精修、显示或录像。查看 `benchmark.log` 的实际精度和 CPU 执行信息，不要仅按文件名判断 FP32/FP16/INT8。

## 3. Tracy：完整执行一次检测，分别测 YOLO 与精修

正常编译测试工具即可：

```bash
cmake -S . -B build-profile -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
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

`check_code.py` 仅检查显式列出的自有 `.cpp/.hpp`，不改写文件，也不检查第三方源码。Python 3.10+，只使用标准库。先完成 CMake 配置，静态分析使用 `build/compile_commands.json` 中真实的编译参数。

```bash
python tests/tools/check_code.py --tool format auto_buff/debug/buff_debug.cpp auto_buff/debug/buff_debug.hpp
python tests/tools/check_code.py --tool tidy auto_buff/debug/buff_debug.cpp
python tests/tools/check_code.py --tool cppcheck auto_buff/debug/buff_debug.cpp
# 三项一起执行；缺少工具会明确报错，不静默跳过。
python tests/tools/check_code.py --tool all auto_buff/debug/buff_debug.cpp
```

Windows 可使用 Visual Studio 自带的 clang-format/clang-tidy；脚本通过 vswhere 定位。clang-tidy 需在 VS x64 开发者终端运行，以取得 MSVC/Windows SDK 的系统头文件环境。Windows 静态分析会额外关闭旧 bundled fmt 的 consteval 解析，以绕过 Clang 与本机 MSVC STL 的兼容错误；正式构建及格式字符串检查不变。Linux 将相应工具加入 PATH。cppcheck 需单独安装支持 C++23 的版本。`.clang-tidy` 先启用核心分析、生命周期与移动语义检查，不做全仓库自动修复。

## 异步显示检查

```bash
./build/buff_debug_test output/profiling/async_debug_checks
rerun rrd compare --unordered --ignore-timeline log_tick output/profiling/async_debug_checks/ownership_reference.rrd output/profiling/async_debug_checks/ownership_reused.rrd
```

Windows 程序为 `build/buff_debug_test.exe`。检查禁用、离线顺序、实机积压覆盖、末帧排空和异常退出。两份 ownership RRD 在提交后分别保留/立即改写源图像与结果，官方比较应一致，验证后台持有独立快照。

`timing.json` 给出 80 帧压力输入的同步绘图耗时和异步提交耗时 P50/P95（最近秩），以及实际绘制/覆盖帧数。异步使用最新帧策略，两者绘制帧数不同；这是调试提交开销检查，不是 YOLO FPS 或目标 CPU 的性能保证。
