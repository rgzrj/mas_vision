# 海康拍摄照片：YOLO实机性能对照

目标是测清处理吞吐、CPU/内存占用、分阶段耗时与结果年龄。使用现有performance.csv和Linux pidstat，不修改检测/跟踪/火控算法。

## 1. 固定输入和运行条件

照片放在海康前，固定位置、距离、曝光、增益和分辨率。先短暂开启detect.debug确认目标确实被检测到，留一张检测截图，再按下文关闭显示测基线。对照期间不要同时运行第二份mas_buff、benchmark_app或训练程序。

先固定一个模型，以启动日志中实际打印的完整路径和输入输出为准。FP32与INT8分开测试，不在显示/精修对照中途更换模型。仓库auto_buff.yaml目前默认traditional，测速前必须选择yolo；模型路径保留目标机现有有效值。

实机线程要求电控mode在2～5、四元数流有效，才会进入检测。模式/颜色应与照片对应；0 frames时先检查这些输入，不先归因于模型性能。照片用于检测和耗时对照，不能验收真实三维运动预测；若反复出现rune diverged，一并保留日志，并将track耗时解释为这一照片场景的结果。

## 2. 第一轮：无显示、无录像、无精修

修改Linux机器现有config/auto_buff.yaml中的对应项，保留模型、颜色和其他参数，设备标定仍在各自固定配置文件，不要用这个片段覆盖整个文件：

```yaml
auto_buff:
  detect:
    backend: yolo
    debug: false
    tracy_debug: false
    feature_points: none
    threads: 6
    opencv_threads: 4
    reuse_buffers: false
  track:
    debug: false
  shoot:
    debug: false
runtime:
  profile: true
  max_runtime_s: 60
```

6个推理线程/4个OpenCV线程沿用此前12900HK的工作基线，本轮先不继续调整线程。三个算法仍正常运行；关闭debug仅关闭诊断。实机不录像，仍保留JSONL、frames.csv与性能CSV，这轮测的是现有实机链路，不是纯网络benchmark。

### 一次运行同时记录CPU/内存

先检查是否已有pidstat：

```bash
command -v pidstat
```

仅在缺失时安装`sysstat`：

```bash
sudo apt install sysstat
```

从build执行，结束后自动生成汇总：

```bash
BENCH_DIR="../output/profiling/$(date +%y.%m.%d_%H%M%S)"
mkdir -p "$BENCH_DIR"
LC_ALL=C pidstat -u -r 1 -e ./mas_buff run \
  > "$BENCH_DIR/resources.log" 2>&1

RUN_DIR="$(sed -n 's/^Output: //p' "$BENCH_DIR/resources.log" | tail -n 1)"
if [ -n "$RUN_DIR" ] && [ -f "$RUN_DIR/performance.csv" ]; then
  printf '%s\n' "$RUN_DIR" > "$BENCH_DIR/run_dir.txt"
  python3 ../tools/summarize_performance.py "$RUN_DIR/performance.csv" --warmup-s 3 \
    > "$BENCH_DIR/performance_summary.json"
  cat "$BENCH_DIR/performance_summary.json"
else
  printf '本轮未生成性能CSV，请查看日志：\n'
  tail -n 40 "$BENCH_DIR/resources.log"
fi
printf 'CPU和内存日志：%s/resources.log\n' "$BENCH_DIR"
```

等待60秒后自动结束，运行过程的输出保存在resources.log。RUN_DIR从本轮日志的Output读取，避免误取上轮目录。若程序报错或0 frames，先查看resources.log；没有有效样本时汇总脚本会明确报错。

pidstat的-e会启动并监测目标程序，程序结束后停止；-u记录CPU，-r记录驻留内存RSS等信息。不加-I时，100%约等于每秒消耗一个逻辑CPU秒，600%约等于六个逻辑CPU秒，不是“整机600%”。这不是六个物理核心满载的证明，也不是模型TOPS。[pidstat官方手册](https://manpages.ubuntu.com/manpages/noble/man1/pidstat.1.html)

CPU/RSS反映整个mas_buff进程，包括相机转换、推理、跟踪和本进程内的诊断线程，不能直接称为YOLO独占CPU。该日志包含启动阶段；performance汇总另排除首个记录样本之后的3秒，两者统计窗口不完全相同。Rerun Viewer是独立进程，不在这个进程统计中。

## 3. 后续对照：每次只改一个因素

每轮60秒，每轮都保存独立日志。先重复基线确认稳定，再进行其他轮次；测试较久时在中间复测基线，检查整机负载或温度漂移。

| 轮次 | 相对基线的改动 | 主要看什么 |
| --- | --- | --- |
| A 基线 | 三个debug=false、none | 无窗口时的真实处理能力 |
| B 精修 | 仅feature_points改rmcs_pixels | refine_ms、refine_calls、detect_ms与FPS变化 |
| C SDL | 精修恢复none，三个debug=true，display.backend=sdl | 快照/后台绘图、CPU竞争带来的额外成本 |
| D Rerun | 在C基础上仅切backend=rerun，并启动Viewer | 图像编码/记录和Viewer负载的额外影响 |

C/D不改变各模块显示内容；D沿用本机有效的Viewer安装，spawn=true。录像性能使用独立record入口测量，不与YOLO同轮测试混为一个指标。

Rerun轮若变慢，另开终端观察进程资源：`pidstat -u -r -G 'mas_buff|rerun' 1`。这份输出用于补看Viewer占用，不与第2节的单进程CPU平均值混为同一个指标。窗口上屏率和NoMachine观感仍需单独观察。

## 4. 如何读出瓶颈

| 指标 | 实际含义与判断 |
| --- | --- |
| observed_processing_fps | 去掉预热后，根据连续host_s计算的实际处理吞吐；作为平均FPS主指标 |
| summary.json的frames/elapsed_seconds | 含启动等待和收尾的整段平均，可作辅助，不能与稳态FPS混用 |
| preprocess_ms | letterbox、归一化、通道/张量准备 |
| infer_ms | OpenVINO同步推理墙钟耗时；均值表示常态，P95/P99观察抖动 |
| decode_ms | 坐标还原、点有效性、候选筛选及去重 |
| refine_ms/refine_calls | 实际像素精修耗时和调用数；没有调用不能据此断言精修很快 |
| detect_ms | 完整检测；还包含判色、观测转换、灯臂轮廓等，不能全部归入推理或解码 |
| track_ms/shoot_ms | 当前照片场景的跟踪/火控；持续未初始化时不能代表有效跟踪负载 |
| record_ms | 同步叠图、录像和报告写入；关闭视频仍可能有报告成本 |
| debug_submit_ms | 复制/提交快照的成本，不含后台绘制；需同时看退出时Debug统计 |
| result_age_ms | 曝光到进入记录回调时的结果年龄，不包含之后的写盘、串口送达或上屏 |

`yolo_ms`仅覆盖预处理+推理+解码，结束于判色/观测转换之前。`detect_ms - yolo_ms - refine_ms`可作为其他检测工作开销的近似，不是精确的独立计时项。CPU占用、阶段墙钟耗时、处理FPS要一起看，不能用单个数值认定瓶颈。

当前汇总脚本只统计毫秒列；refine_calls需要查原始performance.csv，不能从refine_ms=0单独判断原因。

75 FPS对应平均每帧约13.33ms预算。在当前单业务线程同步推理链路下，如果infer_ms本身长期超过13.33ms，就无法靠关掉文字达到75 FPS。反过来，即使推理很快，帧率也可能受相机供帧、姿态等待、输出或显示竞争限制。

Frame drop中的grab/queue只覆盖驱动统计位置；业务队列取最新帧也会跳过旧帧，不能因queue=0就认定整条链路没有丢帧。record.fps=80只指定录像名义FPS，不等于相机实际80 FPS。

## 5. 需要回传什么

第一轮先提供performance_summary.json和resources.log，再附目标检测截图。保留本轮Output目录中的performance.csv、detections.jsonl、frames.csv和app_*.log；后续需要时再用它们细查。

模型路径/接口、线程数从启动日志核对；识别状态从截图/报告核对。暂时不需要改RMCS，也不根据照片上的投影漂移调整算法模型。

如果推理和同步处理都有余量但FPS仍低，再单独测相机供帧，停止实机程序后从build执行：

```bash
cmake .. -DBUILD_TESTING=ON
make -j8 camera_test
./camera_test 20
```

这一步使用现有camera_test，不运行YOLO或串口；需要完整测试源码及真实MVS SDK，先核对其曝光时间戳帧率，再决定是否继续改推理。
