# 模块逻辑回归

测试直接调用生产模块，使用固定输入和断言检查结果。需要人工观察的设备诊断、曲线和测速入口见 [tools/README.md](../tools/README.md)。

| 目录 | 被测内容 |
|---|---|
| auto_aim/ | 装甲板检测、位姿、业务绘制与自瞄回放 |
| auto_buff/ | 传统/YOLO 检测、推理队列、位姿、跟踪、预测、火控和调试输出 |
| hardware/ | 包布局、CRC、串口状态和帧池 |
| threads/ | 路由、模式切换、指令与帧有效期和进程级 OpenCV 线程设置 |
| rm_utils/display/ | 显示生命周期、图元、事件、RRD 曲线清理与读回 |
| rm_utils/io/ | 帧元数据格式与错误输入 |
| rm_utils/plotter_test.cpp | UDP 数据发送与错误恢复 |
| apps/ | 录制/配对读取、回放和播放控制的集成行为 |
| third_party/ | 项目 mas_log 封装的过滤、初始化、并发和退出收尾 |
| cmake/ | SDK 查找规则 |
| common/ | 断言、临时目录、合成观测和内存串口替代实现 |
| data/ | 固定图片与输入素材 |

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

`BUILD_TESTING` 与 `BUILD_TOOLS` 分别控制测试和工具。`VISION_GPU_PROFILE_ONLY=ON` 保留 OpenVINO GPU 轻量入口，构建 gpu_live_profile、live_profile_test、yolo_execution_test，CTest 运行后两个执行回归。TensorRT 不接受这个选项。

OpenVINO 下执行 CPU/GPU 生产检测接口检查。TensorRT 下执行通用几何和像素精修回归，engine 检查需要目标设备生成的模型，未传入时返回 77。模型和真实 GPU 依赖应单独准备。

```bash
./build/tensorrt_execution_test output/engines/rune_szu_fp16.engine
python3 tests/rm_utils/display/check_rerun_scalars.py ./build/rerun_scalar_test
python3 tests/cmake/test_openvino_discovery.py
```

Rerun 的 C++ 程序生成 RRD，Python 脚本读回并检查有限值、NaN/Inf 清理、历史与恢复。配置时 Python 环境支持 `rerun.chunk.RrdReader` 才登记该读回测试，否则显示依赖提示。

固定图片为 data/img/input.png 和 szu_reference.jpg，检测示例配置为 auto_buff/yolo_pose.yaml。人工图片／视频预览使用 `buff_preview config.yaml`，`buff_detect_test` 只运行固定输入自检。

测试构建仍需要工程的 Linux 依赖。测试按模块访问模型和显示后端，硬件逻辑回归使用内存传输，真实相机和串口诊断由 tools 入口执行。默认超时 60 秒，router_runtime_test 为 10 秒，Plotter 端口不可用时按 77 跳过。临时目录由测试清理，历史 tests/output 材料保留。
