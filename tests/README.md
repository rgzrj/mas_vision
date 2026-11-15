# 测试与独立检查

所有目标在tests/CMakeLists.txt登记，直接链接生产模块；不复制滤波、检测或火控实现。

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j8
ctest --test-dir build --output-on-failure
```

- auto_buff：检测、位姿、大符灯臂几何、跟踪、运动拟合、火控。
- app/replay_test.cpp：配对IO、旧session兼容、快照、模式颜色、暂停相关控制及播放速度不影响算法。
- app/armor_replay_test.cpp：armor的paired/media输入、模式过滤和缺姿态处理。
- app/armor_visual_test.cpp：执行生产绘制和算法内部直接调用，验证灯条/角点、位姿重投影、预热模型、完整跟踪/火控文本、YAML全关与回放禁用显示；同时核对SendPacket及打符算法内部调试与最终过滤时机，不启动窗口。
- app/display_lifecycle_test.cpp：真实Display/SDL，验证全关时不创建会话，领域注册前不启动后端，以及注册后的独立窗口与资源收尾。
- app/armor_plotter_test.cpp：关闭GUI后实际接收UDP，核对预热期手眼原始四点、同帧旋转矩阵、缺观测时有效标志及shoot原字段。测试占用127.0.0.1:9870，端口被monitor占用时跳过；关掉monitor后再运行即可。
- hardware/vision_routing_test.cpp：切模式中的旧结果、A→B→A、命令过期和姿态插值范围。
- hardware/serial_state_test.cpp：使用生产user_serial.cpp解析器，内存传输只替换串口IO，验证UserSerial内部姿态插值、越界/大间隔拒绝、重连丢弃旧姿态，以及满队列仍更新mode和通信时间、13字节发送与过期处理。
- hardware/router_runtime_test.cpp：生产router配内存串口，验证配置读取debug开关、armor/小符/大符任务及帧有效期传递、无图像时定时退出和非法runtime参数拒绝，不访问真实硬件。
- app/display_test.cpp、buff_debug_test.cpp、sdl_display_test.cpp：绘制、无效状态清理、独立窗口和运行时调试开关。
- app/sdl_input_test.cpp：标定保存/清空按键、按住不重复保存、关闭事件，以及原有回放暂停/单步按键。
- hardware/camera_test.cpp、serial_test.cpp：显式传配置才访问设备；CTest仅运行相机帮助和串口协议自检。
- tools：模型检查、官方推理测速、静态检查、yolo_profile、buff_profile。

固定素材为tests/data/img/input.png和szu_reference.jpg，分别用于传统与YOLO自检。检测示例配置在tests/auto_buff/yolo_pose.yaml。

相机/串口工具日志位于output/profiling/<时间>/。自动回归临时目录由测试自行清理；tests/output仅为历史材料，不再作为默认输出。

测试构建仍需正式工程的Linux依赖及真实MVS SDK，帮助、自检和内存串口测试不打开设备。通过回归不代表已完成实机精度或帧率验收。

独立整链路测速：`./build/buff_profile config/auto_buff.yaml image.png 600`。重复调用生产算法，关闭图像调试，结果在 output/profiling/<时间>/performance.csv；汇总使用 tools/summarize_performance.py。图片没有相机采集延迟，模型无法初始化时也不代表测到了有效跟踪/火控负载。
