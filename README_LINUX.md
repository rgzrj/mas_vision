# Linux编译与运行

更新：2026-11-13。当前生成mas_vision、mas_vision_record、mas_vision_replay三个程序。只维护Linux运行，要求GCC>=13、CMake>=3.22，C++23，不需要ROS。已安装的依赖优先复用，不必每次重新安装或激活虚拟环境。

## 1. 基础依赖

Ubuntu 24.04可使用系统GCC13：

```bash
sudo apt update
sudo apt install build-essential cmake pkg-config python3-venv python3-pip \
  libopencv-dev libeigen3-dev libyaml-cpp-dev libspdlog-dev nlohmann-json3-dev

# SDL为显示依赖；Rerun C++ SDK随工程提供。
sudo apt install libsdl2-dev
```

工程也会优先使用已经存在的.deps/yaml-cpp、spdlog、Eigen源码。不要混用旧Windows构建缓存或MSVC产物。

## 2. 海康SDK

海康固定参与构建，需要真实的Linux MVS SDK，默认/opt/MVS。先确认本机是否已有：

```bash
ls /opt/MVS/include/MvCameraControl.h
find /opt/MVS/lib/64 -maxdepth 1 -name 'libMvCameraControl.so*' -print
```

缺失时安装厂商对应架构SDK。安装位置不同时传入-DMVS_ROOT=/实际路径，也可显式设置MVS_INCLUDE_DIR和MVS_LIBRARY。不能通过关闭相机开关绕过SDK。

## 3. OpenVINO

只需准备一次共享环境，后续更新源码仍可复用：

```bash
bash environment/setup_linux.sh
```

默认位于${XDG_DATA_HOME:-$HOME/.local/share}/mas-vision/venv，固定OpenVINO 2026.4.1。CMake查找顺序是显式OpenVINO_DIR/缓存、项目.venv-openvino、共享环境、系统SDK，不要求每次source。

复用其他安装时，先找到真正配置文件再填写目录：

```bash
find "$HOME" /opt/intel /usr/local -name OpenVINOConfig.cmake 2>/dev/null
cmake -S . -B build -DOpenVINO_DIR=/实际目录/openvino/cmake
```

若缓存绑到已移走的旧环境，使用cmake -S . -B build -U OpenVINO_DIR重新发现。不要直接复制Python虚拟环境到不同路径。

## 4. 编译

```bash
./build_linux.sh
```

也可以进入build使用熟悉的方式：

```bash
mkdir -p build
cd build
cmake ..
make -j8
```

只需要独立采集或离线回放时，也可分别构建：

```bash
cmake -S apps/record -B build-record
cmake --build build-record -j8

cmake -S apps/replay -B build-replay
cmake --build build-replay -j8
```

独立record不需要OpenVINO；独立replay不需要MVS或串口库。两者均不链接主程序的router及算法工作线程。根目录构建仍一次生成三个程序。

已经存在的environment/archives会作为Arrow、xsimd、mimalloc、Boost的本地源码包使用。Arrow失败时查看arrow_cpp-build-err.log/out.log；只看到Command failed: 2不能判断原因。依赖下载失败需检查具体包，不能关闭TLS验证来解决。内存不足可降低到-j4。旧生成器/编译器切换需要新构建目录。

## 5. 三个入口

```bash
./build/mas_vision
./build/mas_vision calibrate_camera
./build/mas_vision calibrate_handeye
./build/mas_vision calibrate_latency
./build/mas_vision_record config/record.yaml
./build/mas_vision_replay config/replay.yaml
./build/mas_vision_replay config/media.yaml
python3 tools/check_pairs.py /path/to/session
```

从build目录运行则使用./mas_vision等。程序默认配置按可执行文件所在的项目/build与项目/config布局定位，不依赖启动目录。文件/模型相对路径按所属YAML解析。

三个calibrate命令是主程序的标定入口，不启动armor/buff。内参标定窗口按s保存圆点板样本，q或ESC结束并求解（至少10组）；手眼标定按s保存板面与曝光姿态，c清空，q或ESC求解（至少3组）。关闭窗口同ESC，参数写回config/hikcamera.yaml。延迟标定使用终端q结束，只有通过原有一致性门限才写入transfer_latency_ms。标定日志位于output/profiling。

实机从固定auto_aim.yaml、auto_buff.yaml、hikcamera.yaml、serial_config.yaml装配。mode0/1为装甲板、2/3小符、4/5大符。两个算法线程长期存在，切模式不重载模型。相机只采一路海康；主程序不录视频，需要采集就运行独立record程序。

main只负责资源装配和生命周期，配置字段由模块内部读取。runtime仍保留在auto_buff.yaml：router读取运行时长和帧有效期，并将同一有效期随帧交给armor/buff线程。max_runtime_s大于0时，即使没有相机帧也会正常通知退出。

Debug子类由各算法持有，在算法内部按YAML开关直接绘制；构建图元和编码的耗时计入处理线程。main不创建或传递ArmorDebug/BuffDebug。Display只准备后端配置，当前领域确有调试视图时才在主线程启动。装甲板火控直接返回公共SendPacket，13字节串口协议不变。

replay使用pipeline: armor | buff以及input.type: paired | media。paired读取会话原始时间和姿态；media没有同步IMU，不能验收移动云台预测精度。空格暂停/继续，n单步，q退出；Viewer自身暂停不会暂停算法。

check_pairs.py通过replay的--validate选项复用生产配对校验，不打开设备、不加载模型、不创建输出目录。它不是另一套Python校验规则。

## 6. 调试和输出

各模块debug控制显示提交，auto_buff.yaml中的display.backend选择sdl或rerun。实机按当前armor/buff切换对应布局，各模块独立SDL窗口；Rerun沿用共享底图布局，避免重复编码。关闭某个窗口不等于停用算法。SDL图像在提交前按30 Hz采样，领域Debug在采样后才准备图元，算法线程只交图像引用和图元，实际绘制由主线程完成；窗口中的Process FPS仍表示算法处理速度。

Rerun Viewer为单独程序。需要时在共享环境安装与C++ SDK一致的版本：

```bash
"${XDG_DATA_HOME:-$HOME/.local/share}/mas-vision/venv/bin/python" -m pip install rerun-sdk==0.41.0
```

两个显示后端互斥：SDL只维护本机窗口，不创建Rerun会话、不提交数值或写入RRD；Rerun独立管理Viewer、图像、曲线和RRD。display中的spawn、address、recording及编码选项仅用于Rerun。比赛时将各模块debug设为false，不创建显示会话、不准备调试图元，日志继续记录。下次调试只需修改YAML，无需重新编译。回放不导出标注视频，录像只在独立record程序进行。

装甲板恢复MAS原有窗口：HikCamera（检测）、HikCamera_track、HikCamera_track_pose、HikCamera_shoot。track使用armor_track_debug，pose使用armor_pose_debug并随track开启。track/shoot的plotter_enable与图像debug独立，继续向本机UDP 9870输出原诊断字段；track包含未进入tracking时也能采集的原始四角点和同帧R_gimbal2world。若比赛不需要UDP诊断，单独关闭这两个plotter_enable。

Plotter接收端沿用MAS的tools/plot_monitor.py。仅记录CSV无需额外Python依赖：

```bash
python3 tools/plot_monitor.py --no-gui
```

不带--no-gui可实时画曲线，需要pyqtgraph和PyQt5。接收结果在output/profiling/plotter；不要同时启动两个占用9870端口的monitor。

运行数据统一在项目output：live实机诊断、record原始配对采集、replay离线结果、profiling测速。主程序文本日志保留当前main设置的logs/mas_vision_<时间>.log。回放只保存简要运行统计；生产入口不输出检测JSONL和性能CSV。测速与Viewer上屏速度分开统计。

## 7. 验证

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j8
ctest --test-dir build --output-on-failure
```

包括切换epoch、断图命令过期、满姿态队列下mode继续更新、曝光姿态插值边界、paired/media回放、模式筛选、快照和显示测试。自动测试不会替代真实相机/串口和标定验收。

本次Windows上的接口、内存传输及离线回归检查不等于目标Linux完整编译。部署后仍需核对内参、手眼、IMU时间、串口13字节协议和实际帧率；不能用拍摄照片的跟踪结果验收动态预测。

## 独立测速

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --target buff_profile yolo_profile -j8
./build/buff_profile config/auto_buff.yaml /path/to/camera_image.png 600
python3 tools/summarize_performance.py output/profiling/<本次目录>/performance.csv --warmup-s 3
```

buff_profile 读取一张图片并重复调用生产 detect/track/shoot，显式关闭图像 debug，不访问相机或串口。图片分辨率应与 hikcamera.yaml 的标定尺寸一致。它测的是文件图片条件下的算法耗时，不包含采集、传输、显示，也不能用照片验收空间预测精度。若画面不能建立模型，track/shoot 只测到未初始化分支；有效跟踪负载需用能建立模型的画面测量。

yolo_profile 保留 YOLO/精修分项与 Tracy，用法见 ./build/yolo_profile --help；测速 CSV 只由测试程序生成。生产 runtime 仅保留帧龄和运行时长控制。
