# 线程模块

- hikcamera_thread：接收config_path，构造HikCamera时由驱动内部读取YAML；取帧后try_push到SPSCQueue。
- serial_thread：复用实机入口已用serial_config.yaml初始化的UserSerial，周期性收发数据。
- buff_thread：接收业务配置路径和跨帧模块，依次处理模式、最新海康帧、detect、曝光姿态、track、shoot和控制结果。
- usbcamera_thread：保留MAS公共USB接入代码，当前实机和采集入口不启动。

g_shutdown只在apps/main.cpp定义，各任务通过extern读取。实机启动海康、串口、打符三个任务；独立record只启动海康和串口。退出时先通知任务，再等待线程结束。

CameraFrame携带图像、曝光时间和接收时间。take_latest_frame跳过积压旧帧；每帧检测后仅查询一次曝光姿态，诊断共用该快照。过期帧、无效姿态或已切换模式的结果不输出有效控制。
