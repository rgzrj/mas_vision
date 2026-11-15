# 固定配置

| 文件 | 用途 |
|---|---|
| auto_aim.yaml | 装甲板检测、跟踪、火控；各模块debug只控制可视化提交 |
| auto_buff.yaml | 打符检测、跟踪、火控；当前共用的runtime和display设置 |
| hikcamera.yaml | 海康参数、内参、畸变和手眼标定 |
| serial_config.yaml | 串口设备、协议和姿态超时 |
| record.yaml | 独立采集存储、编码队列和生成回放配置的默认pipeline |
| replay.yaml | session配对回放：pipeline、输入、算法快照、输出和播放设置 |
| media.yaml | 普通图片/视频回放示例 |
| usbcamera_config.yaml | 保留的公共USB驱动配置；当前正式入口固定使用海康 |

实机读取项目config中的固定文件；模块构造函数接收配置路径并在内部读取。相对资源路径相对所属YAML解析。回放使用session配置快照，避免当前标定替换录制时标定。

main只传路径，不解析配置字段，也不创建ArmorDebug/BuffDebug。各算法内部持有调试子类，按对应开关直接绘制，主线程按需启动显示。auto_buff.yaml中的runtime继续保留：router读取max_frame_age_ms与max_runtime_s，帧有效期同时用于算法线程和串口命令验收。测速由 tests 中独立程序完成。

装甲板沿用MAS的窗口开关：armor_detector.debug、armor_track.armor_track_debug、armor_track.armor_pose_debug、armor_shoot.debug。pose窗口与原MAS一致，随track窗口启用。armor_track.plotter_enable和armor_shoot.plotter_enable单独控制UDP诊断；关闭窗口仍可采集手眼原始四角点与同帧旋转矩阵。

模式0/1为armor，2/3小符，4/5大符。首次2/4保留buff配置颜色，后续红蓝位变化才切色。RMCS的恒速/正弦运动拟合保留，不由mode强制指定。

实机输出到output/live；采集到output/record；配对和普通媒体的离线结果统一到output/replay。比赛时将各模块debug设为false；日志照常输出，之后开启显示只需修改YAML。
