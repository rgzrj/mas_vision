# 独立采集

```bash
./build/mas_vision_record config/record.yaml
```

采集器直接使用海康驱动及串口接收，不运行检测、跟踪或火控。原始图像、曝光和接收时间、曝光时刻姿态及有效标志、mode保存在同一session；缺姿态不丢图、不伪造四元数。编码队列满或写入失败会报告错误。

配置快照包含auto_aim.yaml、auto_buff.yaml、hikcamera.yaml、serial_config.yaml及对应模型。IR的XML/BIN一起复制。recording.pipeline只决定生成的replay.yaml默认使用哪套算法，不影响采集内容。

程序无参数时从项目config读取record.yaml。相机和串口仍各自通过带路径的构造函数读取自己的固定配置。

快照实现在本目录的session.cpp/.hpp。采集器可单独配置构建，不检查OpenVINO、不编译armor/buff算法或实机业务线程：

```bash
cmake -S apps/record -B build-record
cmake --build build-record -j8
./build-record/mas_vision_record config/record.yaml
```

仍需海康MVS、基础图像依赖及显示依赖。
