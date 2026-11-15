# mas_vision

Linux装甲板与打符视觉。实机通过电控mode选择armor/buff，两个算法线程和模型在启动时创建一次。

```bash
./build_linux.sh
./build/mas_vision
./build/mas_vision calibrate_camera
./build/mas_vision calibrate_handeye
./build/mas_vision calibrate_latency
./build/mas_vision_record config/record.yaml
./build/mas_vision_replay config/replay.yaml
./build/mas_vision_replay config/media.yaml
python3 tools/check_pairs.py /path/to/session
```

比赛和调试使用同一份程序，各模块debug决定是否提交显示；全关时不创建显示会话，日志照常输出。在auto_buff.yaml的display.backend选择sdl/rerun。SDL各模块独立窗口，Rerun共用显示出口。没有dev版本或run/read/export子命令。

程序默认从项目config读取固定配置；相机、串口和各算法模块在带config_path的构造函数中读取自己的设置。录制/回放支持传入对应配置文件，回放优先使用session快照。

- [Linux环境与编译](README_LINUX.md)
- [应用职责](apps/README.md)
- [采集说明](apps/record/README.md)
- [回放和配对检查](apps/replay/README.md)
- [接入边界与并发约束](docs/INTEGRATION_20261113.md)
- [配置目录](config/README.md)

相机固定使用真实Hikrobot MVS SDK。mode 0/1装甲板，2/3小符，4/5大符。相机内参、IMU到云台关系和时序必须由实机标定；普通视频或照片不能验收动态预测精度。

output保存live、record、replay和profiling；tests只保存测试代码与固定素材。rm为训练/标注工具工作区，不参与生产构建。
