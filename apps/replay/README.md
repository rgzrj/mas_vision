# 独立回放与媒体处理

```bash
./build/mas_vision_replay config/replay.yaml
./build/mas_vision_replay config/media.yaml
python3 tools/check_pairs.py /path/to/session
```

`pipeline` 选择 armor 或 buff；`input.type` 选择 paired 或 media。paired 使用 session 的 CSV 时间和姿态，缺姿态时只检测，模式不属于所选 pipeline 时不处理。media 使用媒体时间和固定相机标定，没有同步 IMU，不能据此验收运动云台精度。

倍速仅控制等待，不改变算法时间。空格暂停/继续，暂停时 n 单步，q 退出；SDL 和启动终端均可控制。暂停后不继续消耗输入帧。

`armor_process.hpp/.cpp`、`buff_process.hpp/.cpp` 仅组织离线单帧调用与模块生命周期，直接使用生产算法。打符的离线输入包含原图、曝光时刻、可选姿态、颜色和模式；不接受 realtime 标志，也不维护第二套模型。实机由各自算法线程直接调用模块。

模块 debug 与 `playback.display` 控制开窗；display=false 会关闭算法内部图像调试。回放输出目录只保存运行日志、简要 summary.yaml，以及开启显示时的 RRD；不生成检测 JSONL、标注视频或性能 CSV。原始视频、frames.csv、姿态和 mode 由独立 record 程序保存。

`check_pairs.py` 只调用本程序的 --validate，复用 C++ 配对读取规则，不加载模型、不创建显示或输出、不访问设备。旧 session 的 replay.media/frames_csv/buff_yaml 格式继续可读。
