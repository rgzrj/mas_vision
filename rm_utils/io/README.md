# 公共 IO

本目录不依赖打符检测、跟踪或火控。保留已有命名空间和文件格式以减少迁移风险。

- MediaReader：普通图片/视频读取和媒体时间。
- tool::MediaRecorder：原图/标注视频、会话目录及媒体时间表。
- tool::RecordedFrame：图像、曝光/接收时间、可选姿态、模式；不包含检测结果。
- record::PairedRecorder：后台成对保存；复用 MediaRecorder，维护同序 CSV 和有界队列。
- replay::Player：视频或 PNG 序列和 CSV 配对，校验行数、帧数、时间和姿态。
- PlaybackControl：暂停/单步/退出状态，不参与算法计时。
- project_root：依据 /proc/self/exe 找到项目目录，供默认配置和 Viewer 查找。

CSV 的 pose_valid=0 明确保留缺失姿态，不能自动补单位四元数。时间来自记录的曝光纳秒，容器 FPS 只用于视频元数据。配置快照解析打符 YAML，留在 apps/buff/session。
