# 应用入口

- `main.cpp`：实机唯一组装入口，初始化日志、信号、队列和显示，启动相机、串口、router、armor和buff五个任务；不处理逐帧结果。
- `calibration/`：通过主程序的calibrate_camera、calibrate_handeye、calibrate_latency命令启动，分别标定内参、手眼和取图延迟；不启动实机算法线程。
- `record/`：独立采集程序，不创建算法，不发送视觉控制包，不链接实机业务线程。
- `replay/`：独立回放程序，按配置选择armor/buff和paired/media，复用生产逐帧算法。
- `record/session.cpp/.hpp`：录制流程使用的快照工具，复制相机、串口、两套算法配置及模型；回放直接读取生成的文件，不调用本工具运行算法。
- `replay/armor_process.cpp/.hpp`：回放自己的装甲板编排适配，持有生产 detector、tracker、shooter；实机线程直接持有这三个算法，不依赖回放适配。

main按六步组织：信号/日志/标定命令 → 准备显示后端 → 队列与线程池 → 五个常驻任务 → SDL事件与任务检查 → 等待退出并关闭显示。

入口只传配置路径，不解析模块字段。armor/buff 算法在内部读取 debug 开关并持有各自调试子类，调用时直接构建和提交图元；main 和 router 不持有调试对象。Display 按配置选择后端，由主线程 poll 处理 SDL 事件。router 读取 runtime 的帧有效期与运行时长，不再控制性能 CSV。

各模块debug仅控制运行时显示，全部关闭时不创建显示会话；日志独立输出。程序不再提供run/read/export/record/replay/check子命令，也没有mas_vision_dev。

armor/buff线程各自在启动时创建算法并跨帧保留，各算法内部按开关直接绘制；测速由tests独立程序完成。router统一选择armor、小符或大符、颜色和显示布局。入口不持有算法实例，不通过逐帧回调取回结果。主线程直接调用Display的start/poll/shutdown；五个任务退出后再关闭显示。

三个可执行程序各自在自己的main.cpp定义g_shutdown和注册信号，不再使用signals.hpp。实机入口直接提交线程池任务并定期检查future；任务失败时先设置退出标志，再让线程池等待，避免异常退出时挂住。
