# 统一显示出口

自有 SDL/Rerun 适配代码集中在本目录，第三方 Rerun SDK 保留在 third_party/rerun/sdk。SDL2 使用 Linux 系统 libsdl2-dev。

- display.hpp/.cpp：公开图元、会话生命周期、后端路由和 Rerun 图像采样。应用继续调用 Display。提交端通过 image_due 在复制前询问图像采样，BuffDebug 不查询后端类型；Display 的提交前判断与绘制判断共用同一采样实现。
- sdl/sdl_display.hpp/.cpp：缩放叠图、最新画面槽、SDL窗口和事件。窗口操作在入口主线程执行。
- rerun/rerun_display.hpp/.cpp：Viewer、SDK、共享底图、状态区编码、数值与RRD录制。由Display串行调用，不另建调试线程。

实机直接调用Display::start(config_path, output)，只读取并准备公共display配置。领域Debug内部按模块开关调用register_layout，router通过select_layout选择领域名，主线程poll确认该领域有非空视图后才创建后端；全关debug时不初始化SDL、Viewer或RRD会话。回放仍可传入调整后的settings节点立即启动。DisplayView描述通用窗口名称和可选HUD高度，显示层不解析detect/track/shoot开关。领域布局及生成脚本位于auto_buff/debug/blueprints和auto_aim/debug/blueprints，分别嵌入领域debug目标；运行时不依赖Python。

语义绘图分别在auto_buff/debug/visualize和auto_aim/debug/visualize，领域Debug类负责结果快照与异步工作线程。ArmorDebug复制同帧ArmorPose，绘制不借用算法线程的可变对象。没有另一套算法状态，也没有虚类或后端工厂。

SDL 为 detect/track/shoot 分别创建独立窗口，各自具有渲染器与纹理；禁用模块不创建窗口，单个关闭后不会被新帧重新打开。SDL 显示本机实时画面；RRD 仅记录数值和三维点，不启动 Viewer、不编码图像。录制路径为空时可单独使用 SDL。
Rerun 显示并保存图像与曲线。三个视图共享 camera/image，文字与图元放在各自的 detect/image、track/image、shoot/image 子树。HUD 仅编码状态区，不编码整幅透明图。
现有 image_encoding: jpeg/bgr、hud_encoding: png/bgra 用于 Rerun 对照；SDL 忽略它们。

Rerun 图像最高按 60 Hz 采样；SDL 取消该限制，主线程始终取最新已完成画面，允许跳过未显示旧帧。Process FPS 表示处理吞吐，不代表提交帧数或上屏率。
start/poll/shutdown由主线程调用。领域注册与选择只同步布局描述，主线程poll才创建后端、应用布局及操作窗口。处理线程结束自己的Debug工作线程后退出，main等待全部任务，再调用Display::shutdown。Viewer暂停不阻塞算法，关闭SDL窗口也不停止串口。启动Viewer时不持提交锁，算法线程可继续处理。

修改 Rerun 布局时，用安装了 rerun-sdk==0.41.0 的 Python 运行 `python auto_buff/debug/blueprints/generate.py`，随后重新编译。普通使用无需重新生成。

显示目标是buff_display，直接链接图像能力、yaml-cpp、SDL2和Rerun SDK，不链接buff_io。算法模块不链接此目标；保留数值RRD功能，因此完整程序仍链接Rerun/Arrow，选择SDL不等于编译时删除Rerun。Plotter是独立UDP诊断出口，不依赖Display或图像debug开关。

display_add保留MAS原有width/height参数；SDL当前按width等比缩放，height不参与计算，Rerun记录原图并忽略这两个呈现尺寸。

SDL 按 windowID 路由关闭事件；关闭一个不影响其余窗口。窗口初始位置错开，可分别拖动缩放。图像保留最新帧，不再横向拼接。回放按键在任一模块窗口生效；Rerun 保持现有单 Viewer 的分视图布局。
