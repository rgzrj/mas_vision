# 统一显示出口

自有 SDL/Rerun 适配代码集中在本目录，第三方 Rerun SDK 保留在 third_party/rerun/sdk。SDL2 使用 Linux 系统 libsdl2-dev。

- display.hpp/.cpp：公开图元、会话生命周期、后端路由和图像采样。应用继续调用 Display。set_time选定本帧图像后，领域Debug通过image_due()在生成图元前判断是否绘制；values_enabled()只在Rerun会话中为true，SDL跳过曲线数据整理。
- sdl/sdl_display.hpp/.cpp：轻量图像/图元任务、最新任务槽、文字纹理缓存、SDL窗口和事件。纹理上传、缩放和实际绘制在入口主线程执行。
- rerun/rerun_display.hpp/.cpp：Viewer、SDK、共享底图、状态区编码、数值与RRD录制。由Display串行调用，不另建调试线程。

实机直接调用Display::start(config_path, output)，只读取并准备公共display配置。领域Debug内部按模块开关调用register_layout，router通过select_layout选择领域名，主线程poll确认该领域有非空视图后才创建后端；全关debug时不初始化SDL、Viewer或RRD会话。回放仍可传入调整后的settings节点立即启动。DisplayView描述通用窗口名称和可选HUD高度，显示层不解析detect/track/shoot开关。领域布局及生成脚本位于auto_buff/debug/blueprints和auto_aim/debug/blueprints，分别嵌入领域debug目标；运行时不依赖Python。

语义图元分别由auto_buff/debug/visualize和auto_aim/debug/visualize在算法内部准备。领域Debug不持有异步工作线程，也不复制整帧图像；SDL在采样后保存只读cv::Mat引用及按值图元，同一帧的三个窗口共享底图存储。海康相机池检测引用计数，待显示图像仍被持有时使用其他存储，避免覆盖。其他调用方同样不能原地改写已提交的存储。

SDL 为 detect/track/shoot 分别创建独立窗口，各自具有渲染器与纹理；禁用模块不创建窗口，单个关闭后不会被新帧重新打开。SDL只显示本机实时画面，不创建Rerun对象、不提交SDK数据、不写RRD；spawn、address、recording、image_encoding和hud_encoding均为Rerun专用配置，SDL不解析。
Rerun独立管理Viewer、远程连接、图像、曲线和RRD；它不创建SDL窗口。三个视图共享 camera/image，文字与图元放在各自的 detect/image、track/image、shoot/image 子树。HUD 仅编码状态区，不编码整幅透明图。
现有 image_encoding: jpeg/bgr、hud_encoding: png/bgra 用于 Rerun 对照；SDL 忽略它们。

Rerun图像按最高60 Hz采样；SDL按最高30 Hz在提交前采样，不复制、不绘制被跳过的图像。仅Rerun逐算法帧记录数值曲线，图像采样不影响曲线频率；SDL暂停回放后的单步绕过图像采样，保证显示刚处理的帧。主线程取走最新一组任务后释放提交锁，再执行SDL绘制，落后时待显示任务整组替换。Process FPS继续统计算法调用吞吐，不代表窗口刷新率。
start/poll/shutdown由主线程调用。领域注册与选择只同步布局描述，主线程poll才创建后端、应用布局及操作窗口。算法任务先退出，main等待全部任务，再调用Display::shutdown。Viewer暂停不阻塞算法，关闭SDL窗口也不停止串口。启动Viewer时不持提交锁，算法线程可继续处理。

修改 Rerun 布局时，用安装了 rerun-sdk==0.41.0 的 Python 运行 `python auto_buff/debug/blueprints/generate.py`，随后重新编译。普通使用无需重新生成。

显示目标是buff_display，直接链接图像能力、yaml-cpp、SDL2和Rerun SDK，不链接buff_io。算法模块通过自己的Debug子类使用此目标；两个后端运行时互斥，完整程序仍链接SDL2和Rerun/Arrow，选择SDL不等于编译时删除Rerun。Plotter是独立UDP诊断出口，不依赖Display或图像debug开关。

display_add保留MAS原有width/height参数；SDL窗口初始按width等比设置大小，底图由SDL纹理缩放，图元使用原图坐标；height不参与计算，Rerun记录原图并忽略这两个呈现尺寸。

SDL 按 windowID 路由关闭事件；关闭一个不影响其余窗口。窗口初始位置错开，可分别拖动缩放。图像保留最新帧，不再横向拼接。回放按键在任一模块窗口生效；Rerun 保持现有单 Viewer 的分视图布局。

SDL点线直接由渲染器绘制；文字使用现有OpenCV生成透明纹理，每窗口缓存最近50种内容/字号/颜色/粗细组合，位置变化复用纹理，不依赖SDL_ttf或外部字体文件。窗口析构统一释放底图与文字纹理。
