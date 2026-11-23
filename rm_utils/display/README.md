# 公共显示模块

业务模块准备图像和图元，Display 选择后端并控制采样。显示层不解释 armor、buff 或 detect/track/shoot 的业务含义。

| 文件 | 职责 |
|---|---|
| `display.hpp/.cpp` | 公共图元、会话、布局切换、图像采样和后端分派 |
| `sdl/sdl_display.hpp/.cpp` | 最新帧任务、独立窗口、事件、纹理和文字缓存 |
| `rerun/rerun_display.hpp/.cpp` | Viewer、底图、图元、HUD、曲线和 RRD |
| `CMakeLists.txt` | 登记 `buff_display` 及其直接依赖 |

## 启动与关闭

实机调用 `start(config_path, output)` 只保存配置。领域 Debug 按 YAML 开关注册布局，router 选择领域，主线程 `poll()` 看到非空布局后才初始化选定后端。全关 debug 时不创建 SDL、Viewer 或 RRD 会话。

回放、采集可调用 `start(settings, config_path, output)` 立即创建会话。两个后端整局互斥：SDL 不构造 Rerun 对象，Rerun 不创建 SDL 窗口。

`start/poll/shutdown` 由主线程调用。退出时先停止算法提交者并等待任务结束，再关闭显示。关闭单个 SDL 窗口不停止业务，也不会被后续帧重新打开；Viewer 暂停仅影响观看。

## 一帧提交

算法线程持有 `frame_mutex()` 后依次调用：

```text
set_time(曝光/媒体时间)
    → image_due()：决定是否整理图像和图元
    → values_enabled()：决定是否整理曲线
    → 提交图像、图元、数值
    → end_frame()
```

图像采样不影响算法、采集、串口或处理 FPS。SDL 上限 30 Hz，Rerun 图像上限 60 Hz；数值曲线仍逐算法帧记录。SDL 回放暂停后的单步绕过采样，显示刚处理的帧。

SDL 提交只保留只读 `cv::Mat` 引用和按值图元，不缩放、不叠图。同一帧可由多个窗口共享底图。提交者不得覆盖仍由显示持有的存储；海康相机池通过引用计数避免复用。

`end_frame()` 整组发布最新帧，覆盖尚未显示的旧任务。主线程取走任务后释放锁，再上传纹理和绘制。

## SDL

- 每个启用视图对应一个可独立拖动、缩放和关闭的窗口，关闭事件按 windowID 路由。
- 窗口初始按 `width` 等比显示，图元使用原图坐标；`height` 参数仅保留兼容。
- 点线直接使用 SDL 绘制；文字由 OpenCV 生成透明纹理，每窗口缓存最近 50 种内容和样式，不依赖 SDL_ttf。
- 硬件渲染器优先使用原生 BGR24；只支持 BGRA32 时，同次 `poll()` 的同源窗口共享一次颜色转换。软件渲染保留 BGR24 路径。
- 同源按地址、尺寸和步长判断。每次 `poll()` 清除源图标识，只复用已分配缓冲，避免相机复用地址后显示旧帧。

`spawn/address/recording/image_encoding/hud_encoding` 仅用于 Rerun，SDL 不解析这些配置，也不写 RRD。

## Rerun

有领域布局时，视图共享 `camera/image`；各自图元和 HUD 位于 `<视图名>/image` 下。独立预览使用各自底图路径。`DisplayView::hud_height` 提供状态区高度，显示层不猜测业务名称。

`image_encoding` 支持 `jpeg/bgr`，`hud_encoding` 支持 `png/bgra`。底图、图元、HUD 分开提交，HUD 使用状态区高度。图元即使为空也提交，清除上一帧观测。

`scalar()` 的 NaN/Inf 记录非递归 `Clear::FLAT`，清除当前路径并保留历史；后续有限值恢复曲线。`point3d()` 的非有限坐标记录空点集。SDL 不处理这些数值接口。

领域布局位于 `auto_aim/debug/blueprints`、`auto_buff/debug/blueprints`。修改布局后，使用安装了 `rerun-sdk==0.41.0` 的 Python 运行对应 `generate.py`，再重新编译；普通运行不需要 Python 生成布局。

## 构建与验证

`buff_display` 显式依赖 OpenCV、日志、yaml-cpp、SDL2 和 Rerun SDK，不依赖 `buff_io`。SDK 位于 `third_party/rerun/sdk`；SDL2 使用 Linux 系统 `libsdl2-dev`。运行时选择 SDL 仍会链接 Rerun/Arrow，未做编译期裁剪。

现有显示测试覆盖独立窗口、资源生命周期、采样、暂停单步、按键及曲线清理。`tools/sdl_upload_profile.cpp` 独立比较 BGR24/共享 BGRA32 的转换、上传和绘制成本，不给实机增加统计开关。Windows 软件渲染结果不能代替目标 Linux/OpenGL 和 NoMachine 环境实测。
