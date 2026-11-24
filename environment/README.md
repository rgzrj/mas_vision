# 依赖管理

| 位置 | 管理规则 |
|---|---|
| `third_party/` | 随 Git 维护的固定依赖源码、SDK 和项目适配代码，更新时保留来源与许可证。 |
| `.deps/` | 本机准备的依赖源码或 SDK，Git 忽略，可按下述步骤重新获取。 |
| `environment/` | 依赖准备脚本及离线源码归档，记录版本、来源和恢复方法。 |

新增依赖先复用系统包或已有 CMake 目标。需要随工程维护的代码放入 `third_party/`，本机下载或解压的副本放入 `.deps/`，并在本文件记录获取方式。源码形式同样可以放在 `.deps/`。

`third_party/mas_log` 是项目自有日志封装。`rm_utils` 的四个目标分别管理算法、录制回放、显示和 UDP 诊断，公开接口使用的依赖设为 `PUBLIC`，实现依赖设为 `PRIVATE`。

## 版本与来源

| 依赖 | 版本与来源 | 准备方式 |
|---|---|---|
| Eigen / yaml-cpp / spdlog | 随工程提供 `sources/` 源码归档，版本为 3.4.0 / 0.8.0 / 1.14.1。 | 使用下述解压命令，或安装系统开发包。 |
| Arrow / xsimd / mimalloc / Boost | 随工程提供 `archives/` 源码归档，版本为 18.0.0 / 13.0.0 / 2.0.6 / 1.81.0。 | CMake 自动使用本地归档，下载地址覆盖方式见下文。 |
| Rerun / Tracy | 随仓库提供 SDK 与源码，版本和许可证见 `third_party/rerun/sdk/rerun_cpp_sdk/`、`third_party/tracy/`。 | 获取完整仓库，按上游版本更新对应目录。 |
| OpenCV / SDL2 / nlohmann-json | 使用系统开发包，OpenCV 要求 4.x，JSON 要求至少 3.2。 | 安装命令见 [Linux 部署说明](../README_LINUX.md#1-基础依赖)。 |
| OpenVINO | `setup_linux.sh` 固定安装上游 PyPI 包 2026.4.1。 | 运行 `bash environment/setup_linux.sh`。 |
| TensorRT / CUDA / MVS | 使用目标设备对应的 NVIDIA / 海康厂商 SDK。 | 版本与准备步骤见 [YOLO 部署说明](../auto_buff/buff_detect/yolo/README.md#orin-nx构建与运行)和 [海康 SDK](../README_LINUX.md#2-海康sdk)。 |

系统包和显式 SDK 路径可使用部署机已安装的版本。CMake 对 yaml-cpp、spdlog 优先使用指定布局的 `.deps` 源码，Eigen 优先查找已安装的 CMake 包，再查找 `.deps/eigen-3.4.0`。

## 本地恢复

换机器后先按 [Linux 部署说明](../README_LINUX.md) 安装系统开发包和对应架构的厂商 SDK；需要恢复 `.deps` 源码时使用下面的命令。共享 OpenVINO 环境由 `setup_linux.sh` 单独管理。

archives 内已有 Arrow 18、xsimd 13、mimalloc 2.0.6、Boost 1.81 源码包，CMake 自动选用并缓存路径；Arrow 子构建会从缓存设置自己的下载环境，后续 make 不依赖终端 export。
不再需要手动 source use_local_dependencies.sh 或传 RERUN_ARROW_URL。该脚本仅为旧流程保留，显式环境变量和 CMake -D URL 优先。

sources 内是已有 Eigen、yaml-cpp、spdlog 源码归档，不是编译库。推荐先按 README_LINUX.md 安装系统开发包；需要复用源码时，在项目根目录：

```bash
mkdir -p .deps
unzip -o environment/sources/eigen-3.4.0.zip -d .deps
unzip -o environment/sources/yaml-cpp-0.8.0.zip -d .deps
unzip -o environment/sources/spdlog-1.14.1.zip -d .deps
```

不包含 Linux OpenVINO/Rerun wheel、OpenCV/SDL 系统软件包或 MVS 安装程序；不能将 Windows .venv、DLL 直接复制到 Linux。OpenVINO 安装在项目 .venv-openvino 后无需激活；只有使用 Viewer 才需要另装 rerun-sdk==0.41.0。

共享 OpenVINO 环境：从项目根运行一次 `bash environment/setup_linux.sh`，安装位置为 `${XDG_DATA_HOME:-$HOME/.local/share}/mas-vision/venv`。之后所有更新源码目录都能自动查找；已有项目环境和显式 SDK 仍优先。首次从旧缓存切换时，在原 build 执行 `cmake .. -U OpenVINO_DIR`，后续不需要激活或 export。脚本可能从 pip 缓存或网络安装，失败会停止，不自动运行 sudo/apt。
SDL 不依赖 Viewer；需要 Rerun 窗口时再在共享环境安装 rerun-sdk==0.41.0。
