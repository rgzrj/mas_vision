# 已有依赖

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
