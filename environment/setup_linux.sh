#!/usr/bin/env bash
set -eu
if [ "$(uname -s)" != "Linux" ]; then
  printf '%s\n' 'Run this setup on Linux.' >&2
  exit 1
fi

# 安装一次，后续源码解压/更新不更换此环境。系统 apt 依赖仍由用户安装。
data_home="${XDG_DATA_HOME:-${HOME:?HOME is not set}/.local/share}"
venv="$data_home/mas-vision/venv"
if [ ! -x "$venv/bin/python" ]; then
  python3 -m venv "$venv"
fi
"$venv/bin/python" -m pip install --disable-pip-version-check openvino==2026.4.1
"$venv/bin/python" -c 'import openvino as ov; from openvino.utils import get_cmake_path; print("OpenVINO:", ov.__version__, "devices:", ov.Core().available_devices); print("OpenVINO config:", get_cmake_path())'
printf 'Shared environment: %s\n' "$venv"
printf '%s\n' 'No activation is needed. In your existing build: cmake .. -U OpenVINO_DIR'
