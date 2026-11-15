#!/usr/bin/env bash
set -eu
if [ "$(uname -s)" != "Linux" ]; then
  printf '%s\n' 'mas_vision now targets Linux only; run this script on Ubuntu.' >&2
  exit 1
fi
project_root="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
cmake -S "$project_root" -B "$project_root/build" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@"
cmake --build "$project_root/build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-8}"
