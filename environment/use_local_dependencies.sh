#!/usr/bin/env bash
# Source in the same shell used for the first configure/build of Arrow.
buff_env_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export RERUN_ARROW_URL="$buff_env_dir/archives/apache-arrow-18.0.0.tar.gz"
export ARROW_XSIMD_URL="$buff_env_dir/archives/xsimd-13.0.0.tar.gz"
export ARROW_MIMALLOC_URL="$buff_env_dir/archives/mimalloc-v2.0.6.tar.gz"
export ARROW_BOOST_URL="$buff_env_dir/archives/boost-1.81.0.tar.gz"
unset buff_env_dir
