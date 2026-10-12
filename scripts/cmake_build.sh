#!/bin/bash
# 在项目根目录执行: scripts/cmake_build.sh <build_dir> [cmake --build 参数...]
# 例: CC=gcc-11 CXX=g++-11 scripts/cmake_build.sh build_cmake
# 例: scripts/cmake_build.sh build_cmake --target leju-hardware
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=build_env.sh
source "${SCRIPT_DIR}/build_env.sh"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <build_dir> [cmake --build args...]" >&2
    exit 1
fi

leju_cmake_build "$@"
