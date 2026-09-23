#!/bin/bash
# 在项目根目录执行:
#   CC=gcc-11 CXX=g++-11 scripts/cmake_configure.sh build_cmake -DCMAKE_BUILD_TYPE=Release
#   CC=gcc-11 CXX=g++-11 scripts/cmake_configure.sh --fast build_cmake   # 跳过 tests/examples，更快
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
# shellcheck source=build_env.sh
source "${SCRIPT_DIR}/build_env.sh"

FAST_BUILD=0
if [ "${1:-}" = "--fast" ]; then
    FAST_BUILD=1
    shift
fi

BUILD_DIR="${1:-build_cmake}"
shift || true

leju_detect_build_cores

if command -v ccache >/dev/null 2>&1; then
    export CCACHE_DIR="${CCACHE_DIR:-${HOME}/.cache/ccache/lejulab}"
    export CCACHE_MAXSIZE="${CCACHE_MAXSIZE:-10G}"
    export CCACHE_COMPILERCHECK=content
    mkdir -p "$CCACHE_DIR"
    echo "[configure] ccache enabled: ${CCACHE_DIR}"
else
    echo "[configure] tip: sudo apt install ccache 可显著加速增量编译"
fi

generator_args=()
if command -v ninja >/dev/null 2>&1; then
    generator_args=(-G Ninja)
    echo "[configure] generator: Ninja"
else
    echo "[configure] tip: sudo apt install ninja-build 可加速编译"
fi

extra_args=()
if [ -n "$(leju_compiler_launcher_path)" ]; then
    read -r -a extra_args <<< "$(leju_cmake_configure_extra_args)"
    echo "[configure] RK3588: compiler launcher round-robin on cores ${LEJU_BUILD_CORE_FIRST}-${LEJU_BUILD_CORE_LAST}"
fi

fast_args=()
if [ "$FAST_BUILD" -eq 1 ]; then
    fast_args=(
        -DBUILD_TESTS=OFF
        -DBUILD_TESTING=OFF
        -DBUILD_EXAMPLES=OFF
    )
    echo "[configure] fast mode: skip tests/examples"
fi

cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/$BUILD_DIR" \
    "${generator_args[@]}" \
    "${extra_args[@]}" \
    "${fast_args[@]}" \
    "$@"
