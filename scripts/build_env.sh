#!/bin/bash
# RK3588 big.LITTLE 编译环境辅助
# isolcpus=4-7 时 nproc 仅统计小核(0-3)，需 taskset 显式使用大核(4-7)编译

leju_detect_build_cores() {
    if grep -q 'isolcpus=4-7' /proc/cmdline 2>/dev/null && command -v taskset >/dev/null 2>&1; then
        LEJU_BUILD_CPUSET="4-7"
        # Ninja 下略超订阅：编译与链接可重叠，4 核常用 -j6
        if [ -f "${1:-}/build.ninja" ] || [ -f "${1:-}/.ninja_log" ]; then
            LEJU_BUILD_JOBS=6
        else
            LEJU_BUILD_JOBS=4
        fi
        LEJU_BUILD_CORE_FIRST=4
        LEJU_BUILD_CORE_LAST=7
    else
        LEJU_BUILD_CPUSET=""
        LEJU_BUILD_JOBS="$(nproc)"
        LEJU_BUILD_CORE_FIRST=""
        LEJU_BUILD_CORE_LAST=""
    fi
}

# 轮询绑定每个编译器进程到不同大核；需在 cmake 配置阶段传入
leju_compiler_launcher_path() {
    leju_detect_build_cores
    if [ -n "$LEJU_BUILD_CPUSET" ]; then
        local dir
        dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        echo "${dir}/bigcore_compiler_launcher.sh"
    fi
}

leju_cmake_configure_extra_args() {
    local launcher
    launcher="$(leju_compiler_launcher_path)"
    if [ -n "$launcher" ]; then
        echo "-DCMAKE_C_COMPILER_LAUNCHER=${launcher} -DCMAKE_CXX_COMPILER_LAUNCHER=${launcher}"
    fi
}

# Usage: leju_cmake_build <build_dir> [cmake --build args...]
leju_cmake_build() {
    local build_dir="$1"
    shift

    leju_detect_build_cores "$build_dir"

    if [ -n "$LEJU_BUILD_CPUSET" ]; then
        echo "[build] RK3588: big cores ${LEJU_BUILD_CPUSET}, -j${LEJU_BUILD_JOBS}"
        taskset -c "$LEJU_BUILD_CPUSET" cmake --build "$build_dir" -j"$LEJU_BUILD_JOBS" "$@"
    else
        cmake --build "$build_dir" -j"$LEJU_BUILD_JOBS" "$@"
    fi
}

# Usage: leju_cmake_build_dir [build_subdir] [cmake --build args...]
# 在第三方库脚本中使用，默认 build 子目录
leju_cmake_build_dir() {
    local build_dir="build"
    if [ $# -gt 0 ] && [[ "$1" != --* ]] && [[ "$1" != -* ]]; then
        build_dir="$1"
        shift
    fi
    leju_cmake_build "$build_dir" "$@"
}
