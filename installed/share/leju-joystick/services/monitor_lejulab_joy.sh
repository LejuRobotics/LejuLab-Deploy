#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${LEJULAB_WS_ROOT:-}" ]; then
    case "${SCRIPT_DIR}" in
        */src/leju-joystick/services)
            LEJULAB_WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
            ;;
        */share/leju-joystick/services)
            LEJULAB_WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
            ;;
        *)
            echo "错误: 无法从 ${SCRIPT_DIR} 推导 LEJULAB_WS_ROOT" >&2
            exit 1
            ;;
    esac
fi

BASHRC_PATH="${HOME:-/root}/.bashrc"

if [ -n "${BUILD_DIR:-}" ]; then
    : # 已由环境变量指定
elif [ -d "${LEJULAB_WS_ROOT}/build" ]; then
    BUILD_DIR="${LEJULAB_WS_ROOT}/build"
else
    BUILD_DIR="${LEJULAB_WS_ROOT}/build_cmake"
fi
INSTALL_DIR="${INSTALL_DIR:-${LEJULAB_WS_ROOT}/installed}"

find_binary() {
    local pkg="$1"
    local bin="$2"
    local build_path="${BUILD_DIR}/src/${pkg}/${bin}"
    if [ -x "${build_path}" ]; then
        printf '%s\n' "${build_path}"
        return
    fi
    local install_path="${INSTALL_DIR}/bin/${bin}"
    if [ -x "${install_path}" ]; then
        printf '%s\n' "${install_path}"
        return
    fi
    if command -v "${bin}" >/dev/null 2>&1; then
        command -v "${bin}"
        return
    fi
    echo "错误: 未找到二进制 '${bin}'" >&2
    echo "  build:   ${build_path}" >&2
    echo "  install: ${install_path}" >&2
    echo "  PATH:    (not found)" >&2
    exit 1
}

if [ -f "${BASHRC_PATH}" ]; then
    PS1="${PS1:-autostart}"
    set +eu
    # shellcheck disable=SC1090
    source "${BASHRC_PATH}" >/dev/null 2>&1 || true
    set -eu
fi

AUTOSTART_MANAGER_BIN="$(find_binary leju-joystick leju_joy_autostart_manager)"
JOYSTICK_BIN="$(find_binary leju-joystick leju-joystick)"

export LEJULAB_WS_ROOT
# LD_LIBRARY_PATH 需覆盖 installed/lib —— 源码模式下 liblejusdk-lowlevel.so（stub）
# 与 libddscxx.so 等预编译依赖都在那里；闭源开发树无该目录时此项为 no-op。
export LD_LIBRARY_PATH="${BUILD_DIR}/src/leju-joystick:${BUILD_DIR}/src/lejusdk/lejusdk-vr:${LEJULAB_WS_ROOT}/installed/lib:${LD_LIBRARY_PATH:-}"

child_pid=""
joystick_pid=""
stop_requested=0

# 结束一个子进程(存在才 kill, 并回收), 供 cleanup 与循环复用
kill_child() {
    local pid="$1"
    [ -n "${pid}" ] || return 0
    if kill -0 "${pid}" 2>/dev/null; then
        kill "${pid}" 2>/dev/null || true
        wait "${pid}" 2>/dev/null || true
    fi
}

cleanup() {
    stop_requested=1
    kill_child "${joystick_pid}"
    kill_child "${child_pid}"
    joystick_pid=""
    child_pid=""
}

trap cleanup SIGINT SIGTERM

while true; do
    joystick_pid=""
    child_pid=""

    "${JOYSTICK_BIN}" --headless &
    joystick_pid=$!

    "${AUTOSTART_MANAGER_BIN}" &
    child_pid=$!

    # 同时监管两个子进程: 任一退出即跳出, 统一清理后整体重启。
    # 修复点: 原实现只 wait autostart_manager, 导致 leju-joystick 单独崩溃
    # 时永不重启(stand 站立依赖 leju-joystick, 后果是遥控器 stand 无响应)。
    stop_restart=0
    while :; do
        if ! kill -0 "${joystick_pid}" 2>/dev/null; then
            if [ "${stop_requested}" -ne 0 ]; then
                stop_restart=1
                break
            fi
            echo "[monitor_lejulab_joy] leju-joystick exited, restarting joystick and autostart_manager"
            kill_child "${child_pid}";  child_pid=""
            kill_child "${joystick_pid}"; joystick_pid=""
            break
        fi
        if ! kill -0 "${child_pid}" 2>/dev/null; then
            echo "[monitor_lejulab_joy] autostart_manager exited, restarting joystick and autostart_manager"
            kill_child "${joystick_pid}"; joystick_pid=""
            kill_child "${child_pid}";  child_pid=""
            break
        fi
        sleep 0.5
    done

    if [ "${stop_restart}" -ne 0 ]; then
        exit 0
    fi
    sleep 2
done
