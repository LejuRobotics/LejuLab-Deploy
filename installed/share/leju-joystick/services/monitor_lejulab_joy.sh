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
stop_requested=0

cleanup() {
    stop_requested=1
    if [ -n "${child_pid}" ] && kill -0 "${child_pid}" 2>/dev/null; then
        kill "${child_pid}" 2>/dev/null || true
        wait "${child_pid}" 2>/dev/null || true
    fi
}

trap cleanup SIGINT SIGTERM

while true; do
    "${JOYSTICK_BIN}" --headless &
    joystick_pid=$!

    "${AUTOSTART_MANAGER_BIN}" &
    child_pid=$!

    if wait "${child_pid}"; then
        exit_code=0
    else
        exit_code=$?
    fi

    kill "${joystick_pid}" 2>/dev/null || true
    wait "${joystick_pid}" 2>/dev/null || true
    child_pid=""

    if [ "${stop_requested}" -ne 0 ]; then
        exit 0
    fi

    echo "[monitor_lejulab_joy] autostart_manager 退出，2 秒后重启 (exit=${exit_code})"
    sleep 2
done
