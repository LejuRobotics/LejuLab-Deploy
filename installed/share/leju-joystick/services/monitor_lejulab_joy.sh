#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${LEJULAB_WS_ROOT:-}" ]; then
    case "${SCRIPT_DIR}" in
        */installed/share/leju-joystick/services)
            LEJULAB_WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
            ;;
        */src/leju-joystick/services)
            LEJULAB_WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
            ;;
        *)
            echo "错误: 无法从 ${SCRIPT_DIR} 推导 LEJULAB_WS_ROOT" >&2
            exit 1
            ;;
    esac
fi
SETUP_BASH="${LEJULAB_WS_ROOT}/devel/setup.bash"
BASHRC_PATH="/root/.bashrc"

if [ ! -f "${SETUP_BASH}" ]; then
    echo "错误: 未找到 ${SETUP_BASH}" >&2
    exit 1
fi

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "错误: 未找到 /opt/ros/noetic/setup.bash" >&2
    exit 1
fi

set +u
if [ -f "${BASHRC_PATH}" ]; then
    PS1="${PS1:-autostart}"
    source "${BASHRC_PATH}" >/dev/null 2>&1 || true
fi
source /opt/ros/noetic/setup.bash
source "${SETUP_BASH}"
set -u

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
    roslaunch leju-joystick joy_autostart_service.launch &
    child_pid=$!

    if wait "${child_pid}"; then
        exit_code=0
    else
        exit_code=$?
    fi

    child_pid=""

    if [ "${stop_requested}" -ne 0 ]; then
        exit 0
    fi

    echo "[monitor_lejulab_joy] roslaunch 退出，2 秒后重启 (exit=${exit_code})"
    sleep 2
done
