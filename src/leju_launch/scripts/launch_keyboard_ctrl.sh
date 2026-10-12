#!/usr/bin/env bash
# launch_keyboard_ctrl.sh — Real hardware + keyboard motor controller
#
# hardware_node 在独立进程组运行（不受 Ctrl+C 影响）
# keyboard_ctrl 前台运行（接收键盘输入和 Ctrl+C）
# 退出顺序: keyboard_ctrl 回零退出 → 脚本 SIGTERM 停止 hardware_node

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_launch_common.sh"

# 分离 launch 参数和 keyboard_ctrl 参数
# launch 参数: --coredump, --no-shm, --robot-version, --bypass-ankle-solver
# 其余参数 (--mode, --freq, --step 等) 透传给 keyboard_ctrl
LAUNCH_ARGS=()
EXTRA_ARGS=()
HW_EXTRA_ARGS=()     # 额外透传给 hardware_node 的参数
skip_next=false
for arg in "$@"; do
    if [[ "${arg}" == "--" ]]; then
        continue
    fi
    if ${skip_next}; then
        LAUNCH_ARGS+=("${arg}")
        skip_next=false
        continue
    fi
    case "${arg}" in
        --coredump|--no-shm|-h|--help)
            LAUNCH_ARGS+=("${arg}") ;;
        --bypass-ankle-solver)
            HW_EXTRA_ARGS+=("${arg}")
            EXTRA_ARGS+=("${arg}") ;;
        --robot-version=*)
            LAUNCH_ARGS+=("${arg}") ;;
        --robot-version)
            LAUNCH_ARGS+=("${arg}"); skip_next=true ;;
        --arm-freq=*)
            export CAN_ARM_FREQ="${arg#*=}" ;;
        --arm-freq)
            skip_next=true; _arm_freq_next=true ;;
        *)
            if [[ "${_arm_freq_next:-}" == "true" ]]; then
                export CAN_ARM_FREQ="${arg}"
                _arm_freq_next=false
            else
                EXTRA_ARGS+=("${arg}")
            fi
            ;;
    esac
done

parse_args "${LAUNCH_ARGS[@]}"
setup_env

export ROBOT_VERSION="${ROBOT_VERSION:-14}"

# ---- 启动 hardware_node (独立进程组，不接收终端 SIGINT) ----
HW_BIN="$(find_binary leju-hardware leju-hardware)"
HW_ARGS="$(find_package leju-hardware)/"

echo "[launcher] Starting hardware_node..."
setsid "${SCRIPT_DIR}/start_node.sh" "${HW_BIN}" "${HW_ARGS}" "${HW_EXTRA_ARGS[@]}" &
HW_PID=$!
echo "[launcher] hardware_node PID=${HW_PID} (isolated process group)"

# 等待 hardware_node 启动
sleep 3

# 确认 hardware_node 还活着
if ! kill -0 "${HW_PID}" 2>/dev/null; then
    echo "[launcher] ERROR: hardware_node exited prematurely"
    exit 1
fi

# ---- 清理函数 ----
cleanup_hw() {
    # keyboard_ctrl 已通过 DDS 发送 StopRobot 并等待电机失能完成
    # 此时 hardware_node 可能已经自行退出，检查一下
    if ! kill -0 "${HW_PID}" 2>/dev/null; then
        echo "[launcher] hardware_node already exited."
        wait "${HW_PID}" 2>/dev/null || true
        return
    fi
    echo "[launcher] Stopping hardware_node..."
    # 发 SIGTERM 给 hardware_node 进程组
    kill -TERM -"${HW_PID}" 2>/dev/null || kill -TERM "${HW_PID}" 2>/dev/null || true
    # 等待 hardware_node 完成失能序列后退出 (最长 8 秒)
    for i in $(seq 1 16); do
        if ! kill -0 "${HW_PID}" 2>/dev/null; then
            break
        fi
        sleep 0.5
    done
    # 确保进程已退出
    kill -0 "${HW_PID}" 2>/dev/null && kill -KILL -"${HW_PID}" 2>/dev/null || true
    wait "${HW_PID}" 2>/dev/null || true
    echo "[launcher] hardware_node stopped."
}

# ---- 启动 lejusdk_recorder (后台，录制 DDS 话题到 MCAP) ----
RECORDER_BIN="$(find_binary lejusdk/lejusdk-recorder/recorder lejusdk_recorder 2>/dev/null || true)"
RECORDER_CONFIG="$(find_package leju_launch)/config/recorder.yaml"
RECORDER_PID=""
if [[ -n "${RECORDER_BIN}" && -x "${RECORDER_BIN}" ]]; then
    echo "[launcher] Starting lejusdk_recorder..."
    setsid "${RECORDER_BIN}" --config="${RECORDER_CONFIG}" &
    RECORDER_PID=$!
    echo "[launcher] lejusdk_recorder PID=${RECORDER_PID}"
else
    echo "[launcher] WARN: lejusdk_recorder not found, skipping DDS recording"
fi

# ---- 前台运行键盘控制器 ----
KEYBOARD_BIN="$(find_binary leju-controllers/leju-keyboard-controller motor_keyboard_ctrl)"
echo "[launcher] Starting motor_keyboard_ctrl in foreground..."

trap '' INT  # 脚本自身忽略 SIGINT，只让 keyboard_ctrl 收到
"${KEYBOARD_BIN}" "${EXTRA_ARGS[@]}"
trap - INT

# ---- keyboard_ctrl 已退出，停止 recorder ----
if [[ -n "${RECORDER_PID}" ]] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
    echo "[launcher] Stopping lejusdk_recorder..."
    kill -TERM -"${RECORDER_PID}" 2>/dev/null || kill -TERM "${RECORDER_PID}" 2>/dev/null || true
    wait "${RECORDER_PID}" 2>/dev/null || true
    echo "[launcher] lejusdk_recorder stopped."
fi

# ---- 清理 hardware_node ----
echo "[launcher] Keyboard controller exited, cleaning up..."
cleanup_hw
