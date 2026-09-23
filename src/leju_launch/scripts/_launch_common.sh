#!/usr/bin/env bash
# _launch_common.sh — shared functions for launch_*.sh scripts.
# Sourced, not executed directly.

set -euo pipefail

# ── Path resolution ──────────────────────────────────────────────────
# 路径解析支持两种模式:
#   1. 源码模式（开发者）: 从 build_cmake/ 找可执行
#   2. 安装模式（aarch64 闭源发布产物）: setup.bash 注入 LEJULAB_INSTALL_ROOT，
#      从其 bin/ 找可执行（PATH 也已注入，第 3 重 fallback 自动命中）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${LEJULAB_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
SRC_DIR="${PROJECT_DIR}/src"
# 自动探测构建目录：优先环境变量，其次 build/，最后 build_cmake/
if [[ -n "${BUILD_DIR:-}" ]]; then
    : # 已由环境变量指定，直接使用
elif [[ -d "${PROJECT_DIR}/build" ]]; then
    BUILD_DIR="${PROJECT_DIR}/build"
else
    BUILD_DIR="${PROJECT_DIR}/build_cmake"
fi
INSTALL_DIR="${INSTALL_DIR:-${LEJULAB_INSTALL_ROOT:-${PROJECT_DIR}/installed}}"

# ── Defaults ─────────────────────────────────────────────────────────
COREDUMP=false
NO_SHM=false
AUTO_START=false
PRE_START_FALL_RECOVERY=false

# ROBOT_VERSION: 优先用已有环境变量，否则从 /root/.bashrc 中读取
if [[ -z "${ROBOT_VERSION:-}" ]]; then
    _detected="$(bash -ic 'printf %s "${ROBOT_VERSION:-}"' 2>/dev/null || true)"
    if [[ -n "${_detected}" ]]; then
        ROBOT_VERSION="${_detected}"
    else
        echo "错误: 未设置 ROBOT_VERSION，请在 /root/.bashrc 中添加:" >&2
        echo "  export ROBOT_VERSION=17" >&2
        exit 1
    fi
fi
export ROBOT_VERSION
CONTROLLER_MANAGER_CONFIG=""
TELEOP_CONFIG=""
URDF_PATH=""
SIM_KEYFRAME=""

# ── PID tracking ─────────────────────────────────────────────────────
REQUIRED_PIDS=()
ALL_PIDS=()

# ── Argument parsing ─────────────────────────────────────────────────
parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --coredump)              COREDUMP=true ;;
            --no-shm)                NO_SHM=true ;;
            --robot-version=*)       ROBOT_VERSION="${1#*=}" ;;
            --robot-version)         shift; ROBOT_VERSION="$1" ;;
            --auto-start)            AUTO_START=true ;;
            --pre-start-fall-recovery) PRE_START_FALL_RECOVERY=true ;;
            --config=*)              CONTROLLER_MANAGER_CONFIG="${1#*=}" ;;
            --config)                shift; CONTROLLER_MANAGER_CONFIG="$1" ;;
            --teleop-config=*)       TELEOP_CONFIG="${1#*=}" ;;
            --teleop-config)         shift; TELEOP_CONFIG="$1" ;;
            --urdf-path=*)           URDF_PATH="${1#*=}" ;;
            --urdf-path)             shift; URDF_PATH="$1" ;;
            --sim-keyframe=*)        SIM_KEYFRAME="${1#*=}" ;;
            --sim-keyframe)          shift; SIM_KEYFRAME="$1" ;;
            -h|--help)
                echo "Usage: $(basename "$0") [OPTIONS]"
                echo ""
                echo "Options:"
                echo "  --coredump                    Enable core dump collection"
                echo "  --no-shm                      Disable iceoryx shared memory (use standard CycloneDDS)"
                echo "  --robot-version=XX            Set robot version (e.g. 14, 45, 46, 52)"
                echo "  --auto-start                  Skip joystick launch (headless / auto-start mode)"
                echo "  --pre-start-fall-recovery     Replay the LB+RB+X event that started runtime"
                echo "  --config=PATH                 Path to controller_manager.yaml (overrides default)"
                echo "  --teleop-config=PATH          Path to teleop_bindings.yaml (overrides default)"
                echo "  --urdf-path=PATH              Path to robot URDF file (passed to run_rl_controller)"
                echo "  --sim-keyframe=NAME           MuJoCo startup keyframe (e.g. home or lie0)"
                exit 0
                ;;
            *)
                echo "Unknown argument: $1" >&2
                echo "Usage: $(basename "$0") [--coredump] [--no-shm] [--robot-version=XX]" >&2
                echo "       [--auto-start] [--config=PATH] [--teleop-config=PATH] [--urdf-path=PATH]" >&2
                echo "       [--sim-keyframe=NAME]" >&2
                exit 1
                ;;
        esac
        shift
    done
}

# ── Bag cleanup ──────────────────────────────────────────────────────

cleanup_old_bags() {
    # 机器人启动时只保留最近 N 个 bag 文件，删除更早的以释放磁盘空间。
    local bag_dir="${HOME}/.ros/lejulab/mcap"
    local keep_count="${BAG_KEEP_COUNT:-50}"

    if [[ ! -d "${bag_dir}" ]]; then
        return 0
    fi

    # Collect .mcap files sorted by modification time (oldest first)
    local old_IFS="$IFS"
    IFS=$'\n'
    local files
    files=($(find "${bag_dir}" -maxdepth 1 -type f -name '*.mcap' -printf '%T@ %p\n' 2>/dev/null | sort -n | cut -d' ' -f2-))
    IFS="$old_IFS"

    local total=${#files[@]}
    if (( total <= keep_count )); then
        echo "[launcher] Bag cleanup: ${total} files, 无需清理 (阈值: ${keep_count})"
        return 0
    fi

    local remove_count=$(( total - keep_count ))
    echo "[launcher] Bag cleanup: ${total} files, 将删除 ${remove_count} 个旧文件"

    local i
    for (( i = 0; i < remove_count; i++ )); do
        echo "[launcher] Bag cleanup: 删除 $(basename "${files[$i]}")"
        rm -f "${files[$i]}"
    done

    echo "[launcher] Bag cleanup: 完成，保留最新 ${keep_count} 个文件"
}

# ── Environment setup ────────────────────────────────────────────────
setup_env() {
    cleanup_old_bags
    local config_dir="${SRC_DIR}/leju_launch/config"

    if [[ "${NO_SHM}" == "true" ]]; then
        # --no-shm: 直接用工作区的非共享内存版本
        export CYCLONEDDS_URI="file://${config_dir}/cyclonedds.xml"
    elif [[ -f "/etc/cyclonedds/cyclonedds_shm.xml" ]]; then
        # ETC 有通过 setup_cyclonedds_config.sh 部署的 shm 配置，优先使用
        # 保证与遥控器自启动服务等系统级进程使用同一份 DDS 配置，避免接口隔离
        export CYCLONEDDS_URI="file:///etc/cyclonedds/cyclonedds_shm.xml"
    else
        # 兜底：ETC 没有，使用工作区自带配置
        export CYCLONEDDS_URI="file://${config_dir}/cyclonedds_shm.xml"
    fi
}

# ── Path helpers ─────────────────────────────────────────────────────

# Resolve source package path (replaces roslaunch $(find pkg))
find_package() {
    local pkg="$1"
    echo "${SRC_DIR}/${pkg}"
}

# Resolve binary path from build or install dir
find_binary() {
    local pkg="$1"
    local bin="$2"

    # 1. Build tree (for development)
    local build_path="${BUILD_DIR}/src/${pkg}/${bin}"
    if [[ -x "${build_path}" ]]; then
        echo "${build_path}"
        return
    fi

    # 2. Install tree (for deployment)
    local install_path="${INSTALL_DIR}/bin/${bin}"
    if [[ -x "${install_path}" ]]; then
        echo "${install_path}"
        return
    fi

    # 3. Fallback: PATH
    if command -v "${bin}" &>/dev/null; then
        command -v "${bin}"
        return
    fi

    echo "ERROR: binary '${bin}' not found in:" >&2
    echo "  build:   ${build_path}" >&2
    echo "  install: ${install_path}" >&2
    echo "  PATH:    (not found)" >&2
    exit 1
}

# ── Node launchers ───────────────────────────────────────────────────

_launch() {
    local required="$1"; shift
    local pkg="$1"; shift
    local bin="$1"; shift
    # remaining args are passed to the binary

    local binary_path
    binary_path="$(find_binary "${pkg}" "${bin}")"

    local coredump_arg=""
    [[ "${COREDUMP}" == "true" ]] && coredump_arg="--with-coredump"

    # shellcheck disable=SC2086
    "${SCRIPT_DIR}/start_node.sh" ${coredump_arg} "${binary_path}" "$@" &
    local pid=$!
    ALL_PIDS+=("${pid}")
    if [[ "${required}" == "true" ]]; then
        REQUIRED_PIDS+=("${pid}")
    fi
    echo "[launcher] Started ${bin} (PID ${pid}, required=${required})"
}

launch_required_node() {
    _launch true "$@"
}

launch_node() {
    _launch false "$@"
}

# ── Process management ───────────────────────────────────────────────

_cleanup() {
    echo ""
    echo "[launcher] Shutting down all nodes..."
    for pid in "${ALL_PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -TERM "${pid}" 2>/dev/null || true
        fi
    done
    # 等待节点优雅退出后再 SIGKILL 兜底。硬件节点退出前要对电机掉使能
    # (RUIWO 最坏 1s 线程等待 + 逐电机 CAN 往返), 固定 0.5s 会把它杀在半路。
    # 轮询等待最多 5s, 全部退出则提前结束, 正常路径不变慢。
    local waited=0
    while (( waited < 50 )); do
        local alive=false
        for pid in "${ALL_PIDS[@]}"; do
            if kill -0 "${pid}" 2>/dev/null; then
                alive=true
                break
            fi
        done
        [[ "${alive}" == "false" ]] && break
        sleep 0.1
        waited=$((waited + 1))
    done
    for pid in "${ALL_PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            echo "[launcher] PID ${pid} 优雅退出超时, 强制 SIGKILL"
            kill -KILL "${pid}" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true

    # 清理 iceoryx 残留锁文件 (进程被 kill 后不会自动清理, 积累后会阻塞新进程)
    local iox_locks
    iox_locks=$(ls /tmp/iceoryx_rt_*.lock 2>/dev/null)
    if [[ -n "${iox_locks}" ]]; then
        rm -f /tmp/iceoryx_rt_*.lock
        echo "[launcher] Cleaned iceoryx runtime lock files"
    fi

    echo "[launcher] All nodes stopped."
}

wait_and_cleanup() {
    trap _cleanup EXIT INT TERM

    # Monitor required processes — exit if any required node dies
    while true; do
        for pid in "${REQUIRED_PIDS[@]}"; do
            if ! kill -0 "${pid}" 2>/dev/null; then
                echo "[launcher] Required node (PID ${pid}) exited."
                exit 1
            fi
        done
        sleep 1
    done
}
