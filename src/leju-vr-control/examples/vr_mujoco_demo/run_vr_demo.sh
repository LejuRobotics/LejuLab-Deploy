#!/usr/bin/env bash
# ==============================================================================
# run_vr_demo.sh — VR + MuJoCo 全链路演示启动脚本
#
# 架构：
#   quest_udp_to_dds_node  ← UDP ← Quest3
#         │ DDS: QuestBonePoses
#         ▼
#   quest_vr_abs_control_node（IK 求解 → 写 VR_STATE_FILE）
#         │ DDS: ArmTrajectory（发送到机器人，可选）
#         │ 文件: /tmp/vr_ik_state.json
#         ▼
#   mujoco_viewer.py（读状态文件 → MuJoCo 可视化）
#
# 用法：
#   bash run_vr_demo.sh [选项]
#
# 选项：
#   --no-mujoco       不启动 MuJoCo 可视化窗口
#   --build-dir DIR   构建目录（默认 ${REPO_ROOT}/build）
#   --assets-dir DIR  leju_assets 目录（默认 ${REPO_ROOT}/src/leju_assets）
#   --display DISP    X display（默认 :1，用于 VNC）
#   --hz FLOAT        MuJoCo 刷新频率（默认 30）
#   --base-z FLOAT    机器人底座高度（默认 0.95m）
# ==============================================================================

set -euo pipefail

# ── 脚本自身路径 ──────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../../" && pwd)"

# ── 默认参数 ──────────────────────────────────────────────────────────────────
BUILD_DIR="${REPO_ROOT}/build"
ASSETS_DIR="${REPO_ROOT}/src/leju_assets"
X_DISPLAY=":1"
MUJOCO_HZ="30"
BASE_Z="0.95"
ENABLE_MUJOCO=true
VR_STATE_FILE="/tmp/vr_ik_state.json"
ROBOT_VERSION="${ROBOT_VERSION:-17}"
VR_CONTROL_DT="${VR_CONTROL_DT:-0.01}"

# ── 参数解析 ──────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-mujoco)     ENABLE_MUJOCO=false ;;
    --build-dir)     BUILD_DIR="$2";  shift ;;
    --assets-dir)    ASSETS_DIR="$2"; shift ;;
    --display)       X_DISPLAY="$2";  shift ;;
    --hz)            MUJOCO_HZ="$2";  shift ;;
    --base-z)        BASE_Z="$2";     shift ;;
    *) echo "[run_vr_demo] Unknown option: $1"; exit 1 ;;
  esac
  shift
done

# ── 可执行文件路径 ─────────────────────────────────────────────────────────────
UDP_DDS_NODE="${BUILD_DIR}/src/leju-remote/quest_udp_to_dds_node"
ABS_CTRL_NODE="${BUILD_DIR}/src/leju-vr-control/quest_vr_abs_control_node"
VIEWER_SCRIPT="${SCRIPT_DIR}/../../test/mujoco_viewer.py"

# ── 检查 ──────────────────────────────────────────────────────────────────────
check_file() {
  if [[ ! -f "$1" ]]; then
    echo "[run_vr_demo] ERROR: $1 not found."
    echo "  请先编译项目: cmake --build ${BUILD_DIR} -j4"
    exit 1
  fi
}
check_file "${UDP_DDS_NODE}"
check_file "${ABS_CTRL_NODE}"
check_file "${VIEWER_SCRIPT}"

if [[ ! -d "${ASSETS_DIR}" ]]; then
  echo "[run_vr_demo] ERROR: assets dir not found: ${ASSETS_DIR}"
  echo "  Set --assets-dir or LEJU_ASSETS_PATH"
  exit 1
fi

echo "============================================================"
echo " VR + MuJoCo 全链路演示"
echo "  构建目录  : ${BUILD_DIR}"
echo "  资源目录  : ${ASSETS_DIR}"
echo "  状态文件  : ${VR_STATE_FILE}"
echo "  MuJoCo   : ${ENABLE_MUJOCO}"
echo "  Display  : ${X_DISPLAY}"
echo "============================================================"

# ── 子进程管理 ────────────────────────────────────────────────────────────────
PIDS=()

cleanup() {
  echo ""
  echo "[run_vr_demo] 正在停止所有子进程..."
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait "${PIDS[@]}" 2>/dev/null || true
  echo "[run_vr_demo] 已退出。"
}
trap cleanup EXIT INT TERM

# ── 启动 quest_udp_to_dds_node ────────────────────────────────────────────────
echo "[run_vr_demo] 启动 quest_udp_to_dds_node ..."
LEJU_ASSETS_PATH="${ASSETS_DIR}" \
  "${UDP_DDS_NODE}" &
PIDS+=($!)
sleep 1

# ── 启动 quest_vr_abs_control_node ───────────────────────────────────────────
echo "[run_vr_demo] 启动 quest_vr_abs_control_node ..."
LEJU_ASSETS_PATH="${ASSETS_DIR}" \
ROBOT_VERSION="${ROBOT_VERSION}" \
VR_CONTROL_DT="${VR_CONTROL_DT}" \
VR_STATE_FILE="${VR_STATE_FILE}" \
  "${ABS_CTRL_NODE}" &
PIDS+=($!)
sleep 1

# ── 启动 MuJoCo 可视化 ────────────────────────────────────────────────────────
if [[ "${ENABLE_MUJOCO}" == true ]]; then
  echo "[run_vr_demo] 启动 MuJoCo 可视化 (DISPLAY=${X_DISPLAY}) ..."
  DISPLAY="${X_DISPLAY}" \
  LEJU_ASSETS_PATH="${ASSETS_DIR}" \
    python3 "${VIEWER_SCRIPT}" \
      --state-file "${VR_STATE_FILE}" \
      --hz "${MUJOCO_HZ}" \
      --base-z "${BASE_Z}" &
  PIDS+=($!)
else
  echo "[run_vr_demo] 已跳过 MuJoCo 可视化（--no-mujoco）"
fi

echo ""
echo "[run_vr_demo] 所有节点已启动。按 Ctrl-C 退出。"
echo "  Quest3 连接提示：大拇指+食指捏合 1~2 秒开始遥操作"
echo ""

# 等待任意子进程退出
wait -n "${PIDS[@]}" 2>/dev/null || true
