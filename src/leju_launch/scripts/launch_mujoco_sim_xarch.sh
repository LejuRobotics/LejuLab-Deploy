#!/usr/bin/env bash
# launch_mujoco_sim_xarch.sh — 跨架构联合仿真编排。
# 与 launch_mujoco_sim.sh 类似，但：
#   - 控制器用交叉编译的 *aarch64* run_rl_controller(QEMU 模拟，板子同款二进制)
#   - DDS 走禁多播 loopback(docker/cyclonedds_sim.xml)，不用 iceoryx SHM/RouDi
# 需在跨架构仿真容器 lejulab-sim-xarch 内运行(由 docker/run_sim_xarch.sh 进入)。
#
# 用法(容器内)： bash src/leju_launch/scripts/launch_mujoco_sim_xarch.sh [--robot-version=17]
#
# 依赖：build_sim/ 有 x86 leju-mujoco-sim；build_cross/ 有 aarch64 run_rl_controller。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${LEJULAB_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
cd "$PROJECT_DIR"

ROBOT_VERSION="${ROBOT_VERSION:-17}"
for a in "$@"; do
    case "$a" in
        --robot-version=*) ROBOT_VERSION="${a#*=}" ;;
        -h|--help) echo "Usage: $(basename "$0") [--robot-version=XX]"; exit 0 ;;
        *) echo "未知参数: $a" >&2; exit 1 ;;
    esac
done

SIM_BUILD="${SIM_BUILD:-build_sim}"
XC_BUILD="${XC_BUILD:-build_cross}"

# DDS：禁多播 loopback(QEMU 下 lo 多播不支持)。容器里 run_sim_xarch.sh 已注入此 URI，
# 未注入时退回 docker/cyclonedds_sim.xml。不调用 _launch_common 的 setup_env(它会强制 SHM)。
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${PROJECT_DIR}/docker/cyclonedds_sim.xml}"

SCENE="src/leju_assets/models/biped_s${ROBOT_VERSION}/xml/scene_rl.xml"
SIM_BIN="${SIM_BUILD}/src/leju-mujoco-sim/leju-mujoco-sim"
JOY_BIN="${SIM_BUILD}/src/leju-joystick/leju-joystick"
CTRL_BIN="${XC_BUILD}/src/leju-controllers/leju-rl-controller/run_rl_controller"
CTRL_CFG="src/leju-controllers/leju-rl-controller/config/${ROBOT_VERSION}/controller_manager.yaml"
# aarch64 控制器运行时库(QEMU 用 QEMU_LD_PREFIX 解析系统库，这里补仓库内的 onnx + lowlevel)
CTRL_LIBS="${CTRL_LIBS:-${PROJECT_DIR}/${XC_BUILD}/src/lejusdk/lejusdk-lowlevel:${PROJECT_DIR}/src/leju-controllers/leju-rl-controller/3rd_party/onnxruntime_toolkit/onnxruntime-linux-aarch64/lib}"

# 校验必需产物
for f in "$SIM_BIN" "$CTRL_BIN" "$SCENE" "$CTRL_CFG"; do
    [ -e "$f" ] || { echo "错误: 缺 $f" >&2; exit 1; }
done
file "$CTRL_BIN" | grep -q "ARM aarch64" || echo "警告: $CTRL_BIN 不是 aarch64，可能不是交叉编译产物"

PIDS=()
cleanup() {
    echo; echo "[xarch] 关闭所有节点..."
    for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
    sleep 0.5
    for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
    rm -f /tmp/iceoryx_rt_*.lock 2>/dev/null || true
    wait 2>/dev/null || true
    echo "[xarch] 已停止."
}
trap cleanup EXIT INT TERM

echo "=== 跨架构联合仿真  ROBOT_VERSION=${ROBOT_VERSION} ==="
echo "  DDS:        ${CYCLONEDDS_URI}"
echo "  sim(x86):   ${SIM_BIN}"
echo "  ctrl(arm64):${CTRL_BIN}"

echo "[xarch] 启动 x86 仿真 (GUI)"
"$SIM_BIN" "$SCENE" &
SIM_PID=$!; PIDS+=("$SIM_PID")
sleep 4

if [ -x "$JOY_BIN" ]; then
    echo "[xarch] 启动 x86 手柄 (可选)"
    "$JOY_BIN" &
    PIDS+=("$!")
fi

echo "[xarch] 启动 aarch64 控制器 (QEMU 模拟)"
LD_LIBRARY_PATH="$CTRL_LIBS" "$CTRL_BIN" "$CTRL_CFG" &
CTRL_PID=$!; PIDS+=("$CTRL_PID")

# 监控必需节点(仿真 + 控制器)：任一退出即收尾
while true; do
    kill -0 "$SIM_PID"  2>/dev/null || { echo "[xarch] 仿真已退出"; break; }
    kill -0 "$CTRL_PID" 2>/dev/null || { echo "[xarch] 控制器已退出"; break; }
    sleep 1
done
