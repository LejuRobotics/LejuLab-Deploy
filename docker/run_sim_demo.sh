#!/usr/bin/env bash
# run_sim_demo.sh — 一键：在仿真容器里编译并起 MuJoCo 仿真 + RL 控制器闭环。
# 等价于手动 run_sim.sh 进容器后 build + launch_mujoco_sim.sh 的全过程。
#
# 用法：
#   ./run_sim_demo.sh                 # 默认 ROBOT_VERSION=17
#   ./run_sim_demo.sh 46              # 指定机器人版本(需有 config/<v> 与 scene_rl.xml)
#   FORCE_BUILD=1 ./run_sim_demo.sh   # 强制重新 configure
#
# 容器内走项目自带 launch_mujoco_sim.sh：sim + joystick + run_rl_controller，
# DDS 用 --no-shm(普通 CycloneDDS loopback，免 iceoryx RouDi)。Ctrl-C 退出。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/_ensure_image.sh"
IMAGE_NAME="lejulab-sim-x86:latest"
ROBOT_VERSION="${1:-17}"
BUILD_DIR_NAME="build_sim"
FORCE_BUILD="${FORCE_BUILD:-0}"

# 本地无 → 云端拉取 → 回退本地构建
ensure_image "$IMAGE_NAME" bash "$SCRIPT_DIR/build_sim_image.sh" || exit 1

# 校验模型存在
SCENE="src/leju_assets/models/biped_s${ROBOT_VERSION}/xml/scene_rl.xml"
if [ ! -f "$PROJECT_DIR/$SCENE" ]; then
    echo "错误：找不到 $SCENE" >&2
    echo "  该机器人版本可能没有 scene_rl.xml；可用版本：" >&2
    ls "$PROJECT_DIR"/src/leju_assets/models/*/xml/scene_rl.xml 2>/dev/null \
        | sed -E 's#.*/biped_s([0-9]+)/.*#    \1#' >&2 || true
    exit 1
fi

# GPU 透传参数
GPU_ARGS=()
if command -v nvidia-container-runtime >/dev/null 2>&1; then
    GPU_ARGS=(--gpus all --runtime nvidia
        -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all,display)
else
    echo "警告：未检出 nvidia-container 运行时，GUI 将用软件渲染(慢)。"
    GPU_ARGS=(-e LIBGL_ALWAYS_SOFTWARE=1)
fi

xhost +local:root >/dev/null 2>&1 || echo "警告：xhost 不可用，GUI 窗口可能起不来。"

echo "=== 仿真闭环 demo: ROBOT_VERSION=${ROBOT_VERSION} (Ctrl-C 退出) ==="

# shellcheck disable=SC2016
INNER='
set -e
cd /workspace
CFG_STAMP=build_sim/.demo_configured
if [ ! -f "$CFG_STAMP" ] || [ "'"$FORCE_BUILD"'" = "1" ]; then
    echo "=== configure ==="
    cmake -B build_sim -G Ninja \
        -DBUILD_MUJOCO_SIM=ON -DBUILD_RL_CONTROLLER=ON -DBUILD_JOYSTICK=ON \
        -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
    touch "$CFG_STAMP"
fi
echo "=== build (sim + controller + joystick) ==="
cmake --build build_sim --target leju-mujoco-sim run_rl_controller leju-joystick -j$(nproc)
echo "=== launch 闭环 ==="
BUILD_DIR=/workspace/build_sim ROBOT_VERSION='"$ROBOT_VERSION"' \
    bash src/leju_launch/scripts/launch_mujoco_sim.sh --no-shm --robot-version='"$ROBOT_VERSION"'
'

exec docker run --rm -it --net host "${GPU_ARGS[@]}" \
    -e DISPLAY="${DISPLAY:-:1}" \
    -e ROBOT_VERSION="$ROBOT_VERSION" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$PROJECT_DIR":/workspace \
    -w /workspace \
    "$IMAGE_NAME" bash -lc "$INNER"
