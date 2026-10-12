#!/usr/bin/env bash
# 启动仿真容器（GUI + NVIDIA 硬件渲染 + X11 透传 + host 网络给 DDS）。
# 参考 kuavo-ros-control/docker/run_with_gpu.sh 的 GPU/X11 透传模式精简而来。
#
# 用法：
#   ./run_sim.sh            # 进入容器(已挂载仓库到 /workspace，GPU/显示就绪)
#   ./run_sim.sh <cmd...>   # 在容器内直接执行命令后退出
#
# 进容器后编译 + 跑仿真：
#   cmake -B build_sim -G Ninja -DBUILD_MUJOCO_SIM=ON \
#         -DBUILD_RL_CONTROLLER=OFF -DBUILD_JOYSTICK=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
#   cmake --build build_sim --target leju-mujoco-sim -j$(nproc)
#   export ROBOT_VERSION=17   # 必须与模型匹配(17/45/46/...)，否则启动即抛异常
#   SIM=build_sim/src/leju-mujoco-sim/leju-mujoco-sim
#   $SIM src/leju_assets/models/biped_s17/xml/scene.xml             # GUI(需显卡/显示)
#   $SIM --headless src/leju_assets/models/biped_s17/xml/scene.xml  # 无界面(物理+DDS)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/_ensure_image.sh"
IMAGE_NAME="lejulab-sim-x86:latest"
CONTAINER_NAME="lejulab_sim_$(echo "$PROJECT_DIR" | md5sum | cut -c1-8)"

# 本地无 → 云端拉取 → 回退本地构建
ensure_image "$IMAGE_NAME" bash "$SCRIPT_DIR/build_sim_image.sh" || exit 1

# 允许容器访问宿主 X 服务器（GUI 必需）
xhost +local:root >/dev/null 2>&1 || echo "警告：xhost 不可用，GUI 窗口可能起不来（headless 不受影响）"

# GPU 参数：有 nvidia 运行时才加，否则退回软件渲染
GPU_ARGS=()
if docker info 2>/dev/null | grep -qi nvidia || command -v nvidia-container-runtime >/dev/null 2>&1; then
    GPU_ARGS=(--gpus all --runtime nvidia
        -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all,display)
else
    echo "警告：未检出 nvidia-container 运行时，将用软件渲染(慢)。装 nvidia-container-toolkit 可硬件加速。"
    GPU_ARGS=(-e LIBGL_ALWAYS_SOFTWARE=1)
fi

# 复用已有容器
if docker ps -aq -f name="^${CONTAINER_NAME}$" | grep -q .; then
    docker start "$CONTAINER_NAME" >/dev/null 2>&1 || true
    if [ "$#" -gt 0 ]; then
        exec docker exec "$CONTAINER_NAME" bash -lc "$*"
    fi
    exec docker exec -it "$CONTAINER_NAME" bash
fi

RUN_ARGS=(docker run -it --name "$CONTAINER_NAME"
    --net host
    "${GPU_ARGS[@]}"
    -e DISPLAY="${DISPLAY:-:0}"
    -e ROBOT_VERSION="${ROBOT_VERSION:-17}"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    -v "$PROJECT_DIR":/workspace
    -w /workspace
    "$IMAGE_NAME")

if [ "$#" -gt 0 ]; then
    exec "${RUN_ARGS[@]}" bash -lc "$*"
fi
exec "${RUN_ARGS[@]}" bash
