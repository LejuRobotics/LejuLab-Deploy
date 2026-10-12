#!/usr/bin/env bash
# run_sim_xarch.sh — 进入单镜像跨架构仿真容器(交互 shell)。
# 一个容器里可跑：x86 原生 MuJoCo 仿真(GPU GUI) + aarch64 控制器(QEMU 模拟)。
# 默认进 shell 自己操作；加 --demo 则自动起 sim+控制器闭环。
#
# 用法：
#   ./run_sim_xarch.sh            # 进容器 shell (默认 ROBOT_VERSION=17)
#   ./run_sim_xarch.sh 46         # 指定版本
#   ./run_sim_xarch.sh --demo     # 一键自动起闭环
#
# 前置：① 宿主已注册 binfmt: docker run --privileged tonistiigi/binfmt --install arm64
#       ② aarch64 控制器已交叉编译(build_cross)；缺则脚本用交叉编译镜像自动补编。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
source "${SCRIPT_DIR}/_ensure_image.sh"
IMAGE="lejulab-sim-xarch:latest"
XC_IMAGE="lejulab-cross-aarch64:latest"

DEMO=0; RV=17
for a in "$@"; do
    case "$a" in
        --demo) DEMO=1 ;;
        [0-9]*) RV="$a" ;;
        *) echo "未知参数: $a" >&2; exit 1 ;;
    esac
done

CTRL=build_cross/src/leju-controllers/leju-rl-controller/run_rl_controller
CTRL_LIBS="/workspace/build_cross/src/lejusdk/lejusdk-lowlevel:/workspace/src/leju-controllers/leju-rl-controller/3rd_party/onnxruntime_toolkit/onnxruntime-linux-aarch64/lib"
cd "$PROJECT_DIR"

# 镜像：本地无 → 云端拉取 → 回退本地构建
ensure_image "$IMAGE" docker build -t "$IMAGE" -f "$SCRIPT_DIR/Dockerfile.sim-xarch" "$SCRIPT_DIR"

# aarch64 控制器(交叉编译产物)；缺则拉/用交叉编译镜像补编
if [ ! -x "$CTRL" ]; then
    if ensure_image "$XC_IMAGE"; then
        echo "=== 缺 aarch64 控制器，用交叉编译镜像补编 ==="
        docker run --rm -v "$PROJECT_DIR":/workspace "$XC_IMAGE" bash -lc '
            cmake -B build_cross -G Ninja -DCMAKE_TOOLCHAIN_FILE=/opt/toolchain-aarch64.cmake \
                -DCMAKE_BUILD_TYPE=Release -DBUILD_MUJOCO_SIM=OFF -DBUILD_JOYSTICK=OFF \
                -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF >/dev/null &&
            cmake --build build_cross --target run_rl_controller -j$(nproc)'
    else
        echo "提示：尚无 aarch64 控制器($CTRL)，也无法获得交叉编译镜像 $XC_IMAGE。" >&2
        echo "      进容器后只能跑 x86 仿真；要跑 aarch64 控制器请先交叉编译(见 howto-cross-compile-docker)。" >&2
    fi
fi

# GPU 透传
GPU_ARGS=()
if command -v nvidia-container-runtime >/dev/null 2>&1; then
    GPU_ARGS=(--gpus all --runtime nvidia
        -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all,display)
else
    echo "警告：无 nvidia-container 运行时，GUI 走软件渲染(慢)。"
    GPU_ARGS=(-e LIBGL_ALWAYS_SOFTWARE=1)
fi
xhost +local:root >/dev/null 2>&1 || echo "警告：xhost 不可用，GUI 可能起不来。"

COMMON=(--rm "${GPU_ARGS[@]}"
    -e DISPLAY="${DISPLAY:-:1}" -e ROBOT_VERSION="$RV"
    -e CYCLONEDDS_URI="file:///workspace/docker/cyclonedds_sim.xml"
    -e CTRL_LIBS="$CTRL_LIBS"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$PROJECT_DIR":/workspace -w /workspace
    "$IMAGE")

if [ "$DEMO" -eq 1 ]; then
    # 一键闭环
    exec docker run -it "${COMMON[@]}" bash -c '
        set -e
        cmake -B build_sim -G Ninja -DBUILD_MUJOCO_SIM=ON -DBUILD_RL_CONTROLLER=OFF \
            -DBUILD_JOYSTICK=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF >/dev/null
        cmake --build build_sim --target leju-mujoco-sim -j$(nproc)
        build_sim/src/leju-mujoco-sim/leju-mujoco-sim src/leju_assets/models/biped_s'"$RV"'/xml/scene_rl.xml &
        SIM_PID=$!; sleep 4
        trap "kill $SIM_PID 2>/dev/null||true" EXIT INT TERM
        LD_LIBRARY_PATH="$CTRL_LIBS" build_cross/src/leju-controllers/leju-rl-controller/run_rl_controller \
            src/leju-controllers/leju-rl-controller/config/'"$RV"'/controller_manager.yaml'
fi

# 默认：进交互 shell，打印速查
cat <<EOF
=== 进入跨架构仿真容器 (ROBOT_VERSION=$RV) ===
容器内速查（复制即用）：

  # ★ 一条命令起跨架构闭环(sim x86 + 控制器 aarch64/QEMU)：
  bash src/leju_launch/scripts/launch_mujoco_sim_xarch.sh --robot-version=$RV

  # ── 或手动分步 ──
  # 1) 编 x86 仿真 (gcc-9)
  cmake -B build_sim -G Ninja -DBUILD_MUJOCO_SIM=ON -DBUILD_RL_CONTROLLER=OFF \\
        -DBUILD_JOYSTICK=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
  cmake --build build_sim --target leju-mujoco-sim -j\$(nproc)

  # 2) 起 x86 仿真 (GUI，后台)
  build_sim/src/leju-mujoco-sim/leju-mujoco-sim src/leju_assets/models/biped_s$RV/xml/scene_rl.xml &

  # 3) 起 aarch64 控制器 (QEMU 模拟)  —— \$CTRL_LIBS 已预置库路径
  LD_LIBRARY_PATH=\$CTRL_LIBS \\
    build_cross/src/leju-controllers/leju-rl-controller/run_rl_controller \\
    src/leju-controllers/leju-rl-controller/config/$RV/controller_manager.yaml

EOF

exec docker run -it "${COMMON[@]}" bash
