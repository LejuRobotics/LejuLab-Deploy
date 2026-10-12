#!/usr/bin/env bash
# 在交叉编译镜像内编译本仓库到 aarch64，输出 build_cross/。
#
# 用法：
#   ./run_build.sh                       # 默认 BUILD_RL_CONTROLLER=ON
#   ./run_build.sh --target test_robot_version
#   CMAKE_ARGS="-DBUILD_JOYSTICK=ON" ./run_build.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="lejulab-cross-aarch64:latest"
BUILD_DIR="${BUILD_DIR:-build_cross}"
TARGET=""

while [ $# -gt 0 ]; do
    case "$1" in
        --target) TARGET="$2"; shift 2 ;;
        *) echo "未知参数: $1" >&2; exit 1 ;;
    esac
done

if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "错误：镜像 $IMAGE_NAME 不存在。先运行 ./build_image.sh 或 docker load -i lejulab-cross-aarch64.tar.gz" >&2
    exit 1
fi

CMAKE_ARGS="${CMAKE_ARGS:--DBUILD_RL_CONTROLLER=ON}"
BUILD_CMD="cmake --build $BUILD_DIR -j\$(nproc)"
[ -n "$TARGET" ] && BUILD_CMD="cmake --build $BUILD_DIR --target $TARGET -j\$(nproc)"

echo "=== 交叉编译 ($IMAGE_NAME) ==="
echo "Project:   $PROJECT_DIR"
echo "Build dir: $BUILD_DIR"
echo "CMake:     $CMAKE_ARGS"

docker run --rm \
    -v "$PROJECT_DIR":/workspace \
    -w /workspace \
    "$IMAGE_NAME" \
    bash -c 'source scripts/build_env.sh && cmake -B '"$BUILD_DIR"' -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE=/opt/toolchain-aarch64.cmake \
        -DCMAKE_BUILD_TYPE=Release $CMAKE_ARGS && $BUILD_CMD'

echo "=== 完成，产物在 $PROJECT_DIR/$BUILD_DIR ==="
