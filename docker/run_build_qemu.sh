#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="lejulab-aarch64:latest"
BUILD_DIR="build_docker"

echo "=== Running aarch64 build in Docker ==="
echo "Project: $PROJECT_DIR"
echo "Build dir: $BUILD_DIR"

docker run --rm --platform linux/arm64 \
    -v "$PROJECT_DIR":/workspace \
    "$IMAGE_NAME" \
    bash -c "cmake -B $BUILD_DIR -DBUILD_RL_CONTROLLER=ON -DBUILD_JOYSTICK=OFF && cmake --build $BUILD_DIR -j\$(nproc)"

echo "=== Build complete ==="
