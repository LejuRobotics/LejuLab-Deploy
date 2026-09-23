#!/usr/bin/env bash
# 构建 x86 仿真镜像（GUI/NVIDIA）。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="lejulab-sim-x86:latest"

echo "=== 构建仿真镜像 $IMAGE_NAME ==="
docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile.sim" "$SCRIPT_DIR"
echo "=== 完成。用 ./run_sim.sh 进入容器 ==="
