#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="lejulab-aarch64:latest"

echo "=== Registering QEMU binfmt for arm64 ==="
docker run --rm --privileged tonistiigi/binfmt --install arm64

echo "=== Building aarch64 Docker image ==="
docker buildx build \
    --platform linux/arm64 \
    --load \
    -t "$IMAGE_NAME" \
    -f "$SCRIPT_DIR/Dockerfile.aarch64" \
    "$SCRIPT_DIR"

echo "=== Done: $IMAGE_NAME ==="
