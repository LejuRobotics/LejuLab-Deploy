#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_FILE="$SCRIPT_DIR/lejulab-aarch64.tar.gz"

if [ ! -f "$IMAGE_FILE" ]; then
    echo "Error: $IMAGE_FILE not found"
    exit 1
fi

echo "=== Loading lejulab-aarch64 image from $IMAGE_FILE ==="
docker load -i "$IMAGE_FILE"
echo "=== Done ==="
docker images lejulab-aarch64
