#!/usr/bin/env bash
# 构建 RK3588 交叉编译镜像，并可选导出自包含 tar。
#
# 用法：
#   ./build_image.sh             # sysroot 不存在则先提取，再 build，再 save tar
#   ./build_image.sh --no-save   # 只 build 不导出 tar
#   ./build_image.sh --no-extract# sysroot 缺失也不自动提取（直接报错）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="lejulab-cross-aarch64:latest"
TAR_FILE="$SCRIPT_DIR/lejulab-cross-aarch64.tar.gz"
SYSROOT="$SCRIPT_DIR/sysroot"

DO_SAVE=1
DO_EXTRACT=1
for arg in "$@"; do
    case "$arg" in
        --no-save) DO_SAVE=0 ;;
        --no-extract) DO_EXTRACT=0 ;;
        *) echo "未知参数: $arg" >&2; exit 1 ;;
    esac
done

if [ ! -d "$SYSROOT" ] || [ -z "$(ls -A "$SYSROOT" 2>/dev/null)" ]; then
    if [ "$DO_EXTRACT" -eq 1 ]; then
        echo "=== sysroot 不存在，先提取 ==="
        bash "$SCRIPT_DIR/extract_sysroot.sh"
    else
        echo "错误：$SYSROOT 不存在，请先运行 extract_sysroot.sh" >&2
        exit 1
    fi
fi

echo "=== 构建镜像 $IMAGE_NAME ==="
docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile.cross-aarch64" "$SCRIPT_DIR"

if [ "$DO_SAVE" -eq 1 ]; then
    echo "=== 导出自包含镜像 -> $TAR_FILE ==="
    docker save "$IMAGE_NAME" | gzip > "$TAR_FILE"
    ls -lh "$TAR_FILE"
fi

echo "=== 完成 ==="
echo "在别的机器加载：  docker load -i $(basename "$TAR_FILE")"
echo "编译：           ./run_build.sh"
