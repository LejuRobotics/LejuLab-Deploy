#!/usr/bin/env bash

set -euo pipefail

show_usage() {
    cat <<'EOF'
Usage:
  ./docker/build.sh [--base-image image] [--tag tag]

Examples:
  ./docker/build.sh
  ./docker/build.sh --tag release
  ./docker/build.sh --base-image kuavo_opensource_mpc_wbc_img:latest --tag dev
EOF
}

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
BASE_IMAGE=""
IMAGE_TAG="dev"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --base-image)
            BASE_IMAGE="${2:-}"
            shift 2
            ;;
        --tag)
            IMAGE_TAG="${2:-}"
            shift 2
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            echo "错误: 未知参数 $1"
            show_usage
            exit 1
            ;;
    esac
done

if [ -z "${BASE_IMAGE}" ]; then
    BASE_IMAGE="$(docker images kuavo_opensource_mpc_wbc_img --format "{{.Repository}}:{{.Tag}}" | sort -V | tail -n1)"
fi

if [ -z "${BASE_IMAGE}" ]; then
    echo "错误: 未找到 kuavo 基底镜像，请先执行 ./docker/run.sh 下载导入，或手动 docker load。"
    exit 1
fi

cd "$SCRIPT_DIR"
docker build \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    -t "lejulab_platform:${IMAGE_TAG}" \
    .
