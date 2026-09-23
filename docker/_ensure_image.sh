#!/usr/bin/env bash
# _ensure_image.sh — 被各 run_*.sh source。确保镜像存在：本地无 → 云端拉取 → 回退本地构建。
#
# 云端备份目录(可用环境变量覆盖)：
IMAGE_BASE_URL="${IMAGE_BASE_URL:-https://kuavo.lejurobot.com/r3588_docker_image_backup}"

# 云端有备份的镜像名单(空格分隔)。只有这些才会去云端拉；其余直接本地构建。
# 目前云端只有 lejulab-cross-aarch64(它需板子 sysroot，无板子无法本地重建)；
# sim 镜像可由 Dockerfile 本地构建，未上传，故不在此列。新上传的镜像加进来即可，
# 或用环境变量 CLOUD_IMAGES 覆盖。
CLOUD_IMAGES="${CLOUD_IMAGES:-lejulab-cross-aarch64}"

# ensure_image <image[:tag]> [本地构建命令...]
#   1) docker 已有该镜像 → 直接返回
#   2) 镜像在 CLOUD_IMAGES 名单 → 从 ${IMAGE_BASE_URL}/<image名>.tar.gz 下载并 docker load
#   3) 仍没有且给了构建命令 → 执行本地构建
ensure_image() {
    local image="$1"; shift || true

    if docker image inspect "$image" >/dev/null 2>&1; then
        return 0
    fi

    local name="${image%%:*}"                      # 去掉 :tag

    # 不在云端名单 → 跳过下载，直接本地构建
    if ! printf '%s\n' $CLOUD_IMAGES | grep -qx "$name"; then
        if [ "$#" -gt 0 ]; then
            echo "本地无镜像 '${image}'，本地构建..." >&2
            "$@"; return $?
        fi
        echo "错误：无镜像 '${image}'(不在云端名单且未提供构建方式)。" >&2
        return 1
    fi

    local url="${IMAGE_BASE_URL}/${name}.tar.gz"
    local tmp="/tmp/${name}.tar.gz"

    echo "本地无镜像 '${image}'，尝试从云端拉取：" >&2
    echo "  ${url}" >&2
    local ok=0
    if command -v curl >/dev/null 2>&1; then
        curl -fSL --retry 3 -o "$tmp" "$url" && ok=1
    elif command -v wget >/dev/null 2>&1; then
        wget -O "$tmp" "$url" && ok=1
    else
        echo "  无 curl/wget，无法下载。" >&2
    fi

    if [ "$ok" -eq 1 ]; then
        echo "下载完成，docker load..." >&2
        if docker load -i "$tmp"; then
            rm -f "$tmp"
            docker image inspect "$image" >/dev/null 2>&1 && return 0
            echo "  注意：load 的镜像名/标签与 '${image}' 不一致。" >&2
        fi
    fi
    rm -f "$tmp" 2>/dev/null || true
    echo "云端拉取失败。" >&2

    if [ "$#" -gt 0 ]; then
        echo "回退到本地构建：$*" >&2
        "$@"
        return $?
    fi
    echo "错误：无法获得镜像 '${image}'（云端拉取失败且未提供本地构建方式）。" >&2
    return 1
}
