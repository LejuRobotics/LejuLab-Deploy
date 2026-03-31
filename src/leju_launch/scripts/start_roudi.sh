#!/bin/bash
#
# 手动启动 RouDi (调试用)
# 生产环境请使用: ./setup_cyclonedds_config.sh
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "${SCRIPT_DIR}")"
PROJECT_DIR="$(dirname "$(dirname "${PKG_DIR}")")"

# 自动检测 iceoryx 路径 (兼容 lejulab-platform 和 lejulab)
ICEORYX_PATHS=(
    "${PROJECT_DIR}/src/lejusdk/3rd_party/iceoryx"
    "${PROJECT_DIR}/installed/iceoryx"
)
ICEORYX_DIR=""
for path in "${ICEORYX_PATHS[@]}"; do
    [ -x "$path/bin/iox-roudi" ] && ICEORYX_DIR="$path" && break
done

CONFIG_FILE="${PKG_DIR}/config/roudi_config.toml"

if [ -z "${ICEORYX_DIR}" ]; then
    echo "错误: 未找到 iox-roudi，搜索路径:"
    printf "  %s\n" "${ICEORYX_PATHS[@]}"
    exit 1
fi

# 检查 RouDi 是否已在运行
if [ -S /tmp/roudi ]; then
    echo "RouDi 已在运行 (可能由 systemd 管理)，无需重复启动"
    exit 0
fi

# 清理残留的 socket/lock 文件
rm -f /tmp/roudi /tmp/roudi.lock

echo "=== RouDi 调试模式 (Ctrl+C 停止) ==="
echo "Config: ${CONFIG_FILE}"
exec "${ICEORYX_DIR}/bin/iox-roudi" -c "${CONFIG_FILE}"
