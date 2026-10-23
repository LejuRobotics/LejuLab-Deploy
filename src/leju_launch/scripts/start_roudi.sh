#!/bin/bash
#
# 手动启动 RouDi (调试用)
# 生产环境请使用: ./setup_cyclonedds_config.sh
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "${SCRIPT_DIR}")"
PROJECT_DIR="$(dirname "$(dirname "${PKG_DIR}")")"

# 自动检测 iox-roudi (兼容按架构打包、旧目录和本地安装布局)
ARCH="$(uname -m)"
ROUDI_SEARCH_PATHS=(
    "${PROJECT_DIR}/src/lejusdk/3rd_party/${ARCH}/iceoryx/bin/iox-roudi"
    "${PROJECT_DIR}/src/lejusdk/3rd_party/iceoryx/bin/iox-roudi"
    "${PROJECT_DIR}/installed/iceoryx/bin/iox-roudi"
    "${PROJECT_DIR}/installed/bin/iox-roudi"
    "/opt/iceoryx/bin/iox-roudi"
)
ROUDI_BIN=""
for path in "${ROUDI_SEARCH_PATHS[@]}"; do
    if [ -x "$path" ]; then
        ROUDI_BIN="$path"
        break
    fi
done

CONFIG_FILE="${PKG_DIR}/config/roudi_config.toml"

if [ -z "${ROUDI_BIN}" ]; then
    echo "错误: 未找到 iox-roudi，搜索路径:"
    printf "  %s\n" "${ROUDI_SEARCH_PATHS[@]}"
    exit 1
fi

if [ ! -f "${CONFIG_FILE}" ]; then
    echo "错误: 未找到 RouDi 配置文件: ${CONFIG_FILE}"
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
echo "Binary: ${ROUDI_BIN}"
echo "Config: ${CONFIG_FILE}"
exec "${ROUDI_BIN}" -c "${CONFIG_FILE}"
