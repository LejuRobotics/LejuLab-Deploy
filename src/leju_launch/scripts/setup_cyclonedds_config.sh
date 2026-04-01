#!/bin/bash
#
# CycloneDDS + iceoryx 共享内存部署
#
# 功能:
#   1. 安装 RouDi 为系统服务 (开机自启)
#   2. 部署 CycloneDDS 配置文件
#   3. 设置 CYCLONEDDS_URI 环境变量
#
# 部署后 roslaunch 和独立程序均可直接使用共享内存
#
# 用法:
#   ./setup_cyclonedds_config.sh          # 部署
#   ./setup_cyclonedds_config.sh --remove # 卸载
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "${SCRIPT_DIR}")"
PROJECT_DIR="$(dirname "$(dirname "${PKG_DIR}")")"

ICEORYX_INSTALL="/opt/iceoryx"
ROUDI_CONFIG="${PKG_DIR}/config/roudi_config.toml"
NORMAL_XML="${PKG_DIR}/config/cyclonedds.xml"
SHM_XML="${PKG_DIR}/config/cyclonedds_shm.xml"

# 自动检测 iox-roudi 路径 (兼容 lejulab-platform 和 lejulab)
ROUDI_SEARCH_PATHS=(
    "${PROJECT_DIR}/src/lejusdk/3rd_party/iceoryx/bin/iox-roudi"
    "${PROJECT_DIR}/installed/bin/iox-roudi"
)

install_dependencies() {
    local REQUIRED_PKGS=(libacl1-dev)
    local MISSING_PKGS=()
    for pkg in "${REQUIRED_PKGS[@]}"; do
        if ! dpkg -s "$pkg" &>/dev/null; then
            MISSING_PKGS+=("$pkg")
        fi
    done
    if [ ${#MISSING_PKGS[@]} -gt 0 ]; then
        echo "安装缺失依赖: ${MISSING_PKGS[*]}"
        sudo apt-get update && sudo apt-get install -y "${MISSING_PKGS[@]}"
    else
        echo "依赖已安装: ${REQUIRED_PKGS[*]}"
    fi
}

find_roudi() {
    for path in "${ROUDI_SEARCH_PATHS[@]}"; do
        [ -x "$path" ] && echo "$path" && return
    done
    echo "错误: 未找到 iox-roudi，搜索路径:" >&2
    printf "  %s\n" "${ROUDI_SEARCH_PATHS[@]}" >&2
    exit 1
}

update_cyclonedds_env() {
    # 系统级: /etc/profile.d/ 对所有用户生效 (包括 root)
    sudo tee /etc/profile.d/cyclonedds.sh > /dev/null << EOF
# CycloneDDS
export CYCLONEDDS_URI="$1"
EOF
    sudo chmod 644 /etc/profile.d/cyclonedds.sh

    # 兼容: 更新当前用户和 root 的 rc 文件 (非登录 shell 也能生效)
    local uri="$1"
    local rc_files=("$HOME/.bashrc" /root/.bashrc)
    [ -f "$HOME/.zshrc" ] && rc_files+=("$HOME/.zshrc")
    [ -f /root/.zshrc ] && rc_files+=("/root/.zshrc")

    for rc in "${rc_files[@]}"; do
        sudo sed -i '/# CycloneDDS/d' "$rc"
        sudo sed -i '/CYCLONEDDS_URI/d' "$rc"
        echo -e "\n# CycloneDDS\nexport CYCLONEDDS_URI=\"${uri}\"" | sudo tee -a "$rc" > /dev/null
    done
}

deploy() {
    echo "=== CycloneDDS 共享内存部署 ==="

    install_dependencies

    local ROUDI_BIN
    ROUDI_BIN=$(find_roudi)
    if [ ! -f "${ROUDI_CONFIG}" ]; then
        echo "错误: 未找到 ${ROUDI_CONFIG}"
        exit 1
    fi
    echo "使用 iox-roudi: ${ROUDI_BIN}"
    echo "使用配置: ${ROUDI_CONFIG}"

    # 停掉旧服务并清理
    sudo systemctl stop leju-roudi.service 2>/dev/null || true
    sudo rm -f /tmp/roudi /tmp/roudi.lock
    sudo rm -f /dev/shm/iceoryx_mgmt /dev/shm/iceoryx

    # 创建 iceoryx 用户组 (多用户共享访问)
    getent group iceoryx > /dev/null || sudo groupadd iceoryx
    sudo usermod -aG iceoryx "$USER"

    # 安装 iox-roudi 到系统目录
    sudo mkdir -p "${ICEORYX_INSTALL}/bin"
    sudo cp "${ROUDI_BIN}" "${ICEORYX_INSTALL}/bin/iox-roudi"

    # 安装配置文件
    sudo mkdir -p /etc/iceoryx /etc/cyclonedds
    sudo cp "${ROUDI_CONFIG}" /etc/iceoryx/roudi_config.toml
    sudo cp "${NORMAL_XML}" /etc/cyclonedds/cyclonedds.xml
    sudo cp "${SHM_XML}" /etc/cyclonedds/cyclonedds_shm.xml

    # systemd 服务
    # UMask=0000: 确保 RouDi 创建的 socket/shm 不受 umask 限制
    # ExecStartPost: 等待 RouDi 就绪后显式放开权限，不依赖用户组
    sudo tee /etc/systemd/system/leju-roudi.service > /dev/null << EOF
[Unit]
Description=iceoryx RouDi daemon
After=network.target

[Service]
Type=simple
User=root
Group=iceoryx
UMask=0000
Environment="CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds_shm.xml"
ExecStartPre=-/bin/rm -f /tmp/roudi /tmp/roudi.lock
ExecStart=${ICEORYX_INSTALL}/bin/iox-roudi -c /etc/iceoryx/roudi_config.toml
ExecStartPost=/bin/bash -c 'for i in \$(seq 1 50); do [ -S /tmp/roudi ] && break; sleep 0.1; done; chmod 0666 /tmp/roudi 2>/dev/null; chmod 0666 /dev/shm/iceoryx_mgmt 2>/dev/null; chmod 0666 /dev/shm/iceoryx 2>/dev/null; true'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl daemon-reload
    sudo systemctl enable leju-roudi.service
    sudo systemctl start leju-roudi.service

    # 设置环境变量
    update_cyclonedds_env "file:///etc/cyclonedds/cyclonedds_shm.xml"

    # 检查服务状态
    echo ""
    systemctl status leju-roudi.service --no-pager

    echo ""
    echo "=== 部署完成 ==="
    echo ""
    echo -e "\033[1;33m【重要】当前用户已加入 iceoryx 组，需要注销当前用户重新登录或重启系统才能生效\033[0m"
}

remove() {
    echo "=== CycloneDDS 共享内存卸载 ==="

    echo "停止 RouDi 服务..."
    sudo systemctl stop leju-roudi.service 2>/dev/null || true
    sudo systemctl disable leju-roudi.service 2>/dev/null || true
    sudo pkill -f iox-roudi 2>/dev/null || true

    echo "删除 systemd 服务文件..."
    sudo rm -f /etc/systemd/system/leju-roudi.service
    sudo systemctl daemon-reload

    echo "删除 iceoryx 安装目录与配置文件..."
    sudo rm -rf /opt/iceoryx /etc/iceoryx
    sudo rm -f /etc/cyclonedds/cyclonedds_shm.xml

    echo "清理共享内存与临时文件..."
    sudo rm -f /tmp/roudi /tmp/roudi.lock
    sudo rm -f /dev/shm/iceoryx_mgmt /dev/shm/iceoryx

    echo "还原 CYCLONEDDS_URI 为普通配置..."
    update_cyclonedds_env "file:///etc/cyclonedds/cyclonedds.xml"

    echo ""
    echo "=== 卸载完成 ==="
}

case "$1" in
    --remove) remove ;;
    "") deploy ;;
    *) echo "用法: $0 [--remove]"; exit 1 ;;
esac
