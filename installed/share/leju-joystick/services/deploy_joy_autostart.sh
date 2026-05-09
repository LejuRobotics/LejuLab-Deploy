#!/bin/bash

set -euo pipefail

if systemctl is-active --quiet roban_joy_monitor.service; then
    echo "服务 roban_joy_monitor.service 已开启，正在停止..."
    sudo systemctl stop roban_joy_monitor.service
    sudo systemctl disable roban_joy_monitor.service
    echo "服务 roban_joy_monitor.service 已停止。"
else
    echo "服务 roban_joy_monitor.service 未开启。"
fi

if systemctl is-active --quiet ocs2_h12pro_monitor.service; then
    echo "服务 ocs2_h12pro_monitor.service 已开启，正在停止..."
    sudo systemctl stop ocs2_h12pro_monitor.service
    sudo systemctl disable ocs2_h12pro_monitor.service
    echo "服务 ocs2_h12pro_monitor.service 已停止。"
else
    echo "服务 ocs2_h12pro_monitor.service 未开启。"
fi

if systemctl is-active --quiet lejulab_joy_monitor.service; then
    echo "服务 lejulab_joy_monitor.service 已开启，正在停止..."
    sudo systemctl stop lejulab_joy_monitor.service
    sudo systemctl disable lejulab_joy_monitor.service
    echo "服务 lejulab_joy_monitor.service 已停止。"
else
    echo "服务 lejulab_joy_monitor.service 未开启。"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 该脚本在两种仓库布局下都会被调用:
#   闭源 lejulab_platform: <ws>/src/leju-joystick/services/
#   开源 lejulab:          <ws>/installed/share/leju-joystick/services/
case "${SCRIPT_DIR}" in
    */installed/share/leju-joystick/services)
        WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
        ;;
    */src/leju-joystick/services)
        WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
        ;;
    *)
        echo "错误: 无法从 ${SCRIPT_DIR} 推导工作区根目录" >&2
        exit 1
        ;;
esac

SERVICE_NAME="lejulab_joy_monitor.service"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"
TEMPLATE_FILE="${SCRIPT_DIR}/lejulab_joy_monitor.service.template"
MONITOR_SCRIPT="${SCRIPT_DIR}/monitor_lejulab_joy.sh"
SET_ACTIVE_PROFILE_SCRIPT="${SCRIPT_DIR}/set_active_profile.sh"
CYCLONEDDS_SETUP_SCRIPT="${WS_ROOT}/src/leju_launch/scripts/setup_cyclonedds_config.sh"
AUTOSTART_ROOT="/root/.config/lejulab/auto_start_config"
PROFILES_DIR="${AUTOSTART_ROOT}/profiles"
DEPLOYED_SET_ACTIVE_PROFILE_SCRIPT="${AUTOSTART_ROOT}/set_active_profile.sh"
ROOT_BASHRC="/root/.bashrc"

if [ "${EUID}" -ne 0 ]; then
    echo "错误: 请以 root 身份执行该脚本" >&2
    exit 1
fi

if [ -d "${WS_ROOT}/installed" ]; then
    BUILD_MODE="open"
else
    BUILD_MODE="closed"
fi

escape_sed_replacement() {
    printf '%s' "$1" | sed -e 's/[\/&]/\\&/g'
}

resolve_robot_version() {
    if [ -n "${ROBOT_VERSION:-}" ]; then
        printf '%s' "${ROBOT_VERSION}"
        return
    fi

    if [ -f "${ROOT_BASHRC}" ]; then
        local detected_version=""
        detected_version="$(HOME=/root bash -ic 'printf %s "${ROBOT_VERSION:-}"' 2>/dev/null || true)"
        if [ -n "${detected_version}" ]; then
            printf '%s' "${detected_version}"
            return
        fi
    fi

    printf '46'
}

stop_and_disable_service() {
    systemctl stop "${SERVICE_NAME}" 2>/dev/null || true
    systemctl disable "${SERVICE_NAME}" 2>/dev/null || true
}

build_packages() {
    if [ ! -f /opt/ros/noetic/setup.bash ]; then
        echo "错误: 未找到 /opt/ros/noetic/setup.bash" >&2
        exit 1
    fi

    set +u
    source /opt/ros/noetic/setup.bash

    if [ "${BUILD_MODE}" = "open" ]; then
        if [ ! -f "${WS_ROOT}/installed/setup.bash" ]; then
            echo "错误: open 模式下未找到 ${WS_ROOT}/installed/setup.bash" >&2
            exit 1
        fi
        source "${WS_ROOT}/installed/setup.bash"
    fi
    set -u

    echo "开始编译整个工作区 (mode=${BUILD_MODE})"
    (
        cd "${WS_ROOT}"
        catkin build
    )
}

install_service() {
    if [ ! -f "${TEMPLATE_FILE}" ]; then
        echo "错误: 未找到模板文件 ${TEMPLATE_FILE}" >&2
        exit 1
    fi
    if [ ! -x "${MONITOR_SCRIPT}" ]; then
        echo "错误: monitor 脚本不可执行 ${MONITOR_SCRIPT}" >&2
        exit 1
    fi
    if [ ! -x "${SET_ACTIVE_PROFILE_SCRIPT}" ]; then
        echo "错误: 激活配置脚本不可执行 ${SET_ACTIVE_PROFILE_SCRIPT}" >&2
        exit 1
    fi
    if [ ! -x "${CYCLONEDDS_SETUP_SCRIPT}" ]; then
        echo "错误: CycloneDDS 配置脚本不可执行 ${CYCLONEDDS_SETUP_SCRIPT}" >&2
        exit 1
    fi

    stop_and_disable_service
    build_packages
    "${CYCLONEDDS_SETUP_SCRIPT}"

    mkdir -p "${AUTOSTART_ROOT}" "${PROFILES_DIR}"
    cp "${SET_ACTIVE_PROFILE_SCRIPT}" "${DEPLOYED_SET_ACTIVE_PROFILE_SCRIPT}"
    chmod 755 "${DEPLOYED_SET_ACTIVE_PROFILE_SCRIPT}"

    local robot_version=""
    local default_profile_dir=""
    robot_version="$(resolve_robot_version)"
    default_profile_dir="${WS_ROOT}/src/leju-controllers/leju-rl-controller/config/${robot_version}"
    if [ ! -f "${default_profile_dir}/controller_manager.yaml" ]; then
        echo "错误: 默认配置目录缺少 controller_manager.yaml: ${default_profile_dir}" >&2
        exit 1
    fi

    "${DEPLOYED_SET_ACTIVE_PROFILE_SCRIPT}" \
        --target-dir "${default_profile_dir}" \
        --source deploy \
        --ws-root "${WS_ROOT}" \
        --robot-version "${robot_version}"

    cp "${TEMPLATE_FILE}" "${SERVICE_FILE}"
    sed -i "s|@LEJULAB_WS@|$(escape_sed_replacement "${WS_ROOT}")|g" "${SERVICE_FILE}"
    sed -i "s|@MONITOR_SCRIPT@|$(escape_sed_replacement "${MONITOR_SCRIPT}")|g" "${SERVICE_FILE}"

    systemctl daemon-reload
    systemctl enable "${SERVICE_NAME}"
    systemctl restart "${SERVICE_NAME}"

    echo "已安装并启动 ${SERVICE_NAME}"
    echo "工作区: ${WS_ROOT}"
    echo "编译模式: ${BUILD_MODE}"
    echo "CycloneDDS 配置脚本: ${CYCLONEDDS_SETUP_SCRIPT}"
    echo "默认激活配置: ${default_profile_dir}"
    echo "前端切换脚本: ${DEPLOYED_SET_ACTIVE_PROFILE_SCRIPT}"
    echo "运行环境: ${WS_ROOT}/devel/setup.bash"
    systemctl status "${SERVICE_NAME}" --no-pager || true
}

remove_service() {
    stop_and_disable_service
    rm -f "${SERVICE_FILE}"
    systemctl daemon-reload
    echo "已移除 ${SERVICE_NAME}"
}

case "${1:-}" in
    --remove)
        remove_service
        ;;
    "")
        remove_service
        install_service
        ;;
    *)
        echo "用法: $0 [--remove]" >&2
        exit 1
        ;;
esac
