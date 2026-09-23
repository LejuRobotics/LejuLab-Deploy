#!/bin/bash

set -euo pipefail

TELEOP_CONFIG_POLICY=""
ACTION="install"

usage() {
    cat <<'EOF'
用法: deploy_joy_autostart.sh [--keep-teleop-config|--overwrite-teleop-config] [--remove]

部署手柄自启动服务时，同时安装运行时 teleop_bindings.yaml：
  --keep-teleop-config       保留已有现场配置
  --overwrite-teleop-config  使用当前机器人版本的标准配置覆盖，并创建 .bak 备份

未指定策略时，首次部署会直接安装；配置已存在时会交互询问是否覆盖。
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --keep-teleop-config)
            if [ -n "${TELEOP_CONFIG_POLICY}" ] && [ "${TELEOP_CONFIG_POLICY}" != "keep" ]; then
                echo "错误: 手柄配置保留与覆盖选项不能同时使用" >&2
                exit 2
            fi
            TELEOP_CONFIG_POLICY="keep"
            ;;
        --overwrite-teleop-config)
            if [ -n "${TELEOP_CONFIG_POLICY}" ] && [ "${TELEOP_CONFIG_POLICY}" != "overwrite" ]; then
                echo "错误: 手柄配置保留与覆盖选项不能同时使用" >&2
                exit 2
            fi
            TELEOP_CONFIG_POLICY="overwrite"
            ;;
        --remove)
            ACTION="remove"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "错误: 未知参数 $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

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
#   安装模式:              <ws>/share/leju-joystick/services/
case "${SCRIPT_DIR}" in
    */share/leju-joystick/services)
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

escape_sed_replacement() {
    printf '%s' "$1" | sed -e 's/[\/&]/\\&/g'
}

resolve_robot_version() {
    if [ -n "${ROBOT_VERSION:-}" ]; then
        printf '%s' "${ROBOT_VERSION}"
        return
    fi

    # 优先读 sudo 调用者的 bashrc（部署时通常是 sudo su 进来的普通用户）
    local caller_home=""
    if [ -n "${SUDO_USER:-}" ]; then
        caller_home="$(getent passwd "${SUDO_USER}" | cut -d: -f6)"
    fi

    for bashrc in "${caller_home}/.bashrc" "${ROOT_BASHRC}"; do
        if [ -f "${bashrc}" ]; then
            local detected_version=""
            detected_version="$(HOME="$(dirname "${bashrc}")" bash -ic 'printf %s "${ROBOT_VERSION:-}"' 2>/dev/null || true)"
            if [ -n "${detected_version}" ]; then
                printf '%s' "${detected_version}"
                return
            fi
        fi
    done

    echo "错误: 未能自动检测到 ROBOT_VERSION，请手动指定:" >&2
    echo "  ROBOT_VERSION=14 sudo bash deploy_joy_autostart.sh" >&2
    echo "  ROBOT_VERSION=46 sudo bash deploy_joy_autostart.sh" >&2
    exit 1
}

stop_and_disable_service() {
    systemctl stop "${SERVICE_NAME}" 2>/dev/null || true
    systemctl disable "${SERVICE_NAME}" 2>/dev/null || true
}

install_deps() {
    if ! command -v tmux >/dev/null 2>&1; then
        echo "安装依赖: tmux"
        apt-get install -y tmux
    fi
}

build_packages() {
    echo "开始编译整个工作区 (pure CMake)"

    # 如果由 sudo 调用，切回原用户编译，避免 build/ 文件属主变为 root
    if [ -n "${SUDO_USER:-}" ] && [ "${SUDO_USER}" != "root" ]; then
        echo "以用户 ${SUDO_USER} 身份编译，确保 build/ 目录权限正确..."
        # 确保 build/ 属主是原用户（之前可能被 sudo 污染过）
        chown -R "${SUDO_USER}:${SUDO_USER}" "${WS_ROOT}/build" 2>/dev/null || true
        sudo -u "${SUDO_USER}" env CC=gcc-11 CXX=g++-11 \
            cmake -B"${WS_ROOT}/build" -S"${WS_ROOT}" -DCMAKE_BUILD_TYPE=Release
        sudo -u "${SUDO_USER}" env CC=gcc-11 CXX=g++-11 \
            cmake --build "${WS_ROOT}/build" -j"$(nproc)"
    else
        CC=gcc-11 CXX=g++-11 cmake -B"${WS_ROOT}/build" -S"${WS_ROOT}" -DCMAKE_BUILD_TYPE=Release
        CC=gcc-11 CXX=g++-11 cmake --build "${WS_ROOT}/build" -j"$(nproc)"
    fi
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
    install_deps
    setup_python_symlink

    build_packages

    "${CYCLONEDDS_SETUP_SCRIPT}"

    # service 以 root 运行，HOME=/root，RuiWo SDK / hardware 节点从
    # /root/.config/lejuconfig/ 读取配置，需确保该目录下文件齐全
    mkdir -p /root/.config/lejuconfig

    # 从仓库拷贝 RuiWo 电机配置（不覆盖已有的用户自定义文件）
    local ruiwo_config_src="${WS_ROOT}/src/leju-hardware/config/config.yaml"
    if [ -f "${ruiwo_config_src}" ] && [ ! -f "/root/.config/lejuconfig/config.yaml" ]; then
        cp "${ruiwo_config_src}" "/root/.config/lejuconfig/config.yaml"
        echo "已拷贝 RuiWo 电机配置: /root/.config/lejuconfig/config.yaml"
    fi

    # 从 SUDO_USER 的 lejuconfig 目录同步运行时配置（CanbusWiringType、ImuType 等）
    if [ -n "${SUDO_USER:-}" ]; then
        local user_lejuconfig
        user_lejuconfig="$(getent passwd "${SUDO_USER}" | cut -d: -f6)/.config/lejuconfig"
        if [ -d "${user_lejuconfig}" ]; then
            for f in CanbusWiringType.ini ImuType.ini canbus_device_cofig.yaml hipimuEulerOffset.csv; do
                if [ -f "${user_lejuconfig}/${f}" ] && [ ! -f "/root/.config/lejuconfig/${f}" ]; then
                    cp "${user_lejuconfig}/${f}" "/root/.config/lejuconfig/${f}"
                    echo "已同步配置: /root/.config/lejuconfig/${f}"
                fi
            done
        fi
    fi

    sync_resources_to_lejuconfig

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

    install_teleop_bindings "${robot_version}" "${default_profile_dir}"

    "${DEPLOYED_SET_ACTIVE_PROFILE_SCRIPT}" \
        --target-dir "${default_profile_dir}" \
        --source deploy \
        --ws-root "${WS_ROOT}" \
        --robot-version "${robot_version}"

    cp "${TEMPLATE_FILE}" "${SERVICE_FILE}"
    # 自动探测构建目录：优先 build/，其次 build_cmake/
    local build_dir="${WS_ROOT}/build"
    if [ ! -d "${build_dir}" ]; then
        build_dir="${WS_ROOT}/build_cmake"
    fi

    sed -i "s|@LEJULAB_WS@|$(escape_sed_replacement "${WS_ROOT}")|g" "${SERVICE_FILE}"
    sed -i "s|@BUILD_DIR@|$(escape_sed_replacement "${build_dir}")|g" "${SERVICE_FILE}"
    sed -i "s|@MONITOR_SCRIPT@|$(escape_sed_replacement "${MONITOR_SCRIPT}")|g" "${SERVICE_FILE}"
    sed -i "s|@ROBOT_VERSION@|$(escape_sed_replacement "${robot_version}")|g" "${SERVICE_FILE}"

    systemctl daemon-reload
    systemctl enable "${SERVICE_NAME}"
    systemctl restart "${SERVICE_NAME}"

    echo "已安装并启动 ${SERVICE_NAME}"
    echo "工作区: ${WS_ROOT}"
    echo "CycloneDDS 配置脚本: ${CYCLONEDDS_SETUP_SCRIPT}"
    echo "默认激活配置: ${default_profile_dir}"
    echo "前端切换脚本: ${DEPLOYED_SET_ACTIVE_PROFILE_SCRIPT}"
    systemctl status "${SERVICE_NAME}" --no-pager || true
}

install_teleop_bindings() {
    local robot_version="$1"
    local default_profile_dir="$2"
    local installer="${WS_ROOT}/scripts/install_teleop_bindings.sh"
    local config_user="${SUDO_USER:-root}"
    local config_home=""
    local -a policy_args=()

    if [ ! -x "${installer}" ]; then
        echo "错误: 手柄配置安装脚本不存在或不可执行: ${installer}" >&2
        exit 1
    fi

    config_home="$(getent passwd "${config_user}" | cut -d: -f6)"
    if [ -z "${config_home}" ]; then
        echo "错误: 无法确定配置用户 ${config_user} 的 HOME" >&2
        exit 1
    fi

    if [ -n "${TELEOP_CONFIG_POLICY}" ]; then
        policy_args=("--${TELEOP_CONFIG_POLICY}")
    fi

    echo "安装手柄运行配置（用户: ${config_user}，策略: ${TELEOP_CONFIG_POLICY:-交互选择}）..."
    if [ "${config_user}" = "root" ]; then
        HOME="${config_home}" ROBOT_VERSION="${robot_version}" \
            "${installer}" \
            --source "${default_profile_dir}/teleop_bindings.yaml" \
            --target "${config_home}/.config/lejuconfig/teleop_bindings.yaml" \
            "${policy_args[@]}"
    else
        sudo -u "${config_user}" -H env ROBOT_VERSION="${robot_version}" \
            "${installer}" \
            --source "${default_profile_dir}/teleop_bindings.yaml" \
            --target "${config_home}/.config/lejuconfig/teleop_bindings.yaml" \
            "${policy_args[@]}"
    fi
}

remove_service() {
    stop_and_disable_service
    rm -f "${SERVICE_FILE}"
    systemctl daemon-reload
    echo "已移除 ${SERVICE_NAME}"
}

sync_resources_to_lejuconfig() {
    local src_resources="${WS_ROOT}/resources"

    # 优先用 sudo 调用者的 home，回退到 $HOME
    local user_home=""
    if [ -n "${SUDO_USER:-}" ]; then
        user_home="$(getent passwd "${SUDO_USER}" | cut -d: -f6)"
    fi
    user_home="${user_home:-${HOME}}"

    local dst_config="${user_home}/.config/lejuconfig"

    if [ ! -d "${src_resources}" ]; then
        echo "警告: 资源目录不存在 ${src_resources}，跳过同步"
        return
    fi

    mkdir -p "${dst_config}"

    for dir in action_files music; do
        local src_dir="${src_resources}/${dir}"
        local dst_dir="${dst_config}/${dir}"

        if [ ! -d "${src_dir}" ]; then
            echo "警告: 源目录不存在 ${src_dir}，跳过"
            continue
        fi

        if [ -d "${dst_dir}" ]; then
            read -r -p "目录 ${dst_dir} 已存在，是否替换？[y/N] " answer
            if [[ "${answer,,}" != "y" && "${answer,,}" != "yes" ]]; then
                echo "跳过 ${dir}"
                continue
            fi
            rm -rf "${dst_dir}"
        fi

        cp -r "${src_dir}" "${dst_dir}"
        echo "已复制: ${src_dir} -> ${dst_dir}"

        # 修复权限: 以 root 执行 cp 导致文件属主为 root，
        # NX 后续通过 lab 用户 SCP 覆盖动作文件时报 Permission denied
        if [ -n "${SUDO_USER:-}" ]; then
            chown -R "${SUDO_USER}:${SUDO_USER}" "${dst_dir}"
            echo "已修正 ${dst_dir} 属主为 ${SUDO_USER}"
        fi
    done
}

setup_python_symlink() {
    if [ -L /usr/bin/python ] && [ "$(readlink /usr/bin/python)" = "/usr/bin/python3" ]; then
        echo "Python 软链接已正确指向: /usr/bin/python -> /usr/bin/python3"
        return
    fi

    ln -sf /usr/bin/python3 /usr/bin/python
    echo "已创建 Python 软链接: /usr/bin/python -> /usr/bin/python3"
}

case "${ACTION}" in
    remove)
        remove_service
        ;;
    install)
        remove_service
        install_service
        ;;
esac
