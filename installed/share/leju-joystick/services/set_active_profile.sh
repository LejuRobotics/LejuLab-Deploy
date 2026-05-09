#!/bin/bash

set -euo pipefail

AUTOSTART_ROOT="/root/.config/lejulab/auto_start_config"
PROFILES_DIR="${AUTOSTART_ROOT}/profiles"
CURRENT_LINK="${AUTOSTART_ROOT}/current"
METADATA_FILE="${AUTOSTART_ROOT}/current.meta.json"

usage() {
    echo "用法:" >&2
    echo "  直接激活已有目录: $0 --target-dir <abs_dir> --source <deploy|frontend>" >&2
    echo "  部署并写定位信息: $0 --target-dir <abs_dir> --source deploy --ws-root <abs_dir> --robot-version <version>" >&2
    echo "  前端导入并替换:   $0 --import-from <abs_dir> --profile-name <name> --source frontend" >&2
    exit 1
}

if [ "${EUID}" -ne 0 ]; then
    echo "错误: set_active_profile.sh 必须以 root 身份运行" >&2
    exit 1
fi

TARGET_DIR=""
IMPORT_FROM_DIR=""
PROFILE_NAME=""
SOURCE=""
WS_ROOT=""
ROBOT_VERSION_ARG=""

while [ "$#" -gt 0 ]; do
    case "$1" in
        --target-dir)
            [ "$#" -ge 2 ] || usage
            TARGET_DIR="$2"
            shift 2
            ;;
        --import-from)
            [ "$#" -ge 2 ] || usage
            IMPORT_FROM_DIR="$2"
            shift 2
            ;;
        --profile-name)
            [ "$#" -ge 2 ] || usage
            PROFILE_NAME="$2"
            shift 2
            ;;
        --source)
            [ "$#" -ge 2 ] || usage
            SOURCE="$2"
            shift 2
            ;;
        --ws-root|--WS_ROOT)
            [ "$#" -ge 2 ] || usage
            WS_ROOT="$2"
            shift 2
            ;;
        --robot-version|--robot_version)
            [ "$#" -ge 2 ] || usage
            ROBOT_VERSION_ARG="$2"
            shift 2
            ;;
        *)
            usage
            ;;
    esac
done

[ -n "${SOURCE}" ] || usage

case "${SOURCE}" in
    deploy|frontend)
        ;;
    *)
        echo "错误: --source 仅支持 deploy 或 frontend" >&2
        exit 1
        ;;
esac

if [ -n "${WS_ROOT}" ] || [ -n "${ROBOT_VERSION_ARG}" ]; then
    if [ "${SOURCE}" != "deploy" ]; then
        echo "错误: --ws-root/--robot-version 只能与 --source deploy 一起使用" >&2
        exit 1
    fi
    [ -n "${WS_ROOT}" ] || usage
    [ -n "${ROBOT_VERSION_ARG}" ] || usage
    if [[ "${WS_ROOT}" != /* ]]; then
        echo "错误: --ws-root 必须是绝对路径: ${WS_ROOT}" >&2
        exit 1
    fi
    if [ ! -d "${WS_ROOT}" ]; then
        echo "错误: --ws-root 目录不存在: ${WS_ROOT}" >&2
        exit 1
    fi
    WS_ROOT="$(readlink -f "${WS_ROOT}")"
fi

mkdir -p "${AUTOSTART_ROOT}" "${PROFILES_DIR}"

validate_profile_dir() {
    local dir="$1"

    if [[ "${dir}" != /* ]]; then
        echo "错误: 目录必须是绝对路径: ${dir}" >&2
        exit 1
    fi

    dir="$(readlink -f "${dir}")"
    if [ ! -d "${dir}" ]; then
        echo "错误: 目标目录不存在 ${dir}" >&2
        exit 1
    fi

    if [ ! -f "${dir}/controller_manager.yaml" ]; then
        echo "错误: 目标目录缺少 controller_manager.yaml: ${dir}" >&2
        exit 1
    fi

    printf '%s\n' "${dir}"
}

write_metadata() {
    local active_dir="$1"
    local source_name="$2"
    local ws_root="${3:-}"
    local robot_version="${4:-}"
    local updated_at=""
    local updated_by=""
    local hostname_value=""

    updated_at="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
    updated_by="${SUDO_USER:-${USER:-root}}"
    hostname_value="$(hostname)"

    python3 - "${METADATA_FILE}" "${active_dir}" "${source_name}" "${updated_at}" "${updated_by}" "${hostname_value}" "${ws_root}" "${robot_version}" <<'PY'
import json
import os
import sys

metadata_path, active_dir, source, updated_at, updated_by, hostname, ws_root, robot_version = sys.argv[1:]
data = {}
if os.path.exists(metadata_path):
    try:
        with open(metadata_path, encoding="utf-8") as f:
            data = json.load(f) or {}
    except (json.JSONDecodeError, OSError):
        data = {}
data["active_dir"] = active_dir
data["source"] = source
data["updated_at"] = updated_at
data["updated_by"] = updated_by
data["hostname"] = hostname
if source == "deploy" and ws_root and robot_version:
    data["ws_root"] = ws_root
    data["robot_version"] = robot_version
with open(metadata_path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2)
    f.write("\n")
PY
}

switch_current_link() {
    local target_dir="$1"
    local source_name="$2"
    local write_meta="${3:-yes}"
    local ws_root="${4:-}"
    local robot_version="${5:-}"

    local tmp_link="${CURRENT_LINK}.tmp.$$"
    ln -sfn "${target_dir}" "${tmp_link}"
    mv -Tf "${tmp_link}" "${CURRENT_LINK}"

    if [ "${write_meta}" = "yes" ]; then
        write_metadata "${target_dir}" "${source_name}" "${ws_root}" "${robot_version}"
    fi
}

synthesize_frontend_manager() {
    local staging_dir="$1"

    if [ ! -f "${METADATA_FILE}" ]; then
        echo "错误: 缺少 ${METADATA_FILE},请重跑 deploy_joy_autostart.sh 部署自启动服务" >&2
        return 1
    fi
    if [ ! -f "${staging_dir}/config_mimic.yaml" ]; then
        echo "错误: profile 缺少 config_mimic.yaml: ${staging_dir}" >&2
        return 1
    fi

    python3 - "${METADATA_FILE}" "${staging_dir}" <<'PY'
import json
import sys
from pathlib import Path

import yaml

metadata_path = Path(sys.argv[1])
staging_dir = Path(sys.argv[2])

try:
    with metadata_path.open(encoding="utf-8") as f:
        meta = json.load(f) or {}
except (json.JSONDecodeError, OSError) as e:
    print(f"错误: 读取 {metadata_path} 失败: {e}", file=sys.stderr)
    sys.exit(1)

ws_root = meta.get("ws_root") or ""
robot_version = meta.get("robot_version") or ""
if not ws_root or not robot_version:
    print(
        f"错误: {metadata_path} 缺少 ws_root 或 robot_version,"
        "请重跑 deploy_joy_autostart.sh 部署自启动服务",
        file=sys.stderr,
    )
    sys.exit(1)

repo_config_dir = Path(ws_root) / "src/leju-controllers/leju-rl-controller/config" / str(robot_version)
if not repo_config_dir.is_dir():
    print(f"错误: 仓库配置目录不存在 {repo_config_dir}", file=sys.stderr)
    sys.exit(1)

base_manager_path = repo_config_dir / "controller_manager.yaml"
if not base_manager_path.is_file():
    print(f"错误: 仓库底板不存在 {base_manager_path}", file=sys.stderr)
    sys.exit(1)

amp_candidates = [
    repo_config_dir / "controllers" / "config_amp.yaml",
    repo_config_dir / "config_amp.yaml",
]
amp_abs = next((c.resolve() for c in amp_candidates if c.is_file()), None)
if amp_abs is None:
    print(
        "错误: 未在仓库目录找到 config_amp.yaml: "
        + ", ".join(str(c) for c in amp_candidates),
        file=sys.stderr,
    )
    sys.exit(1)

mimic_name = "mimic_user"
teleop_path = repo_config_dir / "teleop_bindings.yaml"
if teleop_path.is_file():
    with teleop_path.open(encoding="utf-8") as f:
        bindings = yaml.safe_load(f) or {}
    for entry in bindings.get("joy_bindings") or []:
        if not isinstance(entry, dict):
            continue
        action = entry.get("action") or {}
        if action.get("type") != "SwitchController":
            continue
        name = (action.get("args") or {}).get("name")
        if isinstance(name, str) and name and name != "amp":
            mimic_name = name
            break

with base_manager_path.open(encoding="utf-8") as f:
    manager = yaml.safe_load(f) or {}
if not isinstance(manager, dict):
    print(f"错误: 仓库底板格式异常 {base_manager_path}", file=sys.stderr)
    sys.exit(1)

manager["default_controller"] = "amp"
manager["controllers"] = [
    {"name": "amp", "type": "GenericRLController", "config": str(amp_abs), "enabled": True},
    {"name": mimic_name, "type": "GenericRLController", "config": "config_mimic.yaml", "enabled": True},
]

out_path = staging_dir / "controller_manager.yaml"
with out_path.open("w", encoding="utf-8") as f:
    class FlowListDumper(yaml.SafeDumper):
        def increase_indent(self, flow=False, indentless=False):
            return super().increase_indent(flow=flow, indentless=False)

    def represent_list(dumper, data):
        flow_style = bool(data) and all(isinstance(x, (str, int, float, bool)) for x in data)
        return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=flow_style)

    FlowListDumper.add_representer(list, represent_list)
    yaml.dump(manager, f, Dumper=FlowListDumper, sort_keys=False, allow_unicode=True, default_flow_style=False)

print(f"已合成 {out_path} (amp={amp_abs}, mimic_name={mimic_name})")
PY
}

import_profile_and_activate() {
    local import_from_dir="$1"
    local profile_name="$2"
    local source_name="$3"
    local managed_target_dir="${PROFILES_DIR}/${profile_name}"
    local staging_dir="${PROFILES_DIR}/.${profile_name}.staging.$$"
    local current_target=""

    import_from_dir="$(validate_profile_dir "${import_from_dir}")"
    if [[ "${profile_name}" == *"/"* ]] || [[ "${profile_name}" == "." ]] || [[ "${profile_name}" == ".." ]]; then
        echo "错误: --profile-name 不合法: ${profile_name}" >&2
        exit 1
    fi

    rm -rf "${staging_dir}"
    cp -a "${import_from_dir}" "${staging_dir}"
    validate_profile_dir "${staging_dir}" >/dev/null

    if [ "${source_name}" = "frontend" ]; then
        if ! synthesize_frontend_manager "${staging_dir}"; then
            rm -rf "${staging_dir}"
            exit 1
        fi
    fi

    current_target="$(readlink -f "${CURRENT_LINK}" 2>/dev/null || true)"
    if [ -n "${current_target}" ] && [ "${current_target}" = "${managed_target_dir}" ]; then
        switch_current_link "${staging_dir}" "${source_name}" "no"
    fi

    rm -rf "${managed_target_dir}"
    mv "${staging_dir}" "${managed_target_dir}"

    switch_current_link "${managed_target_dir}" "${source_name}" "yes"
    echo "已导入并激活 profile: ${profile_name} -> ${managed_target_dir}"
}

activate_existing_dir() {
    local target_dir="$1"
    local source_name="$2"
    local ws_root="${3:-}"
    local robot_version="${4:-}"

    target_dir="$(validate_profile_dir "${target_dir}")"
    switch_current_link "${target_dir}" "${source_name}" "yes" "${ws_root}" "${robot_version}"
    echo "已切换 current -> ${target_dir}"
}

if [ -n "${IMPORT_FROM_DIR}" ] || [ -n "${PROFILE_NAME}" ]; then
    [ "${SOURCE}" = "frontend" ] || {
        echo "错误: --import-from/--profile-name 仅支持 --source frontend" >&2
        exit 1
    }
    [ -n "${IMPORT_FROM_DIR}" ] || usage
    [ -n "${PROFILE_NAME}" ] || usage
    [ -z "${TARGET_DIR}" ] || {
        echo "错误: 使用 --import-from 模式时不要再传 --target-dir" >&2
        exit 1
    }
    import_profile_and_activate "${IMPORT_FROM_DIR}" "${PROFILE_NAME}" "${SOURCE}"
    exit 0
fi

[ -n "${TARGET_DIR}" ] || usage
activate_existing_dir "${TARGET_DIR}" "${SOURCE}" "${WS_ROOT}" "${ROBOT_VERSION_ARG}"
