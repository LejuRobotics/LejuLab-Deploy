#!/bin/bash

set -euo pipefail

AUTOSTART_ROOT="/root/.config/lejulab/auto_start_config"
PROFILES_DIR="${AUTOSTART_ROOT}/profiles"
CURRENT_LINK="${AUTOSTART_ROOT}/current"
METADATA_FILE="${AUTOSTART_ROOT}/current.meta.json"

usage() {
    echo "用法:" >&2
    echo "  直接激活已有目录: $0 --target-dir <abs_dir> --source <deploy|frontend>" >&2
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
    local updated_at=""
    local updated_by=""
    local hostname_value=""

    updated_at="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
    updated_by="${SUDO_USER:-${USER:-root}}"
    hostname_value="$(hostname)"

    python3 - "${METADATA_FILE}" "${active_dir}" "${source_name}" "${updated_at}" "${updated_by}" "${hostname_value}" <<'PY'
import json
import sys

metadata_path, active_dir, source, updated_at, updated_by, hostname = sys.argv[1:]
payload = {
    "active_dir": active_dir,
    "source": source,
    "updated_at": updated_at,
    "updated_by": updated_by,
    "hostname": hostname,
}
with open(metadata_path, "w", encoding="utf-8") as f:
    json.dump(payload, f, ensure_ascii=False, indent=2)
    f.write("\n")
PY
}

switch_current_link() {
    local target_dir="$1"
    local source_name="$2"
    local write_meta="${3:-yes}"

    local tmp_link="${CURRENT_LINK}.tmp.$$"
    ln -sfn "${target_dir}" "${tmp_link}"
    mv -Tf "${tmp_link}" "${CURRENT_LINK}"

    if [ "${write_meta}" = "yes" ]; then
        write_metadata "${target_dir}" "${source_name}"
    fi
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

    target_dir="$(validate_profile_dir "${target_dir}")"
    switch_current_link "${target_dir}" "${source_name}" "yes"
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
activate_existing_dir "${TARGET_DIR}" "${SOURCE}"
