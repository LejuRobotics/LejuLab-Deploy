#!/bin/bash

set -euo pipefail

AUTOSTART_ROOT="/root/.config/lejulab/auto_start_config"
PROFILES_DIR="${AUTOSTART_ROOT}/profiles"
CURRENT_LINK="${AUTOSTART_ROOT}/current"
METADATA_FILE="${AUTOSTART_ROOT}/current.meta.json"
USER_DANCE_NAME="user_dance"
USER_DANCE_LINK="${CURRENT_LINK}/${USER_DANCE_NAME}"

usage() {
    echo "用法:" >&2
    echo "  直接激活已有目录: $0 --target-dir <abs_dir> --source <deploy|frontend>" >&2
    echo "  部署并写定位信息: $0 --target-dir <abs_dir> --source deploy --ws-root <abs_dir> --robot-version <version>" >&2
    echo "  前端导入舞蹈文件: $0 --import-from <abs_dir> --profile-name <name> --source frontend" >&2
    echo "    (在 current 下挂载 user_dance -> profiles/<name>, 并向 base controller_manager.yaml 追加 mimic_user)" >&2
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

# ---------------------------------------------------------------------------
# validate_profile_dir  - 用于 deploy / activate_existing_dir 路径
#   校验目标目录包含 controller_manager.yaml（底板配置必须）
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# validate_import_dir  - 用于前端导入路径
#   校验导入目录包含 env.yaml / .csv 轨迹 / .onnx 模型
#   返回规范化路径
# ---------------------------------------------------------------------------
validate_import_dir() {
    local dir="$1"

    if [[ "${dir}" != /* ]]; then
        echo "错误: 导入目录必须是绝对路径: ${dir}" >&2
        exit 1
    fi

    dir="$(readlink -f "${dir}")"
    if [ ! -d "${dir}" ]; then
        echo "错误: 导入目录不存在: ${dir}" >&2
        exit 1
    fi

    local yaml_count csv_count onnx_count
    yaml_count="$(find "${dir}" -maxdepth 1 -type f \( -iname '*.yaml' -o -iname '*.yml' \) | wc -l)"
    csv_count="$(find "${dir}" -maxdepth 1 -type f -iname '*.csv' | wc -l)"
    onnx_count="$(find "${dir}" -maxdepth 1 -type f -iname '*.onnx' | wc -l)"

    if [ "${yaml_count}" -eq 0 ]; then
        echo "错误: 导入目录缺少 .yaml / .yml 文件 (env.yaml): ${dir}" >&2
        exit 1
    fi
    if [ "${yaml_count}" -gt 1 ]; then
        echo "错误: 导入目录包含多个 .yaml/.yml 文件 (${yaml_count}), 无法确定哪个是 env.yaml: ${dir}" >&2
        exit 1
    fi
    if [ "${csv_count}" -eq 0 ]; then
        echo "错误: 导入目录缺少 .csv 轨迹文件: ${dir}" >&2
        exit 1
    fi
    if [ "${onnx_count}" -eq 0 ]; then
        echo "错误: 导入目录缺少 .onnx 模型文件: ${dir}" >&2
        exit 1
    fi

    printf '%s\n' "${dir}"
}

# ---------------------------------------------------------------------------
# detect_import_files  - 在导入目录中检测具体文件名
#   输出格式（每行一个 var=value，供 eval 使用）:
#     ENV_YAML=<basename>
#     CSV_FILE=<basename>
#     ONNX_FILE=<basename>
# ---------------------------------------------------------------------------
detect_import_files() {
    local dir="$1"

    local env_yaml csv_file onnx_file

    env_yaml="$(find "${dir}" -maxdepth 1 -type f \( -iname '*.yaml' -o -iname '*.yml' \) -printf '%f\n' | head -1)"
    csv_file="$(find "${dir}" -maxdepth 1 -type f -iname '*.csv' -printf '%f\n' | head -1)"
    onnx_file="$(find "${dir}" -maxdepth 1 -type f -iname '*.onnx' -printf '%f\n' | head -1)"

    printf 'ENV_YAML=%s\n' "${env_yaml}"
    printf 'CSV_FILE=%s\n' "${csv_file}"
    printf 'ONNX_FILE=%s\n' "${onnx_file}"
}

# ---------------------------------------------------------------------------
# resolve_ws_root  - 推导工作空间根目录
#   优先级: current.meta.json.ws_root > $LEJULAB_WS_ROOT > 脚本路径推导
# ---------------------------------------------------------------------------
resolve_ws_root() {
    local ws_root=""

    # 1) 从 metadata 读取 (deploy 时写入)
    if [ -f "${METADATA_FILE}" ]; then
        ws_root="$(python3 -c "
import json, sys
try:
    with open('${METADATA_FILE}') as f:
        d = json.load(f) or {}
    print(d.get('ws_root', ''))
except Exception:
    pass
" 2>/dev/null || true)"
        if [ -n "${ws_root}" ] && [ -d "${ws_root}" ]; then
            printf '%s\n' "${ws_root}"
            return 0
        fi
    fi

    # 2) 环境变量 (systemd 设置)
    if [ -n "${LEJULAB_WS_ROOT:-}" ] && [ -d "${LEJULAB_WS_ROOT}" ]; then
        printf '%s\n' "${LEJULAB_WS_ROOT}"
        return 0
    fi

    # 3) 从脚本自身路径推导
    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    case "${script_dir}" in
        */share/leju-joystick/services)
            ws_root="$(cd "${script_dir}/../../../.." && pwd)"
            ;;
        */src/leju-joystick/services)
            ws_root="$(cd "${script_dir}/../../.." && pwd)"
            ;;
        *)
            echo "错误: 无法推导工作空间根目录 (ws_root)，请设置 LEJULAB_WS_ROOT 环境变量" >&2
            exit 1
            ;;
    esac

    if [ ! -d "${ws_root}" ]; then
        echo "错误: 推导的工作空间根目录不存在: ${ws_root}" >&2
        exit 1
    fi
    printf '%s\n' "${ws_root}"
}

# ---------------------------------------------------------------------------
# write_metadata
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# write_user_dance_metadata  - 在 current.meta.json 合并 user_dance 字段
#   记录当前挂载的用户舞蹈信息，不改动 active_dir 等既有字段。
# ---------------------------------------------------------------------------
write_user_dance_metadata() {
    local profile_name="$1"
    local profile_dir="$2"
    local source_name="$3"
    local updated_at
    updated_at="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"

    python3 - "${METADATA_FILE}" "${profile_name}" "${profile_dir}" "${source_name}" "${updated_at}" <<'PY'
import json
import os
import sys

path, name, dance_dir, source, updated_at = sys.argv[1:]
data = {}
if os.path.exists(path):
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f) or {}
    except (json.JSONDecodeError, OSError):
        data = {}
data["user_dance"] = {
    "name": name,
    "dir": dance_dir,
    "source": source,
    "updated_at": updated_at,
}
with open(path, "w", encoding="utf-8") as f:
    json.dump(data, f, ensure_ascii=False, indent=2)
    f.write("\n")
PY
}

# ---------------------------------------------------------------------------
# switch_current_link  - 原子更新 current 符号链接
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# patch_controller_manager  - 在 controller_manager.yaml 的 controllers
#   列表末尾追加 mimic_user 条目（若不存在）。采用文本插入保留原文件注释，
#   mimic_user 的 config 指向 user_dance/config_mimic.yaml。
# ---------------------------------------------------------------------------
patch_controller_manager() {
    local target_file="$1"

    python3 - "${target_file}" "${USER_DANCE_NAME}" <<'PY'
import os
import re
import sys
from pathlib import Path

target = Path(sys.argv[1])
user_dance = sys.argv[2]
config_value = f"{user_dance}/config_mimic.yaml"

if not target.is_file():
    print(f"error: {target} not found", file=sys.stderr)
    sys.exit(1)

text = target.read_text(encoding="utf-8")
lines = text.splitlines()

# 幂等：已存在 name: "mimic_user" 则跳过（兼容 - name: 与 name: 两种写法、有无引号）
name_re = re.compile(r'^\s*-?\s*name:\s*"?mimic_user"?\s*$')
if any(name_re.match(line) for line in lines):
    print("mimic_user entry already exists, skip")
    sys.exit(0)


def entry_block(indent):
    return [
        f'{indent}- name: "mimic_user"',
        f'{indent}  type: "GenericRLController"',
        f'{indent}  config: "{config_value}"',
        f'{indent}  enabled: true',
    ]


# 定位顶层 controllers: 键
ctrl_idx = None
for i, line in enumerate(lines):
    if re.match(r'^controllers:\s*$', line):
        ctrl_idx = i
        break

if ctrl_idx is None:
    # 无 controllers 键：在文件末尾追加一个 controllers 段
    new_lines = lines + ["", "controllers:"] + entry_block("  ")
else:
    # 探测列表条目缩进（取 controllers: 之后第一个列表项的前导空格）
    item_indent = None
    for line in lines[ctrl_idx + 1:]:
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        m = re.match(r'^(\s*)-\s+', line)
        if m:
            item_indent = m.group(1)
        break
    if item_indent is None:
        item_indent = "  "

    # 定位插入点 = controllers 列表结束处：
    #   列表项及其字段均有缩进；首个缩进 0 的非空非注释行即为下一个顶层键，否则 EOF
    insert_idx = len(lines)
    for j in range(ctrl_idx + 1, len(lines)):
        line = lines[j]
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if line[0] in (" ", "\t"):
            # 缩进行：属于 controllers 列表（条目或其字段）
            continue
        # 缩进 0 的非空非注释行 -> 新的顶层键 -> 边界
        insert_idx = j
        break

    block = entry_block(item_indent)
    new_lines = lines[:insert_idx] + block + lines[insert_idx:]

out = "\n".join(new_lines)
if not out.endswith("\n"):
    out += "\n"

# 原子写回
tmp = f"{target}.tmp.{os.getpid()}"
Path(tmp).write_text(out, encoding="utf-8")
os.replace(tmp, str(target))
print("added mimic_user entry")
PY
}

# ---------------------------------------------------------------------------
# import_dance_profile  - 前端导入舞蹈文件主流程（user_dance 挂载模式）
#   不替换 current；current 始终指向 deploy 部署的 base profile。
#   1. 校验导入目录 (.yaml + .csv + .onnx)
#   2. 拷贝所有文件到 staging
#   3. 调用训练脚本生成 config_mimic.yaml
#   4. 原子替换 profiles/<name>
#   5. (re)point current/user_dance -> profiles/<name>
#   6. 就地修补 base controller_manager.yaml 追加 mimic_user (config: user_dance/config_mimic.yaml)
#   7. 记录 user_dance 元数据
# ---------------------------------------------------------------------------
import_dance_profile() {
    local import_from_dir="$1"
    local profile_name="$2"
    local source_name="$3"

    # --- 校验 profile-name ---
    if [[ "${profile_name}" == *"/"* ]] || [[ "${profile_name}" == "." ]] || [[ "${profile_name}" == ".." ]]; then
        echo "error: invalid --profile-name: ${profile_name}" >&2
        exit 1
    fi

    # --- 校验导入目录 ---
    import_from_dir="$(validate_import_dir "${import_from_dir}")"

    # --- 检测文件名 ---
    eval "$(detect_import_files "${import_from_dir}")"

    # --- 推导 ws_root 及训练脚本路径 ---
    local ws_root
    ws_root="$(resolve_ws_root)"
    local train_script="${ws_root}/scripts/mimic_config_train_to_deploy.py"
    if [ ! -f "${train_script}" ]; then
        echo "error: train script not found: ${train_script}" >&2
        exit 1
    fi

    # --- 解析当前 active (base) 目录，必须含 controller_manager.yaml ---
    local base_dir
    base_dir="$(readlink -f "${CURRENT_LINK}" 2>/dev/null || true)"
    if [ -z "${base_dir}" ] || [ ! -f "${base_dir}/controller_manager.yaml" ]; then
        echo "error: active profile missing controller_manager.yaml, run deploy_joy_autostart.sh first" >&2
        exit 1
    fi

    # --- 创建 staging 目录 ---
    local managed_target_dir="${PROFILES_DIR}/${profile_name}"
    local staging_dir="${PROFILES_DIR}/.${profile_name}.staging.$$"
    rm -rf "${staging_dir}"
    mkdir -p "${staging_dir}"

    # --- Step 1: 拷贝导入目录所有文件到 staging ---
    cp -a "${import_from_dir}/." "${staging_dir}/"
    echo "copied import files to staging: ${staging_dir}"

    # --- Step 2: 调用训练脚本生成 config_mimic.yaml ---
    # aarch64 (RK3588) 未编译 OpenVINO 后端，强制 onnxruntime；x86 保持 openvino 默认
    local engine_arg=""
    local engine_name="openvino"
    if [ "$(uname -m)" = "aarch64" ]; then
        engine_arg="--inference-engine onnxruntime"
        engine_name="onnxruntime"
    fi
    echo "generating config_mimic.yaml (inference_engine: ${engine_name}) ..."
    python3 "${train_script}" \
        --env-yaml "${staging_dir}/${ENV_YAML}" \
        --dance-name "${profile_name}" \
        --policy-path "${ONNX_FILE}" \
        --motion-file "${CSV_FILE}" \
        ${engine_arg} \
        --out "${staging_dir}/config_mimic.yaml" || {
        echo "error: train script failed" >&2
        rm -rf "${staging_dir}"
        exit 1
    }
    echo "generated config_mimic.yaml"

    # --- Step 3: 原子替换 profiles/<name> ---
    rm -rf "${managed_target_dir}"
    mv "${staging_dir}" "${managed_target_dir}"

    # --- Step 4: (re)point current/user_dance -> profiles/<name> (原子) ---
    local tmp_link="${USER_DANCE_LINK}.tmp.$$"
    ln -sfn "${managed_target_dir}" "${tmp_link}"
    mv -Tf "${tmp_link}" "${USER_DANCE_LINK}"
    echo "mounted ${USER_DANCE_LINK} -> ${managed_target_dir}"

    # --- Step 5: 就地修补 base controller_manager.yaml 追加 mimic_user ---
    patch_controller_manager "${base_dir}/controller_manager.yaml"

    # --- Step 6: 记录 user_dance 元数据 ---
    write_user_dance_metadata "${profile_name}" "${managed_target_dir}" "${source_name}"

    echo "imported user dance: ${profile_name} -> ${managed_target_dir}"
    echo "note: restart runtime (BACK+START to quit, then START) to load the new dance"
}

# ---------------------------------------------------------------------------
# activate_existing_dir  - deploy 路径: 直接激活已有配置目录
# ---------------------------------------------------------------------------
activate_existing_dir() {
    local target_dir="$1"
    local source_name="$2"
    local ws_root="${3:-}"
    local robot_version="${4:-}"

    target_dir="$(validate_profile_dir "${target_dir}")"
    switch_current_link "${target_dir}" "${source_name}" "yes" "${ws_root}" "${robot_version}"
    echo "已切换 current -> ${target_dir}"
}

# ===========================================================================
# 主分发逻辑
# ===========================================================================
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
    import_dance_profile "${IMPORT_FROM_DIR}" "${PROFILE_NAME}" "${SOURCE}"
    exit 0
fi

[ -n "${TARGET_DIR}" ] || usage
activate_existing_dir "${TARGET_DIR}" "${SOURCE}" "${WS_ROOT}" "${ROBOT_VERSION_ARG}"
