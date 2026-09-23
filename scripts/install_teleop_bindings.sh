#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: install_teleop_bindings.sh [--overwrite|--keep] [--source PATH] [--target PATH]

Install the repository teleop binding defaults into the controller user's
~/.config/lejuconfig directory. Existing files require an explicit policy in
non-interactive mode.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROBOT_VERSION="${ROBOT_VERSION:-17}"
SOURCE="${TELEOP_STANDARD_SOURCE:-${REPO_ROOT}/src/leju-controllers/leju-rl-controller/config/${ROBOT_VERSION}/teleop_bindings.yaml}"
TARGET="${TELEOP_RUNTIME_TARGET:-${HOME}/.config/lejuconfig/teleop_bindings.yaml}"
POLICY=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --overwrite) POLICY="overwrite" ;;
        --keep) POLICY="keep" ;;
        --source) shift; SOURCE="${1:?--source requires PATH}" ;;
        --target) shift; TARGET="${1:?--target requires PATH}" ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

validate_yaml() {
    "${TELEOP_PYTHON:-/usr/bin/python3}" - "$1" <<'PY'
import sys
import yaml

path = sys.argv[1]
with open(path, encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
if not isinstance(config, dict) or not isinstance(config.get("joy_bindings"), list):
    raise SystemExit("teleop YAML must contain a joy_bindings list: " + path)
PY
}

[[ -f "$SOURCE" ]] || { echo "ERROR: standard config not found: $SOURCE" >&2; exit 1; }
validate_yaml "$SOURCE"

if [[ -e "$TARGET" && -z "$POLICY" ]]; then
    if [[ -t 0 ]]; then
        printf '[teleop-install] 手柄运行配置已存在：%s\n' "$TARGET"
        read -r -p '[teleop-install] 是否先创建备份并覆盖现有配置？[y/N] ' answer
        [[ "$answer" =~ ^[Yy]$ ]] && POLICY="overwrite" || POLICY="keep"
    else
        echo "ERROR: target exists; specify --overwrite or --keep" >&2
        exit 2
    fi
fi

if [[ -e "$TARGET" && "$POLICY" == "keep" ]]; then
    echo "[teleop-install] kept existing config: $TARGET"
    sha256sum "$TARGET"
    exit 0
fi

target_dir="$(dirname "$TARGET")"
mkdir -p "$target_dir"
tmp="$(mktemp "${target_dir}/.teleop_bindings.XXXXXX")"
trap 'rm -f "$tmp"' EXIT
cp -- "$SOURCE" "$tmp"
validate_yaml "$tmp"

if [[ -e "$TARGET" ]] && cmp -s -- "$tmp" "$TARGET"; then
    echo "[teleop-install] unchanged: $TARGET"
    exit 0
fi

if [[ -e "$TARGET" ]]; then
    backup="${TARGET}.bak"
    cp -p -- "$TARGET" "$backup"
    chmod --reference="$TARGET" "$tmp"
    echo "[teleop-install] backup: $backup"
else
    chmod 0644 "$tmp"
fi

mv -f -- "$tmp" "$TARGET"
trap - EXIT
echo "[teleop-install] installed: $SOURCE -> $TARGET"
sha256sum "$TARGET"
