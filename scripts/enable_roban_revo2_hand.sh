#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PWD_PROJECT_DIR="$(pwd)"

if [[ -f "${SCRIPT_PROJECT_DIR}/src/leju-hardware/config/roban_v17/kuavo.json" ]]; then
  PROJECT_DIR="${SCRIPT_PROJECT_DIR}"
elif [[ -f "${PWD_PROJECT_DIR}/src/leju-hardware/config/roban_v17/kuavo.json" ]]; then
  PROJECT_DIR="${PWD_PROJECT_DIR}"
else
  PROJECT_DIR="${SCRIPT_PROJECT_DIR}"
fi

KUAVO_JSON="${PROJECT_DIR}/src/leju-hardware/config/roban_v17/kuavo.json"
CAN_CONFIG="${HOME}/.config/lejuconfig/canbus_device_cofig.yaml"
CHECK_ONLY=0
UPDATE_KUAVO=1
UPDATE_CAN=1

usage() {
  cat <<'EOF'
Usage: scripts/enable_roban_revo2_hand.sh [options]

Enable Revo2 dexterous hands for a Roban v17 test machine.
The script is idempotent and preserves one original backup per config outside
the repository under ~/.local/state/lejulab/config_backups/.

Options:
  --check                 Only print current status; do not modify files.
  --kuavo-json PATH       kuavo.json path. Default:
                          ./src/leju-hardware/config/roban_v17/kuavo.json
  --can-config PATH       CAN device yaml path. Default:
                          ~/.config/lejuconfig/canbus_device_cofig.yaml
  --no-kuavo              Do not update EndEffectorType in kuavo.json.
  --no-can                Do not update CAN device yaml.
  -h, --help              Show this help.

What it does:
  1. Set EndEffectorType to ["revo2", "revo2"] in roban_v17/kuavo.json.
  2. Ensure Lhand_revo2_hand exists in bcan2_devices if available,
     otherwise canbus0_devices.
  3. Ensure Rhand_revo2_hand exists in bcan3_devices if available,
     otherwise canbus1_devices.

Notes:
  - This is a site/test-machine helper. It does not change repository defaults
    unless you point --kuavo-json at a tracked config file intentionally.
  - Run hardware after this script so leju-hardware reloads the config.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check)
      CHECK_ONLY=1
      shift
      ;;
    --kuavo-json)
      KUAVO_JSON="${2:?missing value for --kuavo-json}"
      shift 2
      ;;
    --can-config)
      CAN_CONFIG="${2:?missing value for --can-config}"
      shift 2
      ;;
    --no-kuavo)
      UPDATE_KUAVO=0
      shift
      ;;
    --no-can)
      UPDATE_CAN=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

python3 - "$KUAVO_JSON" "$CAN_CONFIG" "$CHECK_ONLY" "$UPDATE_KUAVO" "$UPDATE_CAN" <<'PY'
import json
import os
import re
import shutil
import sys
from pathlib import Path

kuavo_json = Path(sys.argv[1]).expanduser()
can_config = Path(sys.argv[2]).expanduser()
check_only = sys.argv[3] == "1"
update_kuavo = sys.argv[4] == "1"
update_can = sys.argv[5] == "1"
state_home = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state"))
backup_dir = state_home / "lejulab" / "config_backups"


def info(msg: str) -> None:
    print(f"[INFO] {msg}")


def ok(msg: str) -> None:
    print(f"[OK] {msg}")


def warn(msg: str) -> None:
    print(f"[WARN] {msg}")


def fail(msg: str) -> None:
    print(f"[ERROR] {msg}", file=sys.stderr)
    raise SystemExit(1)


def backup(path: Path) -> None:
    if check_only:
        return
    backup_dir.mkdir(parents=True, exist_ok=True)
    backup_name = str(path.resolve()).lstrip("/").replace("/", "__") + ".bak"
    bak = backup_dir / backup_name
    if bak.exists():
        info(f"backup already preserved: {bak}")
        return
    shutil.copy2(path, bak)
    info(f"backup: {bak}")


def update_end_effector_type(path: Path) -> None:
    if not path.exists():
        fail(f"kuavo json not found: {path}")

    raw = path.read_text()
    data = json.loads(raw)
    current = data.get("EndEffectorType")
    if current == ["revo2", "revo2"]:
        ok(f"{path}: EndEffectorType already ['revo2', 'revo2']")
        return

    info(f"{path}: EndEffectorType {current!r} -> ['revo2', 'revo2']")
    if check_only:
        return

    backup(path)
    new_raw, count = re.subn(
        r'"EndEffectorType"\s*:\s*\[[^\]]*\]',
        '"EndEffectorType": ["revo2", "revo2"]',
        raw,
        count=1,
        flags=re.S,
    )
    if count != 1:
        fail(f"{path}: cannot locate EndEffectorType field")
    path.write_text(new_raw)
    ok(f"{path}: updated EndEffectorType")


def section_bounds(text: str, section: str):
    pattern = re.compile(rf"(?m)^{re.escape(section)}:\s*$")
    m = pattern.search(text)
    if not m:
        return None
    start = m.end()
    next_section = re.search(r"(?m)^[A-Za-z0-9_]+_devices:\s*$|^canbus_interfaces:\s*$", text[start:])
    end = start + next_section.start() if next_section else len(text)
    return start, end


def choose_section(text: str, preferred: str, fallback: str, side: str) -> str:
    if section_bounds(text, preferred):
        return preferred
    if section_bounds(text, fallback):
        warn(f"{preferred} not found; use {fallback} for {side} hand")
        return fallback
    fail(f"cannot find {preferred} or {fallback} in CAN config")


def ensure_device(text: str, section: str, name: str, device_id: str):
    if re.search(rf"(?m)^\s*-\s*name:\s*{re.escape(name)}\s*(?:#.*)?$", text):
        return text, False, f"{name} already exists"

    bounds = section_bounds(text, section)
    if not bounds:
        fail(f"section not found: {section}")
    _start, end = bounds
    snippet = (
        "  # ///////////////\n"
        f"  - name: {name}\n"
        "    class: revo2_hand\n"
        f"    device_id: {device_id}\n"
        "    ignore: false\n"
    )
    if end > 0 and text[end - 1] != "\n":
        snippet = "\n" + snippet
    return text[:end] + snippet + text[end:], True, f"insert {name} into {section}"


def update_can_devices(path: Path) -> None:
    if not path.exists():
        fail(f"CAN config not found: {path}")

    text = path.read_text()
    left_section = choose_section(text, "bcan2_devices", "canbus0_devices", "left")
    right_section = choose_section(text, "bcan3_devices", "canbus1_devices", "right")

    new_text, left_changed, left_msg = ensure_device(
        text, left_section, "Lhand_revo2_hand", "0x01"
    )
    new_text, right_changed, right_msg = ensure_device(
        new_text, right_section, "Rhand_revo2_hand", "0x02"
    )

    ok(left_msg) if not left_changed else info(left_msg)
    ok(right_msg) if not right_changed else info(right_msg)

    if not left_changed and not right_changed:
        ok(f"{path}: CAN hand devices already configured")
        return

    if check_only:
        warn(f"{path}: CAN config would be modified")
        return

    backup(path)
    path.write_text(new_text)
    ok(f"{path}: updated CAN hand devices")


info(f"mode: {'check only' if check_only else 'apply'}")
if update_kuavo:
    update_end_effector_type(kuavo_json)
else:
    info("skip kuavo.json update")

if update_can:
    update_can_devices(can_config)
else:
    info("skip CAN config update")

print("")
ok("done")
print("Next: restart leju-hardware/run_rl_controller so configs and binaries reload.")
PY
