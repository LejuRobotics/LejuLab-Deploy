#!/usr/bin/env bash
# launch_mujoco_sim_fall.sh — MuJoCo sim for fall testing
#
# 行为:
#   1) 启动后仍按 AMP 的 joint_default_pos 做初始插值（与原 launch_mujoco_sim 一致）
#   2) 遥控器按 START 后进入 HoldPose，保持当前关节角，不跑 AMP 策略
#   3) 仍可通过遥操（如 X）切换到 amp / 舞蹈等控制器

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_launch_common.sh"

parse_args "$@"
setup_env

ROBOT_VERSION="${ROBOT_VERSION:-46}"

MUJOCO_XML="$(find_package leju_assets)/models/biped_s${ROBOT_VERSION}/xml/scene_rl.xml"
RL_PKG_DIR="$(find_package leju-controllers/leju-rl-controller)"

if [[ -z "${CONTROLLER_MANAGER_CONFIG}" ]]; then
    CONTROLLER_MANAGER_CONFIG="${RL_PKG_DIR}/config/${ROBOT_VERSION}/controller_manager.yaml"
fi

if [[ ! -f "${CONTROLLER_MANAGER_CONFIG}" ]]; then
    echo "ERROR: controller_manager not found: ${CONTROLLER_MANAGER_CONFIG}" >&2
    exit 1
fi

# 生成临时 manager：default=hold（AMP 姿态插值 + START 后保持），保留原有控制器
CONTROLLER_MANAGER_CONFIG="$(python3 - "${CONTROLLER_MANAGER_CONFIG}" <<'PY'
import sys
import tempfile
from pathlib import Path

try:
    import yaml
except ImportError as e:
    sys.stderr.write("ERROR: need python3-yaml\n")
    raise SystemExit(1) from e

mgr_path = Path(sys.argv[1])
mgr_dir = mgr_path.parent
mgr = yaml.safe_load(mgr_path.read_text(encoding="utf-8")) or {}

def resolve(cfg: str) -> Path:
    p = Path(cfg)
    return p if p.is_absolute() else (mgr_dir / p).resolve()

# 找 amp 配置，抽出初始插值所需字段
amp_entry = next((c for c in (mgr.get("controllers") or []) if c.get("name") == "amp"), None)
if not amp_entry:
    sys.stderr.write("ERROR: controller 'amp' not found in controller_manager\n")
    raise SystemExit(1)

amp_cfg_path = resolve(str(amp_entry.get("config") or ""))
if not amp_cfg_path.is_file():
    sys.stderr.write(f"ERROR: amp config not found: {amp_cfg_path}\n")
    raise SystemExit(1)

amp_cfg = yaml.safe_load(amp_cfg_path.read_text(encoding="utf-8")) or {}
try:
    robot = amp_cfg["HumanoidRobotCfg"]["env"]["robot"]
    loop_dt = amp_cfg["HumanoidRobotCfg"].get("loop_dt", 0.001)
except (KeyError, TypeError) as e:
    sys.stderr.write(f"ERROR: invalid amp config structure: {amp_cfg_path}\n")
    raise SystemExit(1) from e

for key in ("joint_names", "joint_direction", "joint_default_pos", "actuator_kp", "actuator_kd"):
    if key not in robot:
        sys.stderr.write(f"ERROR: amp config missing robot.{key}\n")
        raise SystemExit(1)

hold_cfg = {
    "loop_dt": loop_dt,
    "kp": 100.0,
    "kd": 10.0,
    "robot": {
        "joint_names": robot["joint_names"],
        "joint_direction": robot["joint_direction"],
        "joint_default_pos": robot["joint_default_pos"],
        "actuator_kp": robot["actuator_kp"],
        "actuator_kd": robot["actuator_kd"],
    },
}

tmp_dir = Path(tempfile.mkdtemp(prefix="leju_fall_hold_"))
hold_cfg_path = tmp_dir / "config_hold_from_amp.yaml"
hold_cfg_path.write_text(
    yaml.dump(hold_cfg, allow_unicode=True, sort_keys=False, default_flow_style=False),
    encoding="utf-8",
)

mgr["default_controller"] = "hold"
controllers = [c for c in list(mgr.get("controllers") or []) if c.get("name") != "hold"]
for c in controllers:
    if isinstance(c, dict) and "config" in c:
        c["config"] = str(resolve(str(c["config"])))

hold_entry = {
    "name": "hold",
    "type": "HoldPoseController",
    "config": str(hold_cfg_path),
    "enabled": True,
}
mgr["controllers"] = [hold_entry] + controllers

out_mgr = tmp_dir / "controller_manager.yaml"
out_mgr.write_text(
    yaml.dump(mgr, allow_unicode=True, sort_keys=False, default_flow_style=False),
    encoding="utf-8",
)
print(
    "[launcher] fall: init interpolate=AMP default pose; START=HoldPose (no AMP policy)",
    file=sys.stderr,
)
print(str(out_mgr))
PY
)"

if [[ -z "${TELEOP_CONFIG}" ]]; then
    _default_teleop="${RL_PKG_DIR}/config/${ROBOT_VERSION}/teleop_bindings.yaml"
    if [[ -f "${_default_teleop}" ]]; then
        TELEOP_CONFIG="${_default_teleop}"
    fi
fi

rl_args=(-c "${CONTROLLER_MANAGER_CONFIG}")
[[ -n "${TELEOP_CONFIG}" ]]  && rl_args+=(-t "${TELEOP_CONFIG}")
[[ -n "${URDF_PATH}" ]]      && rl_args+=(-u "${URDF_PATH}")

launch_required_node leju-mujoco-sim leju-mujoco-sim \
    "${MUJOCO_XML}"

if [[ "${AUTO_START}" != "true" ]]; then
    launch_node leju-joystick leju-joystick
fi

launch_required_node leju-controllers/leju-rl-controller run_rl_controller \
    "${rl_args[@]}"
launch_node lejusdk/lejusdk-recorder/recorder lejusdk_recorder \
    "--config=$(find_package leju_launch)/config/recorder.yaml"

wait_and_cleanup
