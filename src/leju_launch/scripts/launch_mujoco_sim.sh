#!/usr/bin/env bash
# launch_mujoco_sim.sh — MuJoCo simulation + RL controller (controller_manager mode)
# Shell equivalent of load_mujoco_sim.launch
#
# 启动后由 run_rl_controller 根据 IMU 姿态自动选择：
#   倒地 → mimic_fall_stand（START 待机 hold，LB+RB+X 两阶段起身）
#   直立 → amp

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_launch_common.sh"

parse_args "$@"
setup_env

ROBOT_VERSION="${ROBOT_VERSION:-46}"

MUJOCO_XML="$(find_package leju_assets)/models/biped_s${ROBOT_VERSION}/xml/scene_rl.xml"

# ── Default config paths (can be overridden by --config / --teleop-config / --urdf-path) ──
RL_PKG_DIR="$(find_package leju-controllers/leju-rl-controller)"

if [[ -z "${CONTROLLER_MANAGER_CONFIG}" ]]; then
    CONTROLLER_MANAGER_CONFIG="${RL_PKG_DIR}/config/${ROBOT_VERSION}/controller_manager.yaml"
fi

if [[ -z "${TELEOP_CONFIG}" ]]; then
    _default_teleop="${RL_PKG_DIR}/config/${ROBOT_VERSION}/teleop_bindings.yaml"
    if [[ -f "${_default_teleop}" ]]; then
        TELEOP_CONFIG="${_default_teleop}"
    fi
fi

# ── Build run_rl_controller argument list ──────────────────────────────────
rl_args=(-c "${CONTROLLER_MANAGER_CONFIG}")
[[ -n "${TELEOP_CONFIG}" ]]  && rl_args+=(-t "${TELEOP_CONFIG}")
[[ -n "${URDF_PATH}" ]]      && rl_args+=(-u "${URDF_PATH}")
[[ "${PRE_START_FALL_RECOVERY}" == "true" ]] && rl_args+=(--pre-start-fall-recovery)

# ── Launch nodes ──────────────────────────────────────────────────────────
if [[ "${PRE_START_FALL_RECOVERY}" == "true" && "${ROBOT_VERSION}" == "17" && -z "${SIM_KEYFRAME}" ]]; then
    SIM_KEYFRAME="lie0"
fi

sim_args=("${MUJOCO_XML}")
[[ -n "${SIM_KEYFRAME}" ]] && sim_args+=("--initial-keyframe=${SIM_KEYFRAME}")
launch_required_node leju-mujoco-sim leju-mujoco-sim \
    "${sim_args[@]}"

# Mirror dev's `unless="$(arg auto_start)"`: skip joystick in auto-start mode
if [[ "${AUTO_START}" != "true" ]]; then
    launch_node leju-joystick leju-joystick
fi

launch_required_node leju-controllers/leju-rl-controller run_rl_controller \
    "${rl_args[@]}"
launch_node lejusdk/lejusdk-recorder/recorder lejusdk_recorder \
    "--config=$(find_package leju_launch)/config/recorder.yaml"

wait_and_cleanup
