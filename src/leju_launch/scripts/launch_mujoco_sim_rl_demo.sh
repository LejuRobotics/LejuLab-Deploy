#!/usr/bin/env bash
# launch_mujoco_sim_rl_demo.sh — MuJoCo simulation + RL demo controller
# Shell equivalent of demo/load_mujoco_sim_rl_demo.launch

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_launch_common.sh"

parse_args "$@"
setup_env

ROBOT_VERSION="${ROBOT_VERSION:-46}"

MUJOCO_XML="$(find_package leju_assets)/models/biped_s${ROBOT_VERSION}/xml/scene_rl.xml"

launch_required_node leju-mujoco-sim leju-mujoco-sim \
    "${MUJOCO_XML}"
launch_required_node leju-joystick leju-joystick
launch_required_node leju-controllers/leju-rl-controller run_rl_demo_controller \
    "$(find_package leju-controllers/leju-rl-controller)/config/${ROBOT_VERSION}/config_amp.yaml"
launch_node lejusdk/lejusdk-recorder/recorder lejusdk_recorder \
    "--config=$(find_package leju_launch)/config/recorder.yaml"

wait_and_cleanup
