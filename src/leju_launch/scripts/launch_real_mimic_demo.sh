#!/usr/bin/env bash
# launch_real_mimic_demo.sh — Real hardware + mimic demo controller
# Shell equivalent of demo/load_real_mimic_demo.launch

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_launch_common.sh"

parse_args "$@"
setup_env

export ROBOT_VERSION="${ROBOT_VERSION:-14}"

launch_required_node leju-hardware leju-hardware \
    "$(find_package leju-hardware)/"
launch_required_node leju-joystick leju-joystick
launch_required_node leju-controllers/leju-rl-controller run_rl_mimic_controller \
    "$(find_package leju-controllers/leju-rl-controller)/config/${ROBOT_VERSION}/config_mimic_HPNY_dance.yaml"
launch_node lejusdk/lejusdk-recorder/recorder lejusdk_recorder \
    "--config=$(find_package leju_launch)/config/recorder.yaml"

wait_and_cleanup
