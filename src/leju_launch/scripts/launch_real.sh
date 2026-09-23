#!/usr/bin/env bash
# launch_real.sh — Real hardware + RL controller (controller_manager mode)
# Shell equivalent of load_real.launch
#
# 启动后由 run_rl_controller 根据 IMU 姿态自动选择：
#   倒地 → mimic_fall_stand（START 待机 hold，LB+RB+X 两阶段起身）
#   直立 → amp

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_launch_common.sh"
source "${SCRIPT_DIR}/_teleop_config_path.sh"

parse_args "$@"
setup_env

# CAN 中断亲和: 分散到小核 1-3, 避免硬中断/irq线程/NET_RX 全挤在 CPU0
# (详见 set_can_irq_affinity.sh 注释; 无 CAN/无权限时自动跳过)
bash "${SCRIPT_DIR}/set_can_irq_affinity.sh"

# ── Default config paths (can be overridden by --config / --teleop-config / --urdf-path) ──
RL_PKG_DIR="$(find_package leju-controllers/leju-rl-controller)"

if [[ -z "${CONTROLLER_MANAGER_CONFIG}" ]]; then
    CONTROLLER_MANAGER_CONFIG="${RL_PKG_DIR}/config/${ROBOT_VERSION}/controller_manager.yaml"
fi

if [[ -z "${TELEOP_CONFIG}" ]]; then
    _default_teleop="${RL_PKG_DIR}/config/${ROBOT_VERSION}/teleop_bindings.yaml"
    select_real_teleop_config "${_default_teleop}"
fi

# ── Build run_rl_controller argument list ──────────────────────────────────
rl_args=(-c "${CONTROLLER_MANAGER_CONFIG}")
[[ -n "${TELEOP_CONFIG}" ]]  && rl_args+=(-t "${TELEOP_CONFIG}")
[[ -n "${URDF_PATH}" ]]      && rl_args+=(-u "${URDF_PATH}")
[[ "${PRE_START_FALL_RECOVERY}" == "true" ]] && rl_args+=(--pre-start-fall-recovery)

# ── Launch nodes ──────────────────────────────────────────────────────────
launch_required_node leju-hardware leju-hardware \
    "$(find_package leju-hardware)/"

# Mirror dev's `unless="$(arg auto_start)"`: skip joystick in auto-start mode
if [[ "${AUTO_START}" != "true" ]]; then
    launch_required_node leju-joystick leju-joystick
fi

launch_required_node leju-controllers/leju-rl-controller run_rl_controller \
    "${rl_args[@]}"
launch_node lejusdk/lejusdk-recorder/recorder lejusdk_recorder \
    "--config=$(find_package leju_launch)/config/recorder.yaml"
# lejusdk-csv-recorder 已移出默认启动（数据与 mcap 重复且 CSV 无大小上限），
# 需要时手动运行: lejusdk-csv-recorder --dir <目录>

# 麦克风采集 → DDS（可选节点：无 USB 麦克风时节点自行退出，不影响主控制链路）
launch_node leju-audio micphone_to_dds_node

# 音频播放（可选节点：无播放设备时节点自行退出，不影响主控制链路）
launch_node leju-audio audio_player_node

# 系统监控（可选节点：5Hz 发布 /monitor/system_info/* + RL 线程所在核心）
launch_node leju-monitor system_info_node

wait_and_cleanup
