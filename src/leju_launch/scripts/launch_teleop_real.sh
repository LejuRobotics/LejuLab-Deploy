#!/usr/bin/env bash
# launch_teleop_real.sh — Real hardware + Pico VR teleoperation
# Shell equivalent of real.launch (pico_teleop.launch with simulation:=false)
#
# 启动后由 run_rl_controller 根据 IMU 姿态自动选择：
#   倒地 → mimic_fall_stand（START 待机 hold，LB+RB+X 两阶段起身）
#   直立 → amp（与原默认行为一致）
# Pico 校准 (RT+B / LT+B) 后 RG+A 切到 heft_pico 实时跟踪，RG+B 返回 amp。

# Pico teleop 仅支持 Roban S17（config/17 下有 heft_pico + pico_bindings）
# 必须在 source _launch_common.sh 之前设置，避免其从 bashrc 解析出其他版本
export ROBOT_VERSION="${ROBOT_VERSION:-17}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_launch_common.sh"
source "${SCRIPT_DIR}/_teleop_config_path.sh"
source "${SCRIPT_DIR}/_motion_teleop_common.sh"

# 先消费 pico 专属参数，其余交给公共 parse_args
parse_motion_args "$@"
parse_args "${MOTION_REMAINING[@]}"
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

# ── Launch nodes ──────────────────────────────────────────────────────────
launch_required_node leju-hardware leju-hardware \
    "$(find_package leju-hardware)/"

# Mirror dev's `unless="$(arg auto_start)"`: skip joystick in auto-start mode
if [[ "${AUTO_START}" != "true" ]]; then
    launch_required_node leju-joystick leju-joystick
fi

launch_required_node leju-controllers/leju-rl-controller run_rl_controller \
    "${rl_args[@]}"

# Pico VR 前端：UDP :12345 → GMR 重定向 → DDS /rt/pico/joysticks + /rt/pico/heft_reference
# 放在 recorder 等可选节点之前，避免可选二进制缺失时打断 Pico 启动
launch_motion_gmr

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
