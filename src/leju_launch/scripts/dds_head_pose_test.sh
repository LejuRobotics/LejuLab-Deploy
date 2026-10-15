#!/usr/bin/env bash
# dds_head_pose_test.sh — 头部指令精度测试（期望 vs 实测）
#
# 用法：
#   bash src/leju_launch/scripts/dds_head_pose_test.sh
#   bash src/leju_launch/scripts/dds_head_pose_test.sh --cases 'yaw:+60,pitch:-18'
#   bash src/leju_launch/scripts/dds_head_pose_test.sh --tolerance-deg 3 --json /tmp/head.json
#
# Requires: pip3 install cyclonedds==0.10.2
# 必须先启动 launch_real.sh（或等价 launch），控制器处于 Running 且已在走路/站立。
#
# S17 /rt/head_trajectory 约定：
#   q[0] = head_yaw (rad), q[1] = head_pitch (rad, 正值 = 低头)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${LEJULAB_PROJECT_ROOT:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
ARCH="$(uname -m)"
CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-${PROJECT_DIR}/src/lejusdk/3rd_party/${ARCH}/cyclonedds-0.10.2}"

export CYCLONEDDS_LIB_DIR="${CYCLONEDDS_HOME}/lib"
export LD_LIBRARY_PATH="${CYCLONEDDS_LIB_DIR}:${LD_LIBRARY_PATH:-}"

if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    if [[ -f "/etc/cyclonedds/cyclonedds_shm.xml" ]]; then
        export CYCLONEDDS_URI="file:///etc/cyclonedds/cyclonedds_shm.xml"
    else
        export CYCLONEDDS_URI="file://${PROJECT_DIR}/src/leju_launch/config/cyclonedds_shm.xml"
    fi
fi

if [[ ! -f "${CYCLONEDDS_LIB_DIR}/libddsc.so" ]]; then
    echo "error: CycloneDDS library not found at ${CYCLONEDDS_LIB_DIR}/libddsc.so" >&2
    exit 1
fi

export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"
exec python3 "${SCRIPT_DIR}/dds_head_pose_test.py" "$@"
