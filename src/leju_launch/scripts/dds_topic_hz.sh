#!/usr/bin/env bash
# dds_topic_hz.sh — Measure DDS topic publish rate (rostopic hz equivalent)
#
# Usage:
#   bash src/leju_launch/scripts/dds_topic_hz.sh
#   bash src/leju_launch/scripts/dds_topic_hz.sh rt/depth_camera/frame_mm_240x424
#   bash src/leju_launch/scripts/dds_topic_hz.sh rt/depth_camera/frame_meters_36x64 --expected-len 2304
#
# Requires: pip3 install cyclonedds==0.10.2
# Run while launch_real.sh / depth publisher is active.

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
exec python3 "${SCRIPT_DIR}/dds_topic_hz.py" "$@"
