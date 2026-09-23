#!/usr/bin/env bash
# dds_depth_policy_input_video.sh — Record depth_walk policy 64x36 input as video
#
# Subscribes rt/depth_camera/frame_mm_240x424 and runs the same processing as
# DepthImageProcessor in depth_walk (resize, inpaint, Gaussian blur, /2.5).
#
# Usage:
#   bash src/leju_launch/scripts/dds_depth_policy_input_video.sh
#   bash src/leju_launch/scripts/dds_depth_policy_input_video.sh -o /tmp/policy_depth.mp4 --duration 10 --no-show
#   bash src/leju_launch/scripts/dds_depth_policy_input_video.sh --side-by-side -o compare.mp4
#
# Requires: pip3 install cyclonedds==0.10.2 opencv-python numpy

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
exec python3 "${SCRIPT_DIR}/dds_depth_policy_input_video.py" "$@"
