#!/bin/bash
# Launch the standalone Xsens→GMR→DDS HEFT reference frontend (leju_xsens_gmr).
#
# Usage:
#   bash src/leju_launch/scripts/run_xsens_gmr.sh [extra leju-xsens-gmr args...]
#
# Environment:
#   XSENS_GMR_VENV        venv path (default: ~/.cache/leju_xsens_gmr/venv)
#   CYCLONEDDS_URI       optional CycloneDDS config (network interface etc.)
#   CYCLONEDDS_LIB_DIR   optional libddsc.so dir to preload (ARM deployments)
set -e
# Keep ROS/host Python modules out of the dedicated environment.
unset PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PKG_DIR="$REPO_ROOT/src/leju_xsens_gmr"
VENV="${XSENS_GMR_VENV:-${HOME}/.cache/leju_xsens_gmr/venv}"
VENV_PYTHON="$VENV/bin/python3"

if [ ! -x "$VENV_PYTHON" ]; then
    echo "usable Xsens/GMR venv not found at $VENV"
    echo "Create it first:"
    echo "  $REPO_ROOT/src/leju_launch/scripts/setup_xsens_gmr_env.sh"
    exit 1
fi

source "$SCRIPT_DIR/_gmr_python_env.sh"
if ! gmr_supported_python "$VENV_PYTHON"; then
    echo "This GMR environment needs Python 3.10-3.12. Re-run its setup script."
    exit 1
fi
gmr_configure_mujoco_gl "$@"
# Default on: launcher restarts must not lose 9764 to a leftover process.
# Tests that invoke this script with a fake interpreter set XSENS_REPLACE_STALE=0.
if [[ "${XSENS_REPLACE_STALE:-1}" != 0 ]]; then
    gmr_stop_stale_xsens
fi

# Default: use the repo's bundled cyclonedds runtime lib if present
ARCH=$(uname -m)
BUNDLED_DDS_HOME="$REPO_ROOT/src/lejusdk/3rd_party/$ARCH/cyclonedds-0.10.2"
if [ -d "$BUNDLED_DDS_HOME" ]; then
    export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-$BUNDLED_DDS_HOME}"
    export CYCLONEDDS_LIB_DIR="${CYCLONEDDS_LIB_DIR:-$BUNDLED_DDS_HOME/lib}"
fi

# roslaunch appends its private remapping arguments to every node, including
# non-ROS executables.  Do not pass those through to argparse.
APP_ARGS=()
for arg in "$@"; do
    case "$arg" in
        __name:=*|__log:=*|__ns:=*) ;;
        *) APP_ARGS+=("$arg") ;;
    esac
done

exec "$VENV_PYTHON" -m leju_xsens_gmr.main "${APP_ARGS[@]}"
