#!/usr/bin/env bash
# Run inside the host/container where the frontend will run.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_gmr_python_env.sh"
gmr_setup_environment XSENS leju_xsens_gmr 'Xsens/GMR'
