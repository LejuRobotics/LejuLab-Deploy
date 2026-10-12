#!/usr/bin/env bash
# Xsens body frontend; --with-pico-joy optionally starts a separate joy process.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${SCRIPT_DIR}/launch_teleop_real.sh" "$@" --motion-source xsens
