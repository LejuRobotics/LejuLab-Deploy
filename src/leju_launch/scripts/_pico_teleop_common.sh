#!/usr/bin/env bash
# _pico_teleop_common.sh — shared Pico VR teleop helpers for launch_teleop_{real,sim}.sh
# Sourced, not executed directly.

# ── Pico teleop defaults ─────────────────────────────────────────────
PICO_HOST="${PICO_HOST:-0.0.0.0}"                    # UDP bind (headset streams here)
PICO_PORT="${PICO_PORT:-12345}"
PICO_FPS="${PICO_FPS:-100}"
PICO_DOMAIN_ID="${PICO_DOMAIN_ID:-0}"
PICO_BROADCAST_NAME="${PICO_BROADCAST_NAME:-ROBAN}"
PICO_ENABLE_BROADCAST=true
PICO_ENABLE_VIEWER=false
PICO_GMR_VENV="${PICO_GMR_VENV:-${HOME}/.cache/leju_pico_gmr/venv}"

# ── Pico argument parsing ────────────────────────────────────────────
# Consume pico-specific args; everything else lands in PICO_REMAINING
# so the caller can hand it to _launch_common.sh's parse_args().
parse_pico_args() {
    PICO_REMAINING=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --pico-host=*)            PICO_HOST="${1#*=}" ;;
            --pico-host)              shift; PICO_HOST="$1" ;;
            --pico-port=*)            PICO_PORT="${1#*=}" ;;
            --pico-port)              shift; PICO_PORT="$1" ;;
            --pico-fps=*)             PICO_FPS="${1#*=}" ;;
            --pico-fps)               shift; PICO_FPS="$1" ;;
            --pico-domain=*)          PICO_DOMAIN_ID="${1#*=}" ;;
            --pico-domain)            shift; PICO_DOMAIN_ID="$1" ;;
            --pico-broadcast-name=*)  PICO_BROADCAST_NAME="${1#*=}" ;;
            --pico-broadcast-name)    shift; PICO_BROADCAST_NAME="$1" ;;
            --no-broadcast)           PICO_ENABLE_BROADCAST=false ;;
            --viewer)                 PICO_ENABLE_VIEWER=true ;;
            --pico-venv=*)            PICO_GMR_VENV="${1#*=}" ;;
            --pico-venv)              shift; PICO_GMR_VENV="$1" ;;
            *)                        PICO_REMAINING+=("$1") ;;
        esac
        shift
    done
}

# ── Pico/GMR frontend node ───────────────────────────────────────────
# Mirrors the `pico_gmr` node in pico_teleop.launch (run_pico_gmr.sh +
# start_node.sh). Non-required: if it dies the robot keeps the current
# controller; teleop just stops producing a live reference.
# HEFT CSV recording is done in C++ heft_pico (LT+Y/X), not here.
launch_pico_gmr() {
    local pico_gmr_args=(
        --robot roban_s17
        --host "${PICO_HOST}"
        --port "${PICO_PORT}"
        --fps "${PICO_FPS}"
        --domain-id "${PICO_DOMAIN_ID}"
        --broadcast-name "${PICO_BROADCAST_NAME}"
    )
    [[ "${PICO_ENABLE_BROADCAST}" != "true" ]] && pico_gmr_args+=(--no-broadcast)
    [[ "${PICO_ENABLE_VIEWER}" == "true" ]] && pico_gmr_args+=(--viewer)

    local coredump_arg=""
    [[ "${COREDUMP}" == "true" ]] && coredump_arg="--with-coredump"

    PICO_GMR_VENV="${PICO_GMR_VENV}" \
        "${SCRIPT_DIR}/start_node.sh" ${coredump_arg} \
        "${SCRIPT_DIR}/run_pico_gmr.sh" "${pico_gmr_args[@]}" &
    local pid=$!
    ALL_PIDS+=("${pid}")
    echo "[launcher] Started pico_gmr (PID ${pid})"
}
