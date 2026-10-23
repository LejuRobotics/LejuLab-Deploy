#!/usr/bin/env bash
# Neutral body-source dispatcher; platform lifecycle stays in _launch_common.sh.
source "${SCRIPT_DIR}/_pico_teleop_common.sh"
MOTION_SOURCE="${MOTION_SOURCE:-pico}"
XSENS_HOST="${XSENS_HOST:-0.0.0.0}"
XSENS_PORT="${XSENS_PORT:-9763}"
XSENS_BVH_PORT="${XSENS_BVH_PORT:-9764}"
XSENS_CHARACTER_ID="${XSENS_CHARACTER_ID:-0}"
XSENS_FPS="${XSENS_FPS:-100}"
XSENS_DOMAIN_ID="${XSENS_DOMAIN_ID:-0}"
XSENS_GMR_VENV="${XSENS_GMR_VENV:-${HOME}/.cache/leju_xsens_gmr/venv}"
XSENS_CONTROL_TOPIC="${XSENS_CONTROL_TOPIC:-/rt/pico/joysticks}"
XSENS_ENABLE_VIEWER=false
XSENS_DRIFT_COMP=false
ENABLE_PICO_JOY=false

parse_motion_args() {
    local remaining=() option value variable
    local effective_robot_version="${ROBOT_VERSION:-17}"
    while [[ $# -gt 0 ]]; do
        option="${1%%=*}"
        variable=""
        case "$option" in
            --robot-version)
                if [[ "$1" == *=* ]]; then
                    effective_robot_version="${1#*=}"
                elif [[ $# -ge 2 ]]; then
                    effective_robot_version="$2"
                fi
                remaining+=("$1") ;;
            --motion-source) variable=MOTION_SOURCE ;;
            --xsens-host) variable=XSENS_HOST ;;
            --xsens-port) variable=XSENS_PORT ;;
            --xsens-bvh-port) variable=XSENS_BVH_PORT ;;
            --xsens-character-id) variable=XSENS_CHARACTER_ID ;;
            --xsens-fps) variable=XSENS_FPS ;;
            --xsens-domain|--xsens-domain-id) variable=XSENS_DOMAIN_ID ;;
            --xsens-venv) variable=XSENS_GMR_VENV ;;
            --xsens-control-topic) variable=XSENS_CONTROL_TOPIC ;;
            --with-pico-joy) ENABLE_PICO_JOY=true ;;
            --no-pico-joy) ENABLE_PICO_JOY=false ;;
            --viewer) XSENS_ENABLE_VIEWER=true; remaining+=("$1") ;;
            --xsens-viewer) XSENS_ENABLE_VIEWER=true ;;
            --xsens-drift-comp) XSENS_DRIFT_COMP=true ;;
            *) remaining+=("$1") ;;
        esac
        if [[ -n "$variable" ]]; then
            if [[ "$1" == *=* ]]; then
                value="${1#*=}"
            else
                if [[ $# -lt 2 || "$2" == --* ]]; then
                    echo "Missing value for $option" >&2; return 1
                fi
                shift; value="$1"
            fi
            printf -v "$variable" '%s' "$value"
        fi
        shift
    done
    case "$MOTION_SOURCE" in
        pico|xsens) ;;
        *) echo "--motion-source must be pico or xsens" >&2; return 1 ;;
    esac
    if [[ "$MOTION_SOURCE" == xsens && "$effective_robot_version" != 17 ]]; then
        echo "Xsens HEFT frontend requires ROBOT_VERSION=17" >&2; return 1
    fi
    parse_pico_args "${remaining[@]}"
    MOTION_REMAINING=("${PICO_REMAINING[@]}")
}

launch_motion_gmr() {
    if [[ "$MOTION_SOURCE" == pico ]]; then
        launch_pico_gmr
        return
    fi
    if [[ "${ROBOT_VERSION:-17}" != 17 ]]; then
        echo "Xsens HEFT frontend requires ROBOT_VERSION=17" >&2; return 1
    fi
    local args=(--host "$XSENS_HOST" --port "$XSENS_PORT"
        --bvh-port "$XSENS_BVH_PORT" --character-id "$XSENS_CHARACTER_ID"
        --fps "$XSENS_FPS" --domain-id "$XSENS_DOMAIN_ID"
        --control-topic "$XSENS_CONTROL_TOPIC")
    [[ "$XSENS_ENABLE_VIEWER" == true ]] && args+=(--viewer)
    [[ "$XSENS_DRIFT_COMP" == true ]] && args+=(--drift-comp)
    local prefix=()
    [[ "${COREDUMP:-false}" == true ]] && prefix+=(--with-coredump)
    XSENS_GMR_VENV="$XSENS_GMR_VENV" XSENS_REPLACE_STALE=1 \
        "${SCRIPT_DIR}/start_node.sh" "${prefix[@]}" \
        "${SCRIPT_DIR}/run_xsens_gmr.sh" "${args[@]}" &
    ALL_PIDS+=("$!")
    if [[ -n "${REQUIRED_PIDS+x}" ]]; then
        REQUIRED_PIDS+=("$!")
    fi
    echo "[launcher] Started xsens_gmr (PID $!, required=true)"
    if [[ "$ENABLE_PICO_JOY" == true && -n "$XSENS_CONTROL_TOPIC" ]]; then
        # Join the Xsens DDS domain and publish the configured generic control topic.
        local joy_args=(--host "$PICO_HOST" --port "$PICO_PORT"
            --domain-id "$XSENS_DOMAIN_ID" --joy-topic "$XSENS_CONTROL_TOPIC"
            --broadcast-name "$PICO_BROADCAST_NAME")
        [[ "$PICO_ENABLE_BROADCAST" != true ]] && joy_args+=(--no-broadcast)
        PICO_GMR_VENV="$PICO_GMR_VENV" \
            "${SCRIPT_DIR}/start_node.sh" "${prefix[@]}" \
            "${SCRIPT_DIR}/run_pico_joy.sh" "${joy_args[@]}" &
        ALL_PIDS+=("$!")
        echo "[launcher] Started pico_joy (PID $!)"
    fi
}
