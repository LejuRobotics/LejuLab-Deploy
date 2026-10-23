#!/usr/bin/env bash
# Shared installer mechanics; Pico and Xsens keep separate environments.
# Python 3.10-3.12: MuJoCo 3.11 needs >=3.10, CycloneDDS 0.10.2 fails on 3.14.

gmr_supported_python() {
    "$1" -c 'import sys; sys.exit(0 if (3, 10) <= sys.version_info[:2] < (3, 13) else 1)' >/dev/null 2>&1
}

gmr_stop_stale_xsens() {
    # Leftover nohup/manual frontends survive launcher Ctrl+C and keep TCP 9764 / UDP 9763.
    # Bracket the first letter so this pgrep does not match itself.
    local pid waited=0
    local pids=()
    mapfile -t pids < <(pgrep -f '[l]eju_xsens_gmr.main' || true)
    if [[ ${#pids[@]} -eq 0 ]]; then
        return 0
    fi
    echo "[gmr] Stopping leftover Xsens frontend (PID ${pids[*]}) so BVH TCP can bind"
    for pid in "${pids[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    while (( waited < 30 )); do
        pgrep -f '[l]eju_xsens_gmr.main' >/dev/null || break
        sleep 0.1
        waited=$((waited + 1))
    done
    mapfile -t pids < <(pgrep -f '[l]eju_xsens_gmr.main' || true)
    for pid in "${pids[@]}"; do
        kill -KILL "$pid" 2>/dev/null || true
    done
}

gmr_configure_mujoco_gl() {
    # bashrc on this robot exports MUJOCO_GL=osmesa. That backend imports
    # PyOpenGL and crashes: AttributeError: 'NoneType' object has no attribute 'glGetError'.
    # Headless GMR only needs MuJoCo IK. --viewer needs a real GL backend.
    local wants_viewer=false arg current
    for arg in "$@"; do
        [[ "$arg" == "--viewer" ]] && wants_viewer=true
    done
    current="$(printf '%s' "${MUJOCO_GL:-}" | tr '[:upper:]' '[:lower:]')"
    if [[ "$wants_viewer" == true ]]; then
        case "$current" in
            osmesa|disable|disabled|off|false|0|"")
                if [[ -n "${DISPLAY:-}" ]]; then
                    export MUJOCO_GL=glfw
                else
                    export MUJOCO_GL=egl
                fi
                ;;
        esac
    else
        case "$current" in
            osmesa|"") export MUJOCO_GL=disable ;;
        esac
    fi
}

gmr_ensure_uv() {
    if [[ -n "${GMR_UV_BIN:-}" && -x "$GMR_UV_BIN" ]]; then return; fi
    if command -v uv >/dev/null 2>&1; then
        GMR_UV_BIN="$(command -v uv)"
        return
    fi
    local bootstrap_dir="${XDG_CACHE_HOME:-$HOME/.cache}/leju_gmr/bootstrap"
    GMR_UV_BIN="$bootstrap_dir/bin/uv"
    if [[ ! -x "$GMR_UV_BIN" ]]; then
        echo "Installing environment bootstrap tool under $bootstrap_dir"
        python3 -m pip install --disable-pip-version-check --target "$bootstrap_dir" 'uv>=0.8,<1'
    fi
}

gmr_select_python() {
    local requested="$1" venv="$2" candidate
    if [[ -n "$requested" ]]; then
        if ! gmr_supported_python "$requested"; then
            echo "Unsupported GMR interpreter: $requested; use Python 3.10-3.12 (recommended: 3.10)." >&2
            return 1
        fi
        GMR_PYTHON_BIN="$requested"
        return
    fi
    if [[ -x "$venv/bin/python3" ]] && gmr_supported_python "$venv/bin/python3"; then
        GMR_PYTHON_BIN="$("$venv/bin/python3" -c 'import sys; print(sys._base_executable)')"
        return
    fi
    for candidate in python3.10 python3.11 python3.12 python3; do
        if command -v "$candidate" >/dev/null 2>&1 && gmr_supported_python "$candidate"; then
            GMR_PYTHON_BIN="$(command -v "$candidate")"
            return
        fi
    done
    echo "No compatible system Python; installing managed Python 3.10 (system/ROS Python stays separate)."
    gmr_ensure_uv
    "$GMR_UV_BIN" python install 3.10
    GMR_PYTHON_BIN="$("$GMR_UV_BIN" python find --managed-python 3.10)"
    gmr_supported_python "$GMR_PYTHON_BIN"
}

gmr_setup_environment() {
    local prefix="$1" package="$2" label="$3"
    local python_var="${prefix}_GMR_PYTHON" venv_var="${prefix}_GMR_VENV"
    local repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
    local venv="${!venv_var:-$HOME/.cache/$package/venv}"
    local requested="${!python_var:-}"
    local common_dir="$repo_root/src/leju_gmr_common" package_dir="$repo_root/src/$package"
    local dds_home="$repo_root/src/lejusdk/3rd_party/$(uname -m)/cyclonedds-0.10.2"
    # ROS setup/PYTHONPATH from the host must not inject a different Python ABI.
    unset PYTHONPATH PYTHONHOME
    export PYTHONNOUSERSITE=1
    if [[ ! -f "$common_dir/pyproject.toml" || ! -f "$package_dir/pyproject.toml" || ! -f "$dds_home/lib/libddsc.so" ]]; then
        echo "Missing local GMR package or bundled CycloneDDS under $repo_root" >&2
        return 1
    fi
    gmr_select_python "$requested" "$venv"
    local selected_version="$("$GMR_PYTHON_BIN" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
    echo "Using Python $selected_version for $label: $GMR_PYTHON_BIN"
    local reuse=false
    if [[ -x "$venv/bin/python3" ]] && gmr_supported_python "$venv/bin/python3"; then
        local current_version="$("$venv/bin/python3" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
        if [[ "$current_version" == "$selected_version" ]] && "$venv/bin/python3" -m pip --version >/dev/null 2>&1; then
            reuse=true
        fi
    fi
    if [[ "$reuse" != true ]]; then
        if [[ -e "$venv" ]]; then
            if [[ ! -f "$venv/pyvenv.cfg" ]]; then
                echo "Refusing to replace a non-venv directory: $venv" >&2
                return 1
            fi
            local backup="${venv}.backup-$(date +%Y%m%d-%H%M%S)-$$"
            mv "$venv" "$backup"
            echo "Previous/incomplete environment saved to $backup"
        fi
        echo "Creating $label environment: $venv"
        if "$GMR_PYTHON_BIN" -c 'import ensurepip' >/dev/null 2>&1; then
            "$GMR_PYTHON_BIN" -m venv "$venv"
        else
            # Ubuntu images may omit ensurepip. uv seeds pip without apt changes.
            gmr_ensure_uv
            UV_DEFAULT_INDEX="${UV_DEFAULT_INDEX:-${PIP_INDEX_URL:-https://pypi.org/simple}}" \
                "$GMR_UV_BIN" venv --seed --python "$GMR_PYTHON_BIN" "$venv"
        fi
    fi
    export CYCLONEDDS_HOME="$dds_home" CYCLONEDDS_LIB_DIR="$dds_home/lib"
    gmr_configure_mujoco_gl
    "$venv/bin/python3" -m pip install --upgrade pip setuptools wheel
    "$venv/bin/python3" -m pip uninstall --yes cyclonedds >/dev/null 2>&1 || true
    "$venv/bin/python3" -m pip install --no-binary=cyclonedds --editable "$common_dir" --editable "$package_dir"
    # Compilation alone is insufficient: catch missing C API symbols and model
    # incompatibility before reporting success. This does not bind device ports.
    "$venv/bin/python3" - "$package" <<'PY'
import importlib
import importlib.metadata
import sys
package = sys.argv[1]
import cyclonedds
import mujoco
module = importlib.import_module(package + '.main')
params = importlib.import_module(package + '.params')
solver = importlib.import_module(package + '.motion_retarget').MotionRetargeter
retarget = solver(next(iter(params.IK_CONFIG_DICT)), 'roban_s17')
print('Verified: Python %s, MuJoCo %s, Mink %s, CycloneDDS %s, S17 nq=%d' % (
    sys.version.split()[0], mujoco.__version__, importlib.metadata.version('mink'),
    importlib.metadata.version('cyclonedds'), retarget.model.nq))
PY
    echo "$label environment is ready: $venv"
    echo "Test with: $repo_root/src/leju_launch/scripts/run_${prefix,,}_gmr.sh --help"
}
