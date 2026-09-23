#!/usr/bin/env bash

# Resolve the real-hardware teleop config without changing an explicit CLI
# override. Runtime lejuconfig is the stable field configuration; the repository
# file remains a first-install fallback.
select_real_teleop_config() {
    local repository_config="$1"
    local runtime_config="${TELEOP_RUNTIME_CONFIG:-${HOME}/.config/lejuconfig/teleop_bindings.yaml}"

    if [[ -n "${TELEOP_CONFIG:-}" ]]; then
        return 0
    fi
    if [[ -f "$runtime_config" ]]; then
        TELEOP_CONFIG="$runtime_config"
    elif [[ -f "$repository_config" ]]; then
        TELEOP_CONFIG="$repository_config"
        echo "[launcher] WARN: runtime teleop config missing; using repository fallback: $repository_config" >&2
    fi
}
