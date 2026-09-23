#!/bin/bash
#
# cleanup.sh - Auto cleanup old stdout logs, MCAP recordings and coredumps
#
# Usage: Called as a roslaunch node at startup
#   <node pkg="leju_launch" type="cleanup.sh" name="cleanup" output="screen"
#         args="$(find leju_launch)/config/cleanup.yaml" />
#

set -euo pipefail

# Fixed paths
STDOUT_DIR="$HOME/.ros/lejulab/stdout"
MCAP_DIR="$HOME/.ros/lejulab/mcap"
COREDUMP_DIR="$HOME/.ros/lejulab/coredumps"

# Defaults
STDOUT_RETENTION_DAYS=30
MCAP_RETENTION_DAYS=10
COREDUMP_RETENTION_DAYS=10
MAX_TOTAL_SIZE_GB=30

##############################################################################
# Parse YAML config (lightweight, no python dependency)
##############################################################################
parse_yaml_value() {
    local file="$1" key="$2" default="$3"
    local val
    val=$(grep -E "^\s+${key}:" "$file" 2>/dev/null | head -1 | sed 's/.*:\s*//' | sed 's/#.*//' | tr -d ' ')
    echo "${val:-$default}"
}

load_config() {
    local config_file="$1"
    if [ ! -f "$config_file" ]; then
        echo "[cleanup] Config file not found: $config_file, using defaults"
        return
    fi

    MAX_TOTAL_SIZE_GB=$(parse_yaml_value "$config_file" "max_total_size" "$MAX_TOTAL_SIZE_GB")
    STDOUT_RETENTION_DAYS=$(parse_yaml_value "$config_file" "retention_days" "$STDOUT_RETENTION_DAYS")

    # Parse per-section retention_days (take them in order: stdout, mcap, coredumps)
    local retention_values
    retention_values=$(grep -E "^\s+retention_days:" "$config_file" 2>/dev/null | sed 's/.*:\s*//' | sed 's/#.*//' | tr -d ' ')
    local i=0
    while IFS= read -r val; do
        case $i in
            0) STDOUT_RETENTION_DAYS="${val:-$STDOUT_RETENTION_DAYS}" ;;
            1) MCAP_RETENTION_DAYS="${val:-$MCAP_RETENTION_DAYS}" ;;
            2) COREDUMP_RETENTION_DAYS="${val:-$COREDUMP_RETENTION_DAYS}" ;;
        esac
        i=$(( i + 1 ))
    done <<< "$retention_values"

    echo "[cleanup] Config: max_total_size=${MAX_TOTAL_SIZE_GB}GB, stdout=${STDOUT_RETENTION_DAYS}d, mcap=${MCAP_RETENTION_DAYS}d, coredumps=${COREDUMP_RETENTION_DAYS}d"
}

##############################################################################
# Cleanup by retention days
##############################################################################
cleanup_by_days() {
    local dir="$1" days="$2" label="$3"
    local count=0

    if [ ! -d "$dir" ]; then
        return
    fi

    # Find and delete entries older than N days
    while IFS= read -r entry; do
        rm -rf "$entry"
        count=$(( count + 1 ))
    done < <(find "$dir" -mindepth 1 -maxdepth 1 -mtime +"$days" | sort)

    if [ "$count" -gt 0 ]; then
        echo "[cleanup] Removed $count expired ${label} entries (>${days} days)"
    fi
}

##############################################################################
# Get directory size in bytes
##############################################################################
dir_size_bytes() {
    local dir="$1"
    if [ -d "$dir" ]; then
        du -sb "$dir" 2>/dev/null | cut -f1
    else
        echo 0
    fi
}

##############################################################################
# Cleanup by total size limit
##############################################################################
cleanup_by_size() {
    local max_bytes=$(( MAX_TOTAL_SIZE_GB * 1024 * 1024 * 1024 ))

    # Calculate current total size
    local stdout_size mcap_size coredump_size total_size
    stdout_size=$(dir_size_bytes "$STDOUT_DIR")
    mcap_size=$(dir_size_bytes "$MCAP_DIR")
    coredump_size=$(dir_size_bytes "$COREDUMP_DIR")
    total_size=$(( stdout_size + mcap_size + coredump_size ))

    if [ "$total_size" -le "$max_bytes" ]; then
        local total_mb=$(( total_size / 1024 / 1024 ))
        local max_mb=$(( max_bytes / 1024 / 1024 ))
        echo "[cleanup] Total size: ${total_mb}MB / ${max_mb}MB, no size cleanup needed"
        return
    fi

    echo "[cleanup] Total size $(( total_size / 1024 / 1024 ))MB exceeds limit $(( max_bytes / 1024 / 1024 ))MB, cleaning oldest entries..."

    # Collect all entries with modification time, sorted oldest first
    local tmpfile
    tmpfile=$(mktemp)

    for dir in "$STDOUT_DIR" "$MCAP_DIR" "$COREDUMP_DIR"; do
        if [ -d "$dir" ]; then
            find "$dir" -mindepth 1 -maxdepth 1 -printf '%T@\t%s\t%p\n' 2>/dev/null >> "$tmpfile"
        fi
    done

    # For directories, we need actual size (du), not the directory entry size
    local sorted_tmpfile
    sorted_tmpfile=$(mktemp)
    while IFS=$'\t' read -r mtime _size path; do
        local actual_size
        if [ -d "$path" ]; then
            actual_size=$(du -sb "$path" 2>/dev/null | cut -f1)
        else
            actual_size=$(stat -c%s "$path" 2>/dev/null || echo 0)
        fi
        echo -e "${mtime}\t${actual_size}\t${path}"
    done < "$tmpfile" | sort -n > "$sorted_tmpfile"

    # Delete oldest entries until under limit
    local removed_count=0
    local freed_bytes=0
    while IFS=$'\t' read -r _mtime entry_size path; do
        if [ "$total_size" -le "$max_bytes" ]; then
            break
        fi
        rm -rf "$path"
        total_size=$(( total_size - entry_size ))
        freed_bytes=$(( freed_bytes + entry_size ))
        removed_count=$(( removed_count + 1 ))
    done < "$sorted_tmpfile"

    rm -f "$tmpfile" "$sorted_tmpfile"

    echo "[cleanup] Size cleanup: removed $removed_count entries, freed $(( freed_bytes / 1024 / 1024 ))MB, now $(( total_size / 1024 / 1024 ))MB / $(( max_bytes / 1024 / 1024 ))MB"
}

##############################################################################
# Main
##############################################################################
main() {
    echo "[cleanup] Starting auto cleanup..."

    # Load config from argument
    if [ $# -ge 1 ]; then
        load_config "$1"
    else
        echo "[cleanup] No config file specified, using defaults"
    fi

    # Step 1: Cleanup by retention days
    cleanup_by_days "$STDOUT_DIR" "$STDOUT_RETENTION_DAYS" "stdout"
    cleanup_by_days "$MCAP_DIR" "$MCAP_RETENTION_DAYS" "mcap"
    cleanup_by_days "$COREDUMP_DIR" "$COREDUMP_RETENTION_DAYS" "coredumps"

    # Step 2: Cleanup by total size limit
    cleanup_by_size

    echo "[cleanup] Done"
}

main "$@"
