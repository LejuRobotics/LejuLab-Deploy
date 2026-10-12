#!/usr/bin/env bash
# check_system.sh — 系统环境诊断
#
# 检查: 内核版本、PREEMPT_RT、CPU 信息、实时调度权限、内存

PASS=0; FAIL=0; WARN=0
pass() { echo "  [PASS] $1"; PASS=$((PASS+1)); }
fail() { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $1"; WARN=$((WARN+1)); }
info() { echo "  [INFO] $1"; }

# ---- 内核版本 ----
KERNEL=$(uname -r)
info "内核: ${KERNEL}"

if uname -r | grep -qi 'rt\|preempt'; then
    pass "PREEMPT_RT 实时内核"
else
    warn "非实时内核 — 控制性能可能不稳定"
fi

if uname -v | grep -qi 'PREEMPT_RT'; then
    pass "内核编译选项包含 PREEMPT_RT"
else
    # 有些内核 uname -r 不含 rt 但 uname -v 有
    if [[ -f /sys/kernel/realtime ]]; then
        RT_VAL=$(cat /sys/kernel/realtime 2>/dev/null || echo "0")
        if [[ "${RT_VAL}" == "1" ]]; then
            pass "内核实时模式已启用 (/sys/kernel/realtime=1)"
        fi
    fi
fi

# ---- CPU 信息 ----
CPU_MODEL=$(grep -m1 'model name\|Hardware' /proc/cpuinfo 2>/dev/null | cut -d: -f2 | xargs)
CPU_CORES=$(nproc)
info "CPU: ${CPU_MODEL:-unknown} (${CPU_CORES} 核)"

# ---- 架构 ----
ARCH=$(uname -m)
info "架构: ${ARCH}"
if [[ "${ARCH}" == "aarch64" ]]; then
    pass "aarch64 架构 (RK3588)"
elif [[ "${ARCH}" == "x86_64" ]]; then
    info "x86_64 架构 (开发机)"
fi

# ---- 实时调度权限 ----
RT_LIMIT=$(ulimit -r 2>/dev/null || echo "0")
if [[ "${RT_LIMIT}" -ge 99 ]] || [[ "$(id -u)" -eq 0 ]]; then
    pass "SCHED_FIFO 权限正常 (rtprio=${RT_LIMIT})"
else
    warn "SCHED_FIFO rtprio=${RT_LIMIT} (可能需要 sudo 或配置 /etc/security/limits.conf)"
fi

# ---- 内存 ----
MEM_TOTAL=$(grep MemTotal /proc/meminfo | awk '{print $2}')
MEM_AVAIL=$(grep MemAvailable /proc/meminfo | awk '{print $2}')
MEM_TOTAL_MB=$((MEM_TOTAL / 1024))
MEM_AVAIL_MB=$((MEM_AVAIL / 1024))
info "内存: ${MEM_AVAIL_MB}MB 可用 / ${MEM_TOTAL_MB}MB 总计"
if [[ ${MEM_AVAIL_MB} -lt 256 ]]; then
    warn "可用内存不足 256MB"
else
    pass "内存充足 (${MEM_AVAIL_MB}MB)"
fi

# ---- CPU 频率调节 (performance governor) ----
if [[ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]]; then
    GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)
    if [[ "${GOV}" == "performance" ]]; then
        pass "CPU governor: performance"
    else
        warn "CPU governor: ${GOV} (建议设为 performance 以减少延迟抖动)"
    fi
fi

# ---- 共享内存大小 ----
SHM_SIZE=$(df /dev/shm 2>/dev/null | awk 'NR==2{print $2}')
if [[ -n "${SHM_SIZE}" ]]; then
    SHM_MB=$((SHM_SIZE / 1024))
    info "/dev/shm 大小: ${SHM_MB}MB"
    if [[ ${SHM_MB} -lt 64 ]]; then
        warn "/dev/shm 小于 64MB — iceoryx 可能空间不足"
    fi
fi

echo ""
echo "  结果: ${PASS} PASS / ${FAIL} FAIL / ${WARN} WARN"
