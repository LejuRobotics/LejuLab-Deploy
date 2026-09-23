#!/usr/bin/env bash
# check_hardware_node.sh — hardware_node 进程诊断
#
# 检查: 进程存活、实时调度、CPU 使用率、日志输出

PASS=0; FAIL=0; WARN=0
pass() { echo "  [PASS] $1"; PASS=$((PASS+1)); }
fail() { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $1"; WARN=$((WARN+1)); }
info() { echo "  [INFO] $1"; }

echo "  --- 进程检测 ---"

HW_PIDS=$(pgrep -f 'leju-hardware' 2>/dev/null || true)
if [[ -n "${HW_PIDS}" ]]; then
    for pid in ${HW_PIDS}; do
        CMDLINE=$(tr '\0' ' ' < /proc/${pid}/cmdline 2>/dev/null || echo "?")
        pass "hardware_node 运行中 (PID=${pid})"
        info "  命令: ${CMDLINE}"
    done
else
    fail "hardware_node 未运行"
    echo ""
    echo "  结果: ${PASS} PASS / ${FAIL} FAIL / ${WARN} WARN"
    exit 0
fi

echo ""
echo "  --- 实时调度 ---"

for pid in ${HW_PIDS}; do
    # 检查主进程和子线程的调度策略
    THREADS=$(ls /proc/${pid}/task/ 2>/dev/null)
    RT_THREADS=0
    NORMAL_THREADS=0
    for tid in ${THREADS}; do
        SCHED_INFO=$(chrt -p "${tid}" 2>/dev/null || true)
        if echo "${SCHED_INFO}" | grep -q 'SCHED_FIFO\|SCHED_RR'; then
            PRIO=$(echo "${SCHED_INFO}" | grep -oP 'priority: \K[0-9]+')
            RT_THREADS=$((RT_THREADS + 1))
        else
            NORMAL_THREADS=$((NORMAL_THREADS + 1))
        fi
    done

    if [[ ${RT_THREADS} -gt 0 ]]; then
        pass "hardware_node 有 ${RT_THREADS} 个实时线程 (+ ${NORMAL_THREADS} 普通线程)"
    else
        warn "hardware_node 无实时线程 — 控制延迟可能不稳定"
    fi

    # 显示实时线程详情
    for tid in ${THREADS}; do
        SCHED_INFO=$(chrt -p "${tid}" 2>/dev/null || true)
        if echo "${SCHED_INFO}" | grep -q 'SCHED_FIFO\|SCHED_RR'; then
            PRIO=$(echo "${SCHED_INFO}" | grep -oP 'priority: \K[0-9]+')
            TNAME=$(cat /proc/${pid}/task/${tid}/comm 2>/dev/null || echo "?")
            info "  RT 线程: ${TNAME} (TID=${tid}, priority=${PRIO})"
        fi
    done
done

echo ""
echo "  --- CPU 使用率 (1s 采样) ---"

for pid in ${HW_PIDS}; do
    # 快速采样 CPU 使用率
    CPU1=$(cat /proc/${pid}/stat 2>/dev/null | awk '{print $14+$15}')
    sleep 1
    CPU2=$(cat /proc/${pid}/stat 2>/dev/null | awk '{print $14+$15}')

    if [[ -n "${CPU1}" ]] && [[ -n "${CPU2}" ]]; then
        CPU_TICKS=$((CPU2 - CPU1))
        CLK_TCK=$(getconf CLK_TCK)
        CPU_PCT=$((CPU_TICKS * 100 / CLK_TCK))
        info "hardware_node CPU: ~${CPU_PCT}%"
        if [[ ${CPU_PCT} -gt 80 ]]; then
            warn "CPU 使用率过高 (${CPU_PCT}%)"
        fi
    fi
done

echo ""
echo "  --- ROBOT_VERSION ---"

# 检查环境变量
for pid in ${HW_PIDS}; do
    RV=$(tr '\0' '\n' < /proc/${pid}/environ 2>/dev/null | grep '^ROBOT_VERSION=' | cut -d= -f2)
    if [[ -n "${RV}" ]]; then
        info "ROBOT_VERSION=${RV}"
        pass "ROBOT_VERSION 已设置"
    else
        warn "ROBOT_VERSION 未在 hardware_node 环境中找到"
    fi
    break
done

echo ""
echo "  --- 控制器进程 ---"

# 检查有无控制器连接
CTRL_PIDS=$(pgrep -f 'motor_keyboard_ctrl\|run_dummy_controller\|rl_controller' 2>/dev/null || true)
if [[ -n "${CTRL_PIDS}" ]]; then
    for pid in ${CTRL_PIDS}; do
        CMDLINE=$(tr '\0' ' ' < /proc/${pid}/cmdline 2>/dev/null | head -c 80)
        info "控制器进程: PID=${pid} (${CMDLINE})"
    done
else
    info "当前无控制器进程运行"
fi

echo ""
echo "  结果: ${PASS} PASS / ${FAIL} FAIL / ${WARN} WARN"
