#!/usr/bin/env bash
# check_all.sh — 全链路系统诊断入口
#
# 依次运行所有诊断模块，给出整体健康报告
# 用法: bash scripts/diagnostics/check_all.sh
#       bash scripts/diagnostics/check_all.sh --remote 192.168.50.157  (在 3588 上远程运行)

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 远程模式: 把脚本 scp 到 3588 执行
if [[ "${1:-}" == "--remote" ]]; then
    if [[ -z "${2:-}" ]]; then
        echo "错误: --remote 需要指定目标 IP，例如: bash check_all.sh --remote <IP>" >&2
        exit 1
    fi
    HOST="$2"
    USER="${3:-linux}"
    echo "=== 远程诊断: ${USER}@${HOST} ==="
    # 上传诊断脚本
    scp -r "${SCRIPT_DIR}" "${USER}@${HOST}:/tmp/diagnostics" 2>/dev/null
    ssh -t "${USER}@${HOST}" "sudo bash /tmp/diagnostics/check_all.sh"
    exit $?
fi

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║         系统全链路诊断                        ║"
echo "║  $(date '+%Y-%m-%d %H:%M:%S')                            ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

TOTAL_PASS=0
TOTAL_FAIL=0
TOTAL_WARN=0

run_module() {
    local name="$1"
    local script="$2"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo " 模块: ${name}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    if [[ -f "${SCRIPT_DIR}/${script}" ]]; then
        bash "${SCRIPT_DIR}/${script}" 2>&1
        local rc=$?
        echo ""
        return ${rc}
    else
        echo "  [SKIP] 脚本不存在: ${script}"
        echo ""
        return 0
    fi
}

run_module "1. 系统环境 (内核/RT/CPU)"     "check_system.sh"
run_module "2. CAN 总线"                    "check_can.sh"
run_module "3. DDS / iceoryx 共享内存"      "check_dds.sh"
run_module "4. hardware_node 进程"          "check_hardware_node.sh"
run_module "5. 电机通信质量"                "check_motor_comm.sh"

echo "╔══════════════════════════════════════════════╗"
echo "║  诊断完成                                     ║"
echo "╚══════════════════════════════════════════════╝"
