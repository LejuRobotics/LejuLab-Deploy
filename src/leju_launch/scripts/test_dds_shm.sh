#!/usr/bin/env bash
# test_dds_shm.sh — 启动 hardware_node，验证 DDS 共享内存是否生效
#
# 用法: sudo bash test_dds_shm.sh [--no-shm]
#   --no-shm  强制使用 UDP 模式（用于对比测试）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_launch_common.sh"

parse_args "$@"
setup_env

export ROBOT_VERSION="${ROBOT_VERSION:-14}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}=== DDS 共享内存通信验证 ===${NC}"
echo ""

# ── 1. 前置检查 ──
echo -e "${CYAN}[1/4] 前置检查${NC}"

echo -n "  CYCLONEDDS_URI = "
echo -e "${GREEN}${CYCLONEDDS_URI}${NC}"

if echo "${CYCLONEDDS_URI}" | grep -q "shm"; then
    echo -e "  模式: ${GREEN}共享内存 (SHM)${NC}"
else
    echo -e "  模式: ${YELLOW}UDP loopback (无共享内存)${NC}"
fi

if pgrep -f iox-roudi > /dev/null 2>&1; then
    echo -e "  RouDi: ${GREEN}运行中${NC}"
else
    echo -e "  RouDi: ${RED}未运行${NC}"
    if echo "${CYCLONEDDS_URI}" | grep -q "shm"; then
        echo -e "  ${RED}错误: SHM 模式需要 RouDi，请先运行 setup_cyclonedds_config.sh${NC}"
        exit 1
    fi
fi

if [ -e /dev/shm/iceoryx ]; then
    echo -e "  /dev/shm/iceoryx: ${GREEN}存在${NC}"
else
    echo -e "  /dev/shm/iceoryx: ${RED}不存在${NC}"
fi

# ── 2. 启动 hardware_node ──
echo ""
echo -e "${CYAN}[2/4] 启动 hardware_node${NC}"

HW_BIN="$(find_binary leju-hardware leju-hardware)"
HW_ARGS="$(find_package leju-hardware)/"

setsid "${SCRIPT_DIR}/start_node.sh" "${HW_BIN}" "${HW_ARGS}" &
HW_PID=$!
echo "  PID=${HW_PID} (等待启动...)"

cleanup() {
    echo ""
    echo -e "${CYAN}[清理] 停止 hardware_node${NC}"
    kill -TERM -"${HW_PID}" 2>/dev/null || kill -TERM "${HW_PID}" 2>/dev/null || true
    sleep 2
    kill -0 "${HW_PID}" 2>/dev/null && kill -KILL -"${HW_PID}" 2>/dev/null || true
    wait "${HW_PID}" 2>/dev/null || true
    echo "  已停止"
}
trap cleanup EXIT

sleep 5

if ! kill -0 "${HW_PID}" 2>/dev/null; then
    echo -e "  ${RED}hardware_node 启动失败${NC}"
    exit 1
fi
echo -e "  ${GREEN}hardware_node 已启动${NC}"

# ── 3. 检查进程是否使用了 iceoryx 共享内存 ──
echo ""
echo -e "${CYAN}[3/4] 检查进程共享内存映射${NC}"

ROUDI_PID=$(pgrep -f iox-roudi 2>/dev/null || echo "")
SHM_COUNT=0

for maps in /proc/[0-9]*/maps; do
    pid=$(echo "$maps" | cut -d/ -f3)
    [ "$pid" = "$ROUDI_PID" ] && continue
    if grep -q '/dev/shm/iceoryx' "$maps" 2>/dev/null; then
        CMDLINE=$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null | cut -c1-60)
        echo -e "  ${GREEN}PID ${pid}${NC} -> ${CMDLINE}"
        ((SHM_COUNT++))
    fi
done 2>/dev/null

if [ "${SHM_COUNT}" -gt 0 ]; then
    echo -e "  ${GREEN}✓ ${SHM_COUNT} 个进程正在使用 iceoryx 共享内存${NC}"
else
    echo -e "  ${YELLOW}⚠ 没有进程使用 iceoryx 共享内存${NC}"
    echo "  DDS 通信走 UDP loopback，延迟较高"
fi

# ── 4. 检查 hardware_node 的 CYCLONEDDS_URI ──
echo ""
echo -e "${CYAN}[4/4] 进程环境变量验证${NC}"

# 找到 hardware_node 的实际进程 (可能是 start_node.sh 的子进程)
HW_REAL_PID=$(pgrep -f "leju-hardware" 2>/dev/null | head -1 || echo "")
if [ -n "${HW_REAL_PID}" ]; then
    HW_DDS_URI=$(tr '\0' '\n' < "/proc/${HW_REAL_PID}/environ" 2>/dev/null | grep "^CYCLONEDDS_URI=" || echo "未设置")
    echo "  hardware_node (PID ${HW_REAL_PID}):"
    echo "    ${HW_DDS_URI}"
    if echo "${HW_DDS_URI}" | grep -q "shm"; then
        echo -e "    ${GREEN}✓ 使用共享内存配置${NC}"
    elif echo "${HW_DDS_URI}" | grep -q "cyclonedds"; then
        echo -e "    ${YELLOW}⚠ 使用 UDP 配置 (非共享内存)${NC}"
    else
        echo -e "    ${RED}✗ CYCLONEDDS_URI 未设置，使用默认配置${NC}"
    fi
else
    echo -e "  ${RED}未找到 hardware_node 进程${NC}"
fi

# ── 结论 ──
echo ""
echo -e "${CYAN}=== 结论 ===${NC}"
if [ "${SHM_COUNT}" -gt 0 ] && echo "${HW_DDS_URI:-}" | grep -q "shm"; then
    echo -e "${GREEN}✓ DDS 共享内存通信正常工作${NC}"
    echo "  预期延迟: < 100μs"
else
    echo -e "${YELLOW}⚠ DDS 未使用共享内存${NC}"
    echo "  当前延迟: ~1000μs (UDP loopback)"
    echo ""
    echo "  修复方法:"
    echo "    1. 确保 RouDi 在运行: systemctl start leju-roudi"
    echo "    2. 启动时不要加 --no-shm"
    echo "    3. 或重新部署: bash setup_cyclonedds_config.sh"
fi
echo ""
echo "按 Ctrl+C 退出并停止 hardware_node"
wait "${HW_PID}" 2>/dev/null || true
