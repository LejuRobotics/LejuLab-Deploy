#!/bin/bash
#
# 检查 CycloneDDS + iceoryx 共享内存是否正常工作
#

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'
PASS=0
FAIL=0
WARN=0

ok() {
    echo -e "  [${GREEN}OK${NC}] $1"
    ((PASS++))
}

fail() {
    echo -e "  [${RED}FAIL${NC}] $1"
    ((FAIL++))
}

warn() {
    echo -e "  [${YELLOW}SKIP${NC}] $1"
    ((WARN++))
}

check() {
    if eval "$2" > /dev/null 2>&1; then
        ok "$1"
    else
        fail "$1"
    fi
}

echo "=== CycloneDDS 共享内存状态检查 ==="

# ── 1. 基础设施 ──
echo ""
echo "基础设施:"
check "leju-roudi.service 正在运行" "systemctl is-active --quiet leju-roudi.service"
check "iox-roudi 进程存在" "pgrep -f iox-roudi"
check "/dev/shm/iceoryx 存在" "[ -e /dev/shm/iceoryx ]"
check "/dev/shm/iceoryx_mgmt 存在" "[ -e /dev/shm/iceoryx_mgmt ]"
check "/tmp/roudi socket 存在" "[ -S /tmp/roudi ]"

# ── 2. 配置 ──
echo ""
echo "配置:"
check "/etc/profile.d/cyclonedds.sh 指向 shm 配置" "grep -q cyclonedds_shm.xml /etc/profile.d/cyclonedds.sh"
check "cyclonedds_shm.xml SharedMemory=true" "grep -q '<Enable>true</Enable>' /etc/cyclonedds/cyclonedds_shm.xml"

if echo "$CYCLONEDDS_URI" | grep -q cyclonedds_shm.xml 2>/dev/null; then
    ok "当前 shell CYCLONEDDS_URI 指向 shm 配置"
else
    fail "当前 shell CYCLONEDDS_URI=${CYCLONEDDS_URI:-<未设置>} (需要重新登录或 source ~/.bashrc)"
fi

# ── 3. 进程级验证: 检查 DDS 进程是否 mmap 了 iceoryx 共享内存 ──
echo ""
echo "进程共享内存使用:"

ROUDI_PID=$(pgrep -f iox-roudi 2>/dev/null)
SHM_PROCS=()

for maps in /proc/[0-9]*/maps; do
    pid=$(echo "$maps" | cut -d/ -f3)
    [ "$pid" = "$ROUDI_PID" ] && continue
    if grep -q '/dev/shm/iceoryx' "$maps" 2>/dev/null; then
        CMDLINE=$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null | cut -c1-80)
        SHM_PROCS+=("${pid}:${CMDLINE}")
    fi
done 2>/dev/null

if [ ${#SHM_PROCS[@]} -gt 0 ]; then
    ok "${#SHM_PROCS[@]} 个进程正在使用 iceoryx 共享内存:"
    for entry in "${SHM_PROCS[@]}"; do
        echo "       PID ${entry%%:*} -> ${entry#*:}"
    done
else
    warn "当前没有 DDS 进程使用共享内存 (启动 ROS 节点后再检查)"
fi

# ── 汇总 ──
echo ""
echo "结果: ${PASS} 通过, ${FAIL} 失败, ${WARN} 跳过"

if [ "${FAIL}" -eq 0 ]; then
    echo -e "${GREEN}共享内存状态正常${NC}"
else
    echo -e "${RED}存在问题，请检查上述失败项${NC}"
    exit 1
fi
