#!/usr/bin/env bash
# check_dds.sh — DDS / iceoryx 共享内存诊断
#
# 检查: RouDi 服务、共享内存、CycloneDDS 配置、DDS topic 活跃度

PASS=0; FAIL=0; WARN=0
pass() { echo "  [PASS] $1"; PASS=$((PASS+1)); }
fail() { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $1"; WARN=$((WARN+1)); }
info() { echo "  [INFO] $1"; }

echo "  --- RouDi 服务 ---"

# 检查 RouDi 进程
if pgrep -f 'iox-roudi' &>/dev/null; then
    ROUDI_PID=$(pgrep -f 'iox-roudi' | head -1)
    pass "RouDi 进程运行中 (PID=${ROUDI_PID})"
else
    fail "RouDi 未运行 — 执行: systemctl start leju-roudi"
fi

# 检查 systemd 服务
if systemctl is-active leju-roudi &>/dev/null; then
    pass "leju-roudi.service active"
else
    STATUS=$(systemctl is-active leju-roudi 2>/dev/null || echo "not-found")
    if [[ "${STATUS}" == "not-found" ]]; then
        warn "leju-roudi.service 未安装 — 执行 setup_cyclonedds_config.sh"
    else
        warn "leju-roudi.service 状态: ${STATUS}"
    fi
fi

echo ""
echo "  --- iceoryx 共享内存 ---"

# 检查 RouDi socket
if [[ -S /tmp/roudi ]] || ls /tmp/roudi* &>/dev/null; then
    pass "RouDi socket 存在 (/tmp/roudi*)"
else
    fail "RouDi socket 不存在 — RouDi 可能未启动"
fi

# 检查 iceoryx 共享内存段
ICE_SHM_COUNT=$(ls /dev/shm/iceoryx* 2>/dev/null | wc -l)
if [[ ${ICE_SHM_COUNT} -gt 0 ]]; then
    pass "iceoryx 共享内存: ${ICE_SHM_COUNT} 个段"
    # 列出段的大小
    for f in /dev/shm/iceoryx*; do
        SIZE_KB=$(($(stat -c%s "$f" 2>/dev/null || echo 0) / 1024))
        info "  ${f##*/}: ${SIZE_KB}KB"
    done
else
    fail "无 iceoryx 共享内存 — RouDi 未正常初始化"
fi

# 检查 iceoryx 组权限
if id -nG 2>/dev/null | grep -qw iceoryx; then
    pass "当前用户在 iceoryx 组中"
else
    CURRENT_USER=$(whoami)
    if [[ "${CURRENT_USER}" == "root" ]]; then
        if id -nG root 2>/dev/null | grep -qw iceoryx; then
            pass "root 用户在 iceoryx 组中"
        else
            warn "root 用户不在 iceoryx 组 — 执行: usermod -aG iceoryx root"
        fi
    else
        warn "用户 ${CURRENT_USER} 不在 iceoryx 组 — 执行: sudo usermod -aG iceoryx ${CURRENT_USER}"
    fi
fi

echo ""
echo "  --- CycloneDDS 配置 ---"

# 检查 CYCLONEDDS_URI
if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    info "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
    # 提取文件路径
    DDS_CONF=$(echo "${CYCLONEDDS_URI}" | sed 's|file://||')
    if [[ -f "${DDS_CONF}" ]]; then
        pass "DDS 配置文件存在: ${DDS_CONF}"
        # 检查 SHM 是否启用
        if grep -qi 'SharedMemory' "${DDS_CONF}" 2>/dev/null; then
            if grep -A2 -i 'SharedMemory' "${DDS_CONF}" | grep -qi 'true\|Enable'; then
                pass "DDS 共享内存模式已启用"
            else
                info "DDS 共享内存模式未启用 (使用 UDP)"
            fi
        fi
    else
        warn "DDS 配置文件不存在: ${DDS_CONF}"
    fi
else
    # 检查默认路径
    if [[ -f /etc/cyclonedds/cyclonedds_shm.xml ]]; then
        warn "CYCLONEDDS_URI 未设置，但配置文件存在于 /etc/cyclonedds/cyclonedds_shm.xml"
    else
        warn "CYCLONEDDS_URI 未设置 — DDS 将使用默认 UDP 通信"
    fi
fi

echo ""
echo "  --- DDS 进程检测 ---"

# 检查是否有进程使用 iceoryx 共享内存
ICE_PROCS=$(lsof /dev/shm/iceoryx* 2>/dev/null | grep -v '^COMMAND' | awk '{print $1, $2}' | sort -u)
if [[ -n "${ICE_PROCS}" ]]; then
    info "使用 iceoryx 共享内存的进程:"
    echo "${ICE_PROCS}" | while read -r proc_info; do
        info "  ${proc_info}"
    done
else
    info "当前无进程使用 iceoryx 共享内存"
fi

echo ""
echo "  结果: ${PASS} PASS / ${FAIL} FAIL / ${WARN} WARN"
