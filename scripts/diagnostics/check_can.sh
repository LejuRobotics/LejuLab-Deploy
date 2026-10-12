#!/usr/bin/env bash
# check_can.sh — CAN 总线诊断
#
# 检查: CAN 接口状态、比特率、错误帧、流量

PASS=0; FAIL=0; WARN=0
pass() { echo "  [PASS] $1"; PASS=$((PASS+1)); }
fail() { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $1"; WARN=$((WARN+1)); }
info() { echo "  [INFO] $1"; }

# Roban v14 预期的 CAN 总线
# bcan0: 左腿 CANFD (1M/5M)
# bcan1: 右腿+腰 CANFD (1M/5M)
# bcan2: 左臂+头 CAN (1M)
# bcan3: 右臂 CAN (1M)
EXPECTED_BUSES=("bcan0" "bcan1" "bcan2" "bcan3")
CANFD_BUSES=("bcan0" "bcan1")
CAN_BUSES=("bcan2" "bcan3")

echo "  --- CAN 接口检测 ---"

for bus in "${EXPECTED_BUSES[@]}"; do
    if ip link show "${bus}" &>/dev/null; then
        STATE=$(ip link show "${bus}" 2>/dev/null | grep -oP 'state \K\S+')
        if [[ "${STATE}" == "UP" ]] || ip link show "${bus}" | grep -q "UP"; then
            pass "${bus} 接口 UP"
        else
            fail "${bus} 接口存在但未 UP (state=${STATE})"
        fi
    else
        fail "${bus} 接口不存在"
    fi
done

echo ""
echo "  --- CAN 比特率检查 ---"

for bus in "${EXPECTED_BUSES[@]}"; do
    if ! ip link show "${bus}" &>/dev/null; then
        continue
    fi

    # 读取比特率
    BITRATE=$(ip -details link show "${bus}" 2>/dev/null | grep -oP 'bitrate \K[0-9]+' | head -1)
    if [[ -n "${BITRATE}" ]]; then
        BITRATE_K=$((BITRATE / 1000))
        info "${bus}: nominal bitrate = ${BITRATE_K} kbps"

        if [[ "${BITRATE}" -eq 1000000 ]]; then
            pass "${bus} nominal bitrate 1Mbps"
        else
            warn "${bus} nominal bitrate ${BITRATE_K}kbps (期望 1000kbps)"
        fi
    fi

    # CANFD 数据比特率
    DBITRATE=$(ip -details link show "${bus}" 2>/dev/null | grep -oP 'dbitrate \K[0-9]+' | head -1)
    if [[ -n "${DBITRATE}" ]]; then
        DBITRATE_K=$((DBITRATE / 1000))
        info "${bus}: data bitrate = ${DBITRATE_K} kbps"

        # bcan0/bcan1 应有 5M 数据比特率
        for fd_bus in "${CANFD_BUSES[@]}"; do
            if [[ "${bus}" == "${fd_bus}" ]]; then
                if [[ "${DBITRATE}" -eq 5000000 ]]; then
                    pass "${bus} CANFD data bitrate 5Mbps"
                else
                    warn "${bus} CANFD data bitrate ${DBITRATE_K}kbps (期望 5000kbps)"
                fi
            fi
        done
    fi
done

echo ""
echo "  --- CAN 错误统计 ---"

for bus in "${EXPECTED_BUSES[@]}"; do
    if ! ip link show "${bus}" &>/dev/null; then
        continue
    fi

    # 读取错误帧计数
    STATS=$(ip -s link show "${bus}" 2>/dev/null)
    TX_ERR=$(echo "${STATS}" | awk '/TX:/{getline; print $3}' 2>/dev/null || echo "?")
    RX_ERR=$(echo "${STATS}" | awk '/RX:/{getline; print $3}' 2>/dev/null || echo "?")

    if [[ "${TX_ERR}" == "0" ]] && [[ "${RX_ERR}" == "0" ]]; then
        pass "${bus} 无错误帧 (TX_err=0, RX_err=0)"
    else
        warn "${bus} 有错误帧 (TX_err=${TX_ERR}, RX_err=${RX_ERR})"
    fi

    # 检查 bus-off 状态
    CAN_STATE=$(ip -details link show "${bus}" 2>/dev/null | grep -oP 'state \K\S+' | tail -1)
    if [[ "${CAN_STATE}" == "ERROR-ACTIVE" ]]; then
        pass "${bus} CAN 状态: ERROR-ACTIVE (正常)"
    elif [[ "${CAN_STATE}" == "ERROR-WARNING" ]]; then
        warn "${bus} CAN 状态: ERROR-WARNING"
    elif [[ "${CAN_STATE}" == "ERROR-PASSIVE" ]]; then
        fail "${bus} CAN 状态: ERROR-PASSIVE — 通信严重受损"
    elif [[ "${CAN_STATE}" == "BUS-OFF" ]]; then
        fail "${bus} CAN 状态: BUS-OFF — 总线已断开，需要重新 up"
    elif [[ -n "${CAN_STATE}" ]]; then
        info "${bus} CAN 状态: ${CAN_STATE}"
    fi
done

echo ""
echo "  --- CAN 流量嗅探 (2秒) ---"

for bus in "${EXPECTED_BUSES[@]}"; do
    if ! ip link show "${bus}" &>/dev/null; then
        continue
    fi

    if ! command -v candump &>/dev/null; then
        warn "candump 工具未安装 (apt install can-utils)"
        break
    fi

    # 2 秒采样，统计帧数
    FRAME_COUNT=$(timeout 2 candump "${bus}" 2>/dev/null | wc -l)
    if [[ ${FRAME_COUNT} -gt 0 ]]; then
        FPS=$((FRAME_COUNT / 2))
        pass "${bus}: ${FRAME_COUNT} 帧/2s ≈ ${FPS} fps"
    else
        info "${bus}: 2 秒内无流量 (电机未运行或未连接)"
    fi
done

echo ""
echo "  结果: ${PASS} PASS / ${FAIL} FAIL / ${WARN} WARN"
