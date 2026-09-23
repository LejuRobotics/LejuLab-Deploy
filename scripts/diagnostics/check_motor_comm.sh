#!/usr/bin/env bash
# check_motor_comm.sh — 电机通信质量诊断
#
# 检查: CAN 帧率、丢帧、延迟抖动、电机响应
# 注意: 需要 hardware_node 运行中且有电机连接

PASS=0; FAIL=0; WARN=0
pass() { echo "  [PASS] $1"; PASS=$((PASS+1)); }
fail() { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $1"; WARN=$((WARN+1)); }
info() { echo "  [INFO] $1"; }

SAMPLE_SEC="${1:-3}"  # 采样时长，默认 3 秒

if ! command -v candump &>/dev/null; then
    fail "candump 未安装 — 执行: apt install can-utils"
    echo "  结果: ${PASS} PASS / ${FAIL} FAIL / ${WARN} WARN"
    exit 1
fi

# ---- 检查 CAN 接口是否存在 ----
BUSES=()
for bus in bcan0 bcan1 bcan2 bcan3; do
    if ip link show "${bus}" &>/dev/null; then
        BUSES+=("${bus}")
    fi
done

if [[ ${#BUSES[@]} -eq 0 ]]; then
    fail "无可用 CAN 接口"
    echo "  结果: ${PASS} PASS / ${FAIL} FAIL / ${WARN} WARN"
    exit 1
fi

echo "  --- CAN 帧率统计 (${SAMPLE_SEC}s 采样) ---"

for bus in "${BUSES[@]}"; do
    # 记录采样前的接口统计
    RX_BEFORE=$(ip -s link show "${bus}" 2>/dev/null | awk '/RX:/{getline; print $2}')
    TX_BEFORE=$(ip -s link show "${bus}" 2>/dev/null | awk '/TX:/{getline; print $2}')
    ERR_BEFORE=$(ip -s link show "${bus}" 2>/dev/null | awk '/RX:/{getline; print $3}')

    sleep "${SAMPLE_SEC}"

    RX_AFTER=$(ip -s link show "${bus}" 2>/dev/null | awk '/RX:/{getline; print $2}')
    TX_AFTER=$(ip -s link show "${bus}" 2>/dev/null | awk '/TX:/{getline; print $2}')
    ERR_AFTER=$(ip -s link show "${bus}" 2>/dev/null | awk '/RX:/{getline; print $3}')

    RX_FRAMES=$((RX_AFTER - RX_BEFORE))
    TX_FRAMES=$((TX_AFTER - TX_BEFORE))
    ERR_FRAMES=$((ERR_AFTER - ERR_BEFORE))
    RX_FPS=$((RX_FRAMES / SAMPLE_SEC))
    TX_FPS=$((TX_FRAMES / SAMPLE_SEC))

    info "${bus}: RX=${RX_FPS} fps, TX=${TX_FPS} fps (${SAMPLE_SEC}s)"

    # CANFD 总线 (bcan0/bcan1) 预期: TX≈500fps, RX≈数千 (多电机反馈)
    if [[ "${bus}" == "bcan0" ]] || [[ "${bus}" == "bcan1" ]]; then
        if [[ ${TX_FPS} -ge 400 ]]; then
            pass "${bus} CANFD TX 帧率正常 (${TX_FPS} fps ≥ 400)"
        elif [[ ${TX_FPS} -gt 0 ]]; then
            warn "${bus} CANFD TX 帧率偏低 (${TX_FPS} fps < 400)"
        else
            info "${bus} CANFD 无发送流量 (可能未运行)"
        fi
    fi

    # CAN 总线 (bcan2/bcan3) 预期: TX≈250fps × N电机
    if [[ "${bus}" == "bcan2" ]] || [[ "${bus}" == "bcan3" ]]; then
        if [[ ${TX_FPS} -ge 200 ]]; then
            pass "${bus} CAN TX 帧率正常 (${TX_FPS} fps)"
        elif [[ ${TX_FPS} -gt 0 ]]; then
            warn "${bus} CAN TX 帧率偏低 (${TX_FPS} fps)"
        else
            info "${bus} CAN 无发送流量"
        fi
    fi

    # 错误帧
    if [[ ${ERR_FRAMES} -gt 0 ]]; then
        warn "${bus}: ${ERR_FRAMES} 个错误帧 (${SAMPLE_SEC}s)"
    else
        pass "${bus}: 无错误帧"
    fi
done

echo ""
echo "  --- CAN 帧间隔抖动分析 ---"

# 对每个活跃总线做帧间隔分析
for bus in "${BUSES[@]}"; do
    TMPFILE="/tmp/candump_diag_${bus}.log"

    # 带时间戳采样 2 秒
    timeout 2 candump -t d "${bus}" > "${TMPFILE}" 2>/dev/null &
    DUMP_PID=$!
    sleep 2
    kill ${DUMP_PID} 2>/dev/null; wait ${DUMP_PID} 2>/dev/null

    LINE_COUNT=$(wc -l < "${TMPFILE}" 2>/dev/null || echo 0)
    if [[ ${LINE_COUNT} -lt 10 ]]; then
        info "${bus}: 帧数不足 (${LINE_COUNT})，跳过抖动分析"
        rm -f "${TMPFILE}"
        continue
    fi

    # candump -t d 输出格式: (0.001234) bcan0 020#...
    # 提取时间戳差值 (秒)，计算统计
    STATS=$(awk '
    BEGIN { prev=-1; n=0; sum=0; max=0; min=999 }
    {
        # 提取括号内的时间戳差值
        gsub(/[()]/, "", $1)
        dt = $1 + 0
        if (dt > 0 && dt < 1) {
            sum += dt
            n++
            if (dt > max) max = dt
            if (dt < min) min = dt
        }
    }
    END {
        if (n > 0) {
            avg = sum / n
            printf "n=%d avg=%.3f min=%.3f max=%.3f (ms)\n", n, avg*1000, min*1000, max*1000
        } else {
            printf "n=0\n"
        }
    }' "${TMPFILE}")

    info "${bus} 帧间隔: ${STATS}"

    # 检查最大间隔
    MAX_MS=$(echo "${STATS}" | grep -oP 'max=\K[0-9.]+')
    if [[ -n "${MAX_MS}" ]]; then
        MAX_INT=$(echo "${MAX_MS}" | awk '{printf "%d", $1}')
        if [[ ${MAX_INT} -gt 20 ]]; then
            warn "${bus}: 最大帧间隔 ${MAX_MS}ms > 20ms — 可能存在丢帧或延迟"
        elif [[ ${MAX_INT} -gt 10 ]]; then
            info "${bus}: 最大帧间隔 ${MAX_MS}ms (轻微抖动)"
        else
            pass "${bus}: 帧间隔抖动正常 (max=${MAX_MS}ms)"
        fi
    fi

    rm -f "${TMPFILE}"
done

echo ""
echo "  --- CAN 接口错误计数器 ---"

for bus in "${BUSES[@]}"; do
    # 读取 CAN 控制器错误计数器
    DETAILS=$(ip -details link show "${bus}" 2>/dev/null)

    # 解析 bus-error 等信息
    RESTART_COUNT=$(echo "${DETAILS}" | grep -oP 'restarts \K[0-9]+' || echo "0")
    if [[ "${RESTART_COUNT}" -gt 0 ]]; then
        warn "${bus}: ${RESTART_COUNT} 次自动重启 (bus-off recovery)"
    fi

    # TEC/REC 计数
    TEC=$(echo "${DETAILS}" | grep -oP 'TEC:\K[0-9]+' 2>/dev/null || true)
    REC=$(echo "${DETAILS}" | grep -oP 'REC:\K[0-9]+' 2>/dev/null || true)
    if [[ -n "${TEC}" ]] || [[ -n "${REC}" ]]; then
        info "${bus}: TEC=${TEC:-?} REC=${REC:-?}"
        if [[ "${TEC:-0}" -gt 96 ]] || [[ "${REC:-0}" -gt 96 ]]; then
            warn "${bus}: 错误计数器过高 (TEC=${TEC} REC=${REC})，接近 bus-off"
        fi
    fi
done

echo ""
echo "  结果: ${PASS} PASS / ${FAIL} FAIL / ${WARN} WARN"
