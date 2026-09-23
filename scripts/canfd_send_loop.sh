#!/bin/bash
#
# canfd_send_loop.sh — 循环发送 CANFD 帧，Ctrl+C 停止
#
# 用法:
#   sudo bash scripts/canfd_send_loop.sh [总线] [帧ID] [数据(hex)] [间隔ms]
#
# 示例:
#   sudo bash scripts/canfd_send_loop.sh bcan0 001 0011223344556677 2
#   sudo bash scripts/canfd_send_loop.sh bcan1 0A1 00112233445566778899AABBCCDDEEFF 4
#
# 默认: bcan0, ID=001, 8字节全零, 2ms间隔

BUS="${1:-bcan0}"
FRAME_ID="${2:-001}"
DATA="${3:-0000000000000000}"
INTERVAL_MS="${4:-2}"

# 数据长度决定是否用 CANFD
DATA_BYTES=$((${#DATA} / 2))
if [ "$DATA_BYTES" -gt 8 ]; then
    # CANFD 帧 (>8 bytes), 需要 ##flags: B=BRS, E=ESI
    FRAME="${FRAME_ID}##0${DATA}"
    FRAME_TYPE="CANFD"
else
    # 标准 CAN 帧 (<=8 bytes)
    FRAME="${FRAME_ID}#${DATA}"
    FRAME_TYPE="CAN"
fi

echo "=== CANFD 循环发送 ==="
echo "  总线:   ${BUS}"
echo "  帧ID:   0x${FRAME_ID}"
echo "  数据:   ${DATA} (${DATA_BYTES} bytes, ${FRAME_TYPE})"
echo "  间隔:   ${INTERVAL_MS} ms"
echo "  Ctrl+C 停止"
echo ""

COUNT=0
trap 'echo ""; echo "已发送 ${COUNT} 帧"; exit 0' INT TERM

while true; do
    cansend "${BUS}" "${FRAME}"
    COUNT=$((COUNT + 1))
    if [ $((COUNT % 500)) -eq 0 ]; then
        echo "[${COUNT}] 已发送 ${COUNT} 帧..."
    fi
    sleep "$(echo "scale=6; ${INTERVAL_MS}/1000" | bc)"
done
