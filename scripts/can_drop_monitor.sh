#!/bin/bash
#
# CAN 丢帧实时监控
# 用法: sudo bash can_drop_monitor.sh bcan2 bcan3
#

IFACES="${@:-bcan2 bcan3}"
INTERVAL=1

RED='\033[31m'
YEL='\033[33m'
GRN='\033[32m'
RST='\033[0m'

get_stats() {
    # 输出: tx_pkts tx_drop tx_err rx_pkts rx_drop rx_err rx_over
    local iface=$1
    local stats=$(ip -s link show "$iface" 2>/dev/null)
    [ -z "$stats" ] && echo "0 0 0 0 0 0 0" && return 1

    local rx_vals=$(echo "$stats" | grep -A1 "RX:" | tail -1 | tr -s ' ')
    local tx_vals=$(echo "$stats" | grep -A1 "TX:" | tail -1 | tr -s ' ')

    # RX: bytes packets errors dropped overrun mcast
    local rx_pkts=$(echo "$rx_vals" | awk '{print $2}')
    local rx_err=$(echo "$rx_vals" | awk '{print $3}')
    local rx_drop=$(echo "$rx_vals" | awk '{print $4}')
    local rx_over=$(echo "$rx_vals" | awk '{print $5}')

    # TX: bytes packets errors dropped carrier collsns
    local tx_pkts=$(echo "$tx_vals" | awk '{print $2}')
    local tx_err=$(echo "$tx_vals" | awk '{print $3}')
    local tx_drop=$(echo "$tx_vals" | awk '{print $4}')

    echo "${tx_pkts:-0} ${tx_drop:-0} ${tx_err:-0} ${rx_pkts:-0} ${rx_drop:-0} ${rx_err:-0} ${rx_over:-0}"
}

# 读初始值
declare -A PREV TOTAL_DROP
for iface in $IFACES; do
    PREV[$iface]=$(get_stats "$iface")
    TOTAL_DROP[$iface]=0
done

echo "══════════════════════════════════════════════════════════════"
echo "  CAN 丢帧监控: $IFACES  (每${INTERVAL}s, Ctrl+C 停止)"
echo "══════════════════════════════════════════════════════════════"

SECONDS=0

while true; do
    sleep $INTERVAL

    echo ""
    printf "  [%ds] %-6s │ %7s %8s %7s │ %7s %8s %7s %8s\n" \
           "$SECONDS" "iface" "TX/s" "TX_drop" "TX_err" "RX/s" "RX_drop" "RX_err" "RX_over"
    printf "  ────────────┼─────────────────────────┼──────────────────────────────────\n"

    for iface in $IFACES; do
        cur=$(get_stats "$iface")
        prev="${PREV[$iface]}"

        read ct_p ct_d ct_e cr_p cr_d cr_e cr_o <<< "$cur"
        read pt_p pt_d pt_e pr_p pr_d pr_e pr_o <<< "$prev"

        dt_p=$((ct_p - pt_p))
        dt_d=$((ct_d - pt_d))
        dt_e=$((ct_e - pt_e))
        dr_p=$((cr_p - pr_p))
        dr_d=$((cr_d - pr_d))
        dr_e=$((cr_e - pr_e))
        dr_o=$((cr_o - pr_o))

        PREV[$iface]="$cur"
        TOTAL_DROP[$iface]=$(( ${TOTAL_DROP[$iface]} + dt_d + dr_d ))

        # 着色
        td_c=$GRN; [ $dt_d -gt 0 ] && td_c=$RED
        te_c=$GRN; [ $dt_e -gt 0 ] && te_c=$RED
        rd_c=$GRN; [ $dr_d -gt 0 ] && rd_c=$RED
        re_c=$GRN; [ $dr_e -gt 0 ] && re_c=$RED
        ro_c=$GRN; [ $dr_o -gt 0 ] && ro_c=$RED

        printf "         %-6s │ %7d ${td_c}%8d${RST} ${te_c}%7d${RST} │ %7d ${rd_c}%8d${RST} ${re_c}%7d${RST} ${ro_c}%8d${RST}  累计=${YEL}%d${RST}\n" \
               "$iface" "$dt_p" "$dt_d" "$dt_e" "$dr_p" "$dr_d" "$dr_e" "$dr_o" "${TOTAL_DROP[$iface]}"
    done
done
