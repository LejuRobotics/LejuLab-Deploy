#!/usr/bin/env bash
# 将 CAN/SPI 等中断分摊到小核 CPU 0-3，减轻 CPU0 压力（不绑大核）
#
# 用法:
#   sudo bash scripts/bind_little_core_irqs.sh
#   sudo bash scripts/bind_little_core_irqs.sh --status   # 仅查看
set -euo pipefail

log() { echo "[bind_irq] $*"; }

require_root() {
  if [ "$(id -u)" != 0 ]; then
    echo "需要 root 权限: sudo bash $0" >&2
    exit 1
  fi
}

get_irqs_by_name() {
  local name="$1"
  grep -w "$name" /proc/interrupts 2>/dev/null | awk -F: '{print $1}' | tr -d ' '
}

bind_irq_to_cpu() {
  local irq="$1"
  local cpu="$2"
  local label="${3:-irq$irq}"

  echo "$cpu" > "/proc/irq/$irq/smp_affinity_list"
  local aff
  aff="$(cat "/proc/irq/$irq/smp_affinity_list")"
  log "$label irq=$irq -> CPU$cpu (aff=$aff)"
}

bind_irq_name_to_cpu() {
  local name="$1"
  local cpu="$2"
  local irq

  irq="$(get_irqs_by_name "$name" | head -1)"
  if [ -z "$irq" ]; then
    log "跳过: 未找到 $name"
    return 0
  fi
  bind_irq_to_cpu "$irq" "$cpu" "$name"
}

bind_all_irqs_by_name_round_robin() {
  local name="$1"
  shift
  local cpus=("$@")
  local irq_list=()
  local idx=0

  mapfile -t irq_list < <(get_irqs_by_name "$name")
  if [ "${#irq_list[@]}" -eq 0 ]; then
    log "跳过: 未找到 $name"
    return 0
  fi

  for irq in "${irq_list[@]}"; do
    local cpu="${cpus[$((idx % ${#cpus[@]}))]}"
    bind_irq_to_cpu "$irq" "$cpu" "$name"
    idx=$((idx + 1))
  done
}

show_status() {
  echo "=== CAN 中断绑核 ==="
  for bus in bcan0 bcan1 bcan2 bcan3; do
    irq="$(get_irqs_by_name "$bus" | head -1)"
    if [ -n "$irq" ]; then
      aff="$(cat "/proc/irq/$irq/smp_affinity_list" 2>/dev/null || echo "?")"
      counts="$(grep -w "$bus" /proc/interrupts | awk '{print "c0="$2,"c1="$3,"c2="$4,"c3="$5}')"
      echo "  $bus irq=$irq aff=$aff $counts"
    else
      echo "  $bus: 未找到"
    fi
  done
}

main() {
  if [ "${1:-}" = "--status" ]; then
    show_status
    exit 0
  fi

  require_root

  # CANFD 负载高，避开 CPU0；标准 CAN 留 CPU0
  bind_irq_name_to_cpu bcan0 1   # 左腿 CANFD
  bind_irq_name_to_cpu bcan1 2   # 右腿+腰 CANFD
  bind_irq_name_to_cpu bcan2 3   # 左臂+头
  bind_irq_name_to_cpu bcan3 0   # 右臂标准 CAN

  # SPI 控制器默认全在 CPU0，分摊到 1/2/3
  bind_irq_name_to_cpu feb00000.spi 1
  bind_irq_name_to_cpu feb10000.spi 2
  bind_irq_name_to_cpu feb30000.spi 3
  bind_irq_name_to_cpu fecb0000.spi 1
  bind_irq_name_to_cpu feb20000.spi 2

  # eMMC / USB 也移一部分离开 CPU0
  bind_irq_name_to_cpu mmc0 1
  bind_all_irqs_by_name_round_robin xhci_hcd 2 3
  bind_irq_name_to_cpu ehci_hcd:usb2 3
  bind_irq_name_to_cpu ehci_hcd:usb4 3

  log "完成"
  show_status
}

main "$@"
