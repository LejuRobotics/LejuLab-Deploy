#!/usr/bin/env bash
# set_can_irq_affinity.sh — 把 CAN 总线中断分散到小核 1-3
#
# 背景: RK3588 GIC 对 Level 中断只投递给 affinity 掩码中的第一个核,
# 默认 4 路 CAN 的硬中断、threaded irq 线程(SCHED_FIFO 50)和 NET_RX
# softirq 全部挤在 CPU0, 跳舞时 4路×500Hz 双向流量使 CPU0 占用 60%+。
#
# 分散原则: 避开 CPU0(DDS/系统杂务聚集), 避开大核 4-7(RL 控制/推理),
# 4 路轮转分配到 CPU 1/2/3。threaded irq 线程会自动跟随 effective 亲和。
#
# 幂等; 无 CAN 接口(x86 开发机/仿真)或无 root 时告警跳过, 不阻塞启动。

TARGET_CPUS=(1 2 3)

idx=0
found=0
while read -r irq name; do
    cpu=${TARGET_CPUS[$((idx % ${#TARGET_CPUS[@]}))]}
    if echo "${cpu}" > "/proc/irq/${irq}/smp_affinity_list" 2>/dev/null; then
        effective=$(cat "/proc/irq/${irq}/effective_affinity_list" 2>/dev/null)
        echo "[can-irq] irq ${irq} (${name}) -> CPU${cpu} (effective=${effective})"
        found=1
    else
        echo "[can-irq] WARN: 设置 irq ${irq} (${name}) 亲和失败 (需要 root?)" >&2
    fi
    idx=$((idx + 1))
done < <(awk '$NF ~ /^b?can[0-9]+$/ {gsub(":","",$1); print $1, $NF}' /proc/interrupts)

if [ "${found}" != "1" ]; then
    echo "[can-irq] 未发现 CAN 中断, 跳过"
fi
