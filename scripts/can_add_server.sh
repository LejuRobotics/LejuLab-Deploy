#!/bin/sh
### BEGIN INIT INFO
# Provides:          can_add_server.sh
# Required-start:    $local_fs $remote_fs $network $syslog
# Required-Stop:     $local_fs $remote_fs $network $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: init can
# Description:       init can
### END INIT INFO

sleep 0.5

CAN_NAME_PREFIX="bcan"

sudo ip link set ${CAN_NAME_PREFIX}0 type can bitrate 1000000 sample-point 0.80 dbitrate 5000000 dsample-point 0.750 fd on loopback off
sudo ip link set up ${CAN_NAME_PREFIX}0
sudo ip link set ${CAN_NAME_PREFIX}1 type can bitrate 1000000 sample-point 0.80 dbitrate 5000000 dsample-point 0.750 fd on loopback off
sudo ip link set up ${CAN_NAME_PREFIX}1
sudo ip link set ${CAN_NAME_PREFIX}2 type can bitrate 1000000 sample-point 0.800 loopback off
sudo ip link set up ${CAN_NAME_PREFIX}2
sudo ip link set ${CAN_NAME_PREFIX}3 type can bitrate 1000000 sample-point 0.800 loopback off
sudo ip link set up ${CAN_NAME_PREFIX}3

sudo ifconfig ${CAN_NAME_PREFIX}0 txqueuelen 5000
sudo ifconfig ${CAN_NAME_PREFIX}1 txqueuelen 5000
sudo ifconfig ${CAN_NAME_PREFIX}2 txqueuelen 5000
sudo ifconfig ${CAN_NAME_PREFIX}3 txqueuelen 5000

# 将 CAN/SPI 中断分摊到小核 0-3（高负载 CANFD 避开 CPU0）
/home/lab/lejulab_platform_zx/scripts/bind_little_core_irqs.sh

exit 0
