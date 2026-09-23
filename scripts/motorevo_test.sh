#!/bin/bash
# Motorevo 关节 CAN FD 控制测试脚本
# 协议: Motorevo 关节 CanFd 协议 V1.0.1 (2025.11.26)
# 接口: bcan2, CAN FD 1Mbps + 5Mbps BRS

set -e

CAN_IF="bcan2"

# Motorevo MIT 参数范围 (电机固件实际值, 协议文档 V1.0.1 有误)
POS_MIN=-12.5
POS_MAX=12.5
VEL_MIN=-10.0
VEL_MAX=10.0
KP_MIN=0.0
KP_MAX=250.0
KD_MIN=0.0
KD_MAX=50.0
TOR_MIN=-50.0
TOR_MAX=50.0

# Motorevo CAN IDs (标准帧, 64字节广播)
CAN_ID_CTRL="010"   # 0x10 控制帧 (使能/失能/零点/清错)
CAN_ID_MIT="020"    # 0x20 MIT模式
CAN_ID_POSVEL="040" # 0x40 位置速度模式
CAN_ID_VEL="060"    # 0x60 速度模式

# CAN FD BRS 标志
CANFD_BRS="##1"

# ============================================================
# 工具函数
# ============================================================

float_to_uint() {
    local val=$1 min=$2 max=$3 bits=$4
    python3 -c "
v = max(min($val, $max), $min)
span = $max - $min
r = int((v - $min) * ((1 << $bits) - 1) / span)
print(r)
"
}

uint_to_float() {
    local raw=$1 min=$2 max=$3 bits=$4
    python3 -c "
span = $max - $min
print(round($raw * span / ((1 << $bits) - 1) + $min, 4))
"
}

# 构造 MIT 控制帧 8 字节 (大端序)
build_mit_frame() {
    local pos_rad=$1 vel_rads=$2 kp=$3 kd=$4 tor_nm=$5

    local pos_raw=$(float_to_uint "$pos_rad" $POS_MIN $POS_MAX 16)
    local vel_raw=$(float_to_uint "$vel_rads" $VEL_MIN $VEL_MAX 12)
    local kp_raw=$(float_to_uint "$kp" $KP_MIN $KP_MAX 12)
    local kd_raw=$(float_to_uint "$kd" $KD_MIN $KD_MAX 12)
    local tor_raw=$(float_to_uint "$tor_nm" $TOR_MIN $TOR_MAX 12)

    python3 -c "
pos = $pos_raw & 0xFFFF
vel = $vel_raw & 0xFFF
kp  = $kp_raw  & 0xFFF
kd  = $kd_raw  & 0xFFF
tor = $tor_raw & 0xFFF

b0 = pos >> 8
b1 = pos & 0xFF
b2 = vel >> 4
b3 = ((vel & 0xF) << 4) | (kp >> 8)
b4 = kp & 0xFF
b5 = kd >> 4
b6 = ((kd & 0xF) << 4) | (tor >> 8)
b7 = tor & 0xFF

print('.'.join(f'{x:02X}' for x in [b0,b1,b2,b3,b4,b5,b6,b7]))
"
}

# MIT 零输出帧 (pos=0, vel=0, kp=0, kd=0, tor=0)
ZERO_FRAME="7F.FF.7F.F0.00.00.07.FF"

# 填充64字节: 关节1数据 + 关节2-8 零
build_broadcast() {
    local joint1=$1 fill=${2:-$ZERO_FRAME}
    local frame="$joint1"
    for i in $(seq 2 8); do
        frame="${frame}.${fill}"
    done
    echo "$frame"
}

# 发送 CAN FD BRS 帧
send() {
    cansend "$CAN_IF" "${1}${CANFD_BRS}${2}"
}

# 使能电机 (CAN ID 0x10)
enable_motor() {
    local cmd="FF.FF.FF.FF.FF.FF.FF.FC"
    send "$CAN_ID_CTRL" "$(build_broadcast "$cmd" "$cmd")"
}

# 失能电机 (CAN ID 0x10)
disable_motor() {
    local cmd="FF.FF.FF.FF.FF.FF.FF.FD"
    send "$CAN_ID_CTRL" "$(build_broadcast "$cmd" "$cmd")"
}

# 清除错误 (CAN ID 0x10)
clear_errors() {
    local cmd="FF.FF.FF.FF.FF.FF.FF.FB"
    send "$CAN_ID_CTRL" "$(build_broadcast "$cmd" "$cmd")"
}

# 保存零点 (CAN ID 0x10)
save_zero() {
    local cmd="FF.FF.FF.FF.FF.FF.FF.FE"
    send "$CAN_ID_CTRL" "$(build_broadcast "$cmd" "$cmd")"
}

# 解析 Motorevo 反馈帧 (8字节)
parse_feedback() {
    local hex_str=$1
    python3 -c "
data = [int(x, 16) for x in '$hex_str'.split()]
n = len(data)

pos_raw = (data[0] << 8) | data[1]
vel_raw = (data[2] << 4) | (data[3] >> 4)
tor_raw = ((data[3] & 0xF) << 8) | data[4]

pos = pos_raw * 25.0 / 65535 + (-12.5)
vel = vel_raw * 130.0 / 4095 + (-65.0)
tor = tor_raw * 60.0 / 4095 + (-30.0)

temp = data[5] - 40 if n > 5 else 'N/A'
status = (data[6] | (data[7] << 8)) if n > 7 else 0
enabled = '是' if (status & 1) else '否'

fault_names = {1:'过压', 2:'过流', 3:'过温', 4:'超速',
               5:'编码器故障', 6:'预驱故障', 7:'过载', 8:'堵转', 9:'超差', 10:'厂家自定义'}
faults = [name for bit, name in fault_names.items() if status & (1 << bit)]

print(f'  位置: {pos:.3f} rad ({pos*180/3.14159:.1f}°)')
print(f'  速度: {vel:.3f} rad/s')
print(f'  力矩: {tor:.3f} Nm')
print(f'  温度: {temp}°C')
print(f'  使能: {enabled}')
print(f'  故障: {\", \".join(faults) if faults else \"无\"}')
print(f'  状态字: 0x{status:04X}')
"
}

# 读取当前位置 (从最近的反馈帧)
# 返回格式: pos_rad
get_current_pos() {
    # 发使能获取一次反馈, 再失能
    enable_motor
    sleep 0.1
    disable_motor
    # 提示用户查看 candump
}

# ============================================================
# 交互式控制
# ============================================================

cmd_interactive() {
    echo "============================================"
    echo "  Motorevo 关节电机交互式控制"
    echo "  协议: CanFd V1.0.1 | 接口: $CAN_IF"
    echo "============================================"
    echo ""
    echo "请选择控制模式:"
    echo ""
    echo "  1) 位置控制 (MIT模式, Kp+Kd)"
    echo "  2) 力矩控制 (MIT模式, 纯力矩)"
    echo "  3) 位置速度控制 (梯形加减速)"
    echo "  4) 速度控制"
    echo "  5) 使能/失能/状态"
    echo "  q) 退出"
    echo ""
    read -rp "选择 [1-5/q]: " mode_choice

    case "$mode_choice" in
        1) interactive_position ;;
        2) interactive_torque ;;
        3) interactive_posvel ;;
        4) interactive_velocity ;;
        5) interactive_basic ;;
        q|Q) echo "退出"; exit 0 ;;
        *) echo "无效选择"; exit 1 ;;
    esac
}

interactive_position() {
    echo ""
    echo "--- 位置控制 (MIT模式) ---"
    echo "参数范围: 位置 [-12.5~12.5] rad, Kp [0~500], Kd [0~5]"
    echo ""

    read -rp "目标位置 (rad): " target_pos
    read -rp "Kp 刚度 [默认 5]: " kp
    read -rp "Kd 阻尼 [默认 1.0]: " kd
    read -rp "发送帧数 [默认 50]: " n_frames
    read -rp "帧间隔秒 [默认 0.02]: " interval

    kp=${kp:-5}
    kd=${kd:-1.0}
    n_frames=${n_frames:-50}
    interval=${interval:-0.02}

    echo ""
    echo "[位置控制] 目标=${target_pos} rad, Kp=${kp}, Kd=${kd}"
    echo "  帧数=${n_frames}, 间隔=${interval}s, 总时长≈$(python3 -c "print(f'{$n_frames * $interval:.1f}')")s"

    local frame=$(build_mit_frame "$target_pos" 0 "$kp" "$kd" 0)
    local broadcast=$(build_broadcast "$frame")
    echo "  MIT帧: $frame"
    echo ""

    read -rp "确认执行? [y/N]: " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "已取消"
        return
    fi

    echo "[1/3] 使能..."
    enable_motor
    sleep 0.02

    echo "[2/3] 位置控制 (0x20) × ${n_frames}..."
    for i in $(seq 1 "$n_frames"); do
        send "$CAN_ID_MIT" "$broadcast"
        sleep "$interval"
    done

    sleep 0.05
    echo "[3/3] 失能..."
    disable_motor
    echo "[位置控制] 完成!"
}

interactive_torque() {
    echo ""
    echo "--- 力矩控制 (MIT模式, kp=0, kd=0) ---"
    echo "参数范围: 力矩 [-30~30] Nm"
    echo "注意: 电机会持续输出指定力矩, 无位置/速度反馈控制"
    echo ""

    read -rp "力矩 (Nm) [默认 0.5]: " torque
    read -rp "持续时间 (秒) [默认 0.5]: " duration
    read -rp "帧间隔秒 [默认 0.02]: " interval

    torque=${torque:-0.5}
    duration=${duration:-0.5}
    interval=${interval:-0.02}

    local n_frames=$(python3 -c "import math; print(max(1, math.ceil($duration / $interval)))")

    echo ""
    echo "[力矩控制] 力矩=${torque} Nm, 持续=${duration}s (${n_frames}帧)"

    local frame=$(build_mit_frame 0 0 0 0 "$torque")
    local broadcast=$(build_broadcast "$frame")
    local zero_broadcast=$(build_broadcast "$ZERO_FRAME")
    echo "  MIT帧: $frame"
    echo ""

    read -rp "确认执行? [y/N]: " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "已取消"
        return
    fi

    echo "[1/4] 使能..."
    enable_motor
    sleep 0.02

    echo "[2/4] 力矩输出 (0x20) × ${n_frames}..."
    for i in $(seq 1 "$n_frames"); do
        send "$CAN_ID_MIT" "$broadcast"
        sleep "$interval"
    done

    echo "[3/4] 释放力矩 (零输出)..."
    for i in $(seq 1 5); do
        send "$CAN_ID_MIT" "$zero_broadcast"
        sleep 0.02
    done

    sleep 0.05
    echo "[4/4] 失能..."
    disable_motor
    echo "[力矩控制] 完成!"
}

interactive_posvel() {
    echo ""
    echo "--- 位置速度控制 (梯形加减速, CAN ID 0x40) ---"
    echo "电机内部自动规划加减速轨迹"
    echo ""

    read -rp "目标位置 (rad): " target_pos
    read -rp "运行速度 (rad/s) [默认 0.5]: " target_vel
    read -rp "发送帧数 [默认 50]: " n_frames
    read -rp "帧间隔秒 [默认 0.02]: " interval

    target_vel=${target_vel:-0.5}
    n_frames=${n_frames:-50}
    interval=${interval:-0.02}

    echo ""
    echo "[位置速度] pos=${target_pos} rad, vel=${target_vel} rad/s"

    local pos_hex vel_hex
    pos_hex=$(python3 -c "import struct; print('.'.join(f'{x:02X}' for x in struct.pack('<f', $target_pos)))")
    vel_hex=$(python3 -c "import struct; print('.'.join(f'{x:02X}' for x in struct.pack('<f', $target_vel)))")

    local joint1="${pos_hex}.${vel_hex}"
    local broadcast=$(build_broadcast "$joint1" "00.00.00.00.00.00.00.00")
    echo "  数据: pos=$pos_hex vel=$vel_hex (little-endian float)"
    echo ""

    read -rp "确认执行? [y/N]: " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "已取消"
        return
    fi

    echo "[1/3] 使能..."
    enable_motor
    sleep 0.02

    echo "[2/3] 位置速度控制 (0x40) × ${n_frames}..."
    for i in $(seq 1 "$n_frames"); do
        send "$CAN_ID_POSVEL" "$broadcast"
        sleep "$interval"
    done

    sleep 0.05
    echo "[3/3] 失能..."
    disable_motor
    echo "[位置速度] 完成!"
}

interactive_velocity() {
    echo ""
    echo "--- 速度控制 (CAN ID 0x60) ---"
    echo ""

    read -rp "目标速度 (rad/s) [默认 0.5]: " target_vel
    read -rp "持续时间 (秒) [默认 1.0]: " duration
    read -rp "帧间隔秒 [默认 0.02]: " interval

    target_vel=${target_vel:-0.5}
    duration=${duration:-1.0}
    interval=${interval:-0.02}

    local n_frames=$(python3 -c "import math; print(max(1, math.ceil($duration / $interval)))")

    echo ""
    echo "[速度控制] vel=${target_vel} rad/s, 持续=${duration}s (${n_frames}帧)"

    local vel_hex
    vel_hex=$(python3 -c "import struct; print('.'.join(f'{x:02X}' for x in struct.pack('<f', $target_vel)))")

    local joint1="${vel_hex}.00.00.00.00"
    local broadcast=$(build_broadcast "$joint1" "00.00.00.00.00.00.00.00")
    echo ""

    read -rp "确认执行? [y/N]: " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "已取消"
        return
    fi

    echo "[1/3] 使能..."
    enable_motor
    sleep 0.02

    echo "[2/3] 速度控制 (0x60) × ${n_frames}..."
    for i in $(seq 1 "$n_frames"); do
        send "$CAN_ID_VEL" "$broadcast"
        sleep "$interval"
    done

    sleep 0.05
    echo "[3/3] 失能..."
    disable_motor
    echo "[速度控制] 完成!"
}

interactive_basic() {
    echo ""
    echo "--- 基础操作 ---"
    echo "  1) 使能电机"
    echo "  2) 失能电机"
    echo "  3) 查看状态 (使能→读反馈→失能)"
    echo "  4) 清除错误"
    echo "  5) 保存零点"
    echo ""
    read -rp "选择 [1-5]: " choice

    case "$choice" in
        1) cmd_enable ;;
        2) cmd_disable ;;
        3) cmd_status ;;
        4) cmd_clear ;;
        5) cmd_savezero ;;
        *) echo "无效选择" ;;
    esac
}

# ============================================================
# 非交互命令
# ============================================================

usage() {
    cat <<'EOF'
用法: ./motorevo_test.sh [命令] [参数]

协议: Motorevo 关节 CanFd V1.0.1
CAN IDs: 0x10(控制) 0x20(MIT) 0x40(位置速度) 0x60(速度)

交互模式 (无参数启动):
  ./motorevo_test.sh              进入交互式菜单

命令行模式:
  enable                          使能电机
  disable                         失能电机
  clear                           清除错误
  savezero                        保存零点
  status                          使能→读反馈→失能

  move <pos> [kp] [kd] [tor] [n] [interval]
                                  MIT位置控制 (CAN ID 0x20)
  torque <tor> [duration] [interval]
                                  MIT力矩控制 (CAN ID 0x20)
  posvel <pos> <vel> [n] [interval]
                                  位置速度模式 (CAN ID 0x40)
  vel <speed> [n] [interval]      速度模式 (CAN ID 0x60)
  parse <hex...>                  解析反馈帧

参数范围:
  位置: [-12.5 ~ 12.5] rad    速度: [-65 ~ 65] rad/s
  Kp:   [0 ~ 500]             Kd:   [0 ~ 5]
  力矩: [-30 ~ 30] Nm

示例:
  ./motorevo_test.sh move 3.7 5 1.0       # 位置控制, Kp=5
  ./motorevo_test.sh torque 0.5 0.5       # 0.5Nm 持续 0.5s
  ./motorevo_test.sh posvel 3.7 0.5       # 位置速度模式
  ./motorevo_test.sh vel 0.5              # 速度模式

注意: 运行前请在另一个终端执行: candump bcan2 -ta -x
EOF
}

cmd_enable() {
    echo "[使能] CAN ID 0x10, BRS..."
    enable_motor
    echo "[使能] 完成"
}

cmd_disable() {
    echo "[失能] CAN ID 0x10, BRS..."
    disable_motor
    echo "[失能] 完成"
}

cmd_clear() {
    echo "[清错] CAN ID 0x10, BRS..."
    clear_errors
    echo "[清错] 完成"
}

cmd_savezero() {
    echo "[零点] CAN ID 0x10, BRS..."
    save_zero
    echo "[零点] 完成"
}

cmd_status() {
    echo "[状态] 使能 → 读反馈 → 失能"
    enable_motor
    sleep 0.2
    disable_motor
    echo "[状态] 完成, 请查看 candump 反馈帧"
}

cmd_move() {
    local target_pos=${1:?请指定目标位置 (rad) [-12.5~12.5]}
    local kp=${2:-5}
    local kd=${3:-1.0}
    local tor=${4:-0}
    local n_frames=${5:-50}
    local interval=${6:-0.02}

    echo "[MIT位置] 目标=${target_pos} rad, Kp=${kp}, Kd=${kd}, Tor=${tor}"

    local frame=$(build_mit_frame "$target_pos" 0 "$kp" "$kd" "$tor")
    local broadcast=$(build_broadcast "$frame")
    echo "  MIT帧: $frame"

    enable_motor
    sleep 0.02
    for i in $(seq 1 "$n_frames"); do
        send "$CAN_ID_MIT" "$broadcast"
        sleep "$interval"
    done
    sleep 0.05
    disable_motor
    echo "[MIT位置] 完成!"
}

cmd_torque() {
    local torque=${1:?请指定力矩 (Nm) [-30~30]}
    local duration=${2:-0.5}
    local interval=${3:-0.02}

    local n_frames=$(python3 -c "import math; print(max(1, math.ceil($duration / $interval)))")

    echo "[MIT力矩] 力矩=${torque} Nm, 持续=${duration}s (${n_frames}帧)"

    local frame=$(build_mit_frame 0 0 0 0 "$torque")
    local broadcast=$(build_broadcast "$frame")
    local zero_broadcast=$(build_broadcast "$ZERO_FRAME")

    enable_motor
    sleep 0.02

    for i in $(seq 1 "$n_frames"); do
        send "$CAN_ID_MIT" "$broadcast"
        sleep "$interval"
    done

    # 释放力矩
    for i in $(seq 1 5); do
        send "$CAN_ID_MIT" "$zero_broadcast"
        sleep 0.02
    done

    sleep 0.05
    disable_motor
    echo "[MIT力矩] 完成!"
}

cmd_posvel() {
    local target_pos=${1:?请指定目标位置 (rad)}
    local target_vel=${2:-0.5}
    local n_frames=${3:-50}
    local interval=${4:-0.02}

    local pos_hex vel_hex
    pos_hex=$(python3 -c "import struct; print('.'.join(f'{x:02X}' for x in struct.pack('<f', $target_pos)))")
    vel_hex=$(python3 -c "import struct; print('.'.join(f'{x:02X}' for x in struct.pack('<f', $target_vel)))")

    local joint1="${pos_hex}.${vel_hex}"
    local broadcast=$(build_broadcast "$joint1" "00.00.00.00.00.00.00.00")

    echo "[位置速度] pos=${target_pos} rad, vel=${target_vel} rad/s"

    enable_motor
    sleep 0.02
    for i in $(seq 1 "$n_frames"); do
        send "$CAN_ID_POSVEL" "$broadcast"
        sleep "$interval"
    done
    sleep 0.05
    disable_motor
    echo "[位置速度] 完成!"
}

cmd_vel() {
    local target_vel=${1:?请指定目标速度 (rad/s)}
    local n_frames=${2:-50}
    local interval=${3:-0.02}

    local vel_hex
    vel_hex=$(python3 -c "import struct; print('.'.join(f'{x:02X}' for x in struct.pack('<f', $target_vel)))")

    local joint1="${vel_hex}.00.00.00.00"
    local broadcast=$(build_broadcast "$joint1" "00.00.00.00.00.00.00.00")

    echo "[速度] vel=${target_vel} rad/s, ${n_frames}帧"

    enable_motor
    sleep 0.02
    for i in $(seq 1 "$n_frames"); do
        send "$CAN_ID_VEL" "$broadcast"
        sleep "$interval"
    done
    sleep 0.05
    disable_motor
    echo "[速度] 完成!"
}

cmd_parse() {
    local hex_str="$*"
    if [ -z "$hex_str" ]; then
        echo "用法: $0 parse <hex bytes>"
        exit 1
    fi
    echo "[解析] $hex_str"
    parse_feedback "$hex_str"
}

# ============================================================
# 主入口
# ============================================================

case "${1:-}" in
    "")        cmd_interactive ;;
    enable)    cmd_enable ;;
    disable)   cmd_disable ;;
    clear)     cmd_clear ;;
    savezero)  cmd_savezero ;;
    status)    cmd_status ;;
    move)      shift; cmd_move "$@" ;;
    torque)    shift; cmd_torque "$@" ;;
    posvel)    shift; cmd_posvel "$@" ;;
    vel)       shift; cmd_vel "$@" ;;
    parse)     shift; cmd_parse "$@" ;;
    -h|--help) usage ;;
    *)         usage ;;
esac
