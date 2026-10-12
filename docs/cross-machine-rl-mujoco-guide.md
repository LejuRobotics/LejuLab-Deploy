# RK3588 RL 推理 + X86 MuJoCo 仿真 — 跨机 UDP 通信操作指南

## 架构概览

```
┌─────────────────────┐          UDP (CycloneDDS)          ┌─────────────────────┐
│   X86 开发机         │  ◄──── /rt/joint_cmd ──────────   │   RK3588 板         │
│                     │  ───── /rt/joint_state ────────►   │                     │
│  leju-mujoco-sim   │  ───── /rt/imu_state  ────────►   │  run_rl_controller  │
│  (物理仿真+可视化)    │  ◄──── /rt/joy       ──────────   │  (RL 策略推理)       │
│                     │                                    │                     │
│  leju-joystick      │                                    │                     │
│  (手柄输入)          │                                    │                     │
└─────────────────────┘                                    └─────────────────────┘
       192.168.50.x                                              192.168.50.80
```

**核心思路**: MuJoCo 仿真和手柄运行在 X86 上提供可视化和输入，RL 推理运行在 3588 上（模拟实机部署）。双方通过 CycloneDDS UDP multicast 自动发现并通信。

---

## 前置条件

| 项目 | X86 开发机 | RK3588 板 |
|------|-----------|----------|
| 网络 | 同一局域网 (192.168.50.x) | 192.168.50.80 或 .242 |
| 编译选项 | `BUILD_MUJOCO_SIM=ON` | `BUILD_RL_CONTROLLER=ON` |
| 推理引擎 | 不需要 | ONNX Runtime (aarch64) |
| iceoryx | **不需要** (用 UDP) | **不需要** (用 UDP) |

---

## Step 1: 准备 CycloneDDS UDP 配置文件

两边都使用 `cyclonedds_udp.xml`，内容如下:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface autodetermine="true" priority="default" multicast="default" />
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
            <EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints>
        </Discovery>
    </Domain>
</CycloneDDS>
```

**关键区别**:
- `cyclonedds_shm.xml` — 共享内存模式，接口绑 `lo`，仅本机通信
- `cyclonedds.xml` — UDP 模式，但接口绑 `lo`，也是仅本机
- **`cyclonedds_udp.xml`** — UDP 模式，`autodetermine="true"` 自动选择物理网卡，**支持跨机通信**

> 如果两台机器不在同一个 multicast 域（例如跨网段），需要把 `autodetermine="true"` 替换为指定网卡名:
> ```xml
> <NetworkInterface name="eth0" multicast="default" />
> ```

---

## Step 2: 编译

### X86 开发机 (MuJoCo + Joystick)

```bash
cd /home/zhongxu/work/master/lejulab_platform_zx

cmake -Bbuild_cmake -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_MUJOCO_SIM=ON \
    -DBUILD_JOYSTICK=ON \
    -DBUILD_RL_CONTROLLER=OFF \
    -DBUILD_TESTS=OFF

cmake --build build_cmake -j$(nproc)
```

### RK3588 板 (RL Controller)

```bash
ssh test@192.168.50.xx
cd /home/test/lejulab_platform_zx

CC=gcc-11 CXX=g++-11 cmake -Bbuild -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_RL_CONTROLLER=ON \
    -DBUILD_MUJOCO_SIM=OFF \
    -DBUILD_JOYSTICK=OFF \
    -DBUILD_TESTS=OFF

CC=gcc-11 CXX=g++-11 cmake --build build -j4
```

---

## Step 3: 启动 (顺序很重要)

### 3.1 X86 — 启动 MuJoCo 仿真 + 手柄

```bash
cd /home/zhongxu/work/master/lejulab_platform_zx

# 关键: 使用 UDP 配置，不走共享内存
export CYCLONEDDS_URI="file://$(pwd)/src/leju_launch/config/cyclonedds_udp.xml"
export ROBOT_VERSION=14    # 与 3588 上的机器人版本一致

# 启动 MuJoCo 仿真器
./build_cmake/src/leju-mujoco-sim/leju-mujoco-sim \
    ./src/leju_assets/models/biped_s${ROBOT_VERSION}/xml/scene_rl.xml &

# 启动手柄 (可选, 用于发送速度指令)
./build_cmake/src/leju-joystick/leju-joystick &
```

### 3.2 RK3588 — 启动 RL 推理控制器

```bash
ssh test@192.168.50.80
cd /home/test/lejulab_platform_zx

# 关键: 使用 UDP 配置，不走共享内存
export CYCLONEDDS_URI="file://$(pwd)/src/leju_launch/config/cyclonedds_udp.xml"
export ROBOT_VERSION=14

# 方式 A: Controller Manager 模式 (生产模式，支持多控制器切换)
./build/src/leju-controllers/leju-rl-controller/run_rl_controller \
    ./src/leju-controllers/leju-rl-controller/config/${ROBOT_VERSION}/controller_manager.yaml

# 方式 B: Demo 模式 (单策略直接推理，调试用，需要 OpenVINO)
# ./build/src/leju-controllers/leju-rl-controller/run_rl_demo_controller \
#     ./src/leju-controllers/leju-rl-controller/config/${ROBOT_VERSION}/config_amp.yaml
```

### 或者用 launch 脚本 (更方便)

**X86 只启动仿真部分:**
```bash
export ROBOT_VERSION=14

# --no-shm 会使用 cyclonedds.xml (loopback)，我们需要手动覆盖为 UDP
export CYCLONEDDS_URI="file://$(pwd)/src/leju_launch/config/cyclonedds_udp.xml"

# 手动启动 MuJoCo + Joystick
./src/leju_launch/scripts/start_node.sh \
    ./build_cmake/src/leju-mujoco-sim/leju-mujoco-sim \
    ./src/leju_assets/models/biped_s${ROBOT_VERSION}/xml/scene_rl.xml &

./src/leju_launch/scripts/start_node.sh \
    ./build_cmake/src/leju-joystick/leju-joystick &
```

---

## Step 4: 验证通信

### 检查 DDS 发现

两边都能看到对方的 participant 就说明 UDP multicast 通路正常。

```bash
# 检查 MuJoCo 是否在发布 joint_state (在任一机器上)
# 看进程日志输出中有无 topic 订阅/发布信息
```

### 网络排查

```bash
# 1. 确认双方能 ping 通
ping 192.168.50.80

# 2. 确认 multicast 路由存在
ip route show | grep multicast
# 如果没有，手动添加:
sudo ip route add 239.255.0.0/16 dev eth0

# 3. 检查防火墙是否放行 UDP multicast
sudo iptables -L -n | grep -i udp
# CycloneDDS 默认使用端口 7400+ (RTPS), 确保未被防火墙拦截
```

### 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 双方无法发现 | multicast 被网络设备屏蔽 | 检查交换机/路由器 multicast 设置，或改用 unicast peer list |
| 发现成功但无数据 | ROBOT_VERSION 不一致导致 topic 名不匹配 | 两边 `ROBOT_VERSION` 必须一致 |
| "RouDi not running" 报错 | `CYCLONEDDS_URI` 指向了 shm 配置 | 确认使用 `cyclonedds_udp.xml` |
| MuJoCo 窗口卡顿 | RL 控制器未启动，仿真等待命令 | 先启动 MuJoCo，再启动 RL |

---

## 配置参考

### RL 推理参数 (config/14/config_amp.yaml)

| 参数 | 值 | 说明 |
|------|-----|------|
| inference_engine | `onnxruntime` | 3588 用 ONNX Runtime (aarch64) |
| loop_dt | 0.001 (1kHz) | 主循环频率 |
| policy_dt | 0.02 (50Hz) | 策略推理频率 |
| 关节数 | 21 (1腰+12腿+8臂) | Roban v14 布局 |
| 速度指令范围 | x: [-0.5, 0.5], y: [-0.5, 0.5] m/s | 手柄控制 |

### DDS Topic 列表

| Topic | 方向 | 说明 |
|-------|------|------|
| `/rt/joint_state` | MuJoCo → RL | 关节状态 (位置/速度/力矩) |
| `/rt/imu_state` | MuJoCo → RL | IMU 数据 (角速度/加速度/姿态) |
| `/rt/joint_cmd` | RL → MuJoCo | 关节控制指令 |
| `/rt/joy` | Joystick → RL | 手柄速度指令 |

---

## 一键停止

```bash
# X86
pkill -f leju-mujoco-sim
pkill -f leju-joystick

# RK3588
pkill -f run_rl_controller
```
