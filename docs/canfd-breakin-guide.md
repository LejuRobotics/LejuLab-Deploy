# CANFD 全身电机磨线工具使用说明

## 概述

`canfd_breakin` 是一个全身电机磨线（break-in）工具，用于新电机线束磨合。程序驱动机器人各关节按预定义的关键帧动作序列做重复运动，使线束内部导线与绝缘层充分贴合。

### 支持的总线和电机

总线布局随机器人版本不同，配置文件分版本提供：

**Roban v14**（`leg_breakin_roban2_v14.yaml`）

| 总线 | 协议 | 频率 | 电机 |
|------|------|------|------|
| bcan0 | CANFD 广播 | 500Hz | 左腿 joint 1-6 |
| bcan1 | CANFD 广播 | 500Hz | 右腿 joint 1-6 + 腰 joint 7 |
| bcan2 | CAN 单帧 | 250Hz | 左臂 joint 1-4 + 头 yaw/pitch |
| bcan3 | CAN 单帧 | 250Hz | 右臂 joint 5-8 |

**Roban v17**（`leg_breakin_roban2_v17.yaml`，全身：腿+腰+臂+头）

v17 的**总线 / 电机 id / kp-kd 不在 yaml 硬编码**，而是运行时从部署配置
`~/.config/lejuconfig/canbus_device_cofig.yaml` 读取（`device_config` 字段）。程序按设备名
`Lleg_joint_/Rleg_joint_/Waist_` 解析各总线电机 id 与 kp/kd（`params` 字段），自动得到关节顺序
并按关节序配对左右腿镜像。yaml 只定义**动作关键帧 + 安全限位**。

真机实测拓扑（以 device_config 为准）：

| 总线 | 协议 | 电机 (device_id) |
|------|------|------|
| bcan0 | CANFD 广播 | **腰=id1** + 左腿 Lleg01-06=**id2-07** |
| bcan1 | CANFD 广播 | 右腿 Rleg01-06=id1-06 |
| bcan2 | CAN 单帧 | 左臂 id1-04 + 头(0x09,0x0A) |
| bcan3 | CAN 单帧 | 右臂 id5-08 |

> 注意：腰是 **id1（最前）**，左腿是 id2-07，与"腿1-6+腰7"的直觉相反；kp/kd 同 id 跨总线
> （腰 id1 vs 右腿 id1）不同，故程序按总线分别保存增益。这正是改为读 device_config 的原因。

v17 左腿关节顺序（依据 `biped_s17/.../biped_v3.urdf` 关节轴，与 v14 不同）：

| 设备序 | URDF | 语义 | v17 限位(°) |
|--------|------|------|------------|
| Lleg_joint_01 | leg_l1 | hip_pitch（斜轴） | ±190 |
| Lleg_joint_02 | leg_l2 | hip_roll | ±180 |
| Lleg_joint_03 | leg_l3 | hip_yaw | ±180 |
| Lleg_joint_04 | leg_l4 | knee（单向，非负） | [-10,180] |
| Lleg_joint_05 | leg_l5 | ankle_pitch | ±90 |
| Lleg_joint_06 | leg_l6 | ankle_roll | ±95 |

> ⚠️ v17 注意：
> - **腰挂在左腿总线（bcan0）**，与 v14（腰在右腿总线）相反。device_config 自动识别腰所在总线。
> - **臂/头一起磨**：`MotorevoActuator` 按总线 `protocol` 分发，bcan2/bcan3 走 **CAN 单帧** 驱动臂/头（kuavo.json 标 `ruiwo`，由单帧路径驱动，与 v14 同机制）。臂的 `arm.canbus_config` 只能含 bcan2/bcan3（否则与腿控制器抢 bcan0/1）；`arm_test_canbus_cofig.yaml` 的臂 id 与真机一致，直接可用。只磨腿+腰加 `--no-arm`。
> - v17 关节顺序与 v14 不同（v17 有 hip_yaw、单膝；v14 是双膝 knee+knee2），关键帧已按 URDF 重排。
> - **kp/kd 全部来自 device_config**（= kuavo.json `ruiwo_kp/kd`）：腿/腰按总线电机读，臂/头按 MotorevoActuator 索引逐个读（臂 14.25/0.907，头Y 10/1，头P 5/1）。yaml 内 `arm_kp/head_kp` 仅为未用 device_config 时的回退。
> - 臂幅度落在 v17 臂限位内（肘 Larm4/Rarm4 `[-130,10]` 单向）。关键帧为保守起点，**上板前现场核对**镜像取反方向。

## 编译

磨线程序位于 motorevo_controller 的 examples 目录下，随整体工程一起编译：

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make canfd_breakin -j$(nproc)
```

编译产物路径：`build/src/leju-hardware/.../bin/canfd_breakin`（具体路径取决于 CMake 安装配置）。

## 便捷脚本（推荐）

`scripts/run_breakin.py` 封装了 `canfd_breakin`：自动定位二进制、按版本选配置、检查/拉起 CAN 总线，并在启动前做 root 检查 + 安全清单确认。`--` 之后的参数原样透传给 `canfd_breakin`。

```bash
# 检查配置（不需 root / CAN）
python3 scripts/run_breakin.py --version 17 -- --dry-run

# 短时间试跑
sudo python3 scripts/run_breakin.py --version 17 -- --duration 30

# 自动拉起 CAN 总线 + 运行 100 轮
sudo python3 scripts/run_breakin.py --version 17 --setup-can -- --rounds 100

# v14 只磨腿
sudo python3 scripts/run_breakin.py --version 14 -- --leg-only
```

脚本选项：

| 选项 | 说明 |
|------|------|
| `--version <17\|14>` | 选版本对应的磨线配置（默认 17） |
| `--config <file>` | 直接指定 YAML（覆盖 `--version`） |
| `--binary <path>` | 直接指定二进制（覆盖自动定位） |
| `--device-config <file>` | 部署 canbus 配置（默认 `~/.config/lejuconfig/canbus_device_cofig.yaml`，v17 默认启用） |
| `--no-device-config` | 不读部署配置，用 yaml 内手写拓扑 |
| `--setup-can` | 用 `ip link` 拉起所需总线（比特率取自 device_config，腿 FD 1M/5M、臂 1M） |
| `-- <参数...>` | 透传给 `canfd_breakin`（见下方完整参数列表） |

> 运行前请先编译 `canfd_breakin`（见上方「编译」），脚本会自动在 `build_cmake` / `build` / `build_full` 中查找二进制。
> v17 默认从 device_config 读总线/电机 id/kp-kd 并据此拉起 CAN；总线名与比特率都来自该文件。

## 使用方法（直接调用 canfd_breakin）

### 基本用法

```bash
# 全身磨线（腿+腰+臂+头），运行 30 分钟
sudo ./canfd_breakin --config config/canfd_breakin/leg_breakin_roban2_v14.yaml --duration 1800

# 运行 100 轮（自动计算时长）
sudo ./canfd_breakin --config config/canfd_breakin/leg_breakin_roban2_v14.yaml --rounds 100

# 只检查配置，不实际运行
sudo ./canfd_breakin --config config/canfd_breakin/leg_breakin_roban2_v14.yaml --dry-run
```

### 分区域磨线

```bash
# 只磨腿（不含腰和手臂）
sudo ./canfd_breakin --config config/canfd_breakin/leg_breakin_roban2_v14.yaml --leg-only

# 只磨手臂和头部
sudo ./canfd_breakin --config config/canfd_breakin/leg_breakin_roban2_v14.yaml --arm-only

# 腿+腰，不磨手臂
sudo ./canfd_breakin --config config/canfd_breakin/leg_breakin_roban2_v14.yaml --no-arm

# 腿+手臂，不磨腰
sudo ./canfd_breakin --config config/canfd_breakin/leg_breakin_roban2_v14.yaml --no-waist
```

### 完整参数列表

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--config <file>` | YAML 配置文件路径（必须） | - |
| `--device-config <file>` | 部署 canbus 配置，按总线读腿/腰 id 与 kp/kd（覆盖 yaml 内 canbus/joint_gains） | - |
| `--arm-config <file>` | 覆盖臂 `canbus_config`（仅 bcan2/3） | yaml 值 |
| `--duration <秒>` | 运行总时长 | 600（10分钟） |
| `--rounds <次>` | 运行轮数（与 --duration 二选一） | - |
| `--log <file.csv>` | CSV 日志路径 | breakin_YYYYMMDD_HHMMSS.csv |
| `--no-log` | 不记录 CSV 日志 | false |
| `--no-waist` | 不控制腰部电机 | false |
| `--no-arm` | 不控制手臂/头部电机 | false |
| `--leg-only` | 只控制腿部（等价 --no-waist --no-arm） | false |
| `--arm-only` | 只控制手臂/头部 | false |
| `--kp <值>` | 覆盖腿部所有关节 kp | 配置文件值 |
| `--kd <值>` | 覆盖腿部所有关节 kd | 配置文件值 |
| `--hz <频率>` | 腿部控制频率 | 500 |
| `--dry-run` | 只打印配置，不实际控制 | false |

## 配置文件说明

配置文件 `config/canfd_breakin/leg_breakin_roban2_v14.yaml` 包含以下几个部分：

### 0. 设备配置 (`device_config`，v17 推荐)

顶层字段 `device_config:` 指向部署的 `~/.config/lejuconfig/canbus_device_cofig.yaml`
（支持 `~`/`$HOME` 展开）。指定后，程序用 `canbus_sdk::ConfigParser` 按设备名
`Lleg_joint_/Rleg_joint_/Waist_` 解析各总线的电机 id 与 kp/kd（`params[1]/[2]`），
**覆盖** 下面手写的 `canbus` 段与 `control.joint_gains`，并按关节序自动配对左右腿镜像。
这样腿/腰拓扑与增益始终与真机一致，且天然支持同 id 跨总线（腰 id1 vs 右腿 id1）不同增益。

### 1. CAN 总线配置 (`canbus`，未用 device_config 时)

定义腿部 CANFD 总线的接口名、电机数量和 ID 列表：

- `left_leg` / `right_leg`：各总线的 `interface`、`motor_count`、`motor_ids`
- `waist`（可选）：`interface` + `motor_id` 指定腰所在总线
  - **不配 `waist` 段**（v14）：腰挂右腿总线，靠 `motor_id > left_leg.motor_count` 自动识别（如 bcan1 上的 id 7）
  - **`waist.interface == left_leg.interface`**（v17）：腰挂左腿总线（bcan0），由左腿循环用 `waist.motor_id` 驱动。此时该 id 需同时列入 `left_leg.motor_ids` 并计入 `motor_count`，以便初始化时一并使能

### 2. 控制参数 (`control`)

- `frequency_hz`: 腿部控制频率（默认 500Hz）
- `joint_gains`: 各关节独立的 kp/kd 增益
  - 大关节（hip/knee）: kp=150, kd=2.0
  - 脚踝: kp=80, kd=1.5

### 3. 腿部动作 (`action`)

- `left_leg_keyframes`: 左腿 6 个关节的关键帧序列（单位：度）
  - 11 个关键帧，每帧 1 秒过渡
  - 单轮周期 = 11 秒
- `right_leg_mirror`: 右腿从左腿镜像映射，可配置是否取反
- `waist_keyframes`: 腰部独立关键帧

### 4. 手臂/头配置 (`arm`)

- `canbus_config`: MotorevoActuator 使用的 CAN 配置文件路径
- `left_arm_keyframes`: 左臂 4 关节关键帧（单位：弧度）
  - 7 帧，每帧 1.9 秒过渡
  - 单轮周期 = 13.3 秒
- `right_arm_mirror`: 右臂镜像映射
- `head_keyframes`: 头部 2 关节关键帧
- `arm_kp/kd`, `head_kp/kd`: 手臂/头部增益

### 5. 安全限位 (`safety`)

- `max_offset_deg`: 腿部各关节最大偏移量（度）
- `arm_max_offset_rad`: 手臂最大偏移量（弧度）
- `head_max_offset_rad`: 头部最大偏移量（弧度）
- `stale_threshold`: 通信丢失检测阈值
- `max_temperature`: 温度保护阈值

## 安全机制

1. **位置限位**: 每个关节有独立的最大偏移量，超过立即停止
2. **通信丢失检测**: 连续 N 帧反馈位置不变时紧急停止
3. **平滑回零**: 停止时先用 minimum-jerk 插值平滑回零（2秒），再失能电机
4. **Ctrl+C 安全停止**: 收到 SIGINT/SIGTERM 后执行回零+失能流程

## 运行前检查清单

1. **吊起机器人**，确保各关节处于零点位置
2. **锁住吊架万向环**，防止机器人旋转
3. **清除周围障碍物**，确保运动范围内无干涉
4. 确认 CAN 总线接口已正确配置（`canbus_config.sh` 或 `ip link set`）
5. 建议先用 `--dry-run` 检查配置是否正确
6. 建议先用较短时间（如 `--duration 30`）试运行，确认动作正常

## CSV 日志格式

日志文件记录 10Hz 采样数据，包含：

| 列 | 说明 |
|----|------|
| `time_s` | 运行时间（秒） |
| `round` | 当前轮次 |
| `L{n}_tgt` | 左腿关节 n 目标位置（度） |
| `L{n}_act` | 左腿关节 n 实际位置（度） |
| `L{n}_tor` | 左腿关节 n 力矩（Nm） |
| `R{n}_tgt/act/tor` | 右腿/腰各关节数据 |
| `LA{n}_tgt/act/tor` | 左臂各关节数据 |
| `RA{n}_tgt/act/tor` | 右臂各关节数据 |
| `HY/HP_tgt/act/tor` | 头部 yaw/pitch 数据 |

## 自定义动作

如需修改磨线动作，编辑 YAML 配置文件中的关键帧数组即可。注意：

- 腿部关键帧单位是**度**
- 手臂/头部关键帧单位是**弧度**
- 修改后建议先 `--dry-run` 检查，再短时间试运行
- 确保新动作不超过 `safety` 中定义的安全限位

## 故障排查

| 问题 | 原因 | 解决方法 |
|------|------|----------|
| CAN 总线打开失败 | 接口未配置 | 运行 `scripts/canbus_config.sh` |
| 电机初始化失败 | 电机未上电或接线问题 | 检查电源和 CAN 线缆 |
| 通信丢失紧急停止 | CAN 总线干扰或线缆松动 | 检查线缆连接，降低 `stale_threshold` |
| 安全限位停止 | 关键帧角度超限 | 调整配置文件中的关键帧或限位值 |
| 手臂初始化失败 | `arm_test_canbus_cofig.yaml` 路径错误 | 检查配置中 `canbus_config` 路径 |
