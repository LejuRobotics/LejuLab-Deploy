# leju-vr-control

`leju-vr-control` 是面向使用者的 Quest3 VR 控制节点。它负责将 Quest3 的输入转换为机器人控制行为，包括手臂控制、头部控制和速度指令控制。

提供两种控制模式，均由 `leju_launch` 的 `vr_teleop.launch` 启动，通过 **`use_incremental_ik`** 一个参数区分：

- `use_incremental_ik:=true` — **增量控制**（Kuavo），按住 `grip` 做手臂增量控制
- `use_incremental_ik:=false` — **绝对控制**（Roban），由 Quest3 骨骼位姿做手臂绝对位姿跟随（**默认**）

## 功能概览

当前支持的主要功能：

- 手臂 VR 增量控制（Kuavo）
- 手臂 VR 绝对控制（Roban）
- 头部控制
- 速度指令控制

## 适用机型与限制

| 机型                 | 手臂控制 | 头部控制 | 速度控制 | 腰部控制 | 备注                              |
| -------------------- | -------- | -------- | -------- | -------- | --------------------------------- |
| `Kuavo4Pro(45/46)` | 增量     | 支持     | 支持     | 不支持   | 无腰部自由度                      |
| `Kuavo5(52)`       | 增量     | 支持     | 支持     | 不支持   | 暂不提供 VR 腰部控制              |
| `Roban2(14/17)`       | 绝对     | 不支持   | 支持     | 不支持   | 走路由控制器自身的 Quest 摇杆处理 |

## 启动前检查

建议启动前确认：

- `robot_version` 设置正确（Roban 绝对控制为 `17`）
- Quest3 与 PC 处于可通信网络环境
- 当前机器人处于安全环境，适合做 VR 遥操作测试

## 启动方式

统一经 `vr_teleop.launch` 启动（launch 会自动拉起 `leju-remote` 接收 Quest 数据），用 `use_incremental_ik` 区分模式：

```bash
# 绝对控制（Roban），use_incremental_ik 默认 false
roslaunch leju_launch vr_teleop.launch robot_version:=17

# 增量控制（Kuavo）
roslaunch leju_launch vr_teleop.launch use_incremental_ik:=true
```

### 启动参数（launch arg）

| 参数                   | 默认值                       | 说明                                                                 |
| ---------------------- | ---------------------------- | -------------------------------------------------------------------- |
| `use_incremental_ik` | `false`                    | `false`=绝对 IK（双手扳机启动）；`true`=增量 IK（`grip` 切换） |
| `robot_version`      | `$ROBOT_VERSION` 或 `46` | 机器人版本；Roban 绝对控制填 `17`                                  |
| `quest_ip`           | 空                           | Quest 的 IP；为空时等待 Quest 广播自动发现                           |

## 增量控制（Kuavo）

### 手臂模式切换

- `X+A`
  - 在 `Auto(1)` 和 `External(2)` 之间切换
- `X+B`
  - 在 `KeepPose(0)` 和 `External(2)` 之间切换

三个模式的效果如下：

| 模式         | 值    | 效果                                                     |
| ------------ | ----- | -------------------------------------------------------- |
| `KeepPose` | `0` | 保持当前手臂姿态，适合临时停在当前位置                   |
| `Auto`     | `1` | 交还给默认自动行为，手臂不再接受当前 VR 增量控制         |
| `External` | `2` | 进入 VR 外部控制模式，按住对应 `grip` 后可进行增量控制 |

### 手臂控制

- 左手 `grip`：控制左臂进入增量控制
- 右手 `grip`：控制右臂进入增量控制
- 双手 `grip`：可同时控制双臂

### 头部控制

头部姿态由头显姿态驱动。

### 按键逻辑

| 操作          | 功能                                         |
| ------------- | -------------------------------------------- |
| `X+Y`       | 进入安全停机/关节松弛状态                    |
| `X+A`       | 在 `Auto(1)` 与 `External(2)` 间切换     |
| `X+B`       | 在 `KeepPose(0)` 与 `External(2)` 间切换 |
| 左手 `grip` | 左臂进入增量控制                             |
| 右手 `grip` | 右臂进入增量控制                             |
| 左摇杆        | 速度平移                                     |
| 右摇杆        | 速度旋转                                     |

### 操作流程示例

1. `roslaunch leju_launch vr_teleop.launch use_incremental_ik:=true`
2. 按 `X+A` 切到 `External(2)`
3. 按住单侧或双侧 `grip`
4. 移动 Quest3 手柄开始控制手臂

> 初次按下 `grip` 时，需要实际手臂发生移动后才会进入有效增量跟随。

## 绝对控制（Roban2）

由 Quest3 骨骼位姿经 IK 求解，使手臂做**绝对位姿跟随**。绝对控制下，**双手扳机 OK 手势**即「进入外部控制并开始绝对跟随」，无需先按 `X+A`。

### 按键逻辑

| 操作             | 功能                                                                                      |
| ---------------- | ----------------------------------------------------------------------------------------- |
| 双手扳机 OK 手势 | 同时按下双手扳机（值 `> 0.5`）并保持约 1～2 秒，进入 `External(2)` 并开始手臂绝对跟随 |
| `X+A`          | 在 `External(2)` 与 `Auto(1)` 间切换；在 `KeepPose(0)` 下按则直达 `External(2)`   |
| `X+B`          | 在 `KeepPose(0)` 与 `Auto(1)` 间切换；在 `External(2)` 下按则切到 `KeepPose(0)`   |
| `X+Y`          | 停机（`StopRobot`）并退出节点，手臂切回 `Auto`                                        |

三个模式的效果：

| 模式         | 值    | 效果                                           |
| ------------ | ----- | ---------------------------------------------- |
| `KeepPose` | `0` | 保持当前手臂姿态                               |
| `Auto`     | `1` | 交还给默认自动摆手，手臂不接受 VR 绝对目标     |
| `External` | `2` | 进入 VR 外部控制，手臂跟随 Quest3 骨骼绝对位姿 |

### 操作流程示例

1. `roslaunch leju_launch vr_teleop.launch robot_version:=17`（默认即绝对）
2. 双手扳机做 OK 手势，进入外部控制，手臂开始绝对跟随
3. 需要暂停时按 `X+A` 回到 `Auto`；再按 `X+A` 回到 `External`
4. 结束时按 `X+Y` 停机退出

## 限制与常见现象

- 增量节点（`quest_vr_control_node`）仅支持 Kuavo，对 Roban 会在启动时直接退出
- 绝对控制（`quest_vr_abs_control_node`）仅支持 Roban，对 Kuavo 会在启动时直接退出
