# 按键绑定配置

## 概述

按键绑定系统将手柄/VR 手柄的按键组合映射为控制器动作（ActionTrigger）。配置文件位于：

```
config/<ROBOT_VERSION>/teleop_bindings.yaml
```

系统支持两类输入设备：
- **Joy 手柄**（`joy_bindings`）
- **Quest VR 手柄**（`quest_bindings`）
- **速度上限**（`velocity_limits`）

## 配置格式

```yaml
velocity_limits:
  stick_deadzone: 0.05
  linear_x: 0.55
  linear_y: 0.30
  angular_z: 0.30

joy_bindings:
  - buttons: ["按键1", "按键2"]    # 按键组合（顺序无关）
    action:
      type: 动作类型
      args:
        参数名: 参数值

quest_bindings:
  - buttons: ["按键1"]
    action:
      type: 动作类型
      args:
        参数名: 参数值
```

## 可用按键

### Joy 手柄

| 按键 | 说明 |
|------|------|
| `A` | A 键（南） |
| `B` | B 键（东） |
| `X` | X 键（西） |
| `Y` | Y 键（北） |
| `LB` | 左肩键 |
| `RB` | 右肩键 |
| `L3` | 左摇杆按下 |
| `R3` | 右摇杆按下 |
| `GUIDE` | 西瓜键 |
| `MISC` | 辅助键 |
| `DPAD_UP` | 方向键上 |
| `DPAD_DOWN` | 方向键下 |
| `DPAD_LEFT` | 方向键左 |
| `DPAD_RIGHT` | 方向键右 |

> **注意：** `START` 和 `BACK` 是系统内置的生命周期控制键（启动/退出），不需要也不能在配置中绑定。

### Quest VR 手柄

| 按键 | 说明 |
|------|------|
| `LEFT_TRIGGER` | 左扳机（阈值 > 0.5） |
| `RIGHT_TRIGGER` | 右扳机（阈值 > 0.5） |
| `LEFT_GRIP` | 左握把（阈值 > 0.5） |
| `RIGHT_GRIP` | 右握把（阈值 > 0.5） |
| `LEFT_FIRST` | 左手第一按键 |
| `LEFT_SECOND` | 左手第二按键 |
| `RIGHT_FIRST` | 右手第一按键 |
| `RIGHT_SECOND` | 右手第二按键 |

## 可用动作类型

| 动作类型 | 说明 | 参数 |
|----------|------|------|
| `SwitchController` | 切换控制器 | `name`: 控制器名称 |
| `SetArmMode` | 设置手臂控制模式 | `name`: 模式名称（`keep_pose` / `auto` / `external`） |
| `SetWaistMode` | 设置腰部控制模式 | `name`: 模式名称（`auto` / `external`） |
| `MotionCommand` | 触发动作/舞蹈播放 | `op`: 操作（`Start`），`name`: 动作名称（可选） |

### SwitchController — 切换控制器

切换到指定控制器。控制器名称需在 `controller_manager.yaml` 中已注册。

```yaml
- buttons: ["X"]
  action:
    type: SwitchController
    args:
      name: amp          # 控制器名称
```

### SetArmMode — 设置手臂模式

切换手臂控制模式。

```yaml
- buttons: ["LB", "X"]
  action:
    type: SetArmMode
    args:
      name: keep_pose    # 手臂模式名称
```

支持的手臂模式：

| 模式名称 | 说明 |
|----------|------|
| `keep_pose` | 保持当前手臂姿态 |
| `auto` | 自动控制模式（行走时跟随 RL 策略，站立时回默认位置） |
| `external` | 外部控制模式（接受 VR/SDK 指令） |

### SetWaistMode — 设置腰部模式

切换腰部控制模式。

```yaml
- buttons: ["LB", "Y"]
  action:
    type: SetWaistMode
    args:
      name: external     # 腰部模式名称
```

支持的腰部模式：

| 模式名称 | 说明 |
|----------|------|
| `auto` | 自动控制模式（行走时跟随 RL 策略，站立时回默认位置） |
| `external` | 外部控制模式（接受外部目标） |

### MotionCommand — 动作播放

触发动作/舞蹈播放。

```yaml
- buttons: ["GUIDE"]
  action:
    type: MotionCommand
    args:
      op: Start
      # name: "dance_1"  # 可选，指定动作名称；省略则播放/重播当前动作
```

## 配置示例

### Roban2（ROBOT_VERSION=14）

```yaml
joy_bindings:
  # 按 X 键切换到 AMP 行走控制器
  - buttons: ["X"]
    action:
      type: SwitchController
      args:
        name: amp

  # 按 Y 键切换到 Mimic 舞蹈控制器
  - buttons: ["Y"]
    action:
      type: SwitchController
      args:
        name: mimic_hpny

  # 按西瓜键播放舞蹈
  - buttons: ["GUIDE"]
    action:
      type: MotionCommand
      args:
        op: Start

quest_bindings:
  # Quest VR 手柄绑定（按需配置）
```

### Kuavo4Pro / Kuavo5（ROBOT_VERSION=46/52）

当前版本默认只有 AMP 控制器，按键绑定默认注释。如需启用，取消对应注释即可。

## 触发机制

### 上升沿检测

按键绑定采用上升沿检测，仅在按键**按下瞬间**触发一次，长按不会重复触发：

- **Joy 手柄**：按键组合发生变化时触发（按下和松开都会检测）
- **Quest VR**：仅在无按键 → 有按键的瞬间触发（只检测按下）

### 组合键

- 支持多按键组合（如 `["LB", "X"]`）
- 按键顺序无关，`["LB", "X"]` 等价于 `["X", "LB"]`
- 需同时按住所有按键才能匹配

### 摇杆控制

摇杆通过 `velocity_limits` 直接映射为连续 `cmd_vel`：

| 摇杆 | 功能 |
|------|------|
| 左摇杆 X/Y | 平移速度指令（前后左右） |
| 右摇杆 X/Y | 旋转速度指令（左右转向） |

## 自定义绑定

1. 编辑 `config/<ROBOT_VERSION>/teleop_bindings.yaml`
2. 在 `joy_bindings` 或 `quest_bindings` 下添加绑定条目
3. 重新启动控制器生效

**注意事项：**
- 同一按键组合不要绑定多个动作
- `SwitchController` 的控制器名称必须在 `controller_manager.yaml` 中已注册
- 建议预留 `START`/`BACK` 不做其他绑定（系统保留）
