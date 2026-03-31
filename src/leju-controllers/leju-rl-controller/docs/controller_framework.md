# 控制器框架

## 概述

leju-rl-controller 采用注册表模式多控制器架构，支持动态控制器注册、运行时平滑切换和多输入源接入。框架通过分层设计将控制策略与执行解耦。

## 架构

```
+-----------------------------------------------------------------------+
|                              main()                                   |
|                                                                       |
|  Initialize RobotData, TriggerBuffer, Lifecycle, ControllerManager    |
|  Register Controllers, Launch InputSources & ControlLoop              |
+-----------------------------------+-----------------------------------+
                                    |
                 +------------------+------------------+
                 |                                     |
    +------------v--------------+        +-------------v--------------+
    |    Async Input Layer      |        |        ControlLoop         |
    |                           |        |     (Fixed Frequency)      |
    |  TeleopInputSource        |        |                            |
    |    - JoyAdapter  (pri=0)  |        |  1. RobotData.getState()   |
    |    - QuestAdapter(pri=1)  |        |  2. TriggerBuffer.drain()  |
    |  ExternalInterface(pri=2) |        |  3. Lifecycle.update()     |
    +---+------------------+----+        |  4. ControlLogic.tick()    |
        |                  |             |  5. ControllerManager.update|
        v                  v             |  6. publishRobotCmd()      |
  +-----------+    +-------------+       +-------------+--------------+
  |TriggerBuf |    |CommandBuffer|                      |
  |(one-shot) |    |(continuous) |                      |
  +-----------+    +-------------+                      |
        |                |                              |
        +-------+--------+                              |
                |                                       |
       +--------v---------+                             |
       |   ControlLogic   |<---------------------------+
       | (Strategy Layer)  |
       |                   |
       | - ActionTrigger   |
       | - Auto Transition |
       | - Fall Logic      |
       +--------+----------+
                |
       +--------v-------------------+
       |    ControllerManager       |
       |                            |
       |  active_controller         |
       |  SwitchTransition          |
       |  Arm/Waist/Head targets    |
       +--------+-------------------+
                |
       +--------v-------------------+
       |  ControllerBase (Plugin)   |
       |                            |
       |  GenericRLController       |
       |    + ArmController         |
       |    + WaistController       |
       +--------+-------------------+
                |
                v
           [RobotCmd]
```

## 核心模块

### 1. ControlLoop

主控制循环，以固定频率运行（通过 `loop_dt` 配置，通常 1000 Hz）。

**单次 tick 执行顺序：**

1. 从 `RobotData` 读取最新的 `RobotState` / `ImuData`
2. 从 `TriggerBuffer` 取出 `ActionTrigger` 事件
3. 更新 `Lifecycle` 状态机
4. 若非 Running 状态，发布保持指令并返回
5. `ControlLogic.tick()` — 策略决策
6. 按优先级合并所有 `InputSource` 的 `CommandBuffer` 快照
7. `ControllerManager.update()` — 执行当前活跃控制器
8. 发布 `RobotCmd` 到机器人

**核心原则：** ControlLoop 不直接访问活跃控制器，只通过 `ControllerManager` 交互。

### 2. Lifecycle

系统级门控 — 决定是否允许控制。

**状态机：**

```
kWaitingForReady --(ready=true)--> kWaitingForStart --(start)--> kRunning
       ^                                  |                          |
       +---------(ready=false)------------+                          |
                                          |                          |
                                          +---(quit)----> kExiting <-+
```

| 状态 | 说明 |
|------|------|
| `kWaitingForReady` | 等待传感器数据就绪 |
| `kWaitingForStart` | 就绪，等待 start 触发 |
| `kRunning` | 正常运行 |
| `kExiting` | 已请求退出 |

**原则：** Lifecycle 只管理是否允许控制，不关心控制什么或如何控制。

### 3. ControlLogic

策略决策层 — 决定控制什么。

**每次 tick 执行：**

1. `handleRunningEntry()` — 进入 Running 时，根据机器人姿态选择初始控制器（如跌倒 → GroundToStand，站立 → AMP）
2. `handleActionTriggers()` — 处理离散事件（SwitchController、SetArmMode 等）
3. `handleAutoTransitions()` — 控制器完成后自动切换（如 GroundToStand → AMP）
4. `handleFallLogic()` — 检测跌倒并触发保护控制

**触发去重策略：**
- 敏感类型（Start、Quit、StartMotion）：每 tick 仅保留一个
- LastWins 类型（SwitchController、SetArmMode）：保留最后一个

### 4. ControllerManager

控制器执行管理器 — 管理控制器生命周期和切换。

**职责：**
- 注册和存储控制器
- 执行活跃控制器
- 处理控制器平滑切换过渡
- 分发手臂/腰部/头部目标和速度指令

**控制器切换 — 两阶段模型：**

```
RequestSwitch("target")
        |
        v
  kTransitioning (MinimumJerk 插值过渡)
        |
        v
  CommitSwitch (old.OnExit(), new.OnEnter())
        |
        v
     kIdle
```

过渡期间：
- 捕获当前手臂/腿部关节参考值
- 使用 `MinimumJerkInterpolator` 插值到新控制器的标称姿态
- 插值完成后自动提交切换

**配置示例**（`controller_manager.yaml`）：

```yaml
loop_dt: 0.001
default_controller: "amp"

switch_interpolation:
  arm_kp: [80.0, ...]
  arm_kd: [8.0, ...]

controllers:
  - name: "amp"
    type: "GenericRLController"
    config: "config_amp.yaml"
    enabled: true
```

### 5. ControllerBase

所有控制器的抽象基类，采用模板方法模式。

**核心虚方法：**

| 方法 | 说明 |
|------|------|
| `initialize()` | 加载配置、模型和资源 |
| `updateImpl()` | 核心控制逻辑（必须重写） |
| `computeObservation()` | 构建观测向量 |
| `computeActions()` | 执行策略推理 |
| `updateRobotCmd()` | 将动作转换为电机指令 |
| `loadPolicy()` | 加载 RL 策略模型 |
| `moveToDefaultPos()` | 移动到初始位置 |
| `startMotion()` | 启动动作回放 |

**update 调用链：**

```
update()
  -> updateImpl()           [子类: observation -> inference -> action]
  -> updateArmCommand()     [启用 ArmController 时覆盖]
  -> updateWaistCommand()   [启用 WaistController 时覆盖]
```

**控制器状态：**

```
kUninitialized -> kRunning -> kPaused -> kRunning
                     |                      |
                     +-> kStopped           +-> kStopped
                     +-> kError
```

### 6. ControllerRegistry

单例工厂，用于动态实例化控制器。

**用法：**

```cpp
// 注册（通过宏，在静态初始化时执行）
REGISTER_CONTROLLER("GenericRLController", GenericRLController, generic_rl);

// 创建（ControllerManager 从 YAML 配置中调用）
auto controller = ControllerRegistry::create("GenericRLController", version, name);
```

### 7. GenericRLController

通用 RL 策略控制器，框架的主要控制器实现。

**功能：**
- ONNX/OpenVINO 策略推理
- 可配置的观测项（base_ang_vel、projected_gravity、velocity_commands、cmd_stance、joint_pos、joint_vel、actions）
- 可配置长度和堆叠顺序的观测历史（isaaclab / classic）
- 动作轨迹回放与残差动作
- 手臂和腰部部件控制器集成
- 速度指令限幅
- DDS 主题日志

**配置示例**（`config_amp.yaml`）：

```yaml
HumanoidRobotCfg:
  inference_engine: "openvino"
  policy_path: "policy/policy.onnx"

  env:
    policy_dt: 0.02

    robot:
      enable_arm_controller: true
      joint_names: [...]
      joint_default_pos: [...]
      actuator_kp: [...]
      actuator_kd: [...]
      action_scale: [...]

    observations:
      history_length: 1
      stack_order: "classic"
      terms:
        base_ang_vel:    { scale: 1.0, clip: [-9999, 9999] }
        projected_gravity: { scale: 1.0, clip: [-9999, 9999] }
        velocity_commands: { scale: 1.0, clip: [-9999, 9999] }
        joint_pos:       { scale: 1.0, clip: [-9999, 9999] }
        joint_vel:       { scale: 1.0, clip: [-9999, 9999] }
        actions:         { scale: 1.0, clip: [-18, 18] }

    command_range:
      lin_vel_x: [-1.0, 1.0]
      lin_vel_y: [-0.6, 0.6]
      ang_vel_z: [-0.3, 0.3]
```

## 输入系统

### 两条并行数据流

```
触发流（一次性）：
  InputSource -> ActionTrigger -> TriggerBuffer -> ControlLogic

连续流（持续性）：
  InputSource -> ContinuousCommand -> CommandBuffer -> Controller
```

### ActionTrigger

一次性离散事件，在当前控制周期生成并消费，消费后立即清除。

**核心特征：**
- 一次性生成、一次性消费
- 仅携带必要参数，不包含系统上下文
- 与 ContinuousCommand（持续性指令）互补

```cpp
enum class ActionType : uint8_t {
  None, Start, Quit,
  SwitchController, SetArmMode, SetWaistMode, MotionCommand,
};

// 参数继承体系
struct ActionArgs { virtual ~ActionArgs() = default; };
struct NamedArgs : public ActionArgs { std::string name; };
struct MotionCommandArgs : public ActionArgs {
  enum Operation { Start };
  Operation op;
  std::string motion_name;
};

struct ActionTrigger {
  ActionType type;
  std::shared_ptr<ActionArgs> args;
};
```

**在系统中的位置：**

```
Input Source (Joy/VR/RPC)
    |
    v
InputAdapter / TeleopAdapter
    |
    v
ActionTrigger
    |
    v
TriggerBuffer (thread-safe)
    |
    v
ControlLogic (consume)
```

#### TriggerBuffer

收集当前控制周期的所有触发事件。不使用优先级队列，而是按语义类别分阶段处理：

| 阶段 | 类别 | 示例 |
|------|------|------|
| 1 | 生命周期触发 | Start、Quit |
| 2 | 安全触发 | RecoverFromFall |
| 3 | 控制器切换触发 | SwitchController |
| 4 | 模式变更触发 | SetArmMode、SetWaistMode |

**优先级来源于语义类别，而非触发事件本身的属性。**

#### 线程模型

多个输入源在不同线程产生触发事件，通过 TriggerBuffer 安全传递到控制循环：

```
TeleopAdapter thread  ----+
ExternalInterface thread --+--> TriggerBuffer --> ControlLoop.drain() --> ControlLogic
Other callback threads ---+        (lock)            (single thread)
```

**去重策略：**
- 敏感类型（Start、Quit、StartMotion）：同一 tick 内仅保留一个
- LastWins 类型（SwitchController、SetArmMode）：同一 tick 内保留最后一个

#### Teleop Binding

手柄按键组合通过 YAML 配置映射到 ActionTrigger：

```yaml
bindings:
  - buttons: [LB, X]
    action:
      type: SwitchController
      args:
        name: amp
```

**运行时逻辑：**
1. 匹配按键组合
2. 上升沿检测（之前未按下，现在按下）
3. 仅在边沿时生成 ActionTrigger
4. 推入 TriggerBuffer

### CommandBuffer

持续性指令状态，使用双缓冲保证线程安全。

```cpp
struct Snapshot {
  MotionCommand cmd_vel;
  std::optional<ExternalJointTarget> arm_target;
  std::optional<ExternalJointTarget> head_target;
  std::optional<ExternalJointTarget> waist_target;
};
```

### InputSource

所有输入源的抽象接口，指令按优先级合并。

| 优先级 | 来源 | 说明 |
|--------|------|------|
| 0 | Joy | 游戏手柄 |
| 1 | Quest | VR 头显 |
| 2 | External | SDK / VR API |

## 部件控制器

### MultiModeArmController

三模式手臂控制，集成到 ControllerBase 中。

| 模式 | 值 | 行为 |
|------|-----|------|
| `kKeepPose` | 0 | 保持当前手臂位置 |
| `kAuto` | 1 | 行走时由 RL 控制，站立时插值回默认位置 |
| `kExternal` | 2 | 两阶段：接近目标 → 低通滤波跟踪 |

External 模式两阶段：
1. **接近阶段** — 速度限制运动趋近目标（可配置 `approach_velocity`）
2. **跟踪阶段** — 低通滤波目标跟随（可配置 `filter_cutoff_freq`）

模式切换使用 `MinimumJerkInterpolator` 实现平滑过渡。

### WaistController

两模式腰部控制。

| 模式 | 值 | 行为 |
|------|-----|------|
| `kAuto` | 0 | 行走时由 RL 控制，站立时三次多项式插值回默认位置 |
| `kExternal` | 1 | 低通滤波外部目标 |

## 跌倒保护与恢复

**RuntimeMode 流程：**

```
Normal (AMP)
    |  (检测到跌倒)
    v
ProtectiveFall (降低刚度，受控下降)
    |  (完成)
    v
Suspended (保持位置，等待恢复)
    |  (恢复触发)
    v
GroundToStand (起身)
    |  (完成)
    v
Normal (AMP)
```

- 跌倒检测基于 roll/pitch 角度、基座高度、接触状态
- 恢复方式支持手动（ActionTrigger）或自动（可配置）
- 所有过渡由 ControlLogic 决策，ControllerManager 执行

## 添加新控制器

1. 创建继承 `ControllerBase` 的类
2. 实现必需的虚方法（`initialize`、`updateImpl`、`computeObservation`、`computeActions`、`updateRobotCmd`、`loadPolicy`）
3. 通过宏注册：
   ```cpp
   REGISTER_CONTROLLER("MyController", MyController, my_controller);
   ```
4. 在 `controller_manager.yaml` 中添加配置：
   ```yaml
   controllers:
     - name: "my_ctrl"
       type: "MyController"
       config: "config_my_ctrl.yaml"
       enabled: true
   ```
5. 可选实现 `NominalArmJoints()` / `NominalLegJoints()` 以支持平滑切换
