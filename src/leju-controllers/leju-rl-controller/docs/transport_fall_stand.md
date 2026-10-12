# 搬运模式与倒地起身

> 适用：`ROBOT_VERSION=17`（Roban 2.2 / RK3588）。组装层仅在该版本注入相关模块。

## 分层

```
交互层（手柄）                              运控层（原语）
────────────────                            ────────────────
Joy 组合键
  → ActionType::TransportFallStand
  → TriggerBuffer
  → ControlLogic Phase 5
  → TransportFallStandScheduler  ──────►  TransportModeCoordinator
       （时序 / 判据 / 多步链）                FallStandCoordinator
                                              ControllerManager
                                                  │
外部上位机 ── DDS 命令/状态 ──────────────────┘
（不经 Scheduler，自行编排）
```

| 层 | 模块 | 职责 |
|----|------|------|
| **运控层** | `TransportModeCoordinator` / `FallStandCoordinator` | 一条命令 = 合法性检查 + 原子副作用；DDS 对外接口；门控查询 |
| **调度层** | `TransportFallStandScheduler` | **仅手柄路径**：把一次事件展开成 coordinator 原语序列 + CM 调用 |
| **策略入口** | `ControlLogic` | Phase 5 转发 `TransportFallStand`；搬运进行中门控其它输入；倒地自动切瘫软 |

原则：

- Coordinator **零调度**：不做回 amp、手臂复位、EXIT 时机、等待/重试。
- Scheduler **只服务手柄**；外部客户端直接打 coordinator DDS，自行负责时序。
- `HAND_OVER` 是运控原语；用户语义「退出」是事件 `transport.exit`（内部再发 `HAND_OVER`）。

## 倾倒检测（Roban 2.2）

`ControlLogic` 发现机器人倒地（含启动时倒地）→ **立即 instant 切** `mimic_fall_stand`
进入 FALL_DOWN 零力矩瘫软，不走 `protective_fall`。非 Roban 2.2 维持原有
`protective_fall` / `ground_to_stand` / `recovery` 回退路径。

瘫软后一切起身动作由手柄 `fallstand.standup`（或外部 DDS）驱动，见下文。

## 运控层

### TransportModeCoordinator

**状态**（`/rt/transport_mode_state`，Float64 0~4）：

| 值 | 名 | 含义 |
|----|----|------|
| 0 | INACTIVE | 未进搬运 |
| 1 | INTERPOLATING | amp 上，手臂 min-jerk 归零中 |
| 2 | READY | 到位，可 LOCK 或取消 |
| 3 | ACTIVE | 已切 `hold_pose`，全身锁死可搬 |
| 4 | HANDING_OVER | 移交挂起；本层不再动控制器 |

**命令**（`/rt/transport_mode_command` 或进程内 `sendCommand`）：

| 命令 | 从 | 副作用 |
|------|----|--------|
| `ENTER` | INACTIVE | → INTERPOLATING：校验 amp+站立；臂 `kExternal` + 归零；到位后自动 READY |
| `LOCK` | READY | pending → **instant** 切 `hold_pose`（无混合），首拍抓 `state.q` 冻住后 ACTIVE |
| `HAND_OVER` | READY / ACTIVE | → HANDING_OVER，**零副作用**（取消未完成 LOCK/FALL_DOWN） |
| `FALL_DOWN` | ACTIVE | pending → instant 切 `mimic_fall_stand`（瘫软，不起播）→ HANDING_OVER |
| `EXIT` | HANDING_OVER | → INACTIVE，无条件；**不**负责回 amp / 清臂模式 |

**给上层的查询：**

- `isInputGating()`：非 INACTIVE → ControlLogic 挡掉切控制器 / 改臂腰 / 起动作等
- `isFallProtectionSuppressed()`：ACTIVE 时豁免跌倒保护

### FallStandCoordinator

独立于搬运通道；仅在当前控制器为 `mimic_fall_stand` 且非 transitioning 时读阶段、吃命令。

**命令**（`/rt/fall_stand_command`）：

| 命令 | 需要阶段 | 副作用 |
|------|----------|--------|
| `PREPARE` | 0 瘫软 | `prepareToMotionStart`（插到固定 READY 姿态，见下） |
| `STAND_UP` | 2 READY 保持 | `startMotion`（播起身轨迹） |

**状态**（`/rt/fall_stand_state`，0~4，仅控制器激活时发布）：

| 值 | 含义 |
|----|------|
| 0 | FALL_DOWN（瘫软） |
| 1 | INTERPOLATING（prepare 插值中） |
| 2 | READY_FOR_STAND_UP（READY 姿态保持） |
| 3 | STAND_UP（起身播放中） |
| 4 | STANDING（起身完成） |

**READY 姿态**（`GenericRLController::prepareToMotionStart`，双策略倒地起身专用）：

- 固定姿态，不取 CSV 首帧：腿部按 IK 解算整体前摆（大腿世界系前摆 15°）、
  膝盖弯 30°、脚踝 -15°、手肘弯 -30°，避免准备阶段蹭到机身壳体。
- 肩膀后摆角度按 **prepare 时刻实时 IMU** 的肚皮法线（body_x·gravity）分档：
  趴（cos>0.5）30°、躺（cos<-0.5）0°、其它（侧躺/斜置/站直）15°。
- 起身轨迹选择用**起播瞬间**重新采样的 IMU，同一肚皮法线判据选趴/躺轨迹
  （对齐 kuavo STAND_UP）；READY 期间挪动机器人会以新姿态选轨迹。
- 起播后前 500 帧为 entry blend：READY 姿态向 CSV/策略目标缓动，
  CSV 与策略仍按原时序推进，解决 CSV 前段的干涉问题。

**不做：** 起身完成后自动切 amp。回切由 `TransportFallStandScheduler`（手柄）或外部
DDS 客户端在看到 STANDING 后自行 `requestSwitch(amp)`。

### ControllerManager（被调用方）

- `requestSwitch` / `setArmMode` / `setHeadTarget` / motion prepare-start
- `startMotion` 起播前同步读最新 IMU 写入控制器（倒地起身轨迹选择依据）
- 切换混合时长 `switchDuration()`（yaml，默认约 2s）
- 过渡中拒绝 `setArmMode`

## 交互层（手柄）

### 按键 → 事件

配置：`config/17/teleop_bindings.yaml`。

| 组合 | 事件名 | 用户语义 |
|------|--------|----------|
| LB+RB+Y | `transport.enter` | 进搬运 |
| LT+RB+A | `transport.lock` | 锁定可搬 |
| LB+RB+A | `transport.exit` | 退出/取消（不是 hand_over） |
| LB+RB+B 长按 2s | `transport.fall_down` | 软倒地 |
| LB+RB+X | `fallstand.standup` | 起身（场景相关） |

路径：

```
JoyTeleopAdapter
  → ActionTrigger{TransportFallStand, name}
  → TriggerBuffer
ControlLoop 每周期（顺序）：
  1. coordinator.tick() ×2 + scheduler.tick()   // 边沿/链/退出编排
  2. ControlLogic Phase 5（不受 transport_gating）
       → scheduler->onEvent(name, imu, now)     // 本周期事件，实时 IMU
  3. ControllerManager.update()
```

`TransportFallStand` **不受**搬运门控；门控只挡 `SwitchController` / `SetArmMode` /
`MotionCommand` 等。所有姿态判据（直立/躺平/趴躺）都吃**当周期实时 IMU**。

### TransportFallStandScheduler

#### 搬运事件

| 事件 | 条件 | 行为 |
|------|------|------|
| `transport.enter` | INACTIVE | `ENTER` |
| `transport.lock` | READY | `LOCK` |
| `transport.exit` | READY | `HAND_OVER` + 抬头；登记「取消搬运」语音 |
| `transport.exit` | ACTIVE 且直立 | `HAND_OVER` + 抬头；登记「退出搬运」语音，走需切控制器退出 |
| `transport.exit` | ACTIVE 非直立 | 拒绝 |
| `transport.fall_down` | ACTIVE | `FALL_DOWN` + 抬头（**不**置普通退出 pending） |

边沿：

- → INTERPOLATING：开保护窗（5s，累计上限 15s；窗内除 lock 重置计时外忽略搬运事件）
  + 播「电机未锁定」语音 + 开始握拳序列
- → ACTIVE：低头（pitch ≈ 20°）+ 播「进入搬运模式」语音
- → HANDING_OVER 且普通退出 pending：开退出编排
- → INACTIVE：**张手** + 清保护窗 / 起身链 / 待播语音

#### 起身 `fallstand.standup`（按搬运状态分叉）

| 当前 ts | 行为 |
|---------|------|
| **ACTIVE** | 硬起身一键：躺平才 `FALL_DOWN`，之后自动 PREPARE → HOLDING 后自动 STAND_UP |
| **HANDING_OVER** | 软起身两按：第 1 次 PREPARE，HOLDING 后再按才 STAND_UP |
| **INACTIVE** | 独立起身两按：倒地已由 ControlLogic 自动切 mimic 瘫软；第 1 次 PREPARE，READY（握拳）后第 2 次 STAND_UP；**握拳握手期间按键暂缓** |

起身播完（fs=STANDING 且本链有进度）→ **倒地路径退出编排**。

#### 退出编排

| 入口 | 步骤 |
|------|------|
| 普通 exit，来自 READY | 已在 amp → 等 `amp && !transitioning` → `setArmMode(auto)` → `EXIT` |
| 普通 exit，来自 LOCK/ACTIVE | 立刻 `requestSwitch(amp)` → 等 `switchDuration()+0.5s` → 查混合完成 → arm auto → `EXIT` |
| 倒地/起身路径 | 轨迹结束立刻 `requestSwitch(amp)`，同一套延时 + 判据 + arm + EXIT（独立起身 ts 已是 INACTIVE 则跳过 EXIT） |

顺序保证：amp 混合完成后再清手臂 `kExternal`，避免混合提交后残臂模式把臂往零位拽。

#### 灵巧手

- **存在性判定**：main 组装层按**首份** `HandState` 一次性固化——双手有效则有手，
  任一手无效或始终无反馈则无手；不处理中途掉线，不用手反馈门控状态机。
- **无手时**：跳过全部手部命令（预握/四指/握拳/张手），握拳序列直接 idle，
  不等 1.0s+0.5s 时序，状态机行为不受影响。
- **握拳时序**（进搬运 INTERPOLATING 边沿 / 倒地 READY 边沿）：
  预握拳 → 1.0s → 四指闭合 → 0.5s → 完整握拳。
- **张手**：搬运到 INACTIVE 边沿统一张手（倒地起身路径先等 AMP 回切/插值完成
  再进 INACTIVE，起身过程中保持握拳）。
- **就绪握手**：倒地 READY 的握拳序列未走完时，第二次 `fallstand.standup` 暂缓，
  防止手没握好就起身。

#### 语音

4 条提示音经 `/rt/audio_play_file` 发布（fire-and-forget，无设备检测）：

| 语音 | 时机 |
|------|------|
| 电机未锁定请扶住背部把手禁止抬起机器人 | → INTERPOLATING 边沿；保护窗内再按 lock 打断重播并重置计时 |
| 进入搬运模式可安全移动 | → ACTIVE 边沿 |
| 取消搬运恢复正常行走状态 | READY 取消：事件登记，**HandingOver 发 EXIT 时**播 |
| 退出搬运模式恢复正常 | ACTIVE 退出 / 倒地路径退出：事件登记，**HandingOver 发 EXIT 时**播 |

## 预启动倒地恢复

`run_rl_controller --pre-start-fall-recovery`（仿真：
`launch_mujoco_sim.sh --pre-start-fall-recovery`）：

- 仅 Roban 2.2 生效，其它机型忽略该参数；`mimic_fall_stand` 不可用同样忽略。
- 开机跳过默认姿态插值，直接进入 `mimic_fall_stand` FALL_DOWN 瘫软。
- 之后流程与常规倒地一致：第一次 `fallstand.standup` prepare（无条件插到
  READY 姿态，prepare 不查 IMU，准备阶段还可挪动机器人），第二次起身。
- 参数不持久化，不影响下次启动。

## 典型路径

**正常搬运**

```
enter →（臂归零 + 握拳）READY → lock → ACTIVE（hold_pose，低头）
  → exit → HAND_OVER → 切 amp → 等混合 → arm auto → EXIT（播退出语音）→ INACTIVE（张手）
```

**READY 取消**

```
enter → READY → exit → HAND_OVER →（不切控制器）arm auto → EXIT（播取消语音）→ 张手
```

**软倒 + 软起**

```
ACTIVE → 长按 fall_down → mimic 瘫软 + HANDING_OVER
  → standup×2 → 起身 → STANDING → 回 amp → arm auto → EXIT
```

**硬起（仍在 ACTIVE）**

```
躺平 + standup → FALL_DOWN + 自动 PREPARE/STAND_UP → 同上退出
```

**非搬运倒地起身**

```
倒地 →（自动）mimic 瘫软 → standup 第1次 → READY（预握拳）
  → standup 第2次 → 起身 → STANDING → 回 amp（无 EXIT）
```

**倒地开机（预启动参数）**

```
--pre-start-fall-recovery 启动 → mimic 瘫软 → 同非搬运倒地起身
```

## 外部 DDS

| 谁 | 入口 |
|----|------|
| 手柄 | 只认 `TransportFallStand` 事件名；由 Scheduler 翻译 |
| 外部 | `/rt/transport_mode_command`、`/rt/fall_stand_command`；**自己**负责回 amp / arm / EXIT 时序 |

运控层对两边暴露同一套原语；只有手柄走进程内 Scheduler。

## 源码入口

| 模块 | 路径 |
|------|------|
| 搬运协调器 | `include/.../runtime/transport_mode_coordinator.h` |
| 倒地起身协调器 | `include/.../runtime/fall_stand_coordinator.h` |
| 手柄调度 | `include/.../runtime/transport_fall_stand_scheduler.h` |
| READY 姿态 / 轨迹选择 | `src/controllers/generic_rl_controller.cpp`（`prepareToMotionStart` / `autoSelectFallStandModel`） |
| 倾倒自动切瘫软 | `src/runtime/control_logic.cpp`（`isFallen` / Fall detected） |
| 组装 / 预启动参数 | `src/main.cpp`（`IS_ROBAN2_2_LEGGED` / `--pre-start-fall-recovery`） |
| 按键 | `config/17/teleop_bindings.yaml` |
| DDS 话题名 | `lejusdk-topic-pubsub/topic_names.h` |
