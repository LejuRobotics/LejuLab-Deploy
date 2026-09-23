# /rt/motor_state 与 /rt/motor_cmd 旁路话题

## 用途
诊断 c2t / ankle solver / 电机本身链路问题，给"上层发了多少 tau、最终写到电机 CAN 帧的是多少 A、电机回来多少"这三个断点做可观测。

## 语义

### /rt/motor_state（IDL: JointState）
- `q[i]`：电机轴角（rad）。**踝部 i ∈ {4,5,10,11} 即 bar 角**，未经 ankle solver 正解。
- `v[i]`：电机轴速度（rad/s）。
- `vd[i]`：电机轴加速度（rad/s²，由 Kalman 滤波器估计）。
- `tau[i]`：**原始电机反馈**，来自 `FeedbackFrameFd::torque()` 解码，**未乘 c2t**、**未经踝部反解**。

### /rt/motor_cmd（IDL: JointCmd）
- `q[i]` / `v[i]`：写电机前的 motor-space 目标（rad / rad·s⁻¹），ankle solver 正解后的 bar 目标。
- `tau[i]`：**写电机前最终 tau 命令**，单位 A：
  - is_ruiwo 路径（RUIWO / MotorEvo CANFD）：对应 PTM `torqueOffset` = `joint_tau / c2t * tau_ratio`（**已做 c2t、tau_ratio 与限幅**）
  - EC_MASTER 路径：对应 `torque` = `joint_tau / c2t`（**已做 c2t 与限幅**；tau_ratio 当前在 EC_MASTER CST 路径中**未应用**，因为 tau_ratio 默认 = 1.0）
- `kp[i]` / `kd[i]`：写帧前的最终 gains。
- `modes[i]`：上层 control mode（CST=0 / CSV=1 / CSP=2）。

## 时间对齐
三条 `/rt/joint_state`、`/rt/motor_state`、`/rt/motor_cmd` **共用同一 timestamp**（来自 `joint_state.timestamp`），离线 join 时无需插值。

## 不变量（健康检查）
对**无 Jacobian 解算的关节**（非踝、非膝），稳态下：

    joint_state.tau[i] ≈ motor_state.tau[i] × c2t[i]

不成立即说明快照取点错误（或 c2t 配置错误）。

## 典型用法（5x 排查）
1. 录 chirp bag：`bash scripts/start_recorder.sh`，跑 keyboard_ctrl CST 扫频。
2. 离线导出 CSV（mcap → csv 工具略）。
3. 画 4 张图：
   - `joint_cmd.tau[4]` vs `joint_state.tau[4]`（baseline 5×）
   - `motor_cmd.tau[4]` vs `motor_cmd.tau[5]`（看 ankle fwd 分配）
   - `motor_cmd.tau[4]` vs `motor_state.tau[4]`（电机层 round-trip）
   - `motor_state.tau[4] × c2t[4]` vs `joint_state.tau[4]`（ankle 反解一致性）

按结果判定根因（详见 `docs/superpowers/specs/2026-05-22-motor-state-cmd-topics-design.md` §4.3）。
