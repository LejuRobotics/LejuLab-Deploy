# 头部关节 DDS 调试工具

本目录下的两个脚本用于在 **launch 运行期间** 通过 DDS 直接操作机器人头部关节：

- `dds_head_set_pose.sh` —— 把头部摆到指定 yaw/pitch 并保持（摆位工具）
- `dds_head_pose_test.sh` —— 下发目标角度并读回实测角度，量化头部控制误差（精度测试）

两个 `.sh` 包装脚本会自动配置 CycloneDDS（`CYCLONEDDS_URI`、`LD_LIBRARY_PATH`），用法与
`dds_topic_hz.sh` 一致。深度链路的调试工具见 [README_depth_dds_tools.md](./README_depth_dds_tools.md)。

## 前置条件

在**上位机或下位机**（与 `run_rl_controller` 相同 DDS 域）执行：

```bash
pip3 install cyclonedds==0.10.2
```

**必须先启动** `launch_real.sh`（或等价 launch），并让控制器进入 Running（手柄 `start` 站立）
后再运行下列工具。

## 链路与话题

```text
下发：/rt/head_trajectory   (leju::msgs::JointTrajectoryPoint, q = [yaw, pitch], 单位 rad)
实测：/rt/joint_state       (leju::msgs::JointState, q[21]/q[22] = 头部 yaw/pitch, 单位 rad)
```

```text
/rt/head_trajectory
  → ExternalInterface::onHeadJointCmd        （维度校验：必须等于 getHeadJointNames().size()）
  → CommandBuffer::writeHeadTarget           （双缓冲 + seq 自增）
  → ControlLoop::mergeHeadTarget             （按输入源优先级选第一个有值者）
  → ControlLogic::processExternalTargets     （按 seq 去重后推送）
  → ControllerManager::setHeadTarget         （缓存 + 刷新接收时间戳）
  → ControllerManager::update()              （覆写 RobotCmd.q 的最后两个槽位）
```

> 头部控制**不在任何控制器内部实现**，而是在任意控制器 `update()` 成功后，由
> `ControllerManager` 统一覆写 `RobotCmd` 末两位（`head_start = cmd.q.size() - 2`），
> 并强制 `modes = 2`（CSP）与固定增益。

## 通用要点

- **单位是弧度**。DDS 上没有任何 `deg→rad` 转换（`vr_msg_converter.hpp` 里是纯赋值），
  填角度会被当成弧度。两个脚本的命令行参数收的是**度**，内部用 `math.radians()` 转换后再发。
- **必须持续下发**。控制器用 0.5s 窗口判定指令新鲜度（`kHeadCmdTimeoutSec`）：超过 0.5s
  无新消息，头部会被判为 VR 掉线并平滑插值回手柄态（默认插值时长 1.0s）。因此两个脚本
  都以远高于 2Hz 的频率持续发布。
- **头部不受搬运模式门控**。手臂/腰部的外部目标在搬运期间会被 `transport_gating` 拦截，
  头部和手部放行。
- **头部不需要切模式**。`setHeadMode` 这条 RPC 在控制器侧没有消费方，指令直接透传。
- **指令维度必须正好 2**。`onHeadJointCmd` 拿 `getHeadJointNames().size()` 比对，不符只告警
  并丢弃：
  ```text
  ExternalInterface: Head joint count mismatch, expected 2, got N
  ```
- **关节顺序**：S17 上 `q[0] = yaw`（`zhead_1_joint`，绕 Z）、`q[1] = pitch`（`zhead_2_joint`，绕 Y）。

---

## 1. 摆位：`dds_head_set_pose.sh`

把头部摆到指定角度并保持，用于快速确认链路是否通、方向是否正确。

```bash
# depth_walk 预设：低头 45°，保持当前 yaw，直到 Ctrl+C
bash src/leju_launch/scripts/dds_head_set_pose.sh --depth-walk

# 指定角度，发 10 秒后退出
bash src/leju_launch/scripts/dds_head_set_pose.sh \
  --pitch-deg 30 --yaw-deg 0 --hold-duration 10
```

**行为**：先从 `/rt/joint_state` 读当前 yaw/pitch（默认下标 21/22），再按 `--move-duration`
线性插值到目标，之后持续发布。

| 参数 | 说明 |
|------|------|
| `--depth-walk` | 预设：pitch 45°，yaw 保持不变 |
| `--pitch-deg D` | 目标 pitch（正值 = 低头；默认：配 `--depth-walk` 为 45，否则 0） |
| `--yaw-deg D` | 目标 yaw（默认保持 `/rt/joint_state` 里的当前值） |
| `--move-duration S` | 从当前姿态线性插值到目标的时间（默认 1.0） |
| `--hold-duration S` | 到位后继续保持的时间，`0` = 直到 Ctrl+C（默认 0） |
| `--rate HZ` | 发布频率（默认 50） |
| `--head-yaw-index` / `--head-pitch-index` | `/rt/joint_state.q` 中的头部下标（默认 21 / 22） |
| `--state-timeout S` | 等待 `/rt/joint_state` 采样以读取当前姿态的超时（默认 3.0） |

---

## 2. 精度测试：`dds_head_pose_test.sh`

对每个用例：读当前实测值 → 插值下发目标 → 保持 → 取**保持阶段最后 1 秒**的实测均值与目标
比较，输出误差、标准差与判定结论。

```bash
# 默认四条用例：yaw +60 / yaw -60 / pitch +30 / pitch -18
bash src/leju_launch/scripts/dds_head_pose_test.sh

# 只测某几条
bash src/leju_launch/scripts/dds_head_pose_test.sh --cases 'yaw:+60,pitch:-18'

# 收紧阈值并把结果写成 JSON（便于归档/CI）
bash src/leju_launch/scripts/dds_head_pose_test.sh \
  --tolerance-deg 3 --json /tmp/head_test.json
```

**输出示例**（格式示意，数值非真机实测）：

```text
case       axis      expect    actual     error    |err|     std  verdict
-------------------------------------------------------------------------
yaw+60     yaw       60.00°    60.32°    +0.32°    0.32°  0.021°  PASS
yaw+60     pitch      0.00°     0.15°    +0.15°    0.15°  0.014°  PASS
-------------------------------------------------------------------------
pitch+30   yaw        0.00°     0.11°    +0.11°    0.11°  0.012°  PASS
pitch+30   pitch     30.00°    30.41°    +0.41°    0.41°  0.018°  PASS
-------------------------------------------------------------------------

结果：全部 4 条通过（阈值 ±5.0°）
```

**说明**：`pitch -18°` 距 s17 的 -20° 下限仅剩 2°（见 [第 3 节](#3-关节行程选目标角度前先看)），
该行数据的解读需先排除触限位；`yaw -60°` 一行已省略，格式与上表一致。

### 判定标记

| 标记 | 含义 | 排查方向 |
|------|------|----------|
| `PASS` | 驱动轴 `\|误差\|` 在 `--tolerance-deg` 内 | — |
| `FAIL` | 超出阈值 | 增益/负载/时间是否足够稳定 |
| `NO_RESP` | 驱动轴几乎没动（位移 < 指令的 5%） | 控制器是否 Running、是否持续下发 |
| `SIGN!` | 动了但方向与指令相反 | 大概率 yaw/pitch 约定写反 |

`NO_RESP` 与 `SIGN!` 只针对该用例的**驱动轴**判定；另一轴仅作展示。

### 退出码

| 码 | 含义 |
|----|------|
| `0` | 全部用例通过 |
| `1` | 有用例失败，或运行中被中断且已完成的用例未全通过 |
| `2` | 参数错误（用例格式、`--rate` 过低）或依赖缺失 |

可直接用于 CI 或 `&&` 串联。`--json` 输出包含每条用例的起止角度、实测均值、误差、标准差、
样本数与判定结果。

### 参数

| 参数 | 说明 |
|------|------|
| `--cases LIST` | 逗号分隔，格式 `axis:deg`，axis 为 `yaw`/`pitch`（默认 `yaw:+60,yaw:-60,pitch:+30,pitch:-18`） |
| `--tolerance-deg D` | 判定阈值（默认 5.0） |
| `--move-duration S` | 每用例插值到目标的时间（默认 1.5） |
| `--settle-duration S` | 到位后继续保持的时间（默认 3.0） |
| `--avg-window S` | 取保持阶段最后这段时间的均值做判定（默认 1.0） |
| `--rate HZ` | 发布频率，**必须 >2.5**，推荐 ≥50（默认 100） |
| `--reset-between` / `--no-reset-between` | 用例之间是否先回中位（默认开启） |
| `--reset-duration S` | 回中位的插值时长（默认 1.0） |
| `--reset-settle S` | 回中位后等待稳定的时长（默认 1.0） |
| `--head-yaw-index` / `--head-pitch-index` | 同摆位工具（默认 21 / 22） |
| `--state-timeout S` | 等待 `/rt/joint_state` 首个样本的超时（默认 5.0） |
| `--json PATH` | 结果写入 JSON，便于归档或 CI 消费 |

---

## 3. 关节行程（选目标角度前先看）

**超出下行行程的目标会被驱动器/关节截断，量到的是限位而不是控制误差。**

| 机型 | yaw | pitch |
|---|---|---|
| s17 / s14 / s52 | ±90° | **-20° / +45°**（不对称） |
| s45 / s46 / s53 / s54 | ±90° | **±30°**（对称） |

s17 的数值出自 `src/leju_assets/models/biped_s17/urdf/biped_s17.urdf`：

- `zhead_1_joint`：axis `0 0 1`，`-1.5708 ~ +1.5708 rad`（±90°），effort 1.5 N·m
- `zhead_2_joint`：axis `0 1 0`，`-0.349066 ~ +0.785398 rad`（-20° ~ +45°），effort 12.0 N·m

MuJoCo XML（`xml/biped_s17.xml`）与 Gazebo URDF 的取值一致。

> **注意**：这是模型/软件的**关节限位**，仓库内没有硬件侧机械极限数据；真机硬限位若更紧需
> 另行确认。另外手柄控头那一路的软件限幅是 yaw ±90° / pitch ±45°（`controller_manager.cpp`），
> pitch 负向比 URDF 松 25°，摇杆压到底会给出超程指令 —— 该路径与本目录脚本无关。

**默认用例的行程检查（s17）**：`yaw ±60°` 在 ±90° 内余量充足；`pitch +30°` 在 +45° 内；
`pitch -18°` **距 -20° 下限仅剩 2°**，若实测异常需先确认是否已触限位。

---

## 推荐排查流程

```bash
# ① 先把头部摆到安全角度，确认链路通、方向对（Ctrl+C 结束）
bash src/leju_launch/scripts/dds_head_set_pose.sh --pitch-deg 30 --yaw-deg 0

# ② 单条用例试跑，确认误差量级和响应方向
bash src/leju_launch/scripts/dds_head_pose_test.sh --cases 'yaw:+60'

# ③ 跑完整默认用例集，并归档结果
bash src/leju_launch/scripts/dds_head_pose_test.sh \
  --tolerance-deg 5 --json /tmp/head_test.json
```

若 ① 头部完全不动，先查控制器是否 Running 与下发频率（见「常见问题」）；若 ② 报 `SIGN!`，
先核对 yaw/pitch 顺序再继续测精度。

---

## 文件结构

```text
src/leju_launch/scripts/
├── dds_head_set_pose.sh / .py     # 头部 yaw/pitch 摆位
├── dds_head_pose_test.sh / .py    # 头部误差测试（期望 vs 实测）
├── dds_leju_types.py              # DDS 类型定义（含 JointTrajectoryPoint / JointState）
└── README_head_dds_tools.md       # 本文档
```

---

## 常见问题

**Q: 脚本收到了 `/rt/joint_state`，但头部完全不跟随指令？**  
A: 依次确认：

1. 控制器处于 Running（未站立/未按 `start` 时不会响应指令）；
2. 指令是否**持续**下发 —— 超过 0.5s 无新消息会被判定 VR 掉线并插值回手柄态；
3. 话题名是否为 `/rt/head_trajectory`（注意与手臂的 `/rt/arm_trajectory` 区分）；
4. 下发维度必须正好 2，否则只告警并丢弃（日志 `ExternalInterface: Head joint count mismatch`）。

**Q: 头部实测角度差得很多，甚至朝反方向走？**  
A: 先核对 yaw/pitch 顺序。S17 的约定是 `q[0] = yaw`（绕 Z）、`q[1] = pitch`（绕 Y）。
注意 `lejusdk-vr` 自带示例（`head_control_example.cpp`、`vr_cmd_publisher.cpp`）里 `q[0]` 写的
是 pitch，与 S17 实际顺序相反，直接照抄会拧错轴。精度测试脚本遇到这种情况会报 `SIGN!`。

**Q: 实测值卡在某个角度上不去？**  
A: 多半是撞到关节限位，见 [第 3 节](#3-关节行程选目标角度前先看)。

**Q: 能收到 Hz 但控制器日志没有头部相关输出？**  
A: 确认 `run_rl_controller` 与脚本使用相同 `CYCLONEDDS_URI`。头部路径没有周期性日志，
可用 `dds_topic_hz.sh rt/joint_state` 确认状态回传是否正常。

**Q: 仿真环境能用吗？**  
A: 能。`load_mujoco_sim.launch` 同样使用 domain 0 + 共享内存配置，脚本无需改动。
