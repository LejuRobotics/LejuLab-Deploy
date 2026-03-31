# RL Mimic Controller

RL Mimic Controller 是模仿学习舞蹈控制示例，支持基于动作捕捉数据的机器人舞蹈控制。

## 功能特性

- **动作模仿**：基于模仿学习策略执行舞蹈动作
- **动作文件播放**：支持加载和播放动作捕捉数据
- **周期性动作**：支持循环播放舞蹈动作
- **平滑过渡**：动作切换时平滑插值
- **手柄触发**：通过手柄按键触发舞蹈播放

## 目录结构

```
examples/
├── include/leju-rl-controller/examples/
│   └── rl_mimic_controller.h     # 控制器头文件
└── src/
    ├── rl_mimic_controller.cpp   # 控制器实现
    └── run_rl_mimic_controller.cpp # 可执行文件入口
```

## 使用方法

### 编译

```bash
catkin build leju-rl-controller
```

### 运行

使用 `leju_launch` 提供的 launch 文件启动（推荐）：

**MuJoCo 仿真：**

```bash
# 设置机器人版本 (目前仅支持 Roban2)
export ROBOT_VERSION=14

# 启动仿真环境 + Mimic 控制器
roslaunch leju_launch demo/load_mujoco_sim_mimic_demo.launch
```

**实物机器人：**

```bash
sudo su  # 需要 root 权限
source devel/setup.bash

export ROBOT_VERSION=14  # Roban2
roslaunch leju_launch demo/load_real_mimic_demo.launch
```

启动后，根据终端提示按下手柄 `start` 按键使机器人站立，然后按 `西瓜键/Y键` 播放舞蹈。

### 配置文件

配置文件示例 `config/14/config_mimic_HPNY_dance.yaml`：

```yaml
policy_path: "path/to/mimic_policy.xml"  # 模仿学习策略路径
inference_engine: "openvino"             # 推理引擎

# 动作配置
motion_file: "path/to/dance_motion.csv"  # 动作数据文件
loop_motion: true                        # 是否循环播放

# 观察项配置
obs_terms:
  - name: "base_ang_vel"
    scale: 0.25
  - name: "projected_gravity"
    scale: 1.0
  - name: "joint_pos"
    scale: 1.0
  - name: "joint_vel"
    scale: 0.05
  # ... 其他观察项
```

## 手柄控制

| 按键 | 功能 |
|------|------|
| `start` | 站立 |
| `back` | 安全停机 |
| `西瓜键/Y键` | 播放/暂停舞蹈 |

## 动作数据格式

动作数据文件为 CSV 格式，每行包含：

```
time, joint1_pos, joint2_pos, ..., jointN_pos, joint1_vel, joint2_vel, ..., jointN_vel
```

其中：
- `time`: 时间戳（秒）
- `jointX_pos`: 各关节位置（弧度）
- `jointX_vel`: 各关节速度（弧度/秒）

## 架构说明

```
+-------------------+     +-------------------+     +-------------------+
|   Motion File     |     |   Robot State     |     |   IMU Data        |
|   (动作数据)       |     |   (当前状态)       |     |   (姿态信息)      |
+---------+---------+     +---------+---------+     +---------+---------+
          |                       |                       |
          v                       v                       v
+-----------------------------------------------------------------------+
|                        RLMimicController                              |
|  +-----------------+  +-----------------+  +-----------------------+  |
|  | Motion Loader   |  | Observation     |  | Policy Inference      |  |
|  | & Playback      |  | Compute         |  | (Reference + State)   |  |
|  +--------+--------+  +--------+--------+  +-----------+-----------+  |
|           |                    |                       |               |
|           v                    v                       v               |
|  +-----------------+  +-----------------+  +-----------------------+  |
|  | Reference Motion|->| Combined Input  |->| Action Generation     |  |
|  | (目标姿态)      |  | (参考+当前状态)  |  | (模仿学习策略输出)     |  |
|  +-----------------+  +-----------------+  +-----------------------+  |
+-----------------------------------------------------------------------+
                                  |
                                  v
+-----------------------------------------------------------------------+
|                        publishRobotCmd()                              |
|                        (发送关节指令)                                  |
+-----------------------------------------------------------------------+
```

## 观察项说明

| 观察项 | 维度 | 说明 |
|--------|------|------|
| base_ang_vel | 3 | 基座角速度 |
| projected_gravity | 3 | 投影重力向量 |
| joint_pos | N | 当前关节位置 |
| joint_vel | N | 当前关节速度 |
| reference_joint_pos | N | 参考关节位置 |
| reference_joint_vel | N | 参考关节速度 |

## 与 RL Demo Controller 的区别

| 特性 | RL Demo Controller | RL Mimic Controller |
|------|-------------------|---------------------|
| 控制目标 | 行走控制 | 舞蹈动作模仿 |
| 输入指令 | 手柄速度指令 | 动作文件参考轨迹 |
| 观察空间 | 本体感受 + 指令 | 本体感受 + 参考轨迹 |
| 动作输出 | 关节位置增量 | 关节位置跟踪 |
| 适用场景 | 日常行走 | 表演展示 |

## 依赖

- lejusdk-lowlevel
- lejusdk-utils
- OpenVINO
- yaml-cpp
- Eigen3
