# RL Demo Controller

RL Demo Controller 是强化学习行走控制示例，支持基于策略网络的机器人行走控制。

## 功能特性

- **策略推理**：支持 OpenVINO 和 ONNX Runtime 推理引擎
- **观察计算**：支持多种观察项（角速度、重力投影、关节位置/速度等）
- **历史帧堆叠**：支持策略输入的历史观察帧堆叠
- **手臂控制**：站立时自动平滑插值到默认姿态
- **手柄遥控**：支持游戏手柄进行行走控制

## 目录结构

```
examples/
├── include/leju-rl-controller/examples/
│   ├── rl_demo_controller.h      # 控制器头文件
│   ├── arm_controller.h          # 手臂控制器
│   └── cmd_stance_calculator.h   # 站立状态计算器
└── src/
    ├── rl_demo_controller.cpp    # 控制器实现
    ├── arm_controller.cpp        # 手臂控制实现
    ├── cmd_stance_calculator.cpp # 站立状态计算实现
    └── run_rl_demo_controller.cpp # 可执行文件入口
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
# 设置机器人版本
export ROBOT_VERSION=52  # Kuavo5
# export ROBOT_VERSION=46  # Kuavo4

# 启动仿真环境 + RL Demo 控制器
roslaunch leju_launch demo/load_mujoco_sim_rl_demo.launch
```

**实物机器人：**

```bash
sudo su  # 需要 root 权限
source devel/setup.bash

export ROBOT_VERSION=52  # Kuavo5
roslaunch leju_launch demo/load_real_rl_demo.launch
```

启动后，根据终端提示按下手柄 `start` 按键使机器人站立，然后使用摇杆控制行走。

### 配置文件

配置文件位于 `config/<ROBOT_VERSION>/`，主要字段：

```yaml
policy_path: "path/to/policy.xml"  # 策略模型路径
inference_engine: "openvino"       # 推理引擎: openvino/onnxruntime
loop_dt: 0.001                     # 控制周期
policy_dt: 0.02                    # 策略推理周期
history_length: 4                  # 历史帧长度

# 观察项配置
obs_terms:
  - name: "base_ang_vel"
    scale: 0.25
    lb: -3.0
    ub: 3.0
  # ... 其他观察项

# 命令范围
command_range_lin_vel_x_lb: -1.0
command_range_lin_vel_x_ub: 1.0
```

## 手柄控制

| 按键 | 功能 |
|------|------|
| `start` | 站立/开始行走 |
| `back` | 安全停机 |
| 左摇杆 | 前后左右平移 |
| 右摇杆 | 左右转向 |

## 架构说明

```
+-------------------+     +-------------------+     +-------------------+
|   Robot State     |     |   IMU Data        |     |   Joy Data        |
|   (关节位置/速度)  |     |   (角速度/加速度)  |     |   (手柄输入)      |
+---------+---------+     +---------+---------+     +---------+---------+
          |                       |                       |
          v                       v                       v
+-----------------------------------------------------------------------+
|                         RLDemoController                              |
|  +-----------------+  +-----------------+  +-----------------------+  |
|  | Observation     |  | Policy Inference|  | Action Scaling        |  |
|  | Compute         |->| (OpenVINO/ONNX) |->| & Joint Command       |  |
|  +-----------------+  +-----------------+  +-----------------------+  |
|                                                                       |
|  +-----------------+  +-----------------+                            |
|  | Arm Controller  |  | CmdStanceCalc   |                            |
|  | (站立插值)       |  | (站立状态检测)   |                            |
|  +-----------------+  +-----------------+                            |
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
| velocity_commands | 3 | 速度指令 (x, y, yaw) |
| cmd_stance | 1 | 站立状态标志 |
| joint_pos | N | 关节位置 |
| joint_vel | N | 关节速度 |
| actions | N | 上一帧动作 |

## 依赖

- lejusdk-lowlevel
- lejusdk-utils
- OpenVINO
- yaml-cpp
- Eigen3
- (可选) ONNX Runtime
