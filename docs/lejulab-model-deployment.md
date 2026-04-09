# LejuLab 模型部署指南

## 1. 如何启动机器人执行的程序

### 编译

首次部署或代码更新后，先在工作区根目录加载环境并编译：

```bash
cd <repo_path>
source installed/setup.bash  # 开源仓库必需
catkin build
```

### 实物启动

AMP 示例：

```bash
sudo su

# Roban2.1
export ROBOT_VERSION=14

# Roban2.2
export ROBOT_VERSION=17

# Kuavo4pro
export ROBOT_VERSION=46

# Kuavo5
export ROBOT_VERSION=52

source devel/setup.bash
roslaunch leju_launch load_real.launch controller_manager_config:=/home/lab/.lejulab/auto_start_config/profiles/amp_walk_v1/controller_manager.yaml
```

Mimic HPNY 示例：

```bash
sudo su

# Roban2.1
export ROBOT_VERSION=14

# Roban2.2
export ROBOT_VERSION=17

source devel/setup.bash
roslaunch leju_launch load_real.launch controller_manager_config:=/home/lab/.lejulab/auto_start_config/profiles/mimic_hpny_v1/controller_manager.yaml
```

### 仿真启动

AMP 示例：

```bash
# Roban2.1
export ROBOT_VERSION=14

# Roban2.2
export ROBOT_VERSION=17

# Kuavo4pro
export ROBOT_VERSION=46

# Kuavo5
export ROBOT_VERSION=52

source devel/setup.bash
roslaunch leju_launch load_mujoco_sim.launch controller_manager_config:=/home/lab/.lejulab/auto_start_config/profiles/amp_walk_v1/controller_manager.yaml
```

### 说明

- 配置对应机器人的版本号 `ROBOT_VERSION`
- 实物需要以 `root` 用户启动
- `controller_manager_config`: LejuLab 模型配置文件(YAML)的绝对路径

## 2. 如何检测机器人是否在执行对应的模型程序

### 检测命令

实物：

```bash
pgrep -af "roslaunch leju_launch load_real.launch"
```

仿真：

```bash
pgrep -af "roslaunch leju_launch load_mujoco_sim.launch"
```

输出示例：

```text
3078626 /usr/bin/python3 /opt/ros/noetic/bin/roslaunch leju_launch load_real.launch controller_manager_config:=/home/lab/.lejulab/auto_start_config/profiles/amp_walk_v1/controller_manager.yaml
```

处理方式：

- 根据当前启动的是实物还是仿真，执行对应的 `pgrep -af "roslaunch ..."` 命令
- 再从输出命令行中读取 `controller_manager_config:=...`
- 判断这个 YAML 路径是否和目标路径一致。若一致，则说明目标 launch 正在运行

## 3. 如何停止机器人执行的程序

### 方式一：直接结束整个 `roslaunch`

实物：

```bash
sudo pkill -INT -f "roslaunch leju_launch load_real.launch"
```

仿真：

```bash
pkill -INT -f "roslaunch leju_launch load_mujoco_sim.launch"
```

适用场景：

- 停止当前整套运行程序

### 方式二：按启动参数精确结束对应控制器进程

参考命令：

```bash
sudo pkill -INT -f "roslaunch leju_launch .*controller_manager_config:=/home/lab/.lejulab/auto_start_config/profiles/amp_walk_v1/controller_manager.yaml"
```

适用场景：

- 按模型路径精确停止
- 因实物运行需要 `root` 权限，故停止时需要 `sudo`

## 4.1 手柄自启动配置切换

### 简要介绍

手柄自启动不会直接读取前端上传目录，而是固定读取当前激活配置：

```text
/root/.config/lejulab/auto_start_config/current/controller_manager.yaml
```

部署自启动服务时，会额外复制一份固定脚本：

```text
/root/.config/lejulab/auto_start_config/set_active_profile.sh
```

前端后续只需要调用这一个脚本，不需要再知道仓库路径。

### 脚本的两种用法

1. 前端导入一个完整模型目录，并替换激活 profile

```bash
sudo /root/.config/lejulab/auto_start_config/set_active_profile.sh \
  --import-from /tmp/project_A \
  --profile-name project_A \
  --source frontend
```

这个模式会做：

- 校验 `/tmp/project_A/controller_manager.yaml` 是否存在
- 将 `/tmp/project_A` 整目录复制到  
  `/root/.config/lejulab/auto_start_config/profiles/project_A`
- 如果 `project_A` 已存在，则整目录替换，而不是叠加覆盖
- 原子更新 `current`
- 写审计信息到  
  `/root/.config/lejulab/auto_start_config/current.meta.json`

2. 直接将 `current` 指到一个已有完整目录，适用于 joy 自启动部署

```bash
sudo /root/.config/lejulab/auto_start_config/set_active_profile.sh \
  --target-dir lejulab/src/leju-controllers/leju-rl-controller/config/${ROBOT_VERSION:-46} \
  --source deploy
```

这个模式主要用于：

- 自启动部署脚本将激活配置重置到仓库默认目录
- 手工排查时切回一个已存在的完整目录

### 前端使用注意事项

- 前端不要对已有 `profiles/project_A` 执行 `cp -r` 叠加覆盖，这会留下历史残留文件
- 正确做法是先准备一个完整目录，再调用 `--import-from ... --profile-name ...`
- 自定义 profile 必须是一整套完整配置目录，不能只上传一个 `controller_manager.yaml`
- `current` 是唯一 source of truth，自启动只看它
- `current.meta.json` 只用于记录谁改了、什么时候改，不参与启动判定

## 4.2 参数文件夹结构

### 配置文件

LejuLab 部署目录包含的文件如下：

- 一个 `controller_manager.yaml`
- 一个控制器 YAML
- 一个 ONNX 模型
- Mimic 类模型额外带一个 CSV 轨迹文件

LejuLab 部署目录：

```text
/home/lab/.lejulab/auto_start_config/profiles/
```

每个模型一个独立目录，例如：

- `/home/lab/.lejulab/auto_start_config/profiles/amp_walk_v1/`
- `/home/lab/.lejulab/auto_start_config/profiles/mimic_hpny_v1/`

### 推荐目录结构

AMP 以仓库当前 `46` 版本为默认参考：

```text
/home/lab/.lejulab/auto_start_config/profiles/amp_walk_v1/
├── controller_manager.yaml
├── config_amp.yaml
└── amp_walk_v1.onnx
```

Mimic 以仓库当前 `14` 版本 `HPNY` 为默认参考：

```text
/home/lab/.lejulab/auto_start_config/profiles/mimic_hpny_v1/
├── controller_manager.yaml
├── config_mimic_HPNY_dance.yaml
├── mimic_hpny_v1.onnx
└── HPNY_dance.csv
```

说明：

- `controller_manager.yaml`：模型入口文件，启动时只需要知道这个路径
- `config_amp.yaml` 或 `config_mimic_*.yaml`：单个控制器定义文件
- `*.onnx`：模型文件
- `*.csv`：Mimic 轨迹文件

### 路径写法建议

推荐优先使用**相对路径**：

- `controller_manager.yaml` 运行时传入绝对路径
- `config_amp.yaml` 或 `config_mimic_*.yaml` 相对于 `controller_manager.yaml`
- `policy_path`、`motion.file` 相对于控制器 YAML

AMP 示例：

```yaml
config: "config_amp.yaml"
policy_path: "amp_walk_v1.onnx"
```

Mimic 示例：

```yaml
config: "config_mimic_HPNY_dance.yaml"
policy_path: "mimic_hpny_v1.onnx"
file: "HPNY_dance.csv"
```

### 重要注意事项

- YAML 里不要写 `~/.lejulab/...`
- 每个模型目录只保留一个入口 `controller_manager.yaml`
- `controller_manager.yaml` 只建议保留**一个启用中的控制器**

## 5. 配置模板说明

### 5.1 `controller_manager.yaml`

AMP 以 `46` 版本默认模板为参考：

```yaml
loop_dt: 0.001                    # controller_manager 自身循环周期，通常 1 kHz
default_controller: "amp"        # 默认启用的控制器名称，必须与下面 name 一致

controllers:
  - name: "amp"                  # 控制器标识
    type: "GenericRLController"  # 当前 LejuLab 部署使用的控制器类型
    config: "config_amp.yaml"    # 控制器 YAML，相对当前 controller_manager.yaml
    enabled: true                # 是否启用，LejuLab 部署建议只保留一个 true
```

Mimic 以 `14` 版本 HPNY 默认模板为参考：

```yaml
loop_dt: 0.001                                      # controller_manager 自身循环周期，通常 1 kHz
default_controller: "mimic_hpny"                    # 默认启用的控制器名称

controllers:
  - name: "mimic_hpny"                              # 控制器标识
    type: "GenericRLController"                     # 当前 LejuLab 部署使用的控制器类型
    config: "config_mimic_HPNY_dance.yaml"          # 控制器 YAML，相对当前 controller_manager.yaml
    enabled: true                                   # 是否启用
```

### 5.2 `config_amp.yaml`

AMP 以当前仓库 `46` 版本模板为默认参考：

```yaml
HumanoidRobotCfg:
  inference_engine: "openvino"              # 推理引擎，可选 openvino / onnxruntime
  loop_dt: 0.001                            # 控制循环周期，通常 1 kHz
  policy_path: "amp_walk_v1.onnx"           # ONNX 模型路径，相对当前 config_amp.yaml；AMP 部署推荐直接放在模型目录根下

  env:
    policy_dt: 0.02                         # 策略执行周期，通常 50 Hz

    robot:
      enable_arm_controller: true           # 是否启用手臂控制器
      arm_joint_names: ["..."]              # 手臂控制器接管的关节名列表

      joint_names: ["..."]                  # 策略使用的关节顺序，必须与训练时一致
      joint_direction: [1.0, 1.0]           # 各关节方向系数，通常为 1 或 -1
      joint_default_pos: [0.0, 0.0]         # 默认姿态 / 站立姿态
      joint_torque_limit: [80.0, 80.0]      # 力矩限幅
      actuator_kp: [40.0, 40.0]             # 位置增益
      actuator_kd: [2.0, 2.0]               # 速度增益
      actuator_control_mode: [2, 2]         # 控制模式：0=CST, 1=CSV, 2=CSP
      action_scale: [0.25, 0.25]            # 模型动作缩放系数

    observations:
      history_length: 1                     # 观测历史帧数
      stack_order: "classic"                # 观测堆叠方式
      terms:
        base_ang_vel:
          scale: 1.0                        # 基座角速度观测的缩放系数
          clip: [-9999.0, 9999.0]           # 基座角速度观测的裁剪范围
        projected_gravity:
          scale: 1.0                        # 重力投影观测
          clip: [-9999.0, 9999.0]
        velocity_commands:
          scale: 1.0                        # 速度命令观测
          clip: [-9999.0, 9999.0]
        cmd_stance:
          scale: 1.0                        # 站立 / 行走状态观测
          clip: [0.0, 1.0]
        joint_pos:
          scale: 1.0                        # 关节位置观测
          clip: [-9999.0, 9999.0]
        joint_vel:
          scale: 1.0                        # 关节速度观测
          clip: [-9999.0, 9999.0]
        actions:
          scale: 1.0                        # 上一帧动作观测
          clip: [-18.0, 18.0]

    command_range:
      lin_vel_x: [-1.0, 1.0]                # 前后速度范围
      lin_vel_y: [-0.6, 0.6]                # 左右速度范围
      ang_vel_z: [-0.3, 0.3]                # 转向角速度范围

  arm_controller:
    enabled: true                           # 是否启用手臂插值控制器
    leg_joint_count: 12                     # 腿部关节数
    waist_joint_count: 0                    # 腰部关节数
    interpolation_velocity: 1.0             # 手臂回中插值速度
    min_duration: 0.2                       # 最短插值时长
    max_duration: 2.0                       # 最长插值时长

  cmd_stance:
    smart_stop:
      enabled: true                         # 是否启用智能停步判断
      torso_velocity_threshold: 0.05        # 躯干速度阈值
      feet_alignment_threshold: 0.08        # 双脚对齐阈值
    velocity_magnitude_threshold: 0.01      # 摇杆速度低于该值视为接近停止

  velocity_input_manager:
    timeout_sec: 1.0                        # 速度输入超时时间
    priorities:
      vr_cmd_vel: 0                         # VR 速度源优先级，数值越小优先级越高
      xbox_joy: 1                           # 手柄速度源优先级
```

### 5.3 `config_mimic_HPNY_dance.yaml`

Mimic 以当前仓库 `14` 版本 `HPNY` 模板为默认参考：

```yaml
HumanoidRobotCfg:
  inference_engine: "onnxruntime"                    # 推理引擎，可选 openvino / onnxruntime
  loop_dt: 0.001                                     # 控制循环周期，通常 1 kHz
  policy_path: "mimic_hpny_v1.onnx"                  # ONNX 模型路径，相对当前 config_mimic_HPNY_dance.yaml；Mimic 部署推荐直接放在模型目录根下
  motion_data_path: "HPNY_dance.csv"                 # 动作轨迹 CSV，相对当前 config_mimic_HPNY_dance.yaml；Mimic 部署推荐直接放在模型目录根下

  motion:
    default_motion: "HPNY_dance"                     # 默认动作名称
    motions:
      - name: "HPNY_dance"                           # 动作名称
        file: "HPNY_dance.csv"                       # 对应轨迹 CSV，相对当前 config_mimic_HPNY_dance.yaml

  env:
    policy_dt: 0.02                                 # 策略执行周期，通常 50 Hz

    robot:
      residual_action: true                         # 是否启用残差动作，true 表示动作叠加到参考轨迹上
      joint_names: ["..."]                          # 策略使用的关节顺序，必须与训练时一致
      joint_direction: [1.0, 1.0]                   # 各关节方向系数
      joint_default_pos: [0.0, 0.0]                 # 默认姿态
      joint_torque_limit: [80.0, 80.0]              # 力矩限幅
      actuator_kp: [40.0, 40.0]                     # 位置增益
      actuator_kd: [2.0, 2.0]                       # 速度增益
      actuator_control_mode: [0, 0]                 # 控制模式：0=CST, 1=CSV, 2=CSP
      action_scale: [0.25, 0.25]                    # 模型动作缩放系数

    observations:
      history_length: 1                             # 观测历史帧数
      stack_order: "classic"                        # 观测堆叠方式
      terms:
        motion_command:
          scale: 1.0                                # 轨迹目标关节位置 / 速度观测
          clip: [-5000.0, 5000.0]
        motion_target_height:
          scale: 1.0                                # 轨迹目标身体高度观测
          clip: [-5000.0, 5000.0]
        motion_anchor_ori_b:
          scale: 1.0                                # 轨迹目标姿态观测
          clip: [-5000.0, 5000.0]
        projected_gravity:
          scale: 1.0                                # 重力投影观测
          clip: [-5000.0, 5000.0]
        base_ang_vel:
          scale: 1.0                                # 基座角速度观测
          clip: [-5000.0, 5000.0]
        joint_pos:
          scale: 1.0                                # 关节位置观测
          clip: [-5000.0, 5000.0]
        joint_vel:
          scale: 1.0                                # 关节速度观测
          clip: [-5000.0, 5000.0]
        actions:
          scale: 1.0                                # 上一帧动作观测
          clip: [-5000.0, 5000.0]
```
