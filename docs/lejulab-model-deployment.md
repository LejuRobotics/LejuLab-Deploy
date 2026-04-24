# LejuLab 模型部署指南

## 1. 部署流程概览

当前推荐使用手柄自启动服务部署实物模型。自启动服务不会直接读取创作平台上传目录，而是固定读取当前激活配置：

```text
/root/.config/lejulab/auto_start_config/current/controller_manager.yaml
```

日常部署一个模型时，流程是：

1. 准备一个完整 profile 目录，目录内包含 `controller_manager.yaml`、控制器 YAML、ONNX 模型，Mimic 还需要 CSV 轨迹文件。
2. 将完整 profile 上传到机器人。
3. 使用 `set_active_profile.sh` 将该 profile 设置为当前激活配置。
4. 用户通过手柄自启动服务拉起当前激活配置对应的模型。

首次装机或服务更新时，还需要部署自启动服务。注意：服务部署脚本会编译工作区，并把当前激活配置重置到仓库默认配置目录；如果已经切换过创作平台上传的 profile，服务部署完成后需要再执行一次 `set_active_profile.sh` 切回目标 profile。

## 2. 编译与自启动服务

### 服务部署

自启动服务部署脚本需要在工作区根目录以 `root` 身份执行。脚本会自动完成编译、CycloneDDS / iceoryx 配置部署、服务安装和启动。

闭源仓库 `lejulab_platform`：

```bash
cd <lejulab_platform_workspace>
sudo ./src/leju-joystick/services/deploy_joy_autostart.sh
```

开源仓库 `lejulab`：

```bash
cd <lejulab_workspace>
sudo ./installed/share/leju-joystick/services/deploy_joy_autostart.sh
```

部署前如果需要让脚本重置到某个机器人版本的仓库默认配置，需要先设置 `ROBOT_VERSION`。

部署脚本会做这些事：

- 停止并禁用旧的 `lejulab_joy_monitor.service`
- 编译整个工作区
- 执行 `src/leju_launch/scripts/setup_cyclonedds_config.sh`
- 复制创作平台可直接调用的配置切换脚本到 `/root/.config/lejulab/auto_start_config/set_active_profile.sh`
- 将当前激活配置重置到仓库默认配置目录
- 安装、启用并启动新的 `lejulab_joy_monitor.service`

### 服务维护

常用维护命令：

```bash
sudo systemctl status lejulab_joy_monitor.service --no-pager
sudo journalctl -u lejulab_joy_monitor.service -f
sudo systemctl restart lejulab_joy_monitor.service
```

卸载服务时执行同一路径并追加 `--remove`：

```bash
cd <lejulab_platform_workspace>
sudo ./src/leju-joystick/services/deploy_joy_autostart.sh --remove

cd <lejulab_workspace>
sudo ./installed/share/leju-joystick/services/deploy_joy_autostart.sh --remove
```

## 3. 上传模型并切换激活配置

### 激活配置脚本

部署自启动服务时，会复制一份固定脚本：

```text
/root/.config/lejulab/auto_start_config/set_active_profile.sh
```

创作平台后续只需要调用这一个脚本，不需要再知道仓库路径。脚本会维护 profiles 目录和 `current` 入口：

```text
/root/.config/lejulab/auto_start_config/profiles/
/root/.config/lejulab/auto_start_config/current
/root/.config/lejulab/auto_start_config/current.meta.json
```

### 创作平台导入完整模型目录

创作平台上传完整模型目录后，使用 `--import-from` 导入并替换激活 profile：

```bash
sudo /root/.config/lejulab/auto_start_config/set_active_profile.sh \
  --import-from /tmp/project_A \
  --profile-name project_A \
  --source frontend
```

这个模式会做：

- 校验 `/tmp/project_A/controller_manager.yaml` 是否存在
- 将 `/tmp/project_A` 整目录复制到 `/root/.config/lejulab/auto_start_config/profiles/project_A`
- 如果 `project_A` 已存在，则整目录替换，而不是叠加覆盖
- 原子更新 `current`
- 写审计信息到
  `/root/.config/lejulab/auto_start_config/current.meta.json`

### 指向已有完整目录

也可以直接将 `current` 指到一个已有完整目录，适用于服务部署脚本重置默认配置，或手工切回仓库内置配置：

```bash
sudo /root/.config/lejulab/auto_start_config/set_active_profile.sh \
  --target-dir <workspace>/src/leju-controllers/leju-rl-controller/config/${ROBOT_VERSION:-46} \
  --source deploy
```

这个模式主要用于：

- 自启动部署脚本将激活配置重置到仓库默认目录
- 手工排查时切回一个已存在的完整目录

### 使用注意事项

- 创作平台不要对已有 `profiles/project_A` 执行 `cp -r` 叠加覆盖，这会留下历史残留文件
- 正确做法是先准备一个完整目录，再调用 `--import-from ... --profile-name ...`
- 自定义 profile 必须是一整套完整配置目录，不能只上传一个 `controller_manager.yaml`
- `current` 是唯一 source of truth，自启动只看它
- `current.meta.json` 只用于记录谁改了、什么时候改，不参与启动判定

## 4.1 自启动和手动启动

### 自启动使用方式

服务部署完成且当前激活配置切换正确后，实物机器人通过手柄自启动：

- 第一次按 `START`：校验 `/root/.config/lejulab/auto_start_config/current/controller_manager.yaml` 后启动实物运行
- 启动命令内部会显式传入 `controller_manager_config:=/root/.config/lejulab/auto_start_config/current/controller_manager.yaml`
- 第二次按 `START`：仅在 runtime 进入等待状态时调用 `startRuntime()`

### 手动启动

用户仍然可以选择手动启动。手动启动时要特别确认 `controller_manager_config` 是否指向自己想要的配置。

实物示例：

```bash
sudo su
source /opt/ros/noetic/setup.bash
source <lejulab_platform_workspace>/devel/setup.bash
roslaunch leju_launch load_real.launch \
  controller_manager_config:=/root/.config/lejulab/auto_start_config/current/controller_manager.yaml
```

仿真示例：

```bash
source /opt/ros/noetic/setup.bash
source <lejulab_platform_workspace>/devel/setup.bash
roslaunch leju_launch load_mujoco_sim.launch \
  controller_manager_config:=/root/.config/lejulab/auto_start_config/profiles/amp_walk_v1/controller_manager.yaml
```

说明：

- 实物运行需要 `root` 权限
- `controller_manager_config` 是 LejuLab 模型入口 YAML 的绝对路径
- 手动指定 profile 路径时，必须确认该目录就是目标模型目录；使用 `current/controller_manager.yaml` 时，必须先确认 `current` 已由 `set_active_profile.sh` 切到目标 profile

## 4.2 参数文件夹结构

### 配置文件

LejuLab 部署目录包含的文件如下：

- 一个 `controller_manager.yaml`
- 一个控制器 YAML
- 一个 ONNX 模型
- Mimic 类模型额外带一个 CSV 轨迹文件

LejuLab 部署目录：

```text
/root/.config/lejulab/auto_start_config/profiles/
```

每个模型一个独立目录，例如：

- `/root/.config/lejulab/auto_start_config/profiles/amp_walk_v1/`
- `/root/.config/lejulab/auto_start_config/profiles/mimic_hpny_v1/`

### 推荐目录结构

AMP 以仓库当前 `46` 版本为默认参考：

```text
/root/.config/lejulab/auto_start_config/profiles/amp_walk_v1/
├── controller_manager.yaml
├── config_amp.yaml
└── model.onnx
```

Mimic 以仓库当前 `14` 版本 `HPNY` 为默认参考：

```text
/root/.config/lejulab/auto_start_config/profiles/mimic_hpny_v1/
├── controller_manager.yaml
├── config_mimic.yaml
├── model.onnx
└── trajectory.csv
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
policy_path: "model.onnx"
```

Mimic 示例：

```yaml
config: "config_mimic.yaml"
policy_path: "model.onnx"
file: "trajectory.csv"
```

### 重要注意事项

- YAML 里不要写 `~/.lejulab/...` 或 `~/.config/...`
- 每个模型目录只保留一个入口 `controller_manager.yaml`
- `controller_manager.yaml` 只建议保留**一个启用中的控制器**

## 4.3 Mimic 训练配置转换脚本

训练仓库导出的 Mimic `env.yaml` 可以用脚本转换成 LejuLab 部署侧的 mimic 控制器 YAML：

```bash
python3 scripts/mimic_config_train_to_deploy.py \
  --env-yaml /path/to/env.yaml \
  --dance-name leju_deploy \
  --out /path/to/config_mimic.yaml
```

适用范围：

- 只适用于 **Mimic** 模型配置转换
- 不适用于 **AMP** 模型；AMP 的 `config_amp.yaml` 仍按 AMP 模板和模型自身配置维护
- 当前脚本内置 roban 2.2 / 17 的 mimic 部署关节顺序、`joint_direction` 和 `actuator_control_mode`

脚本输入：

- `--env-yaml`：训练侧导出的 `env.yaml`
- `--out`：输出的部署侧 mimic 控制器 YAML

常用可选参数：

- `--dance-name`：同时写入 `motion.default_motion` 和 `motion.motions[].name`；不传时默认 `leju_deploy`
- `--policy-path`：写入 `policy_path`；不传时默认 `model.onnx`
- `--motion-file`：写入 `motion.motions[].file`；不传时默认 `trajectory.csv`
- `--inference-engine onnxruntime`：显式使用 ONNX Runtime；不传时默认 `openvino`，且参数值必须精确为 `onnxruntime`
- `--template`：可选旧配置迁移参数，不是必需；仅用于复用旧 YAML 中的 `joint_direction`

扁平化部署默认输出：

```yaml
HumanoidRobotCfg:
  inference_engine: openvino
  policy_path: model.onnx
  motion:
    default_motion: leju_deploy
    motions:
      - name: leju_deploy
        file: trajectory.csv
```

因此创作平台准备 Mimic 部署目录时，通常放置：

```text
<profile>/
├── controller_manager.yaml
├── config_mimic.yaml          # 由 mimic_config_train_to_deploy.py 生成
├── model.onnx
└── trajectory.csv
```

注意：

- 脚本只生成单个 mimic 控制器 YAML，不生成 `controller_manager.yaml`
- `model.onnx` 需要来自本次训练导出的策略模型
- `trajectory.csv` 需要由训练/动作数据转换流程单独生成，并与部署 mimic YAML 的关节顺序一致

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
