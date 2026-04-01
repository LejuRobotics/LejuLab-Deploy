# LejuLab


## 系统架构

```
+-----------------------------------------------------------------------------------+
|                                Controllers                                        |
+-----------------------------------------------------------------------------------+
|                                                                                   |
|                        +-----------------------------+                            |
|                        |     RLDemoController        |                            |
|                        |                             |                            |
|                        |     - ONNX Policy Load      |                            |
|                        |     - OpenVINO Inference    |                            |
|                        |     - Observation Compute   |                            |
|                        |     - Action Scaling        |                            |
|                        +-----------------------------+                            |
|                                                                                   |
+-----------------------------------------------------------------------------------+
                                       |
                        +--------------+--------------+
                        |                             |
                        v                             |
             publishRobotCmd(cmd)          subscribeXxx(callback)
                  [Send Cmd]                   [Recv Data]
                        |                             |
                        v                             |
+-----------------------------------------------------------------------------------+
|                                  lejusdk                                          |
+-----------------------------------------------------------------------------------+
|                                                                                   |
|    +-----------------------------------------------------------------------+      |
|    |                      liblejusdk-lowlevel.so                           |      |
|    |                                                                       |      |
|    |    +-----------------+       +----------------------------------+     |      |
|    |    |  GlobalRobot    |       |          RobotBaseAPI            |     |      |
|    |    |  (Singleton)    |------>|                                  |     |      |
|    |    +-----------------+       |  subscribeImuData(callback)      |     |      |
|    |                              |  subscribeRobotState(callback)   |     |      |
|    |                              |  subscribeJoyData(callback)      |     |      |
|    |                              |  subscribeHardwareState(callback)|     |      |
|    |                              |  publishRobotCmd(cmd)            |     |      |
|    |                              |                                  |     |      |
|    |                              |       +--------------+           |     |      |
|    |                              |       |KuavoHumanoid |           |     |      |
|    |                              |       +--------------+           |     |      |
|    |                              +----------------------------------+     |      |
|    +-----------------------------------------------------------------------+      |
|                                                                                   |
+-----------------------------------------------------------------------------------+
                        |                             ^
                        |                             |
                        v                             |
              +-----------------+           +-----------------+
              |    RobotCmd     |           |    ImuData      |
              |  - q[]  (pos)   |           |    RobotState   |
              |  - v[]  (vel)   |           |    JoyData      |
              |  - tau[] (torq) |           |    HardwareState|
              |  - kp[] (stiff) |           +-----------------+
              |  - kd[] (damp)  |                   ^
              |  - modes[]      |                   |
              +-----------------+                   |
                        |                           |
                        v                           |
+-----------------------------------------------------------------------------------+
|                              Hardware / Sim                                       |
+-----------------------------------------------------------------------------------+
|                                                                                   |
|    +---------------------------+       +---------------------------+              |
|    |     leju-mujoco-sim       |       |      leju-hardware        |              |
|    |                           |       |                           |              |
|    |   +-------------------+   |       |   +-------------------+   |              |
|    |   | MuJoCo Physics    |   |       |   | EtherCAT Driver   |   |              |
|    |   | Sensor Simulation |   |       |   | IMU Data Collect  |   |              |
|    |   | Motor Execution   |   |       |   | Joint Feedback    |   |              |
|    |   +-------------------+   |       |   +-------------------+   |              |
|    |                           |       |                           |              |
|    |       [Simulation]        |       |      [Real Robot]         |              |
|    +---------------------------+       +---------------------------+              |
|                                                                                   |
+-----------------------------------------------------------------------------------+
```

## 目录结构

```bash
tree src -d -L 2
src
├── leju_assets       # 资源模块，自动同步，不要编辑
│   ├── include
│   ├── models
│   └── src
├── leju-controllers  # 控制器
│   ├── leju-dummy-controller
│   └── leju-rl-controller
├── leju_launch       # 提供一键启动所有必要的功能模块
│   ├── launch
│   └── scripts
└── lejusdk           # Leju SDK，自动同步，不要编辑
    ├── examples
    ├── lejusdk-highlevel  # 高层控制 SDK（敬请期待）
    ├── lejusdk-lowlevel   # 底层控制 SDK，提供电机控制与状态读取、IMU 传感器状态读取接口等
    └── lejusdk-utils      # SDK 工具类, 提供辅助函数
```

## LejuSDK 文档

LejuSDK 是 Leju 机器人软件开发工具包，提供多层次的机器人控制接口。详细文档请参考 [LejuSDK 文档](./docs/leju-sdk/index.html)。

您可以clone本仓库在网站打开LejuSDK文档进行浏览

## 依赖安装

```bash
sudo apt-get update && sudo apt-get install -y \
    build-essential cmake \
    libacl1-dev libncurses5-dev
```

## 编译

```bash
source installed/setup.bash  # !!! IMPORTANT !!! 非常重要,不可省略
# source installed/setup.zsh  # !!! IMPORTANT !!! 非常重要,不可省略 如果是zsh
catkin build
sudo su # 实物需要在root用户下运行
```

## 部署 iceoryx 共享内存（首次使用需执行）

项目支持通过 iceoryx 共享内存加速进程间通信，建议部署以获得更优的实时性能：

```bash
./src/leju_launch/scripts/setup_cyclonedds_config.sh
```

部署脚本会完成以下操作：
- 安装 RouDi 守护进程为系统服务（开机自启）
- 部署 CycloneDDS 配置文件到 `/etc/cyclonedds/`
- 设置 `CYCLONEDDS_URI` 环境变量默认为 `cyclonedds_shm.xml`

> **注意：** 部署完成后需要**注销当前用户重新登录**或**重启系统**才能生效。

卸载：

```bash
./src/leju_launch/scripts/setup_cyclonedds_config.sh --remove
```

## 运行

### 通用说明

- **选择机器人版本（数值定义参考 `lejusdk-utils/robot_version.hpp` 中 `RobotVersions` 常量）**
  - `export ROBOT_VERSION=14`  ：Roban 2 代基础版（`RobotVersions::ROBAN2_BASE`）
  - `export ROBOT_VERSION=46`  ：Kuavo 4 代 UAE 版本（`RobotVersions::KUAVO4_UAE`）
  - `export ROBOT_VERSION=52`  ：Kuavo 5 代基础版（`RobotVersions::KUAVO5_BASE`）
- **手柄控制（通用）**
  - `start`：从待机/准备状态切换到运行/站立状态
  - `back`：进入安全停机/关节松弛状态
  - 其他按键和摇杆：根据不同控制器（如 RL demo / mimic）实现行走、转向、舞蹈等功能，详见对应控制器文档

### Mujoco 仿真

```bash
source devel/setup.bash
export ROBOT_VERSION=14
roslaunch leju_launch load_mujoco_sim.launch
```

- 通过如下命令启动控制器、Mujoco 仿真器和手柄控制等功能包
- 根据终端提示，按下`start`按键
- 点击 Mujoco 仿真中的 `Run` 运行按钮
- tips: 如果开始时机器人倒地，可以先`Pause`和`Reset`仿真，然后再`Run`
- **warnings: 由于用户层混用电机 kp/kd，故使用 Kuavo 4 代 UAE 版本和Kuavo 5 代基础版的仿真时需要将  `src/leju-controllers/leju-rl-controller/config/52/config_amp.yaml` 和 `src/leju-controllers/leju-rl-controller/config/46/config_amp.yaml` 中 `hardware_override_kp_kd` 字段注释。**

### 实物机器人

```bash
sudo su  # 需要 root 权限
source devel/setup.bash
export ROBOT_VERSION=14
roslaunch leju_launch load_real.launch
```

- 对于实物机器人，也许您首先需要对电机进行零点标定，但这并不是必须的，因为机器人在出厂时已经标定完毕，如果存在如下情况您可手动执行标定工具重新进行标定:
  - **准备站立时，发现关节角度与零点位置位置存在偏差**
  - 硬件维修更换电机
- 电机零点标定工具参考文档: [电机零点标定工具](./docs/howto-use-motor-cali-tool.md)
- 拉起机器人背后的急停按钮，执行上述命令
- 将移位架升起，等待机器人进入膝盖微曲状态
- 降低移位架，让机器人脚掌刚刚好接触地面
- 使用手柄 `start` 按键使机器人站立
- **结束使用机器人请按手柄 `back` 按键**

### Roban2.1 运行示例

#### Mimic 舞蹈

需要将`src/leju-controllers/leju-rl-controller/config/14/controller_manager.yaml`中的配置改为:
```bash
default_controller: "mimic"  
```

待机器人站立之后，按下西瓜键，即可播放舞蹈，播放结束后再次按西瓜键可重复播放。

#### AMP 行走

需要将`src/leju-controllers/leju-rl-controller/config/14/controller_manager.yaml`中的配置改为:
```bash
default_controller: "amp"
```

待机器人站立之后，左摇杆控制前后左右，右摇杆控制左右转向。

#### 运行

运行命令:
```bash
# 仿真
source devel/setup.bash
export ROBOT_VERSION=14
roslaunch leju_launch load_mujoco_sim.launch

# 实物机器人
sudo su  # 需要 root 权限
source devel/setup.bash
export ROBOT_VERSION=14
roslaunch leju_launch load_real.launch
```

### Kuavo4pro/Kuavo5 AMP 运行示例

#### 仿真运行

```bash
source devel/setup.bash

# kuavo5
export ROBOT_VERSION=52
roslaunch leju_launch load_mujoco_sim_rl_demo.launch

# kauvo4pro
export ROBOT_VERSION=46
roslaunch leju_launch load_mujoco_sim_rl_demo.launch
```

- **注意: 仿真运行需要将 `src/leju-controllers/leju-rl-controller/config/52/config_amp.yaml` 和 `src/leju-controllers/leju-rl-controller/config/46/config_amp.yaml` 中 `hardware_override_kp_kd` 字段注释。**
- 根据终端提示，按下 `start`按键
- 点击 Mujoco 仿真中的 `Run` 运行按钮
- 手柄左摇杆控制前后左右，右摇杆控制左右转向
- **结束使用机器人请按手柄 `back` 按键**

#### 实物运行

```bash
sudo su  # 需要 root 权限
source devel/setup.bash
# export ROBOT_VERSION=46

# kuavo5
export ROBOT_VERSION=52
roslaunch leju_launch load_real_rl_demo.launch

# kuavo4pro
export ROBOT_VERSION=46
roslaunch leju_launch load_real_rl_demo.launch

```
- 拉起机器人背后的急停按钮，执行上述命令
- 将移位架升起，等待机器人进入膝盖微曲状态
- 降低移位架，让机器人脚掌刚刚好接触地面
- 使用手柄 `start` 按键使机器人站立
- 手柄左摇杆控制前后左右，右摇杆控制左右转向
- **结束使用机器人请按手柄 `back` 按键**


## 输入控制设备

### 手柄

![](./docs/images/joystick.png)


## 已知问题
- 用户层和电机层的 kp/kd 混用：在实物上，CSP 模式当前在用户层下发的 joint_cmd 中 kp/kd，实际是电机的 kp/kd，其包含腿部和腰部关节
  - 即将修复: 全部统一为控制器即用户层的 kp/kd
- CST/CSP 不纯粹：用户层 CSP 下发的 joint_cmd 中 joint_q 是关节当前位置而非目标位置。未修改前，用户层 CST 的 kp/kd 下发非 0 数据，导致硬件又计算一次 PD

## 故障排除

### iceoryx 共享内存错误

如果运行时出现以下错误：

```
[leju-mujoco-sim] 错误: iceoryx 共享内存已启用但 RouDi 未运行!
[leju-mujoco-sim] 请先部署: ./src/leju_launch/scripts/setup_cyclonedds_config.sh
[leju-mujoco-sim] 或手动启动: ./src/leju_launch/scripts/start_roudi.sh
```

**解决方案：**

1. **首次使用**：执行 [部署 iceoryx 共享内存](#部署-iceoryx-共享内存首次使用需执行) 部分的部署脚本，然后注销重新登录或重启系统
2. **已部署但未生效**：确认已注销重新登录，或手动启动 RouDi：
   ```bash
   ./src/leju_launch/scripts/start_roudi.sh
   ```
3. **检查服务状态**：
   ```bash
   systemctl status iox-roudi
   ```