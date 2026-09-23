# leju-vr-control

Quest3 VR 遥操作控制包。提供两个可执行节点：

- `quest_vr_control_node` — Kuavo 双足机器人（>= 4.x）的增量/绝对式 VR 控制
- `quest_vr_abs_control_node` — **Roban/biped_s17** 专用绝对式 VR 控制（本文档主要介绍此节点）

---

## quest_vr_abs_control_node

### 功能概述

```
Quest3 ──UDP──> quest_udp_to_dds_node
                        │ DDS: /rt/quest/bone_poses
                        │      /rt/quest/joysticks   （手柄扳机/握把，OK 手势依赖）
                        ▼
              quest_vr_abs_control_node
              ├─ Quest3ArmInfoTransformer（骨骼→手部位姿 + 摇杆 OK 计数）
              ├─ ArmAbsoluteIK（Drake SNOPT 绝对式IK）
              └─ RobanVRAPI::publishArmJointCmd（发布关节指令）
                        │ DDS: ArmTrajectory
                        ▼
                   机器人执行器
```

### 依赖

- Drake（已安装于 `/home/test/drake_install_prefix`）
- lejusdk-vr（CycloneDDS）
- 环境变量 `LEJU_ASSETS_PATH` 指向 `leju_assets` 根目录

### 编译

```bash
cd /path/to/lejulab_platform_zx
cmake -B build \
  -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_DDS_RPC_TESTS=OFF -DBUILD_DDS_RPC_EXAMPLES=OFF
cmake --build build --target quest_vr_abs_control_node -j4
```

### 运行

#### 最简启动
```bash
export LEJU_ASSETS_PATH=/path/to/lejulab_platform_zx/src/leju_assets
export ROBOT_VERSION=17       # biped_s17
./build/src/leju-vr-control/quest_vr_abs_control_node
```

#### 配置参数（环境变量）

| 变量名 | 默认值 | 说明 |
|--------|--------|------|
| `LEJU_ASSETS_PATH` | —（必填）| leju_assets 根目录路径 |
| `ROBOT_VERSION` | `"17"` | 机器人版本，决定 URDF 路径后缀 |
| `VR_CONTROL_DT` | `0.01` | 控制周期（秒），默认 100Hz |
| `VR_UPPER_ARM_LEN` | `0.29` | 上臂长度（米），用于骨骼缩放 |
| `VR_LOWER_ARM_LEN` | `0.29` | 前臂长度（米） |
| `VR_SHOULDER_WIDTH` | `0.15` | 肩宽一半（米） |
| `VR_STATE_FILE` | —（可选）| 实时写入 IK 状态 JSON；除手/肘 IK 目标外还包含 Quest 原始骨骼世界坐标与缩放前躯干系坐标，供 `mujoco_viewer.py` 对比（青球=raw，黄球=pre，红/蓝球=缩放后） |
| `VR_SKIP_OK_GESTURE` | — | 设为 `1` 时跳过「等待 OK」（仅联调 DDS/无手柄） |
| `VR_DEBUG_JOYSTICK` | — | 设为 `1` 时在等待 OK 阶段打印扳机/握把，确认 `/rt/quest/joysticks` 是否有数据 |

#### 与 quest_udp_to_dds_node 联合启动示例

```bash
# 终端1：启动 VR 数据接收节点
export LEJU_ASSETS_PATH=/path/to/leju_assets
./build/src/leju-remote/quest_udp_to_dds_node

# 终端2：启动 IK 控制节点
export LEJU_ASSETS_PATH=/path/to/leju_assets
export ROBOT_VERSION=17
export VR_STATE_FILE=/tmp/vr_ik_state.json  # 可选：供 MuJoCo 可视化, 需要mujoco可视化必选
./build/src/leju-vr-control/quest_vr_abs_control_node
```

#### 连接流程
1. 节点启动后等待 Quest3 骨骼位姿（DDS **`/rt/quest/bone_poses`**）
2. 收到骨骼后等待 **OK**（与 `motion_capture_ik/scripts/tools/quest3_utils.py` 中 **手柄** 分支一致）  
   - **双手扳机同时按下**，数值均 **> 0.5**，并保持约 **50 帧**成功骨骼处理（约 1～2 秒，取决于循环节奏）  
   - 必须同时收到 **`/rt/quest/joysticks`**；若只有骨骼没有手柄 DDS，OK 计数不会前进  
   - 纯手势捏合（手指 IK）当前 **未** 在 C++ `Quest3ArmInfoTransformer` 中实现，需用手柄扳机路径  
3. OK 确认后进入遥操作，发布关节指令

#### 排查：卡在 Waiting for OK gesture

1. 确认 **`quest_udp_to_dds_node` 已运行**，Quest 与板子网络互通（UDP 端口见该节点日志）。
2. 开调试：`export VR_DEBUG_JOYSTICK=1`，看 `has_dds=yes` 且按扳机时 `LT/RT` 是否变化。若一直 `has_dds=no`，说明本机未订阅到手柄话题（检查是否同一 DDS 域、两进程是否都 `initialize` 成功）。
3. 两路话题名（`lejusdk-topic-pubsub`）：**`/rt/quest/bone_poses`**、**`/rt/quest/joysticks`**。若安装有 Cyclone DDS 工具，可用 `ddsperf` / `cyclonedds subscribe` 等查看域内是否有对应 topic 流量（具体命令依本机安装为准）。
4. 联调时可 **`export VR_SKIP_OK_GESTURE=1`** 跳过 OK，直接进主循环（确认 IK 与下游）。

---

## MuJoCo 可视化

提供两种可视化方案：

### 方案一：离线 IK 测试（无需真实 VR 设备）

用合成的正弦波目标位置测试 ArmAbsoluteIK 算法，在 MuJoCo 中实时显示：

```bash
# 终端1：启动 C++ IK 测试节点（合成VR输入）
export LEJU_ASSETS_PATH=/path/to/leju_assets
./build/src/leju-vr-control/test_arm_ik_node

# 终端2：启动 MuJoCo 可视化（与命令行 simulate 一样走 GLFW，不依赖 PyOpenGL/osmesa），DISPLAY需要根据实际情况设置
export LEJU_ASSETS_PATH=/path/to/leju_assets
DISPLAY=:1 python3 src/leju-vr-control/test/mujoco_viewer.py

# 或无显示器（仅打印；脚本内部会设 MUJOCO_GL=egl）
python3 src/leju-vr-control/test/mujoco_viewer.py --no-viewer
```

若曾设置 `MUJOCO_GL=osmesa` 导致 `OpenGL` / `glGetError` 报错：`mujoco_viewer.py` 会在导入前强制改为 `glfw`（有窗口）或 `egl`（`--no-viewer`）。也可手动指定：`VR_MUJOCO_GL=glx|glfw|egl`。

pip 版 `mujoco` 为 namespace 包时，`mujoco.viewer` 不能作为属性访问，脚本内已改为 `from mujoco import viewer`。

### 方案二：真实 VR + MuJoCo 全链路（一键启动脚本）

```bash
# 可选参数：--no-mujoco 表示不启动MuJoCo可视化
bash src/leju-vr-control/examples/vr_mujoco_demo/run_vr_demo.sh

# 不启动 MuJoCo 窗口
bash src/leju-vr-control/examples/vr_mujoco_demo/run_vr_demo.sh --no-mujoco
```

MuJoCo 窗口中可视化内容：
- **机器人手臂**：实时显示 IK 求解后的关节姿态（加载 biped_s17 完整模型）
- **红色球**：左手 VR 输入目标位置
- **蓝色球**：右手 VR 输入目标位置
- **橙色球**：左肘 VR 输入目标位置
- **青色球**：右肘 VR 输入目标位置

---

## IK 算法说明

`ArmAbsoluteIK`（`leju-ik` 库）是 Python `torso_ik.py::ArmIk` 的 C++ 移植版本：

- 加载 `biped_v3_arm.urdf`，将躯干焊接到世界坐标系
- 使用 Drake `InverseKinematics` + SNOPT 求解器
- 手部位置：软代价函数（权重 10·I）
- 肘部位置：软代价函数（权重 10·I）  
- 关节平滑：与上一帧解的二次误差（权重 0.1·I）
- 关节数量：8（左臂 4 + 右臂 4，对应 biped_s17）
