# DDS 独立深度行走控制器实施计划

> **供代理执行者使用：** 按任务逐项执行；每个任务完成后运行该任务的验证命令，再进入下一任务。所有修改只发生在目标仓库，禁止修改源 ROS 仓库，禁止执行 `git push`。

**目标：** 在目标仓库中新增无 ROS 依赖的 S17 深度行走控制器，使用已确认的 `[92550] -> [25]` 模型契约，并完成构建、仿真、安全切换和回归验证。

**架构：** 新增专用 `DepthWalkController` 和四个深度基础模块，不修改 `GenericRLController` 的 AMP/mimic 行为。仿真和未来真实相机均通过 `DepthObservationProvider` 进入同一图像处理、历史缓存和观测构造链路。

**技术栈：** C++17、Eigen、yaml-cpp、目标仓库现有 ONNX Runtime/OpenVINO、MuJoCo、lejusdk-lowlevel、现有 DDS/RobotData/ControllerManager。

---

## 任务 1：建立实施前基线

**文件：**

- 只读：目标仓库 Git 状态、`leju-rl-controller/CMakeLists.txt`
- 只读：`config/17/controller_manager.yaml`
- 只读：`config/17/controllers/config_amp.yaml`

- [ ] **步骤 1：确认工作区和基线提交**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git status --short --branch
git rev-parse HEAD
'
```

预期：工作区只有已知的本地设计文档提交，不能覆盖其他用户修改。

- [ ] **步骤 2：建立当前 AMP/mimic 编译基线**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
cmake --build build --target run_rl_controller -j2
'
```

预期：现有 `run_rl_controller` 构建成功。

- [ ] **步骤 3：记录基线**

将提交号、编译命令和结果写入本地实施日志
`docs/superpowers/verification/depth-walk-baseline.txt`，只在目标仓库保存，不上传。

- [ ] **步骤 4：提交基线记录**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git add -f docs/superpowers/verification/depth-walk-baseline.txt
git commit -m "test: record depth controller migration baseline"
'
```

## 任务 2：实现深度数据基础类型与处理器

**文件：**

- 创建：`src/leju-controllers/leju-rl-controller/include/leju-rl-controller/depth/depth_frame.h`
- 创建：`src/leju-controllers/leju-rl-controller/include/leju-rl-controller/depth/depth_image_processor.h`
- 创建：`src/leju-controllers/leju-rl-controller/src/depth/depth_image_processor.cpp`
- 创建：`src/leju-controllers/leju-rl-controller/tests/depth/test_depth_image_processor.cpp`

- [ ] **步骤 1：先写失败测试**

测试必须覆盖以下输入输出：

```cpp
TEST(DepthImageProcessor, ConvertsMillimetersAndNormalizes) {
  DepthFrame frame = MakeFrame16U(4, 4, 1000);
  DepthImageProcessor processor(MakeTestConfig(2, 2));
  auto result = processor.process(frame);
  ASSERT_TRUE(result.ok);
  ASSERT_EQ(result.image.size(), 4u);
  for (float value : result.image) EXPECT_FLOAT_EQ(value, 1.0f / 2.5f);
}

TEST(DepthImageProcessor, RejectsNaNAndWrongShape) { /* expect invalid */ }
TEST(DepthImageProcessor, ClipsToConfiguredRange) { /* 0 and >2.5 become bounds */ }
```

- [ ] **步骤 2：运行测试确认失败**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx/build
cmake .. -DBUILD_TESTING=ON
cmake --build . --target test_depth_image_processor -j2
ctest -R test_depth_image_processor --output-on-failure
'
```

预期：因类型和处理器尚未定义而失败。

- [ ] **步骤 3：实现最小处理器**

`DepthFrame` 必须保存 `width`、`height`、`unit`、`timestamp`、`sequence` 和
连续像素数组。处理器按以下顺序实现：

```text
单位转换 -> finite/范围检查 -> crop -> 最近邻 resize
-> 可选零值 mask/inpaint -> 可选 Gaussian blur
-> clamp(0, 2.5) -> /2.5 -> 36x64 尺寸检查
```

处理器必须返回结构化错误，不得抛出未捕获异常到 1 kHz 控制线程。

- [ ] **步骤 4：运行处理器测试**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx/build
cmake --build . --target test_depth_image_processor -j2
ctest -R test_depth_image_processor --output-on-failure
'
```

- [ ] **步骤 5：提交**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git add src/leju-controllers/leju-rl-controller/include/leju-rl-controller/depth \
        src/leju-controllers/leju-rl-controller/src/depth \
        src/leju-controllers/leju-rl-controller/tests/depth/test_depth_image_processor.cpp
git commit -m "feat: add transport-neutral depth image processor"
'
```

## 任务 3：实现 43 帧历史缓存和观测提供器

**文件：**

- 创建：`include/leju-rl-controller/depth/depth_history_buffer.h`
- 创建：`src/depth/depth_history_buffer.cpp`
- 创建：`include/leju-rl-controller/depth/depth_observation_provider.h`
- 创建：`src/depth/depth_observation_provider.cpp`
- 创建：`tests/depth/test_depth_history_buffer.cpp`
- 修改：`CMakeLists.txt`

- [ ] **步骤 1：写失败测试**

```cpp
TEST(DepthHistoryBuffer, FirstFrameFillsAllSlots) { /* 8 snapshots equal first */ }
TEST(DepthHistoryBuffer, SelectsOldestToNewest) {
  // push 43 frames with values 0..42
  // expect selected values 0,6,12,18,24,30,36,42
}
TEST(DepthHistoryBuffer, ResetClearsReady) { /* ready=false, no stale data */ }
TEST(DepthHistoryBuffer, SnapshotIsThreadSafe) { /* concurrent push/snapshot */ }
```

- [ ] **步骤 2：实现缓存**

缓存固定为 43 帧，每帧固定 2304 个 `float32`。快照复制后释放锁，禁止控制器持有内部可变引用。快照必须带：

```text
ready, timestamp, sequence, valid_ratio, 8*36*64 data
```

- [ ] **步骤 3：实现 provider**

`submitRawFrame()` 只负责校验、处理并写入缓存；`getSnapshot(now)` 只返回不可变副本，并根据 `depth_timeout_sec` 判断新鲜度。图像处理耗时写入原子统计字段。

- [ ] **步骤 4：运行测试**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx/build
cmake --build . --target test_depth_history_buffer -j2
ctest -R test_depth_history_buffer --output-on-failure
'
```

- [ ] **步骤 5：提交**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git add src/leju-controllers/leju-rl-controller/include/leju-rl-controller/depth \
        src/leju-controllers/leju-rl-controller/src/depth \
        src/leju-controllers/leju-rl-controller/tests/depth/test_depth_history_buffer.cpp \
        src/leju-controllers/leju-rl-controller/CMakeLists.txt
git commit -m "feat: add thread-safe depth history provider"
'
```

## 任务 4：迁移 S17 深度配置和模型资产

**文件：**

- 创建：`config/17/controllers/config_depth_walk.yaml`
- 创建：`config/17/policy/model_15_2026-07-27_18-27-25_187800.onnx`

- [ ] **步骤 1：编写 YAML**

YAML 必须明确包含：

```yaml
controller_type: DepthWalkController
loop_dt: 0.001
policy_dt: 0.02
inference_frequency: 50
policy_path: "policy/model_15_2026-07-27_18-27-25_187800.onnx"
single_obs_dim: 18510
frame_stack: 5
depth:
  history_frames: 8
  source_buffer_frames: 43
  selected_indices: [0, 6, 12, 18, 24, 30, 36, 42]
  height: 36
  width: 64
  max_depth_m: 2.5
  timeout_sec: 0.10
model:
  input_dim: 92550
  output_dim: 25
  action_dim: 21
  gait_frequency_index: 21
  velocity_estimate_start: 22
  velocity_estimate_dim: 3
```

同时迁移源端 action scale、joint direction、default pose、PD、torque limit、速度限制、stance、arm/waist 开关和观测顺序。

- [ ] **步骤 2：复制并校验模型**

模型必须从源仓库复制到目标 `config/17/policy/`，复制后在目标容器执行：

```bash
sha256sum config/17/policy/model_15_2026-07-27_18-27-25_187800.onnx
```

预期 SHA-256 与设计文档一致。

- [ ] **步骤 3：模型签名测试**

使用目标 OpenVINO Core 读取模型，断言：

```text
input shape == [92550]
input type == float32
output shape == [25]
output type == float32
```

- [ ] **步骤 4：提交**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git add config/17/controllers/config_depth_walk.yaml config/17/policy/model_15_2026-07-27_18-27-25_187800.onnx
git commit -m "feat: add S17 depth walk policy configuration"
'
```

## 任务 5：实现 DepthWalkController

**文件：**

- 创建：`include/leju-rl-controller/controllers/depth_walk_controller.h`
- 创建：`src/controllers/depth_walk_controller.cpp`
- 创建：`tests/controllers/test_depth_walk_observation.cpp`
- 创建：`tests/controllers/test_depth_walk_model_contract.cpp`

- [ ] **步骤 1：先写观测构造失败测试**

测试必须使用固定输入值，逐段断言以下偏移：

```cpp
EXPECT_EQ(obs.segment(0, 18432), expected_depth);
EXPECT_EQ(obs.segment(18432, 3), velocity_command);
EXPECT_EQ(obs.segment(18435, 5), command_phase);
EXPECT_EQ(obs.segment(18440, 3), body_ang_vel);
EXPECT_EQ(obs.segment(18443, 3), projected_gravity);
EXPECT_EQ(obs.segment(18446, 21), joint_pos_minus_default);
EXPECT_EQ(obs.segment(18467, 21), joint_vel);
EXPECT_EQ(obs.segment(18488, 21), previous_action);
EXPECT_EQ(obs[18509], frequency_phase);
EXPECT_EQ(obs.size(), 18510);
```

- [ ] **步骤 2：实现控制器配置和观测**

控制器必须复用 `ControllerBase::loadConfig`，从 `RobotState`、`ImuData`、速度命令和 provider 快照构造观测。深度快照不 ready 或过期时，返回安全状态，不构造未初始化观测。

- [ ] **步骤 3：实现 5 帧 frame stack**

使用固定长度容器保存 5 个单帧观测，reset 时全部初始化为零，写入顺序为旧到新。首次推理前必须确认容器已填充。

- [ ] **步骤 4：实现推理和 25 维输出解析**

推理前断言输入为 `92550`。推理后断言输出为 `25`，按以下代码语义解析：

```text
action = output[0..20]
gait_frequency_raw = output[21]
velocity_estimate = output[22..24]
```

速度估计只写入诊断快照，不修改动作或速度命令。

- [ ] **步骤 5：实现动作到 RobotCmd**

严格复用目标端关节顺序和动作输出规则：

```text
q_target = default_q + action * action_scale * per_joint_action_scale
tau = kp * (q_target - q) - kd * v
tau = clamp(tau, -torque_limit, torque_limit)
```

应用现有手臂、腰部控制器覆盖和 ControllerBase 输出模式，不复制 ROS 消息逻辑。

- [ ] **步骤 6：实现 reset、pause、resume 和安全状态**

reset 清空：

```text
policy_history
previous_action
phase
last_output
depth_ready/failure counters
```

推理错误、深度超时和模型输出错误均输出已经验证过的安全 `RobotCmd`。

- [ ] **步骤 7：运行控制器单测**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx/build
cmake --build . --target test_depth_walk_observation test_depth_walk_model_contract -j2
ctest -R "test_depth_walk_(observation|model_contract)" --output-on-failure
'
```

- [ ] **步骤 8：提交**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git add src/leju-controllers/leju-rl-controller/include/leju-rl-controller/controllers/depth_walk_controller.h \
        src/leju-controllers/leju-rl-controller/src/controllers/depth_walk_controller.cpp \
        src/leju-controllers/leju-rl-controller/tests/controllers/test_depth_walk_observation.cpp \
        src/leju-controllers/leju-rl-controller/tests/controllers/test_depth_walk_model_contract.cpp
git commit -m "feat: add standalone DDS depth walk controller"
'
```

## 任务 6：注册控制器并接入构建

**文件：**

- 修改：`src/leju-controllers/leju-rl-controller/src/controllers/controller_registry.cpp`
- 修改：`src/leju-controllers/leju-rl-controller/CMakeLists.txt`
- 修改：`config/17/controller_manager.yaml`

- [ ] **步骤 1：增加注册工厂**

新增类型字符串：

```text
DepthWalkController
```

工厂必须传入目标仓库解析后的配置路径，不得出现源仓库绝对路径或 ROS package path。

- [ ] **步骤 2：增加构建源文件和测试目标**

将深度源文件加入 `rl_controller_framework`，将控制器源文件加入控制器库，将深度测试加入 `BUILD_TESTING`。

- [ ] **步骤 3：增加 manager 条目**

在 `config/17/controller_manager.yaml` 增加：

```yaml
- name: "depth_walk"
  type: "DepthWalkController"
  config: "controllers/config_depth_walk.yaml"
  enabled: true
```

保持默认控制器为 AMP，不改变现有 AMP/mimic 条目。

- [ ] **步骤 4：编译并检查注册**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
cmake --build build --target run_rl_controller -j2
'
```

- [ ] **步骤 5：提交**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git add src/leju-controllers/leju-rl-controller/src/controllers/controller_registry.cpp \
        src/leju-controllers/leju-rl-controller/CMakeLists.txt \
        src/leju-controllers/leju-rl-controller/config/17/controller_manager.yaml
git commit -m "feat: register S17 depth walk controller"
'
```

## 任务 7：接入 MuJoCo 深度相机

**文件：**

- 创建或修改：`src/leju-mujoco-sim/src/depth_camera_source.*`
- 修改：`src/leju-mujoco-sim/src/leju_mujoco_sim.cc`
- 修改：`src/leju-mujoco-sim/CMakeLists.txt`
- 修改：S17 相机和楼梯场景 XML
- 创建：`config/17/controller_manager_depth_sim.yaml`

- [ ] **步骤 1：实现相机源**

使用目标 MuJoCo 的相机/射线接口生成原始深度帧，输出宽高、单位、时间戳和序号。相机线程不能直接修改控制器内存，只能调用 provider 的提交接口。

- [ ] **步骤 2：统一深度范围和坐标**

使用源端参数：

```text
min range = 0.17 m
max range = 2.5 m
策略输入归一化 = z_depth / 2.5
```

在进入处理器前完成 z-depth 转换，确保仿真像素布局与真实相机一致。

- [ ] **步骤 3：增加仿真配置**

仿真 manager 配置必须将 `depth_walk` 注册为可选控制器，并保持 AMP 为默认控制器。

- [ ] **步骤 4：构建仿真**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
cmake --build build --target leju-mujoco-sim run_rl_controller -j2
'
```

- [ ] **步骤 5：提交**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git add src/leju-mujoco-sim config/17/controller_manager_depth_sim.yaml
git commit -m "feat: provide MuJoCo depth camera source for S17"
'
```

## 任务 8：实现控制器切换和安全门禁

**文件：**

- 修改：`controller_manager.cpp` 或目标端现有切换门禁位置
- 修改：`depth_walk_controller.h/.cpp`
- 创建：`tests/controllers/test_depth_walk_switch_safety.cpp`

- [ ] **步骤 1：写失败测试**

测试以下情况必须拒绝切入或进入安全状态：

```cpp
TEST(DepthWalkSwitch, RejectsModelNotReady);
TEST(DepthWalkSwitch, RejectsStaleDepth);
TEST(DepthWalkSwitch, RejectsWrongHistoryShape);
TEST(DepthWalkSwitch, ClearsHistoryOnReset);
TEST(DepthWalkSwitch, BlendsDepthAndAmpCommands);
```

- [ ] **步骤 2：实现进入门禁**

`ControllerManager::requestSwitch("depth_walk", ...)` 在目标控制器 ready 检查失败时返回 false，并记录具体原因。

- [ ] **步骤 3：复用现有平滑切换**

通过 `getDualInferenceBlendReferenceCmd`、力矩限制和重算 mask 接入已有 RL-to-RL 五次曲线切换，不新增独立插值线程。

- [ ] **步骤 4：实现连续无效深度策略**

第一次超时：保持最近安全动作并记录告警；达到连续失败阈值：速度命令归零、进入安全站立或请求退出。策略必须可配置且测试覆盖。

- [ ] **步骤 5：运行测试并提交**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx/build
cmake --build . --target test_depth_walk_switch_safety -j2
ctest -R test_depth_walk_switch_safety --output-on-failure
'
```

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git add src/leju-controllers/leju-rl-controller tests/controllers/test_depth_walk_switch_safety.cpp
git commit -m "feat: gate depth controller switches on sensor readiness"
'
```

## 任务 9：静态 ROS 依赖检查和全量构建

**文件：**

- 只读检查新控制器及其直接依赖

- [ ] **步骤 1：静态扫描**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
grep -RInE "ros/|roscpp|rospy|ros::|NodeHandle|Publisher|Subscriber|ServiceClient|ros::Rate|ros::package|catkin|cv_bridge|image_transport|tf/" \
  src/leju-controllers/leju-rl-controller/include/leju-rl-controller/depth \
  src/leju-controllers/leju-rl-controller/src/depth \
  src/leju-controllers/leju-rl-controller/include/leju-rl-controller/controllers/depth_walk_controller.h \
  src/leju-controllers/leju-rl-controller/src/controllers/depth_walk_controller.cpp
'
```

预期：无输出。

- [ ] **步骤 2：构建全部 RL 目标**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
cmake --build build -j2
'
```

- [ ] **步骤 3：运行全量测试**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx/build
ctest --output-on-failure
'
```

- [ ] **步骤 4：确认 AMP/mimic 配置未改动**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git diff HEAD~1 -- config/17/controllers/config_amp.yaml config/17/controllers/config_mimic_dance.yaml
'
```

预期：无与本次迁移相关的行为变更。

## 任务 10：模型加载、启动、切换和仿真验收

**文件：**

- 创建：`docs/superpowers/verification/depth-walk-verification.md`
- 只读：运行日志和性能统计

- [ ] **步骤 1：模型加载冒烟测试**

启动目标控制器并确认日志包含：

```text
input_dim=92550
output_dim=25
action_dim=21
depth_history=8
depth_shape=36x64
inference_frequency=50
```

- [ ] **步骤 2：启动 S17 MuJoCo 仿真**

使用仓库已有 MuJoCo 启动方式和新增的 `controller_manager_depth_sim.yaml`，记录相机 ready、provider ready 和控制器 ready 日志。

- [ ] **步骤 3：验证控制器切换**

依次验证：

```text
AMP -> depth_walk
depth_walk -> AMP
```

切换前使用站立状态，确认关节目标和力矩连续，没有未初始化动作。

- [ ] **步骤 4：验证三类地形**

分别记录：

- 平路行走；
- 上楼梯；
- 下楼梯。

每种场景至少记录速度命令、控制器状态、深度帧序号、推理频率和异常计数。

- [ ] **步骤 5：验证异常深度**

注入以下情况：

```text
无首帧
超时
全零图
NaN/Inf
错误尺寸
连续丢帧
```

确认控制器拒绝危险动作并进入安全状态。

- [ ] **步骤 6：记录性能**

记录并写入验证文档：

```text
控制循环频率
策略推理频率
图像处理耗时
推理耗时
端到端观测延迟
DDS/内部数据延迟
丢帧和数据竞争结果
```

- [ ] **步骤 7：提交验证结果**

```bash
docker exec kuavo_rl_gpu bash -lc '
cd /root/low/lejulab_platform_zx
git add -f docs/superpowers/verification/depth-walk-verification.md
git commit -m "test: verify S17 depth walk controller"
'
```

## 交付前最终检查

- [ ] 新增深度控制器及直接依赖无 ROS 头文件、API、库和 topic；
- [ ] 输入维度、输出维度、观测顺序和历史方向有自动化测试；
- [ ] 模型文件 SHA-256 和 OpenVINO/ONNX Runtime 加载结果已记录；
- [ ] AMP、mimic 和其他已注册控制器全量构建与测试通过；
- [ ] 平路、上楼梯、下楼梯仿真结果已记录；
- [ ] 深度异常和控制器切换安全测试通过；
- [ ] 所有提交仅在本地目标仓库，未执行 `git push`。
