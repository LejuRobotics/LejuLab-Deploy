# 深度视觉输入对齐与站立稳定性实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**目标：** 将 DDS/MuJoCo 深度图像处理链路逐项对齐源端 ROS 实现，确认 18510 维策略输入完全一致，并解决深度控制器启动后摔倒问题。

**架构：** 保留现有 `DepthDdsPublisher → DepthObservationProvider → DepthImageProcessor → DepthHistoryBuffer → DepthWalkController` 边界。先建立可复现的原始帧、处理帧、历史帧和最终观测快照，再修改最小差异；AMP 和 mimic 不参与深度图像路径。

**技术栈：** MuJoCo 3.2、DDS、C++、OpenVINO、Eigen、GoogleTest、PGM/CSV 诊断文件。

---

### Task 1: 固化源端图像处理基准

**检查文件：**
- `/home/lejurl/work/10/kuavo-ros-control/src/image_processing/scripts/depth_downsample.py`
- `/home/lejurl/work/10/kuavo-ros-control/src/image_processing/scripts/depth_inpainter.py`
- `/home/lejurl/work/10/kuavo-ros-control/src/mujoco/src/mujoco_node.cc`
- `/home/lejurl/work/10/kuavo-ros-control/src/mujoco/include/mujoco_cpp/depth_camera_config.h`

- [ ] 记录原始图像编码、单位、分辨率、相机 FOV、裁剪参数、resize 插值、无效值定义、inpaint 参数、Gaussian 参数、归一化和输出范围。
- [ ] 记录源端 43 帧缓冲的写入方向及 `[0,6,12,18,24,30,36,42]` 对应的时间顺序。
- [ ] 将基准写入 `docs/depth_image_alignment.md`，每个参数给出源文件和行号。
- [ ] 不修改源仓库。

### Task 2: 为目标端增加四层快照诊断

**修改文件：**
- `src/leju-mujoco-sim/src/leju_mujoco_sim.cc`
- `src/leju-controllers/leju-rl-controller/src/depth/depth_observation_provider.cpp`
- `src/leju-controllers/leju-rl-controller/src/depth/depth_image_processor.cpp`
- `src/leju-controllers/leju-rl-controller/src/depth/depth_history_buffer.cpp`

**新增测试：**
- `src/leju-controllers/leju-rl-controller/tests/depth/test_depth_alignment.cpp`

- [ ] 让 `--dump-depth` 同时输出原始射线帧、处理后单帧、8 帧历史拼接的 PGM/CSV；文件名包含层次和序号。
- [ ] 每层记录宽高、单位、min/max/mean、零值比例、时间戳和 sequence。
- [ ] 在 `DepthWalkController` 推理前输出一次最终 18510 维观测的分段摘要：深度 18432、command 3、phase 5、gyro 3、gravity 3、jointPos 21、jointVel 21、action 21、frePhase 1。
- [ ] 测试禁止未初始化帧进入推理，且深度超时必须保持安全命令。

### Task 3: 对齐相机射线与源端深度定义

**修改文件：**
- `src/leju-mujoco-sim/src/leju_mujoco_sim.cc`
- `src/leju-mujoco-sim/model/biped_s17/xml/biped_s17.xml`

- [ ] 核对 `camera_depth_frame` 是否为源端使用的头部光学坐标系；确认局部射线方向、xmat 乘法方向、水平/垂直 FOV 和图像上下方向。
- [ ] 用一个平面、一个楼梯和一个相机前方标记物做射线单元测试，确认距离是 z-depth 还是 ray length。
- [ ] 保持源端有效范围 0.17–2.5 m；超范围值统一按源端规则处理。
- [ ] 若坐标轴或图像上下翻转，加入明确的配置字段和测试，不在控制器中隐式翻转。

### Task 4: 对齐无效值、inpaint、Gaussian 和归一化

**修改文件：**
- `src/leju-controllers/leju-rl-controller/src/depth/depth_image_processor.cpp`
- `src/leju-controllers/leju-rl-controller/include/leju-rl-controller/depth/depth_image_processor.h`
- `src/leju-controllers/leju-rl-controller/config/17/controllers/config_depth_walk.yaml`

- [ ] 先按源端顺序执行：无效值 mask → 白点处理规则 → crop → nearest resize → inpaint → Gaussian 3×3 → clip `[0,2.5]` → `/2.5`。
- [ ] 明确处理 0、NaN、Inf、负值和 ray miss，禁止把无效值直接当成近距离障碍物。
- [ ] 保留现有测试语义；新增测试分别覆盖全空洞、单像素空洞、远距离裁剪、NaN/Inf 和 Gaussian 数值。
- [ ] 只有测试和源端数值都一致后才提交处理器修改。

### Task 5: 对齐历史帧时间顺序和 DDS 时间戳

**修改文件：**
- `src/leju-controllers/leju-rl-controller/src/depth/depth_history_buffer.cpp`
- `src/leju-controllers/leju-rl-controller/src/depth/depth_observation_provider.cpp`
- `src/leju-controllers/leju-rl-controller/tests/depth/test_depth_pipeline.cpp`

- [ ] 验证目标 43 帧环形缓冲的 oldest/newest 索引与源端 `depth_buf[self.selected_ids]` 完全一致。
- [ ] 不使用 DDS 接收时刻覆盖相机采集时刻；若消息没有硬件时间戳，记录明确的接收延迟并按超时策略拒绝旧帧。
- [ ] 用编号帧 `0..42` 测试最终输出必须是 `0,6,12,18,24,30,36,42`，并确认 frameStack 5 组单帧按源端时间顺序排列。

### Task 6: 验证 18510 维观测和 25 维输出契约

**修改文件：**
- `src/leju-controllers/leju-rl-controller/src/controllers/depth_walk_controller.cpp`
- `src/leju-controllers/leju-rl-controller/config/17/controllers/config_depth_walk.yaml`

- [ ] 逐段断言 18510 维观测位置和数值范围，确认 waist-first 的 jointPos/jointVel/action 逻辑只在策略空间使用。
- [ ] 逐项比较源端 commandPhase 与 frePhase，确认 gait frequency 原始值和变换值不混用。
- [ ] 检查模型输入 `[92550]`、输出 `[25]`、action `[0:21]`、gait `[21]`、velocity estimate `[22:25]`。
- [ ] 先用零速度、静态平面和固定深度帧离线推理，记录首个动作和 q_target；若首个动作造成大幅关节偏移，阻止进入控制器并报错。

### Task 7: 站立启动和安全门控验证

**修改文件：**
- `src/leju-controllers/leju-rl-controller/src/controllers/depth_walk_controller.cpp`
- `src/leju-controllers/leju-rl-controller/src/controllers/controller_manager.cpp`

- [ ] 深度历史未满、深度超时、观测未初始化、模型首次推理未完成时，持续发送安全保持命令。
- [ ] 首次有效推理前动作保持零；首次动作采用短时平滑 ramp，确认不会从准备姿态瞬间跳变。
- [ ] 记录 policy action、q_target、实际 q、IMU gyro 和深度 sequence，确认摔倒发生在图像输入、动作输出还是执行映射。
- [ ] 重新验证 AMP/mimic 不受安全门控和观测修改影响。

### Task 8: 端到端验收

- [ ] 构建：`cmake --build build --target leju-mujoco-sim run_rl_controller test_depth_pipeline -j4`。
- [ ] 单测：`./build/src/leju-controllers/leju-rl-controller/test_depth_pipeline` 全部通过。
- [ ] 深度诊断：使用 `--headless --dump-depth`，检查 PGM/CSV 和统计值。
- [ ] 仿真：确认平面站立、平路行走、上楼梯、下楼梯、切换和深度超时安全行为。
- [ ] 只有在图像快照、18510 维观测摘要和首个动作都与源端一致后，才宣称深度行走链路对齐。

