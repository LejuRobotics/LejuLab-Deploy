# ROS 深度行走控制器迁移至 DDS 独立控制器设计

## 1. 目标与范围

将 Roban S17 的 ROS `DepthWalkController` 迁移到
`src/leju-controllers/leju-rl-controller`，实现不依赖任何 ROS
构建或运行时组件的独立控制器。

目标控制器必须：

- 保持训练策略的观测契约；
- 通过目标仓库已有 SDK/DDS 框架获取机器人状态和控制指令；
- 通过线程安全的内部接口获取仿真深度图；
- 支持 MuJoCo 平路、上楼梯和下楼梯验证；
- 不改变现有 AMP 和 mimic 控制器的行为及观测契约。

## 2. 已确认的源端行为

### 2.1 生命周期与周期

ROS 控制器继承 `RLControllerBase`，实现以下生命周期和控制职责：

- 初始化及配置加载；
- reset、pause、resume 和 warm resume；
- 观测更新；
- 策略推理；
- 动作到关节指令的转换；
- 请求退出；
- 判断是否允许控制器切换。

源端关键周期和维度为：

- 控制循环周期：0.001 秒，即目标 1000 Hz；
- 策略推理频率：50 Hz；
- 单帧观测维度：18,510；
- 策略观测堆叠帧数：5；
- 深度历史帧数：8；
- 单帧深度图尺寸：36 × 64。

策略推理与 1 kHz 关节指令循环分离。目标实现保持这种线程分工，避免图像处理或模型推理阻塞控制循环。

### 2.2 单帧观测布局

观测顺序由源端配置加载器和 `singleInputData` 声明顺序确定，并已与
`DepthWalkController::updateObservation` 的实际拼接代码交叉确认。

| 起始偏移 | 维度 | 观测项 |
|---:|---:|---|
| 0 | 18,432 | 深度历史，8 × 36 × 64 |
| 18,432 | 3 | 速度指令：vx、vy、wz |
| 18,435 | 5 | command phase |
| 18,440 | 3 | 机身角速度 |
| 18,443 | 3 | projected gravity |
| 18,446 | 21 | 关节位置减默认位置 |
| 18,467 | 21 | 关节速度 |
| 18,488 | 21 | 上一次策略动作 |
| 18,509 | 1 | frequency phase |

总维度为：

```text
18432 + 3 + 5 + 3 + 3 + 21 + 21 + 21 + 1 = 18510
18510 × 5 = 92550
```

5 帧策略观测按照从旧到新的顺序拼接，模型输入是包含 92,550 个元素的一维 `float32` 数组。

每张深度图按照行优先顺序展开，8 帧深度图的内存布局为：

```text
[history][row][column]
```

### 2.3 模型契约

当前选定模型：

```text
model_15_2026-07-27_18-27-25_187800.onnx
```

SHA-256：

```text
64bcb4051dc889d6002087835ee0b3180220c4ff4882fb8acb8f40128636bbda
```

该模型已在目标容器使用 OpenVINO 2025.2 成功加载并完成推理。实际张量契约为：

- 输入 `obs`：`float32[92550]`；
- 输出 `actions`：`float32[25]`。

25 维输出布局为：

- `[0, 21)`：21 维策略关节动作；
- `[21, 22)`：1 维 gait frequency；
- `[22, 25)`：3 维速度估计。

当前全部模型都不再导出旧版的 128 维 depth latent。目标实现不得要求、读取或发布该 latent。

3 维速度估计仅为保持模型输出解析完整性和可选诊断而保留，不参与实际控制律。

## 3. 架构选择

采用专用 `DepthWalkController`，继承 `ControllerBase`，复用目标仓库已有的：

- 模型加载与推理后端；
- 控制器生命周期；
- 关节映射；
- 动作输出；
- 手臂和腰部控制器；
- 日志与频率统计；
- URI 路径解析；
- ControllerManager 控制器切换及平滑插值。

不在 `GenericRLController` 中加入深度专用分支。这样可以保持 AMP 和 mimic
的代码路径不变，将回归风险限制在新控制器和新增深度模块内。

深度子系统划分如下。

### 3.1 `DepthFrame`

与传输方式无关的原始深度帧，包含：

- 宽度和高度；
- 深度单位；
- 单调时钟采集时间戳；
- 帧序号；
- 连续像素数组。

### 3.2 `DepthImageProcessor`

负责与源端一致的单帧处理：

1. 根据输入单位将毫米转换为米；
2. 拒绝 NaN、Inf、负深度和错误尺寸；
3. 执行配置指定的裁剪；
4. 使用最近邻插值 resize；
5. 根据配置对零深度 mask 膨胀并执行空洞修复；
6. 根据配置执行 Gaussian blur；
7. 将深度裁剪到 `[0, 2.5 m]`；
8. 除以 2.5，得到 `float32 [0, 1]`；
9. 检查输出尺寸严格等于 36 × 64。

所有处理选项都必须显式写入 YAML。仿真和未来真实相机使用同一处理契约。

### 3.3 `DepthHistoryBuffer`

维护线程安全、按时间正序排列的 43 帧缓存。

收到第一张有效图时，使用第一帧初始化全部历史槽位。输出时选择：

```text
[0, 6, 12, 18, 24, 30, 36, 42]
```

输出顺序为最旧到最新，最终维度严格等于：

```text
8 × 36 × 64 = 18432
```

该定义与 Python 真实相机处理链保持一致。ROS MuJoCo 代码中存在另一套
22 帧倒序采样实现，和 Python 链不一致，目标端不保留这套不一致行为。

### 3.4 `DepthObservationProvider`

该模块拥有图像处理器和历史缓存，接收原始深度帧，并以线程安全快照方式提供：

- 8 帧深度历史；
- 最新采集时间戳；
- 有效像素比例；
- 帧序号；
- ready 状态；
- 图像处理耗时。

它是仿真和真实相机共用的稳定边界：

- `MuJoCoDepthCameraSource` 通过进程内线程安全回调提交帧；
- 未来 `DdsDepthCameraSource` 在存在正式 DDS 消息后提交帧。

当前目标 SDK 中未找到可用的深度图 DDS 消息，因此本次不自行虚构或增加公共 DDS 消息类型。

### 3.5 `DepthWalkController`

控制器负责：

- 加载并验证 YAML 和模型张量签名；
- 将 SDK 电机状态映射到 21 维策略关节顺序；
- 按确定的顺序构造 18,510 维单帧观测；
- 维护从旧到新的 5 帧策略观测；
- 以 50 Hz 执行推理；
- 严格解析 21 维 action、1 维 gait frequency 和 3 维速度估计；
- 应用动作缩放、逐关节缩放、方向、默认姿态、PD 增益和力矩限制；
- 复用现有手臂和腰部控制器完成配置指定的外部接管；
- 为 `ControllerManager` 提供切换插值参考命令；
- 仅通过目标仓库原生日志和诊断接口输出状态。

## 4. 接口映射

| ROS 源端接口 | 目标端接口 |
|---|---|
| 关节位置、速度、电流 | `RobotData` 和 `RobotState` |
| IMU 四元数、角速度 | `ImuData` |
| projected gravity | 控制器根据 `ImuData` 内部计算 |
| gait 速度指令 | `ControllerBase::setVelocityCommand` |
| stance 指令 | 目标命令状态及零速防抖 stance |
| `/camera/depth/depth_history_array` | `DepthObservationProvider` 快照 |
| ROS joint command | `RobotCmd` 和 `lejusdk-lowlevel` |
| ROS 手臂 service/topic | `MultiModeArmController` |
| ROS 腰部输出 | `WaistController` |
| ROS 控制器切换 | `ControllerManager::requestSwitch` |
| ROS TopicLogger | 目标 RL 日志和频率记录器 |

仿真数据流：

```text
MuJoCo S17 深度相机
  -> MuJoCoDepthCameraSource
  -> DepthObservationProvider
  -> DepthImageProcessor
  -> DepthHistoryBuffer
  -> DepthWalkController 观测
  -> ONNX/OpenVINO 推理
  -> RobotCmd
  -> 现有 DDS/SDK 仿真底层接口
```

未来真实相机数据流：

```text
真实深度相机 DDS 消息
  -> DdsDepthCameraSource 适配器
  -> 共用 DepthObservationProvider 及后续链路
```

## 5. 安全与线程同步

控制器只有在以下条件全部满足时才允许进入 active：

- 配置合法；
- 模型加载成功；
- 模型输入为 92,550 个 `float32` 元素；
- 模型输出严格为 25 维；
- 策略动作维度为 21；
- provider 已包含 8 张有效的 36 × 64 深度图；
- 最新深度帧未超过配置的超时时间；
- 有效像素比例达到配置阈值。

运行时规则：

- 观测、图像和动作缓存必须同步访问；
- 首次推理前初始化深度、frame stack 和 previous action；
- 模型输出无效时丢弃本次结果；
- 所有 tensor 访问都必须先做维度检查；
- 短时丢帧只允许在超时阈值内沿用最近有效快照；
- 深度超时或连续无效时请求安全站立或退出，禁止继续生成危险新动作；
- reset 清空策略历史、图像历史、旧动作、相位、诊断和错误计数；
- 析构时必须停止并 join 推理线程；
- 图像处理不得运行在 1 kHz 控制路径中；
- 模型加载失败时禁止注册或启动控制器。

控制器切换复用 `ControllerManager` 已有的双策略推理和五次曲线指令插值。
进入深度控制器前额外检查目标控制器 ready 状态。从 AMP 切换到深度控制器、
以及从深度控制器恢复 AMP，都必须满足目标端现有站立状态条件。

## 6. 计划文件变更

在 `leju-rl-controller` 新增：

- `include/leju-rl-controller/depth/depth_frame.h`
- `include/leju-rl-controller/depth/depth_image_processor.h`
- `include/leju-rl-controller/depth/depth_history_buffer.h`
- `include/leju-rl-controller/depth/depth_observation_provider.h`
- `src/depth/depth_image_processor.cpp`
- `src/depth/depth_history_buffer.cpp`
- `src/depth/depth_observation_provider.cpp`
- `include/leju-rl-controller/controllers/depth_walk_controller.h`
- `src/controllers/depth_walk_controller.cpp`
- `config/17/controllers/config_depth_walk.yaml`
- `config/17/controller_manager_depth_sim.yaml`
- `config/17/policy/model_15_2026-07-27_18-27-25_187800.onnx`
- 深度处理、历史顺序、观测布局、模型契约和安全状态测试。

计划修改：

- `leju-rl-controller/CMakeLists.txt`
- 现有控制器注册源文件；
- `config/17/controller_manager.yaml`
- `leju-mujoco-sim/CMakeLists.txt`
- MuJoCo 仿真主代码中与深度相机直接相关的局部代码；
- S17 深度相机和楼梯场景所需 XML。

明确不修改：

- ROS 源仓库中的任何文件；
- `GenericRLController` 的观测和推理行为；
- `config_amp.yaml`；
- mimic 配置和模型；
- 现有公共 SDK/DDS 消息定义。

## 7. 验证方案

### 7.1 静态检查

- 搜索新增控制器及依赖是否包含 ROS 头文件、API、包路径、topic、service、
  catkin、`cv_bridge` 或 `image_transport`；
- 检查 CMake 是否链接 ROS 库。

### 7.2 单元与模型检查

- 深度单位转换和无效值处理；
- resize、归一化及严格输出尺寸；
- 43 帧到 8 帧的时间正序选择；
- 首帧初始化和 reset；
- 深度超时及连续无效处理；
- 单帧观测各项精确偏移；
- 5 帧从旧到新的 frame stack；
- 模型签名严格为 `[92550] -> [25]`；
- 输出解析严格为 `21 + 1 + 3`；
- 关节映射、动作缩放、方向、默认姿态、PD 和力矩限制。

### 7.3 编译与回归检查

- 编译 RL 控制器和 MuJoCo 目标；
- 运行现有全部 RL 控制器测试；
- 执行 AMP 和 mimic 模型加载冒烟测试；
- 运行新增深度模块测试。

### 7.4 仿真检查

- 相机采集和时间戳；
- 8 帧历史内容和顺序；
- 控制器 ready 状态及 50 Hz 推理；
- 平路、上楼梯和下楼梯；
- 速度指令响应；
- AMP 到深度、深度到 AMP 的切换；
- 深度超时、无效和断流处理；
- 记录控制循环、推理、图像处理、端到端观测和 DDS 延迟。

## 8. 验收边界

只有满足以下条件才可判定迁移完成：

- 目标控制器在无 ROS 依赖条件下可编译、可启动；
- 选定模型按已确认的 `[92550] -> [25]` 契约成功加载；
- AMP 和 mimic 行为未改变且通过回归验证；
- 深度控制器在所要求的三类 MuJoCo 地形中完成安全推理和控制器切换验证。

仅通过编译或模型加载不能声称行走效果正常。平路、上楼梯和下楼梯结果必须来自实际仿真运行，并提供日志和机器人行为证据。
