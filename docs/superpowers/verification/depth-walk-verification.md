# S17 DDS 深度行走控制器验证记录

## 已通过

- 目标仓库 `run_rl_controller` 单目标编译通过。
- 深度图像处理和历史缓存测试：5/5 通过。
- 深度模型实际加载成功：
  - 输入：`float32[92550]`
  - 输出：`float32[25]`
- 控制器注册成功：`DepthWalkController`。
- 启动冒烟测试加载了 AMP、深度控制器和 mimic 控制器。
- DDS 深度订阅成功建立：
  `rt/depth_camera/frame_meters_36x64`
- S17 MuJoCo 资产已加入目标仓库，包含 `scene.xml`、`scene_stair.xml`、S17 XML/URDF 和 68 个网格文件。
- MuJoCo headless 深度发布器成功建立 DDS writer，并加载楼梯场景：
  `src/leju-mujoco-sim/model/biped_s17/xml/scene_stair.xml`
- MuJoCo 与控制器联调成功：控制器收到 `/rt/joint_state`、`/rt/imu_state` 后进入
  `Sensor data ready`、默认姿态和 `ControlLoop started`。
- 新增深度控制器及深度模块未发现 ROS API、ROS 头文件或 catkin 依赖。
- 模型 SHA-256：
  `64bcb4051dc889d6002087835ee0b3180220c4ff4882fb8acb8f40128636bbda`

## 启动日志证据

```text
ControllerRegistry: registered controller type 'DepthWalkController'
TopicSubscriber initialized successfully for topic:
rt/depth_camera/frame_meters_36x64
OpenVINOModel::load Input shape: [92550]
OpenVINOModel::load Output shape: [25]
DepthWalkController contract: float32[92550] -> float32[25]
ControllerManager initialized, 9 controllers loaded
```

启动进程随后因没有外部 `/rt/joint_state` 和 `/rt/imu_state` 传感器数据，按目标仓库现有逻辑安全退出；没有发布行走动作。

## 当前限制

本次自动化联调验证了楼梯场景加载、深度 DDS 发布、控制器启动和传感器同步；尚未
在无人工 `start` 按键的条件下做完整行走轨迹评估，因此平路、上楼梯和下楼梯的
稳定性仍需在带启动指令的仿真回放中确认。源资产只提供 `scene_stair.xml`，下楼
场景需要通过场景初始位姿或真实 DDS 相机验证。

全量仓库构建还被已有的 `quest_udp_to_dds_node` 缺少
`google/protobuf/port_def.inc` 阻断；`run_rl_controller`、MuJoCo 目标和深度测试目标均已独立编译成功。

## DDS 深度接入约定

深度源应向 DDS topic：

```text
rt/depth_camera/frame_meters_36x64
```

发布 `leju::msgs::Float64Array`，数组长度必须为 `36*64`，单位为米。
控制器端完成裁剪、归一化、43 帧缓存和 8 帧历史导出。
