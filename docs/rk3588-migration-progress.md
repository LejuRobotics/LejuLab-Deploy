# RK3588 ARM 架构迁移 — 工作成果总结

- **项目**：lejulab_platform → lejulab_platform_zx (ARM 分支)
- **目标平台**：RK3588 (aarch64), Ubuntu 22.04
- **工作分支**：`zhongxu/feat/remove-catkin-pure-cmake`
- **日期**：2026-02-12 ~ 2026-02-13
- **总进度**：约 95%

---

## 一、迁移目标

将 lejulab_platform 项目从 x86 + catkin (ROS1) 架构迁移到 RK3588 aarch64 平台，实现：

1. 去除 ROS/catkin 依赖，改用纯 CMake 构建
2. 所有预编译依赖提供 aarch64 版本
3. 在 RK3588 上实现全量编译和运行
4. 保持 x86 兼容性（双架构并存）

---

## 二、各阶段完成情况

### Phase 0：环境搭建与构建系统改造 ✅ 全部完成

#### 0.1 RK3588 基础环境

- 板子信息：Ubuntu 22.04.3, 内核 5.10.226-rt89 PREEMPT_RT, cmake 3.22.1
- 安装必要依赖：g++-12, libacl1-dev, libeigen3-dev, libyaml-cpp-dev, libusb-1.0-0-dev, libglew-dev, libglfw3-dev
- 板子网络受限，通过 Docker 提取 .deb + scp 离线安装

#### 0.2 catkin → 纯 CMake

**改动量**：21 个文件修改/新增，18 个 `package.xml` 删除

- 创建顶层 `CMakeLists.txt`，按 6 层依赖顺序组织 16 个子模块
- 移除所有 `find_package(catkin)` / `catkin_package()` 调用
- 替换 `${catkin_INCLUDE_DIRS}` / `${catkin_LIBRARIES}` 为显式 CMake target
- `leju_assets` 移除 ROS C++ 依赖（`ros::package::getPath()`）
- x86 环境全量编译零错误通过

依赖层次结构：
```
Layer 0: lejusdk-utils (INTERFACE, 纯头文件)
Layer 1: lejusdk-dds-idl, lejusdk-topic-pubsub (CycloneDDS)
Layer 2: leju_assets
Layer 3: lejusdk-lowlevel (核心 SDK)
Layer 4: leju-hardware, leju-rl-controller, leju-joystick, leju-mujoco-sim
Layer 5: examples, recorder
```

#### 0.3 roslaunch → Shell 脚本

编写 7 个启动脚本替代 roslaunch：

| 脚本 | 说明 |
|------|------|
| `_launch_common.sh` | 公共函数库（`find_binary()`、`find_package()`、进程管理） |
| `launch_real.sh` | 真机启动 |
| `launch_real_rl_demo.sh` | 真机 RL demo |
| `launch_real_mimic_demo.sh` | 真机 Mimic demo |
| `launch_mujoco_sim.sh` | MuJoCo 仿真 |
| `launch_mujoco_sim_rl_demo.sh` | MuJoCo RL demo |
| `launch_mujoco_sim_mimic_demo.sh` | MuJoCo Mimic demo |

实现功能：环境变量设置、多进程启动、required 节点退出守护（`trap` + PID 监控）。

#### 0.4 双架构 3rd_party 支持

- 在 3 个子模块中创建 `arch_select.cmake`，根据 `CMAKE_SYSTEM_PROCESSOR` 自动选择 `x86_64/` 或 `aarch64/` 预编译库
- 涉及：CycloneDDS + iceoryx、SDL3、MuJoCo
- x86_64 和 aarch64 构建均通过验证

#### 0.5 Docker aarch64 交叉编译环境

- 基于 `arm64v8/ubuntu:22.04` + QEMU user-mode
- 镜像大小 697MB / 压缩 223MB
- 新增 `docker/Dockerfile.aarch64`、`build_docker.sh`、`run_build.sh`、`load_image.sh`
- Docker 编译 100% 通过，产物为原生 aarch64 ELF

---

### Phase 1：DDS 通信层 ✅ 已完成

在 RK3588 上从源码编译所有 DDS 组件，替换 x86 预编译产物：

| 组件 | 版本 | 类型 |
|------|------|------|
| CycloneDDS | 0.10.2 | 动态库 (.so) |
| CycloneDDS-CXX | 0.10.2 | 动态库 (.so) |
| iceoryx | 捆绑版 | 静态库 (.a) |

产物放置于 `src/lejusdk/3rd_party/aarch64/` 目录。

---

### Phase 2：仿真与输入 ✅ 已完成

#### 2.1 MuJoCo 3.2.0 aarch64

- 使用官方 aarch64 release 包
- 安装到 `src/leju-mujoco-sim/3rd_party/aarch64/mujoco-3.2.0/`
- ARM NEON 自动启用（`mjUSEPLATFORMSIMD`）
- **新增 `--headless` 模式**：RK3588 Mali-G610 GPU 不支持桌面 OpenGL，新增无渲染纯物理仿真模式，按 timestep 步进 `mj_step()` + DDS 发布

#### 2.2 SDL3 3.3.2 aarch64

- 在 Docker aarch64 容器中从源码编译
- 安装到 `src/leju-joystick/3rd_party/aarch64/SDL/`
- `BUILD_JOYSTICK=ON` 在 Docker 和 RK3588 板子上编译通过
- `libSDL3.so.0.3.2` 已部署到板子 `/usr/local/lib/`

---

### Phase 3：硬件驱动层（部分完成）

#### 3.1 SocketCAN Wrapper ✅ 已完成

RK3588 使用标准 SocketCAN + CAN FD 替代 BUSMUST USB-CAN 适配器：

- 新增 `SocketCANWrapper` 类，继承 `CanBusBase` 抽象基类
- 在工厂方法 `CanBusBase::create()` 中注册
- YAML 配置通过 `type: SOCKETCAN` 选择
- 策略模式，改动侵入性低：
  ```
  CanBusBase (抽象基类)
  ├── BMCanbusWrapper     (BUSMUST, x86)
  ├── LejuCanbusWrapper   (LEJU CANable, 串口)
  └── SocketCANWrapper    (SocketCAN + CAN FD, RK3588) [新增]
  ```

#### 3.2 DexHand SDK ⏳ 待开始

`download-lib.sh` 已内置 aarch64 支持，在 RK3588 上运行即可自动下载 ARM 版本 (v1.1.3)。

---

### Phase 4：推理引擎 ✅ 已完成

**方案**：ONNX Runtime 1.18.0 官方 aarch64 预编译包

- CMakeLists.txt 架构感知检测，aarch64 自动选择 `onnxruntime-linux-aarch64/`
- RK3588 cmake 配置输出：`ONNX Runtime backend: enabled (auto-detected)`
- `run_rl_controller` 和 `run_rl_mimic_controller` 编译链接通过
- 运行时验证：使用 `ROBOT_VERSION=14` 成功加载 policy_1212.onnx 和 base_tf_73999_modify_com_91000.onnx 两个模型
- `docker/run_build.sh` 更新 `BUILD_RL_CONTROLLER=ON`
- 添加推理性能监测（每 100 次打印 avg/min/max 微秒）

目录结构：
```
3rd_party/onnxruntime_toolkit/
├── onnxruntime-linux-x64-1.18.0/      # x86_64 (已有)
└── onnxruntime-linux-aarch64/          # aarch64 (新增)
    ├── include/
    └── lib/
        ├── libonnxruntime.so → libonnxruntime.so.1.18.0
        └── libonnxruntime.so.1.18.0    # 12.7MB
```

---

### Phase 5：构建系统修复与联调（大部分完成）

#### 5.1 硬编码路径修复 ✅

资源路径改为运行时基于可执行文件位置推导。

#### 5.2 EC-Master 条件编译 ✅

- 新增 `ec_types_compat.h` 提供 EC-Master 类型/函数存根
- 通过 `#ifdef ENABLE_ECMASTER` 包裹所有 EC-Master 代码

#### 5.3 Xsens IMU 条件编译 ✅

- 通过 `#ifdef ENABLE_XSENS` 包裹 Xsens 相关代码

#### 5.4 ARM 全量编译修复 ✅

共修复 **15 项** 编译问题：

| 类别 | 数量 | 典型问题 |
|------|------|---------|
| 缺失头文件/库 | 3 | gflags、lcm、GTest |
| x86 预编译库不兼容 | 3 | SDL3、DexHand SDK、BMAPI |
| C++17 兼容性 | 1 | DeviceInfo 聚合初始化 |
| 条件编译 | 5 | EC-Master、Xsens、BMAPI 相关 |
| 链接符号缺失 | 3 | BM_* 函数、BMCanbusWrapper |

新增条件编译标志：

| 标志 | 作用 | ARM 默认值 |
|------|------|-----------|
| `BUILD_RL_CONTROLLER` | RL 控制器 | ON |
| `BUILD_JOYSTICK` | 手柄输入 | OFF |
| `BUILD_MUJOCO_SIM` | MuJoCo 仿真 | OFF |
| `ENABLE_ECMASTER` | EC-Master 驱动 | OFF |
| `ENABLE_XSENS` | Xsens IMU | OFF |
| `HAS_BMAPI` | BUSMUST USB-CAN | OFF (自动检测) |

#### 5.5 集成测试 ⏳ 待完成

---

## 三、额外工作（计划外）

### 跨机 DDS 通信

- 新建 `cyclonedds_udp.xml`，支持 x86 ↔ RK3588 跨机 UDP 多播通信
- 场景：x86 开发机运行 MuJoCo 仿真，RK3588 运行 RL 推理
- 修复 `setup_cyclonedds_config.sh` 中 iox-roudi 搜索路径缺少架构前缀的问题

### MuJoCo Headless 模式

- `leju-mujoco-sim` 新增 `--headless` 参数
- 在 RK3588 上跳过 GLFW/OpenGL，纯物理仿真 + DDS 发布
- 解决 Mali-G610 GPU 无桌面 OpenGL 支持导致的软件渲染性能问题

### joy_trigger 调试工具

命令行模拟手柄输入，无需物理手柄即可触发控制器动作：

```bash
# 按钮模式 — 模拟手柄按键
joy_trigger start           # 启动控制器
joy_trigger guide           # 触发跳舞
joy_trigger back            # 停止控制器

# 行走模式 — 连续发布摇杆轴值 (60Hz)
joy_trigger walk 0 -0.5 0 0 5    # 前进 50% 速度 5秒
joy_trigger walk 0 -0.5 0.3 0    # 前进 + 右转 3秒
```

---

## 四、编译产物

ARM aarch64 全量编译通过，产物清单：

| 产物 | 类型 | 说明 |
|------|------|------|
| `leju-hardware` | 可执行文件 | 主硬件节点 |
| `run_rl_controller` | 可执行文件 | RL 控制器 (ONNX Runtime) |
| `run_rl_mimic_controller` | 可执行文件 | RL Mimic 控制器 (ONNX Runtime) |
| `run_dummy_controller` | 可执行文件 | Dummy 控制器 |
| `leju-joystick` | 可执行文件 | 手柄输入节点 |
| `leju-mujoco-sim` | 可执行文件 | MuJoCo 仿真 (含 headless 模式) |
| `lejusdk_recorder` | 可执行文件 | 数据录制器 |
| `joy_trigger` | 可执行文件 | 调试工具 |
| `liblejusdk-lowlevel.so` | 动态库 | 底层 SDK |
| `liblejusdk_hw.so` | 动态库 | 硬件 SDK |
| `libcanbus_sdk.so` | 动态库 | CAN 总线 SDK |
| `libruiwo_actuatorCXXLib.so` | 动态库 | 瑞沃电机驱动 |
| `libmotorevo_controller.so` | 动态库 | MotorEvo 电机控制 |
| `libkuavo_common.so` | 动态库 | 公共库 |
| `libkuavo_solver.so` | 动态库 | 求解器 |
| `libleju_assets.so` | 动态库 | 资源管理 |

---

## 五、提交记录

| Commit | 说明 |
|--------|------|
| `e0ff3f2` | catkin → 纯 CMake 构建系统迁移 |
| `150ee8d` | roslaunch → shell 启动脚本 (7 个脚本) |
| `07ad693` | DDS 通信层 aarch64 预编译库替换 |
| `c962f92` | ARM 全量编译支持 (15 项修复) |
| `3948dc0` | 双架构 3rd_party + Docker + SDL3 + SocketCAN |
| `963feb9` | 同步 ONNX Runtime 支持代码 |
| `915f1d4` | ONNX Runtime 1.18.0 aarch64 预编译库 |
| `df6b5ae` | MuJoCo headless 模式 + CycloneDDS UDP 配置 |

---

## 六、剩余工作

| 项目 | 说明 | 优先级 |
|------|------|--------|
| Phase 3.2 DexHand SDK | 在板子上运行 `download-lib.sh` 获取 aarch64 版本 | 中 |
| Phase 5.5 集成测试 | DDS 通信性能、硬件节点启动、仿真闭环验证 | 高 |
| ONNX 推理延迟实测 | 跨机运行，观察 `[PERF]` 输出是否满足控制频率 | 高 |
| 手柄实测 | 连接物理手柄验证 SDL3 + leju-joystick 功能 | 低 |
| iceoryx SHM 优化 | 评估 RK3588 上 iceoryx 共享内存是否可用（当前用 UDP） | 低 |
