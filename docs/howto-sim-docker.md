# x86 仿真 Docker 使用指南（GUI / NVIDIA）

在开发机用 Docker 跑 MuJoCo 仿真（`leju-mujoco-sim`），支持 NVIDIA 硬件渲染的 GUI，
也支持无界面 headless（纯物理 + DDS）。与交叉编译镜像（`docker/Dockerfile.cross-aarch64`）
完全分开——那个产 aarch64 板子二进制，**这个在 x86 原生跑仿真**。

> 参考 `kuavo-ros-control/docker` 的 GPU/X11 透传模式精简而来（lejulab 是 DDS/纯 CMake，
> 不需要 ROS/drake/casadi）。

## 原理一句话

`nvidia/opengl:glvnd-runtime-ubuntu20.04`（**gcc-9**，匹配仓库 x86_64 预编译的 GCC9 LTO
库）+ GUI 依赖（glfw/GL/X11），把宿主 NVIDIA 显卡和 X11 显示透传进容器，原生编译并跑仿真。

> ⚠️ 为什么是 20.04 不是 22.04：仓库 `src/lejusdk/3rd_party/x86_64/` 的 CycloneDDS/iceoryx
> 是 **GCC 9** 编的 LTO 静态库，必须 gcc-9 链接（gcc-11 会报 "bytecode generated with GCC
> older than 10.0"）。注意：aarch64 交叉编译那套反而要 gcc-11——两套预编译库 GCC 版本不同。

## 镜像从哪来（自动拉取）

`run_sim*.sh` 都会先 `docker image inspect` 判断本地有没有镜像；**没有就先从云端拉**
（`docker/_ensure_image.sh` 里 `ensure_image`）：
- 云端目录默认 `https://kuavo.lejurobot.com/r3588_docker_image_backup/`，可用环境变量
  `IMAGE_BASE_URL` 覆盖；按 `<镜像名>.tar.gz` 下载后 `docker load`。
- 云端也拉不到时，回退到本地构建（`build_sim_image.sh` / `docker build`）。

所以首次直接 `./run_sim_xarch.sh` 即可——本地没镜像会自动下载。

## 前置：宿主 GPU 配置（GUI 硬件渲染需要）

```bash
# 装 nvidia-container-toolkit（一次）
# 见 https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
nvidia-ctk --version          # 确认已装
nvidia-smi                    # 确认能看到显卡
```
没有 NVIDIA / 没配 toolkit 也能跑，`run_sim.sh` 会自动退回软件渲染（慢），或用 `--headless` 完全不需要显示。

## 三步上手

```bash
cd docker
./build_sim_image.sh          # 构建镜像 lejulab-sim-x86:latest

./run_sim.sh                  # 进容器(GPU/显示/DDS host 网络就绪，仓库挂到 /workspace)

# —— 容器内 ——
cmake -B build_sim -G Ninja -DBUILD_MUJOCO_SIM=ON -DBUILD_TESTS=OFF \
      -DBUILD_RL_CONTROLLER=OFF -DBUILD_JOYSTICK=OFF -DBUILD_EXAMPLES=OFF
cmake --build build_sim --target leju-mujoco-sim -j$(nproc)

export ROBOT_VERSION=17       # 必须与模型匹配(17/45/46/...)，否则启动即抛异常
SIM=build_sim/src/leju-mujoco-sim/leju-mujoco-sim
$SIM            src/leju_assets/models/biped_s17/xml/scene.xml   # GUI(显卡渲染)
$SIM --headless src/leju_assets/models/biped_s17/xml/scene.xml   # 无界面(物理+DDS)
```

`run_sim.sh` 也可直接带命令：`./run_sim.sh "cmake --build build_sim --target leju-mujoco-sim -j8"`。

## 一键闭环 demo（sim + RL 控制器）

要看机器人被 RL 策略驱动「动起来」，用 `run_sim_demo.sh` —— 自动编译 sim+控制器+手柄
并起项目自带的 `launch_mujoco_sim.sh` 闭环：

```bash
cd docker
./run_sim_demo.sh           # 默认 ROBOT_VERSION=17
./run_sim_demo.sh 46        # 指定版本(需有 config/<v> 与 scene_rl.xml)
FORCE_BUILD=1 ./run_sim_demo.sh   # 强制重新 configure
# Ctrl-C 退出
```

它在容器内做：configure(MUJOCO+RL+JOYSTICK) → build → `launch_mujoco_sim.sh --no-shm`。
启动后通过 DDS 闭环：

```
sim   →发布 /rt/joint_state /rt/imu_state，订 /rt/joint_cmd
控制器→发布 /rt/joint_cmd，订 /rt/joint_state /rt/imu_state（ONNX AMP 策略推理）
```

GUI 窗口里能看到 `biped_s${ROBOT_VERSION}` 机器人被策略控制（站立/平衡；给手柄速度则行走）。

注意：
- **ROBOT_VERSION 必须有对应 `config/<v>/controller_manager.yaml` 和
  `models/biped_s<v>/xml/scene_rl.xml`**。某些版本只有 `scene.xml`，需先补 `scene_rl.xml`。
- `--no-shm` 走普通 CycloneDDS loopback，免 iceoryx RouDi 守护进程。
- 容器内无物理手柄，`leju-joystick` 空转(non-required 不影响)；默认零速度→原地站立。
  要遥操走路需透传手柄设备或往 `/rt/joy` 发指令。
- `lejusdk_recorder` 默认没编(non-required，仅日志里一条 not found，无害)。

## run_sim.sh 做了什么（GPU/X11 透传要点）

- `--gpus all --runtime nvidia` + `NVIDIA_DRIVER_CAPABILITIES=all,display` —— 显卡 + OpenGL
- `-e DISPLAY -v /tmp/.X11-unix` + `xhost +local:root` —— X11 窗口
- `--net host` —— DDS（仿真与控制器/RL 通过 DDS topic 互通）
- `-v <repo>:/workspace` —— 挂载仓库，容器内原生编译
- `-e ROBOT_VERSION=${ROBOT_VERSION:-17}` —— 转发机器人版本
- 容器复用：同一仓库目录第二次 `run_sim.sh` 直接 `exec` 进已有容器

## 验证 GPU 透传是否生效

容器内跑：
```bash
glxinfo -B | grep "OpenGL renderer"
# 期望: OpenGL renderer string: NVIDIA GeForce ...   ← 走显卡
# 若显示 llvmpipe/softpipe = 软件渲染(没透传到显卡)
```

## 单镜像跨架构联合仿真（aarch64 控制器 + x86 仿真）

要用**板子同款的 aarch64 控制器二进制**驱动 x86 仿真，验证仓库的 aarch64 部署功能，
用单镜像 `lejulab-sim-xarch`。**一个容器**里就能同时跑：
- `leju-mujoco-sim`（x86 原生，GPU GUI，实时）
- `run_rl_controller`（aarch64，QEMU 模拟，和烧进板子的一模一样）

二者通过**容器内 loopback DDS** 闭环（无需 `--net host`）。

```bash
cd docker
./run_sim_xarch.sh            # 默认：进容器交互 shell(环境全配好，打印速查命令)
./run_sim_xarch.sh 46         # 指定 ROBOT_VERSION
./run_sim_xarch.sh --demo     # 不进 shell，直接一键起闭环
```

**默认进 shell**，里面 GPU/显示/DDS/`QEMU_LD_PREFIX`/仓库挂载都就绪，自己按节奏来：

```bash
# 1) 编 x86 仿真(gcc-9)
cmake -B build_sim -G Ninja -DBUILD_MUJOCO_SIM=ON -DBUILD_RL_CONTROLLER=OFF \
      -DBUILD_JOYSTICK=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
cmake --build build_sim --target leju-mujoco-sim -j$(nproc)
# 2) 起仿真(GUI，后台)
build_sim/src/leju-mujoco-sim/leju-mujoco-sim src/leju_assets/models/biped_s17/xml/scene_rl.xml &
# 3) 起 aarch64 控制器(QEMU)，$CTRL_LIBS 已预置 aarch64 库路径
LD_LIBRARY_PATH=$CTRL_LIBS \
  build_cross/src/leju-controllers/leju-rl-controller/run_rl_controller \
  src/leju-controllers/leju-rl-controller/config/17/controller_manager.yaml
```

或一条命令起完整闭环（跨架构版 launch 脚本，沿用 sim+joystick 编排，控制器换成 aarch64/QEMU）：

```bash
bash src/leju_launch/scripts/launch_mujoco_sim_xarch.sh --robot-version=17
```

> 它与 `launch_mujoco_sim.sh` 的区别：①控制器用 `build_cross` 的 aarch64 二进制(QEMU)
> ②DDS 用禁多播 loopback（不强制 iceoryx SHM/RouDi）③不覆盖 `CYCLONEDDS_URI`。

### 原理（为什么一个容器能跑两种架构）
- 容器跑哪种架构二进制由**宿主内核 binfmt/QEMU** 决定（host 级，对任何容器生效）。
  前置：宿主注册过 `docker run --privileged tonistiigi/binfmt --install arm64`。
- 镜像里 x86 仿真用 20.04/gcc-9 运行时；aarch64 控制器按板子 22.04/glibc 2.35 交叉编译，
  故镜像多阶段拷入一份 aarch64 22.04 运行时库放 `/opt/aarch64-rootfs`，`QEMU_LD_PREFIX`
  指向它，QEMU 据此解析 aarch64 的 loader/glibc/libstdc++。
- QEMU 下 lo 多播不支持 → 用 `docker/cyclonedds_sim.xml`（禁多播 + localhost 单播）。

### 前置产物
- **aarch64 控制器**：脚本会用交叉编译镜像 `lejulab-cross-aarch64` 自动 cross-build 到
  `build_cross/`（需先有该镜像，见 `docs/howto-cross-compile-docker.md`）。
- **x86 仿真**：脚本在 `lejulab-sim-xarch` 容器内自动编（gcc-9）。

### ⚠️ 重要：QEMU 性能
aarch64 控制器是**模拟**跑，ONNX 推理远慢于 x86 仿真的实时步进：
- ✅ **功能/契约验证**：二进制加载、ONNX 推理、DDS 类型匹配、双向通信 —— 都有效
- ❌ **实时/行为验证**：控制器指令滞后，窗口里机器人**可能动作不正常/站不稳**（非代码错，
  是模拟太慢）。要看真实行走效果用纯 x86 demo（`run_sim_demo.sh`）或真板子。

## 与控制器/RL 联调

仿真用 `--net host` + DDS，发布 `/rt/joint_state`、`/rt/imu_state`，订阅 `/rt/joint_cmd`。
在同一台机（或同网段）跑 RL/控制器即可与仿真闭环——参考 `docs/cross-machine-rl-mujoco-guide.md`
（板子跑 RL + x86 跑 MuJoCo）。

## 常见问题

- **启动抛 `ROBOT_VERSION not set`**：`export ROBOT_VERSION=<与模型匹配的版本>`。
- **链接报 `bytecode generated with GCC older than 10.0`**：镜像不是 gcc-9（base 用错成 22.04），
  确认 `Dockerfile.sim` 是 `ubuntu20.04`。
- **`find_package(GTest)` 失败**：镜像缺 `libgtest-dev`（已在 Dockerfile.sim 装）；或编译时加 `-DBUILD_TESTS=OFF`。
- **`iceoryx_binding_c::iceoryx_binding_c not found`**：根 `CMakeLists.txt` 缺顶层
  `find_package(iceoryx_*)`（使 IMPORTED 目标对所有子目录可见）。
- **GUI 黑屏/无法连 X**：宿主先 `xhost +local:root`；确认 `echo $DISPLAY` 非空。
- **GUI 用了软件渲染很卡**：宿主没装 `nvidia-container-toolkit`，或缺 `--gpus all`。
