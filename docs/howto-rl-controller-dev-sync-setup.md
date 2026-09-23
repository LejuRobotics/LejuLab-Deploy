# RL-Controller dev-sync 分支 — 环境搭建与运行文档

> 适用分支：`zhongxu/feat/sync-dev-rl-controller`（已把上游 `dev` 的 rl-controller 新架构整包搬入并改为纯 CMake；lejusdk-vr 增加 runtime RPC；launch 支持 `--auto-start`；leju-joystick 增加自启动 manager）。
> 目标读者：要在**另一台机器/环境**上把这套跑起来的人。
> 典型部署：**x86 机器跑 MuJoCo 仿真 + RK3588(aarch64) 板子跑控制器**，两者通过 DDS 跨机通信。也支持单机。

---

## 0. 架构与约束(必读)

| 角色 | 架构 | 编译器 | 构建系统 |
|------|------|--------|---------|
| 仿真/开发机 | x86_64 | 系统 gcc 即可(gcc-9/11 都行) | 纯 CMake |
| 板载控制器 | aarch64 (RK3588) | **必须 GCC 11** | 纯 CMake |

- aarch64 **必须 gcc-11**：预编译的 CycloneDDS 静态库内嵌 GCC11 LTO 字节码,gcc-12+ 链接会失败。
- **不是 catkin**。本分支已移除 catkin,全程纯 CMake(无 `catkin build`、无 `package.xml`、无 `/opt/ros`、无 `devel/setup.bash`)。
- 第三方预编译库(CycloneDDS / iceoryx / ONNX Runtime / MuJoCo / SDL3)**已随仓库**,且按架构(`3rd_party/aarch64`、`3rd_party/x86_64`)自动选择,无需手装。

---

## 1. 获取代码

该分支**未推送到共享 origin**。两种方式拿到新机器:

### 方式 A:推送到 gitlab 再 clone(推荐,适合全新机器)
在已有该分支的机器上:
```bash
git push origin zhongxu/feat/sync-dev-rl-controller
```
新机器:
```bash
git clone https://www.lejuhub.com/zhongxu/lejulab_platform_zx.git
cd lejulab_platform_zx
git checkout zhongxu/feat/sync-dev-rl-controller
```

### 方式 B:git bundle 离线传输(不依赖网络/origin)
源机器:
```bash
git bundle create /tmp/zx.bundle zhongxu/feat/sync-dev-rl-controller --branches
# scp /tmp/zx.bundle 到目标机器
```
目标机器:
```bash
git clone /tmp/zx.bundle -b zhongxu/feat/sync-dev-rl-controller lejulab_platform_zx
```

> 仓库较大(含预编译 .so / .onnx / mujoco / SDL),完整传输有几百 MB,属正常。

---

## 2. 系统依赖

### x86_64(仿真/开发机)
```bash
sudo apt-get install -y build-essential cmake libacl1-dev libncurses5-dev \
    libeigen3-dev libyaml-cpp-dev libgtest-dev
# MuJoCo viewer 需要 GL(无显示器/headless 可不装):
sudo apt-get install -y libgl1 libglfw3 libxinerama1 libxcursor1 libxi6
```

### aarch64(RK3588 控制器)
```bash
sudo apt-get install -y g++-11 cmake libacl1-dev libeigen3-dev \
    libyaml-cpp-dev libusb-1.0-0-dev libgtest-dev
```

> `cmake` 需 ≥ 3.16。`pinocchio`/`OpenVINO` **不需要**(见 §3 编译开关,默认关闭/可选)。

---

## 3. 编译(纯 CMake)

### aarch64(板子)
```bash
cd lejulab_platform_zx
CC=gcc-11 CXX=g++-11 cmake -Bbuild_cmake -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_PINOCCHIO=OFF -DBUILD_TESTING=ON
CC=gcc-11 CXX=g++-11 cmake --build build_cmake -j$(nproc)
```

### x86_64(仿真机)
```bash
cd lejulab_platform_zx
cmake -Bbuild_cmake -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_PINOCCHIO=OFF -DBUILD_MUJOCO_SIM=ON -DBUILD_TESTING=ON
cmake --build build_cmake -j$(nproc)
```

### 关键编译开关
| 开关 | 默认 | 说明 |
|------|------|------|
| `ENABLE_PINOCCHIO` | ON(找不到自动跳过) | rl-controller 手臂重力补偿依赖 pinocchio。aarch64 无此包 → 设 `OFF`,重力补偿退化为 0(其余功能正常)。x86 若装了 ros-pinocchio 可保留 ON。 |
| `BUILD_MUJOCO_SIM` | ON | 编译 MuJoCo 仿真(x86 跑仿真必须)。 |
| `BUILD_TESTING` | OFF | 编译单元测试(见 §4)。 |
| OpenVINO | 自动 | `find_package(OpenVINO QUIET)`,找不到自动禁用(aarch64 无 OpenVINO,自动走 ONNX)。 |
| ONNX Runtime | 自动 | 按架构自动选 `onnxruntime-linux-aarch64` / `-x64`。 |

构建产物(关键二进制):
- `build_cmake/src/leju-controllers/leju-rl-controller/run_rl_controller`
- `build_cmake/src/leju-mujoco-sim/leju-mujoco-sim`(仅 x86 仿真)
- `build_cmake/tools/joy_trigger`(测试用手柄信号发生器)

---

## 4. 单元测试(验收门)

```bash
# 任一架构,需 -DBUILD_TESTING=ON 编译过
cd build_cmake/src/leju-controllers/leju-rl-controller
ctest --output-on-failure
```
预期:`100% tests passed, 0 tests failed out of 9`(robot_data / motion_trajectory / runtime{action_trigger,trigger_buffer,teleop_adapter,teleop_binding,teleop_integration} / trajectory{minimum_jerk,velocity_limited}）。
> 注:根 CMake 没有顶层 `enable_testing()`,`ctest` 要在该模块的 build 子目录里跑。

lejusdk-vr 的 runtime RPC 往返测试:
```bash
ctest --test-dir build_cmake -R test_vr_api --output-on-failure   # 或直接跑 build_cmake/src/lejusdk/lejusdk-vr/tests/test_vr_api
```

---

## 5. 运行仿真

### 5A. 单机(sim 与 controller 同机)
最简单,用现成 launch 脚本(默认带 MuJoCo viewer):
```bash
export ROBOT_VERSION=17   # 14/17/46/52
# 默认走 iceoryx 共享内存,需先起 roudi:
sudo bash src/leju_launch/scripts/start_roudi.sh   # 另开终端常驻
sudo bash src/leju_launch/scripts/launch_mujoco_sim.sh
#   或不用共享内存(免 roudi):
sudo bash src/leju_launch/scripts/launch_mujoco_sim.sh --no-shm
```
脚本会拉起:`leju-mujoco-sim` + `run_rl_controller` + `leju-joystick` + recorder。
常用参数(Phase 3 新增):
```
--auto-start            不启动 leju-joystick(由外部/自启动管理 START)
--config <path>         覆盖 controller_manager.yaml
--teleop-config <path>  覆盖 teleop_bindings.yaml
--robot-version=XX      机器人版本
--no-shm                关闭 iceoryx,用普通 CycloneDDS
```

### 5B. 跨机(x86 跑 sim + aarch64 跑 controller)—— 推荐验证方式

#### (1) DDS 跨机配置
仓库自带的 `src/leju_launch/config/cyclonedds.xml` 是 **loopback-only**,跨机不通。需各自一份 LAN 配置。把下面 `<X86_IP>`、`<BOARD_IP>` 换成实际 IP(同一网段,例如 192.168.50.23 / 192.168.50.239):

**x86 机器** `~/cyclonedds_x86.xml`:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS><Domain Id="any">
  <General>
    <Interfaces><NetworkInterface address="<X86_IP>" multicast="default" /></Interfaces>
    <AllowMulticast>false</AllowMulticast>
  </General>
  <Discovery>
    <ParticipantIndex>auto</ParticipantIndex>
    <Peers>
      <Peer address="localhost" />   <!-- 同机多进程发现(sim/joy_trigger) -->
      <Peer address="<BOARD_IP>" />  <!-- 对端 -->
    </Peers>
  </Discovery>
</Domain></CycloneDDS>
```

**板子** `~/cyclonedds_board.xml`(对调地址):
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS><Domain Id="any">
  <General>
    <Interfaces><NetworkInterface address="<BOARD_IP>" multicast="default" /></Interfaces>
    <AllowMulticast>false</AllowMulticast>
  </General>
  <Discovery>
    <ParticipantIndex>auto</ParticipantIndex>
    <Peers>
      <Peer address="localhost" />
      <Peer address="<X86_IP>" />
    </Peers>
  </Discovery>
</Domain></CycloneDDS>
```
> `localhost` peer 是同机多个参与者(如 controller 与 joy_trigger 都在板子上)互相发现的**关键**——多播关掉后没有它同机发现不了。两机需同一 `Domain Id`(此处 `any`)。防火墙放行 UDP(7400-7500 区间)。

#### (2) x86 起仿真
```bash
cd lejulab_platform_zx
export ROBOT_VERSION=17
export CYCLONEDDS_URI=file://$HOME/cyclonedds_x86.xml
export LD_LIBRARY_PATH=$PWD/build_cmake/src/lejusdk/lejusdk-lowlevel:$PWD/build_cmake/src/lejusdk/lejusdk-vr:$PWD/src/leju-mujoco-sim/3rd_party/x86_64/mujoco-3.2.0/lib
SCENE=src/leju_assets/models/biped_s${ROBOT_VERSION}/xml/scene_rl.xml
# 带 GUI(本机有显示器/X):
build_cmake/src/leju-mujoco-sim/leju-mujoco-sim "$SCENE"
# 无显示器(纯物理,日志驱动):
build_cmake/src/leju-mujoco-sim/leju-mujoco-sim --headless "$SCENE"
```
> v17 的 `scene_rl.xml` 若仓库缺失,从上游取:`git show upstream/dev:src/leju_assets/models/biped_s17/xml/scene_rl.xml`(同样取 `biped_s17.xml`)。

#### (3) 板子起控制器
```bash
cd lejulab_platform_zx   # 板子上的仓库
export ROBOT_VERSION=17
export CYCLONEDDS_URI=file://$HOME/cyclonedds_board.xml
export LD_LIBRARY_PATH=$PWD/build_cmake/src/lejusdk/lejusdk-lowlevel:$PWD/build_cmake/src/lejusdk/lejusdk-vr:$PWD/src/leju-controllers/leju-rl-controller/3rd_party/onnxruntime_toolkit/onnxruntime-linux-aarch64/lib
RLC=src/leju-controllers/leju-rl-controller
# 首次部署时安装运行配置；已存在时按需使用 --keep 或 --overwrite
bash scripts/install_teleop_bindings.sh
# 重要:加 stdbuf -oL,否则进程被 kill 时日志缓冲会丢,误判"卡死"
stdbuf -oL -eL build_cmake/src/leju-controllers/leju-rl-controller/run_rl_controller \
  -c $RLC/config/${ROBOT_VERSION}/controller_manager.yaml \
  -t $HOME/.config/lejuconfig/teleop_bindings.yaml
```
`~/.config/lejuconfig/teleop_bindings.yaml` 是真机运行及桌面端写回的现场配置；
`$RLC/config/${ROBOT_VERSION}/teleop_bindings.yaml` 仅作为安装时的标准模板。
显式 `-t/--teleop-config` 的优先级最高。

正常启动日志应依次出现:`Sensor data ready` → `Moving to default position` → `Starting ControlLoop` → `Lifecycle: 等待 start 启动信号`(到达 WaitingForStart)。

#### (4) 驱动控制器(joy_trigger,在板子另开终端,同 DDS 环境)
```bash
export CYCLONEDDS_URI=file://$HOME/cyclonedds_board.xml
JT=build_cmake/tools/joy_trigger
$JT start                 # 进 kRunning,开始跑策略
$JT walk 0 0.3 0 0 5      # 速度: lx ly rx ry [秒]; 这里前进 0.3 持续 5s
$JT south                 # = A 键 → 触发 motion(mimic 类控制器跳舞)
$JT north                 # = Y 键 → 切换到 mimic_dance
$JT back                  # 停止 runtime
```
按键映射(SDL 手柄):`south=A  east=B  west=X(joy_trigger 暂不支持)  north=Y  start=START  back=BACK  guide=GUIDE`。
v17 teleop 绑定:`X→切amp  Y→切mimic_dance  A→MotionCommand(起舞)`。

---

## 6. 真机运行(简述)

```bash
sudo su
export ROBOT_VERSION=17
bash src/leju_launch/scripts/launch_real.sh            # 普通
bash src/leju_launch/scripts/launch_real.sh --auto-start --config <path>   # 自启动模式
```
需要 root(CAN 访问 + RT 内存锁定)。手柄自启动服务部署见 `src/leju-joystick/services/deploy_joy_autostart.sh`(板子上 root 执行)。

---

## 7. 已知问题与排坑(本分支当前状态)

1. **必须 `stdbuf -oL` 跑控制器**:否则 `kill -9` 会丢失缓冲日志,看起来像"启动到一半卡死",其实是日志没刷出来。
2. **跨机 DDS 同机发现**:配置里必须有 `<Peer address="localhost"/>`,否则同机的 controller 与 joy_trigger 互相发现不了(START 发不进去)。仓库自带的 `cyclonedds.xml` 是 lo-only,跨机不可用。
3. **VR API 双重初始化失败**(日志 `[VRCommunication] Failed to initialize: Could not create topic`):QuestTeleopAdapter 与 ExternalInterface 都初始化 VR,第二次失败。**已被 `continuing without VR support` 兜底,不阻塞**,核心控制器正常。但这意味着 ExternalInterface 的 startRuntime/stopRuntime RPC 可能未注册成功 → **手柄自启动的 RPC 路径待修**(用 joy_trigger 的内置 START/BACK 不受影响)。
4. **控制器不响应 SIGTERM**:停止需 `kill -9`(优雅退出卡住)。影响自启动 BACK→stop 流程,待修。
5. **仿真不稳定(待调参,非代码 bug)**:v17 下 amp 与 mimic_dance 都会让 MuJoCo 在 `actuator 10`(右腿)报 `NaN/unstable`(~14s)。两个控制器都崩在同一关节 → 系统性,疑点在**关节顺序映射 / CST 力矩下 kp-kd / 模型与 sim 匹配**(`config/17` 当前用的是 **dev 的 ONNX 模型**,非 fork 原 roban 模型,可换回 `git show 310523f0^:.../config/17/policy_roban_2026-05-06_*.onnx` 对比)。
6. **mimic 起舞**:`guide` 键在 v17 teleop_bindings 里**没绑定**(日志提示 "Press guide" 与配置不符),实际起舞键是 **A**(`joy_trigger south`)。
7. **CST 模式 kp/kd**:`actuator_control_mode` 腿=0(CST 力矩)、臂=2(CSP 位置)。CST 下 kp/kd 须为 0(否则双重 PD)——排查仿真不稳时重点核对。

---

## 8. 附录:目录速查

| 路径 | 说明 |
|------|------|
| `~/.config/lejuconfig/teleop_bindings.yaml` | 真机手柄运行配置，也是桌面端绑定写回目标 |
| `src/leju-controllers/leju-rl-controller/config/<ver>/` | 控制器标准配置模板(controller_manager.yaml / controllers/*.yaml / teleop_bindings.yaml / policy/*.onnx) |
| `src/leju-controllers/leju-rl-controller/src/runtime/` | dev 新架构:lifecycle / control_loop / control_logic / input(teleop, external_interface) |
| `src/lejusdk/lejusdk-vr/` | VR SDK + runtime RPC(startRuntime/stopRuntime/getRuntimeState) |
| `src/leju-joystick/services/` | 手柄自启动 manager + systemd 部署脚本 |
| `src/leju_launch/scripts/` | shell 启动体系(launch_real.sh / launch_mujoco_sim.sh / _launch_common.sh / start_roudi.sh) |
| `tools/joy_trigger.cpp` | 程序化发 JoyData(测试驱动) |
| `docs/plans/2026-05-29-rl-controller-dev-sync-and-joy-autostart.md` | 本次迁移的四阶段计划 |
