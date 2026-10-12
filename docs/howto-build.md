# 双构建系统说明

## 背景

`lejulab_platform` 同时支持两套构建系统：

| 构建方式 | 平台 | 说明 |
|----------|------|------|
| **catkin** | x86（ROS noetic） | 传统 ROS workspace 构建，产物到 `devel/lib/` |
| **纯 CMake** | ARM（RK3588）| 独立 CMake 构建，产物到 `src/<pkg>/`，不依赖 ROS |

每个 `CMakeLists.txt` 通过 `find_package(catkin QUIET)` 自动检测环境：

```cmake
find_package(catkin QUIET COMPONENTS ...)
set(_USING_CATKIN ${catkin_FOUND})
```

- ARM 上无 ROS → catkin 找不到 → 自动走纯 CMake
- x86 上有 ROS noetic → catkin 被找到 → **默认走 catkin 路径**

## 问题：x86 上默认走 catkin

x86 开发机安装了 ROS noetic 后，`find_package(catkin QUIET)` 会自动成功，导致：

- 产物输出到 `devel/lib/<pkg>/` 而非 `src/<pkg>/`
- 启动脚本 `launch_*.sh` 在 `build_cmake/src/<pkg>/` 下找不到二进制
- 构建行为和 ARM 实机不一致，不利于开发调试

## 解决方案：`-DFORCE_PURE_CMAKE=ON`

在 x86 开发机上强制走纯 CMake 路径（与 ARM 行为一致）：

```bash
cd build_cmake
cmake .. -DFORCE_PURE_CMAKE=ON
make -j$(nproc)
```

或者先清理旧的 catkin 构建缓存：

```bash
rm -rf build_cmake
mkdir build_cmake && cd build_cmake
cmake .. -DFORCE_PURE_CMAKE=ON
make -j$(nproc)
```

此时 `_USING_CATKIN=false`，各子目录的 `catkin_package()`、`${catkin_LIBRARIES}` 等会跳过，所有产物和 ARM 行为完全一致。

## 构建选项

| 参数 | 默认 | 说明 |
|------|------|------|
| `FORCE_PURE_CMAKE` | `OFF` | 强制纯 CMake 构建，忽略 catkin |
| `BUILD_RL_CONTROLLER` | `ON` | 编译 RL 控制器 |
| `BUILD_JOYSTICK` | `ON` | 编译手柄模块 |
| `BUILD_MUJOCO_SIM` | `ON` | 编译 MuJoCo 仿真 |
| `BUILD_TESTS` | `ON` | 编译测试 |
| `BUILD_EXAMPLES` | `ON` | 编译示例 |

## 各平台推荐命令

### x86 开发机（模拟 ARM 行为）

```bash
cd build_cmake
cmake .. -DFORCE_PURE_CMAKE=ON
make -j$(nproc)
```

### x86 开发机（catkin 模式，用于 ROS launch 调试）

```bash
# 在 catkin workspace 中
catkin build
```

### ARM RK3588

```bash
cd build
cmake ..
make -j$(nproc)
```

ARM 上无 ROS，catkin 自动找不到，无需 `FORCE_PURE_CMAKE`。

## 验证构建模式

编译后检查 `_USING_CATKIN` 状态：

```bash
cd build_cmake
cmake .. -DFORCE_PURE_CMAKE=ON 2>&1 | grep -i catkin
```

纯 CMake 模式下应无 catkin 相关输出（无 `catkin 0.8.10` 等日志）。

## protobuf 格式说明

`leju-remote/protos_c/` 的 `.pb.h/.pb.cc` 因 protoc 版本不兼容，拆为两套：

| 目录 | protoc 版本 | 特征 | 适用 |
|------|------------|------|------|
| `protos_c_aarch64/` | 3.21 | `port_def.inc` | ARM + x86 开发机（protobuf ≥ 3.7） |
| `protos_c_x86/` | 3.6 | `stubs/common.h` | 老 lab x86（libprotobuf 3.6.x） |

CMakeLists.txt 自动检测系统 libprotobuf 版本选择对应目录：

```cmake
if(Protobuf_VERSION ≤ 3.6.99)  →  protos_c_x86
else                           →  protos_c_aarch64
```

如需手动指定：

```bash
cmake .. -DPROTO_FORMAT=3.6    # 强制 3.6 格式
cmake .. -DPROTO_FORMAT=3.21   # 强制 3.21 格式
```
