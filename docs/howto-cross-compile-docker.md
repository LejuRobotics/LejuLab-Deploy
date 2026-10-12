# RK3588 交叉编译 Docker 使用指南

在 x86 开发机上**真交叉编译**整个仓库到 aarch64（RK3588），比 QEMU 模拟快约 10×。
设计与实测见 `docs/plans/2026-05-30-rk3588-cross-compile-docker.md`。

## 原理一句话

x86 宿主里跑原生 `aarch64-linux-gnu-gcc-11`（与板子 gcc 11.4.0 同版本，能链接仓库内
GCC11 LTO 预编译的 CycloneDDS/iceoryx），配一份从真实板子 rsync 出来的 sysroot，
全部烘进一个自包含 Docker 镜像。

## 三步上手

```bash
cd docker

# 1) 从板子提取 sysroot（首次，~6.8G；默认 test@192.168.28.12，含 drake）
./extract_sysroot.sh
#   换板子：BOARD_HOST=192.168.50.239 BOARD_PASS=xxx ./extract_sysroot.sh
#   不要 drake：./extract_sysroot.sh --without-drake

# 2) 构建镜像并导出自包含 tar（lejulab-cross-aarch64.tar.gz）
./build_image.sh
#   只 build 不导出：./build_image.sh --no-save

# 3) 交叉编译整个平台 → build_cross/
./run_build.sh
#   只编某目标：./run_build.sh --target test_robot_version
#   自定义开关：CMAKE_ARGS="-DBUILD_MUJOCO_SIM=OFF" ./run_build.sh
```

产物在 `build_cross/`，全部为 `ELF 64-bit LSB ... ARM aarch64`，可直接 scp 到板子运行。

## 在另一台机器使用（离线）

`./build_image.sh` 产出的 `docker/lejulab-cross-aarch64.tar.gz` 自包含 sysroot，拷到任意
装了 docker 的 x86 机器：

```bash
docker load -i lejulab-cross-aarch64.tar.gz
# 然后在仓库根目录 ./docker/run_build.sh 即可（无需联网、无需连板子）
```

## 关键设计点（排查时有用）

- **toolchain**：`docker/toolchain-aarch64.cmake`，设 cross 编译器 + `CMAKE_SYSROOT` +
  `DRAKE_PREFIX`（指向 sysroot 内 `/opt/drake`）+ `FIND_ROOT_PATH_MODE_PACKAGE BOTH`
  （兼顾 sysroot 与仓库内 3rd_party config）。
- **cmake 3.22**（镜像内 apt 装，与板子 3.22.1 一致）：便于及早暴露板子上会遇到的问题。
- **sysroot 内容**：`/usr/{include,lib,local,share}` + drake，绝对软链已相对化。
  `/usr/share` 必须带（`Eigen3Config.cmake` 等 cmake config 在此）。

> 实现初期曾在 docker 层规避两处仓库 CMake 问题（`tools/joy_trigger` 跨目录 IMPORTED
> 目标作用域、`hipnuc/xsens` 硬编码 `/usr/include/eigen3`），后已修复仓库源码根因
> （根 CMakeLists 顶层 find_package iceoryx；改用 `${EIGEN3_INCLUDE_DIR}`），规避随之移除。

## 与旧 QEMU 方案的关系

旧的 QEMU 模拟方案脚本保留为 `docker/*_qemu.sh` + `Dockerfile.aarch64` 作兜底；
新交叉编译方案为**首选**（更快、不依赖 binfmt/QEMU）。

## 常见问题

- **`extract_sysroot.sh` 卡住/无权限**：板子 `test` 用户需 passwordless sudo（脚本用
  `--rsync-path="sudo rsync"` 读全 `/usr/lib`）。
- **`find_package(X)` 失败**：板子缺该 dev 包，或其 cmake config 在未提取的目录；
  确认板子 `dpkg -l | grep X`，必要时在 extract 增拉对应路径。
- **链接报 LTO 版本不匹配**：cross gcc 不是 11.x；Dockerfile 已固定 `gcc-11-aarch64`，
  勿改成 12+。
