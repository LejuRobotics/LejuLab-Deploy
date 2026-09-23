# RK3588 环境部署指南

在新设备上复制当前 3588 (192.168.50.157) 的完整运行环境。

## 基础信息

| 项目 | 当前值 |
|------|--------|
| 系统 | Ubuntu 22.04.3 LTS (Jammy) aarch64 |
| 内核 | 5.10.226-rt89 PREEMPT_RT (自编译) |
| 用户 | linux / linux |
| 项目路径 | /home/linux/lejulab_platform |
| 网络 | WiFi `Lj_software_2` → 192.168.50.x |

---

## 第 1 步：系统预装 (出厂已有)

以下组件通常由 RK3588 出厂镜像提供，无需手动安装：

- Ubuntu 22.04 aarch64 base system
- xubuntu-core 桌面
- NetworkManager, SSH server
- bcan0-bcan3 CAN 接口（内核驱动 `rockchip_canfd`）
- GPU 驱动 `libmali-bifrost-g52-g13p0-x11-gbm`
- lightdm 显示管理器

---

## 第 2 步：PREEMPT_RT 内核

**当前内核**：`5.10.226-rt89`（自编译，非标准 Ubuntu 内核）

```bash
# 确认内核包已安装
dpkg -l | grep linux-headers-5.10.226-rt89
# 输出: linux-headers-5.10.226-rt89  5.10.226-rt89-135  arm64

uname -r
# 输出: 5.10.226-rt89
uname -v
# 输出: #5 SMP PREEMPT_RT ...
```

> **注意**：RT 内核通常需要从源码编译或从其他设备拷贝 deb 包。
> 如果新设备有相同硬件，可直接从旧设备 `/boot/` 和 `/lib/modules/5.10.226-rt89/` 拷贝。

---

## 第 3 步：安装必要的 apt 包

```bash
sudo apt-get update && sudo apt-get install -y \
    build-essential cmake g++-12 gcc-12-base \
    git vim htop tree gdb \
    can-utils \
    libacl1-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    libncurses5-dev libncursesw5-dev \
    libgtest-dev googletest \
    libprotobuf-dev protobuf-compiler \
    libgflags-dev \
    libcurl4-openssl-dev \
    libglew-dev libglfw3-dev libxkbcommon-dev \
    zlib1g-dev \
    python3-dev python3-pip python3-venv \
    ninja-build autoconf automake libtool pkg-config \
    net-tools ethtool i2c-tools \
    rt-tests stress stress-ng \
    rsync lrzsz zip
```

---

## 第 4 步：CAN 总线开机自启

### 4.1 创建 CAN 初始化脚本

```bash
sudo tee /etc/init.d/can_add_server.sh > /dev/null << 'SCRIPT'
#!/bin/sh
sleep 0.5

CAN_NAME_PREFIX="bcan"

# 配置 CAN FD 接口 (bcan0-3)
for i in 0 1 2 3; do
    sudo ip link set ${CAN_NAME_PREFIX}${i} type can \
        bitrate 1000000 sample-point 0.740 \
        dbitrate 5000000 dsample-point 0.700 \
        fd on loopback off
    sudo ip link set up ${CAN_NAME_PREFIX}${i}
    sudo ifconfig ${CAN_NAME_PREFIX}${i} txqueuelen 5000
done

# 绑定 CAN 中断线程到大核 (CPU 4-7)
echo "=== 绑定 CAN 中断线程 ==="
for i in 0 1 2 3; do
    IRQ_NUM=$((153 + i))
    CPU_NUM=$((4 + i))
    pid=$(ps aux | grep "irq/${IRQ_NUM}-bcan${i}" | grep -v grep | awk '{print $2}')
    if [ ! -z "$pid" ]; then
        echo "绑定 bcan${i} (PID $pid) 到 CPU ${CPU_NUM}"
        taskset -cp ${CPU_NUM} $pid
    fi
done
echo "=== 绑定完成 ==="

exit 0
SCRIPT
sudo chmod +x /etc/init.d/can_add_server.sh
```

> **注意**：IRQ 号 (153-156) 和 CPU 核号 (4-7) 可能因硬件版本不同而变化。
> 新设备上执行 `cat /proc/interrupts | grep bcan` 确认实际 IRQ 号。

### 4.2 注册 systemd 服务

```bash
sudo tee /etc/systemd/system/can_add_server.service > /dev/null << 'EOF'
[Unit]
Description=Initialize CAN bus interfaces
After=network.target

[Service]
Type=oneshot
ExecStart=/etc/init.d/can_add_server.sh

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable can_add_server.service
sudo systemctl start can_add_server.service
```

### 4.3 验证

```bash
ip link show type can
# 应看到 bcan0-bcan3 全部 UP
candump -t d bcan0   # 有电机连接时应有数据
```

---

## 第 5 步：iceoryx + CycloneDDS (DDS 共享内存通信)

### 5.1 安装 iceoryx RouDi

```bash
# RouDi 二进制 (从旧设备或项目编译产物拷贝)
sudo mkdir -p /opt/iceoryx/bin
sudo cp <path_to>/iox-roudi /opt/iceoryx/bin/
sudo chmod +x /opt/iceoryx/bin/iox-roudi
```

> RouDi 也可通过项目脚本自动部署：
> `bash src/leju_launch/scripts/setup_cyclonedds_config.sh`

### 5.2 创建 iceoryx 用户组

```bash
sudo groupadd -f iceoryx
sudo usermod -aG iceoryx linux
sudo usermod -aG iceoryx root
```

### 5.3 RouDi 配置

```bash
sudo mkdir -p /etc/iceoryx
sudo tee /etc/iceoryx/roudi_config.toml > /dev/null << 'EOF'
[general]
version = 1

[[segment]]

[[segment.mempool]]
size = 128
count = 10000

[[segment.mempool]]
size = 1024
count = 5000

[[segment.mempool]]
size = 16384
count = 1000

[[segment.mempool]]
size = 131072
count = 200

[[segment.mempool]]
size = 1048576
count = 50
EOF
```

### 5.4 CycloneDDS 配置

```bash
sudo mkdir -p /etc/cyclonedds

# 共享内存模式 (推荐)
sudo tee /etc/cyclonedds/cyclonedds_shm.xml > /dev/null << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <SharedMemory>
            <Enable>true</Enable>
            <LogLevel>error</LogLevel>
        </SharedMemory>
        <General>
            <Interfaces>
                <NetworkInterface name="lo" multicast="default" />
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
            <EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints>
        </Discovery>
    </Domain>
</CycloneDDS>
EOF

# UDP 回退模式
sudo tee /etc/cyclonedds/cyclonedds.xml > /dev/null << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="lo" multicast="default" />
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
            <EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints>
        </Discovery>
    </Domain>
</CycloneDDS>
EOF
```

### 5.5 RouDi systemd 服务

```bash
sudo tee /etc/systemd/system/leju-roudi.service > /dev/null << 'EOF'
[Unit]
Description=iceoryx RouDi daemon
After=network.target

[Service]
Type=simple
User=root
Group=iceoryx
UMask=0000
Environment="CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds_shm.xml"
ExecStartPre=-/bin/rm -f /tmp/roudi /tmp/roudi.lock
ExecStart=/opt/iceoryx/bin/iox-roudi -c /etc/iceoryx/roudi_config.toml
ExecStartPost=/bin/bash -c 'for i in $(seq 1 50); do [ -S /tmp/roudi ] && break; sleep 0.1; done; chmod 0666 /tmp/roudi 2>/dev/null; chmod 0666 /dev/shm/iceoryx_mgmt 2>/dev/null; chmod 0666 /dev/shm/iceoryx 2>/dev/null; true'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable leju-roudi.service
sudo systemctl start leju-roudi.service
```

### 5.6 验证

```bash
systemctl status leju-roudi
ls /dev/shm/iceoryx*       # 应有 iceoryx 和 iceoryx_mgmt
ls /tmp/roudi              # 应存在 socket
id linux | grep iceoryx    # 确认用户在 iceoryx 组
```

---

## 第 6 步：环境变量

```bash
# 追加到 ~/.bashrc
cat >> ~/.bashrc << 'EOF'
export ROBOT_VERSION=14
export CYCLONEDDS_URI="file:///etc/cyclonedds/cyclonedds_shm.xml"
EOF
source ~/.bashrc
```

---

## 第 7 步：项目配置文件

```bash
mkdir -p ~/.config/lejuconfig

# CAN 总线接线方式
echo "dual_bus" > ~/.config/lejuconfig/CanbusWiringType.ini

# IMU 类型 (none = 不使用 IMU)
echo "none" > ~/.config/lejuconfig/ImuType.ini
```

CAN 总线电机配置文件 `canbus_device_cofig.yaml` 从旧设备拷贝：

```bash
scp linux@<旧设备IP>:~/.config/lejuconfig/canbus_device_cofig.yaml ~/.config/lejuconfig/
```

如有 RUIWO 手臂电机零点标定数据：

```bash
scp linux@<旧设备IP>:~/.config/lejuconfig/ruiwo_zero.yaml ~/.config/lejuconfig/
```

---

## 第 8 步：部署项目

```bash
cd /home/linux
git clone <仓库地址> lejulab_platform
cd lejulab_platform
git checkout zhongxu/feat/remove-catkin-pure-cmake

# 编译（必须使用 gcc-11，与预编译 DDS 库的 LTO 版本匹配）
CC=gcc-11 CXX=g++-11 cmake -Bbuild_cmake \
    -DCMAKE_BUILD_TYPE=Release

CC=gcc-11 CXX=g++-11 cmake --build build_cmake -j4
```

> **首次编译前**需先编译 DDS 第三方库（iceoryx/CycloneDDS），
> 参考 `src/lejusdk/3rd_party/BUILD_CYCLONEDDS.md`。

---

## 第 9 步：验证部署

```bash
# 运行全链路诊断
sudo bash scripts/diagnostics/check_all.sh

# 启动 hardware_node
export ROBOT_VERSION=14
sudo -E ./build_cmake/src/leju-hardware/leju-hardware ./src/leju-hardware

# 启动键盘控制器
sudo bash src/leju_launch/scripts/launch_keyboard_ctrl.sh
```

---

## 快速迁移清单

从旧设备直接拷贝的文件（适用于相同硬件）：

```bash
OLD=linux@192.168.50.157

# 1. RT 内核模块
scp -r ${OLD}:/lib/modules/5.10.226-rt89 /lib/modules/

# 2. iceoryx RouDi 二进制
scp ${OLD}:/opt/iceoryx/bin/iox-roudi /opt/iceoryx/bin/

# 3. 项目配置
scp ${OLD}:~/.config/lejuconfig/canbus_device_cofig.yaml ~/.config/lejuconfig/
scp ${OLD}:~/.config/lejuconfig/ruiwo_zero.yaml ~/.config/lejuconfig/

# 4. 系统服务配置
scp ${OLD}:/etc/init.d/can_add_server.sh /etc/init.d/
scp ${OLD}:/etc/systemd/system/can_add_server.service /etc/systemd/system/
scp ${OLD}:/etc/systemd/system/leju-roudi.service /etc/systemd/system/
scp ${OLD}:/etc/iceoryx/roudi_config.toml /etc/iceoryx/
scp -r ${OLD}:/etc/cyclonedds/ /etc/cyclonedds/

# 5. 刷新服务
sudo systemctl daemon-reload
sudo systemctl enable can_add_server leju-roudi
sudo systemctl start can_add_server leju-roudi
```

---

## 环境分层总结

| 层级 | 来源 | 内容 |
|------|------|------|
| **系统镜像** | 出厂 | Ubuntu 22.04, GPU 驱动, bcan 内核驱动, xubuntu |
| **RT 内核** | 自编译 | `5.10.226-rt89` PREEMPT_RT, headers |
| **apt 包** | 手动安装 | g++-12, can-utils, libeigen3-dev, libgtest-dev, 等 |
| **CAN 服务** | 手动配置 | `/etc/init.d/can_add_server.sh` + systemd |
| **DDS/iceoryx** | 项目编译 | RouDi → `/opt/iceoryx/`, 配置 → `/etc/iceoryx/` + `/etc/cyclonedds/` |
| **用户组** | 手动配置 | `iceoryx` 组 (linux + root) |
| **环境变量** | ~/.bashrc | `ROBOT_VERSION=14`, `CYCLONEDDS_URI` |
| **项目配置** | 手动/标定 | `~/.config/lejuconfig/` (CAN 电机配置, IMU, 零点) |
| **项目代码** | git | `/home/linux/lejulab_platform` |
