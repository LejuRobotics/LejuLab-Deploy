# leju-remote

`leju-remote` 是 Quest3 数据接入与转发工具。它负责接收 Quest3 端发来的 UDP 数据，并将骨骼与手柄信息转换为本地系统可消费的数据格式，供上层 VR 控制节点使用。

该包本身不负责 IK、模式切换或机器人动作策略。

## 功能概览

- 接收 Quest3 端发送的 UDP 数据包
- 解析 Quest3 手柄与骨骼相关 protobuf 消息
- 将 Quest3 数据转发给本地 VR 相关节点使用
- 提供自动发现模式与指定 IP 直连模式


## 使用前准备

### Quest3 端版本

Quest3 端应用需要与当前 PC 端协议版本兼容。当前稳定版本为：

- `0.0.1-181-g2b295f2`
- APK 下载地址：
  - `https://kuavo.lejurobot.com/Quest_apks/leju_kuavo_hand-0.0.1-181-g2b295f2.apk`

如团队后续发布了新的兼容版本，请以团队通知为准，不建议随意混用不同来源或不同阶段的 Quest3 应用。

### Quest3 安装方式

Quest3 安装方式准备环境：

- SideQuest 安装包：
  - `https://kuavo.lejurobot.com/Quest_apks/SideQuest-Setup-0.10.42-x64-win.exe`
- Quest3 安装与更新说明：
  - `https://kuavo.lejurobot.com/beta_manual/basic_usage/kuavo-ros-control/docs/Quest3_VR_basic/index.html`

通常要求：

- Quest3 与 PC 处于可通信网络环境
- Quest3 端应用已正确安装并启动
- PC 端与 Quest3 端协议版本保持一致


## 构建

```bash
cd /path/to/lejulab_platform
catkin build leju-remote
```

## 启动方式

### 方式 1：等待 Quest3 主动连接

```bash
rosrun leju-remote quest_udp_to_dds_node
```

### 方式 2：指定 Quest3 IP

```bash
rosrun leju-remote quest_udp_to_dds_node 192.168.1.100
```

如果需要指定端口：

```bash
rosrun leju-remote quest_udp_to_dds_node 192.168.1.100:10019
```

## Proto 文件生成

`leju-remote` 当前直接维护 protobuf 生成后的 C++ 文件，位于：

- 源 proto：`protos/protos/*.proto`
- 生成结果：`protos_c/*.pb.h`、`protos_c/*.pb.cc`

当 `.proto` 文件发生变更后，需要手动重新生成对应的 C++ 文件。

### 生成脚本

仓库内已提供脚本：

```bash
cd src/leju-remote/protos_c
./generate.sh
```

### 生成方式

该脚本本质上会执行类似如下命令：

```bash
protoc --proto_path="../protos" --cpp_out="." \
  protos/hand_pose.proto \
  protos/hand_wrench_srv.proto \
  protos/robot_info.proto
```

脚本执行后会：

- 生成 `.pb.h` 和 `.pb.cc`
- 修正生成文件中的 include 路径
- 将最终结果整理到 `protos_c/` 目录

### 生成前提

- 需要本机已安装 `protoc`
- 要求 `protoc 3.15+`

## 常见问题

### 启动后收不到 Quest3 数据

先检查：

- Quest3 端应用是否已经启动
- Quest3 与 PC 是否在可通信网络环境
- 指定的 IP 和端口是否正确
- PC 端防火墙是否拦截了 UDP 通信

### 修改了协议字段但程序行为异常

优先检查是否重新生成了 protobuf C++ 文件。
只修改 `.proto` 而未重新执行 `protoc`，常常会导致编译通过但运行行为不一致。

### `leju-remote` 启动成功但机器人没有动作

这是正常的。`leju-remote` 只负责 Quest3 数据接入与转发，机器人是否动作取决于上层节点，例如 `leju-vr-control`。
