# lejusdk-dds-idl

DDS IDL 消息定义包，提供机器人通信所需的数据类型。

## IDL 消息类型

| IDL 文件 | 消息类型 | 说明 |
|----------|----------|------|
| `JointState.idl` | `JointState` | 关节状态反馈（位置、速度、力矩） |
| `JointCmd.idl` | `JointCmd` | 全身关节命令 |
| `JointTrajectory.idl` | `JointTrajectoryPoint` | 关节轨迹点（手臂、头部、腰部） |
| `ImuData.idl` | `ImuData` | IMU 传感器数据 |
| `VelocityCmd.idl` | `VelocityCmd` | 速度指令（线速度、角速度） |
| `Joy.idl` | `Joy` | 手柄输入 |
| `Float64Types.idl` | `Float64` / `Float64Array` | 通用浮点类型 |
| `StringData.idl` | `StringData` | 字符串消息 |
| `TactState.idl` | `TactState` | tact 播放状态回报（state、error_code、error_message，1Hz） |

## 目录结构

```
lejusdk-dds-idl/
├── idl/                 # IDL 定义文件
├── include/             # 生成的 C++ 头文件
│   └── lejusdk-dds-idl/
├── src-generated/       # 生成的 C++ 源文件
├── gen_idl_cxx.sh       # IDL → C++ 生成脚本
├── CMakeLists.txt
└── package.xml
```

## 使用方法

### 新增或修改 IDL

1. 编辑或新增 IDL 文件：
```bash
vim idl/YourMessage.idl
```

2. 运行生成脚本：
```bash
./gen_idl_cxx.sh
```

3. 编译验证：
```bash
catkin build lejusdk-dds-idl
```

### 在代码中使用

```cpp
#include <lejusdk-dds-idl/JointState.hpp>
#include <lejusdk-dds-idl/JointTrajectory.hpp>

leju::msgs::JointState state;
leju::msgs::JointTrajectory traj;
```
