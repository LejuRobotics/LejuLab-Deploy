# leju-ik

`leju-ik` 提供当前项目中的 Quest3 相关 IK 能力，面向上层集成者与节点调用者使用。它负责将抽象后的姿态、手柄与传感器输入转换为机器人可执行的关节目标或姿态结果。

该包是算法层，不是最终用户操作入口。

## 当前支持能力

当前已支持的能力主要包括：

- Quest3 手臂 IK 能力
- Quest3 增量控制主链
- `Quest3IkAPI`
- `Quest3IkIncrementalAPI`
- 手臂模式切换
- 握把(`grip`)进入/退出增量控制
- 当前姿态重建与增量控制运行

当前版本已经可以支撑 `leju-vr-control` 的手臂增量控制主链。手肘平滑与切换细节仍在持续优化中。

## 主要接口

### `Quest3IkAPI`

适用于 Quest3 基础 IK 能力接入，主要提供：

- `init(...)`
- `onBonePoses(...)`
- `onJoystick(...)`
- `setArmMode(...)`
- `runOnce()`

### `Quest3IkIncrementalAPI`

适用于 Quest3 增量控制接入，主要提供：

- `init(...)`
- `onBonePoses(...)`
- `onJoystick(...)`
- `onArmCtrlModeState(...)`
- `onSensorArmJoints(...)`
- `setArmMode(...)`
- `runOnce()`

## 使用方式

一个典型的集成流程如下：

1. 初始化 `Quest3IkAPI` 或 `Quest3IkIncrementalAPI`
2. 持续输入 Quest3 骨骼数据
3. 持续输入手柄数据
4. 如需增量控制，持续输入当前手臂传感器关节角
5. 根据上层模式逻辑调用 `setArmMode(...)`
6. 周期调用 `runOnce()`
7. 通过回调或输出结果消费 IK 解算结果

## 输入边界

`leju-ik` 接收的是抽象输入，不直接解释 Quest3 设备按键语义。

例如：

- 骨骼数据通过 `onBonePoses(...)` 输入
- 手柄模拟量通过 `onJoystick(...)` 输入
- 抽象后的每臂控制模式状态通过 `onArmCtrlModeState(...)` 输入

Quest3 的按键解释、模式逻辑与用户操作入口，通常由 `leju-vr-control` 在上层处理后再传入 `leju-ik`。

## 适用范围与限制

- 当前重点支持 `Kuavo` 系列的 Quest3 增量控制能力
- `leju-ik` 是算法层，不直接提供用户交互入口
- `leju-vr-control` 是其上层调用者之一
- 本 README 不展开底层 `lejusdk-vr` 的内部实现与接口细节

## 与 `leju-vr-control` 的关系

- `leju-vr-control` 负责 VR 输入解释、按键逻辑与模式切换
- `leju-ik` 负责 IK 求解与姿态转换
- `lejusdk-vr` 是底层内部依赖，不作为用户层直接使用入口

如果只是使用 VR 控制功能，优先阅读 `src/leju-vr-control/README.md`。如果需要做算法接入或节点集成，再阅读本 README 与头文件接口。
