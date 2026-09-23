# CAN/CANFD 电机参数调试工具 使用说明

## 简介

`scripts/can_param_tool.py` 通过 CAN/CANFD 总线读写电机内部寄存器参数，用于现场整定 PID、
调节滤波、修改默认增益、标定保护阈值等。

参数协议来源：《CAN/CAN FD 修改参数说明》。
参数表覆盖 index 10-66 共 57 个寄存器，分三级：

- **只读 (RO)**：10 Firmware Version
- **普通模式 (SAFE)**：控制整定、限幅、默认增益、滤波
- **专家模式 (EXPERT)**：机械零位、物理参数、保护阈值——需输入 `UNLOCK` 解锁

## 前置条件

- Python 3.8+，已安装 `rich` (`pip3 install rich`)
- `cansend` / `candump` / `ip` / `pgrep`（标准系统工具）
- 至少一条 bcan* 总线已 UP
- **强烈建议先停掉 h12_hw_node 等硬件进程**（工具启动时会检查并提醒）

## 基本用法

```bash
python3 scripts/can_param_tool.py
```

常用选项：

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `--scan-timeout MS` | 50 | 单帧扫描超时 |
| `--scan-id-max N` | 16 | 扫描电机 ID 上限 |
| `--io-timeout MS` | 200 | 读写响应超时 |
| `--debug` | off | 打印原始 candump 输出 |
| `--no-log` | off | 关闭 `logs/can_param_*.log` 记录 |

## 操作流程

1. 启动后，工具检查是否有硬件进程占用 CAN → 若有给出警告
2. 自动扫描所有 UP 的 bcan* 接口（id 1-16），列出每条总线的在线电机
3. **选总线** (数字序号) → **选电机** (Motor ID 或 b 返回)
4. 主菜单：
   - `[1]` 读单个参数（输入 index）
   - `[2]` 读全部参数（dump 表格）
   - `[3]` 修改参数（白名单 → 选 idx → 旧值/新值 → 确认 → 回读校验）
   - `[4]` 保存到 Flash（独立二次确认，永久生效）
   - `[5]` 切换电机
   - `[e]` 切换专家模式
   - `[q]` 退出

## 安全要点

- **修改只写 RAM**，断电重启丢失。必须点 `[4]` 才能固化到 Flash。
- 回读校验不过时工具不会建议保存。
- 机械零位（22/23）、NPP/GearRatio/TorqueConstant（44-46）、保护开关（56）
  改错风险高，只在专家模式暴露。

## 日志

默认写到 `logs/can_param_YYYYMMDD_HHMMSS.log`，每次读/写/保存操作都记录，方便事后审计和恢复。

## 常见故障

- **"电机无响应"**：检查供电、接线；确认总线 bitrate 与电机匹配；确认 Motor ID 是否正确（`--scan-id-max` 可能不够大）。
- **"回读校验失败"**：多数是值超出电机内部限幅被截断；改用限幅范围内的值，或用专家模式先改 CAN COM MIN/MAX。
- **扫描看不到电机**：接口可能 DOWN，检查 `ip link show`；或 `src/leju-hardware` 相关进程在占用总线，先停掉。
