---
name: mcap-analysis
description: 解析 Leju CycloneDDS MCAP 录制文件 — 查看概览、分析频率、导出消息、生成 CSV
argument-hint: <mcap文件路径> [操作: info | stats | dump | csv | 自由描述]
---

# Leju MCAP 解析工具集

解析 CycloneDDS recorder 产生的 MCAP 文件（`omgidl + CDR` 编码的 `leju::msgs::*`）。
所有脚本位于 `scripts/mcap/`，需要 `pip install mcap`，推荐 `pip install pycdr2`。

## 背景知识

- `mcap` Python 库只读容器（topic 列表、元信息），**不解码 payload**。
- MCAP 内每个 schema 自带完整 OMG IDL 文本（`schema.data`）。
- `leju_msgs.py` 在运行时解析 IDL → 用 `pycdr2` 动态构建解码器 → 返回 plain dict。
  **新 schema 零代码改动**。没装 pycdr2 时回退到 7 个已知 schema 的硬编码解码器。
- `decode()` 返回的 dict 包含 IDL 全量字段，含 `header_sec` / `header_nanosec`（采集时刻）
  和 `JointCmd.modes`（关节控制模式）。

## 可用脚本

| 脚本 | 用途 | 关键参数 |
|------|------|----------|
| `leju_msgs.py` | 共享解码库（IDL-driven + hardcoded fallback），无 CLI | `from leju_msgs import decode, register_from_mcap_summary` |
| `mcap_info.py` | 文件概览：header / 统计 / schema / channel Hz | `--json` |
| `mcap_topic_stats.py` | 每 topic 频率 / p50·p99 dt / 丢帧 | `--topic <t>` `--json` |
| `mcap_dump_topic.py` | 单 topic → JSONL | `--topic <t>` `--head N` `--start/--end` `--raw` `--pretty` |
| `mcap_export_csv.py` | 数值字段 → CSV | `--topic <t>` `--field <f>` `-o <path>` |
| `spike_idl_decode.py` | 回归校验：全量对拍 dynamic vs hardcoded | 无参数 |

## 执行步骤

### 1. 确定 MCAP 文件路径

```bash
# 常见位置
ls ~/.ros/lejulab/mcap/*.mcap | tail -5
```

### 2. 概览（info）

```bash
python3 scripts/mcap/mcap_info.py ~/.ros/lejulab/mcap/recording_xxx.mcap
python3 scripts/mcap/mcap_info.py ~/.ros/lejulab/mcap/recording_xxx.mcap --json | jq '.channels[] | select(.hz > 100) | .topic'
```

输出：消息总数、时长、channel 数、每个 schema 的解码路径（`[dyn]`=IDL 驱动 / `[hc]`=硬编码）、
每个 topic 的消息数和平均 Hz。

### 3. 频率 / 丢帧分析（stats）

```bash
python3 scripts/mcap/mcap_topic_stats.py ~/.ros/lejulab/mcap/recording_xxx.mcap
python3 scripts/mcap/mcap_topic_stats.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /rt/joint_cmd --topic /rt/joint_state
python3 scripts/mcap/mcap_topic_stats.py ~/.ros/lejulab/mcap/recording_xxx.mcap --json | jq '.[] | select(.drops > 0)'
```

关注 `drops > 0` 和 `max_dt` 异常大的 topic。

### 4. 查看消息内容（dump）

```bash
# 前 N 条
python3 scripts/mcap/mcap_dump_topic.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /rt/imu_state --head 5

# 时间切片（秒，相对 bag 起始）
python3 scripts/mcap/mcap_dump_topic.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /rt/joint_cmd --start 10 --end 10.1

# 美化输出
python3 scripts/mcap/mcap_dump_topic.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /rt/joy --head 1 --pretty

# 不认识的 schema → 看原始字节
python3 scripts/mcap/mcap_dump_topic.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /some/topic --head 1 --raw

# 配合 jq
python3 scripts/mcap/mcap_dump_topic.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /amp/q_target --head 100 | jq -c '{t, v0: .data.data[0]}'
```

每行一条消息，格式：`{"t": <秒>, "log_time_ns": <int>, "topic": <str>, "data": <dict>}`

### 5. 导出 CSV（csv）

```bash
python3 scripts/mcap/mcap_export_csv.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /amp/q_target -o q_target.csv
python3 scripts/mcap/mcap_export_csv.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /rt/joint_state --field q -o joint_q.csv
python3 scripts/mcap/mcap_export_csv.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /rt/joint_cmd --field modes -o modes.csv
python3 scripts/mcap/mcap_export_csv.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /rt/imu_state --field acc -o acc.csv
python3 scripts/mcap/mcap_export_csv.py ~/.ros/lejulab/mcap/recording_xxx.mcap --topic /amp/arm_mode -o - | head   # stdout
```

默认字段：Float64/Float64Array → `data`，JointState/JointCmd → `q`，ImuData → `gyro`，Joy → `axes`。
`--field` 可传 IDL 里任意数值字段（`tau` / `kp` / `kd` / `modes` / `quat` / `buttons` / `header_sec` 等）。

### 6. 回归校验

```bash
python3 scripts/mcap/spike_idl_decode.py ~/.ros/lejulab/mcap/recording_xxx.mcap
# 期望输出: ✓ all messages matched
```

### 7. Python 库调用

```python
import leju_msgs
from mcap.reader import make_reader

with open(path, "rb") as f:
    reader = make_reader(f)
    leju_msgs.register_from_mcap_summary(reader.get_summary())  # 读 IDL 构建 decoder
    for sch, ch, m in reader.iter_messages():
        payload = leju_msgs.decode(sch.name, m.data)  # → plain dict
```

## 已知 schema 参考

```
leju::msgs::Float64        → {"header_sec", "header_nanosec", "data": float}
leju::msgs::Float64Array   → {"header_sec", "header_nanosec", "data": [float...]}
leju::msgs::StringData     → {"header_sec", "header_nanosec", "data": "str"}
leju::msgs::JointState     → {header, "q":[23], "v":[23], "vd":[23], "tau":[23]}
leju::msgs::JointCmd       → {header, "q":[23], "v":[23], "tau":[23], "kp":[23], "kd":[23], "modes":[23]}
leju::msgs::ImuData        → {header, "gyro":[3], "acc":[3], "free_acc":[3], "quat":[4]}
leju::msgs::Joy            → {header, "axes":[6 float32], "buttons":[16 int32]}
```

新 schema 只要录制进 mcap 就自动可用（IDL 驱动），无需修改任何脚本。

## CDR 布局速查表

每条消息 payload 前 4 字节是 encapsulation header（`0x0001` = CDR_LE）。
CDR 对齐从 encap 之后的流起点算：`(raw_offset - 4) % N == 0`。

所有 `leju::msgs::*` 以 `int32 header_sec; uint32 header_nanosec;` 开头（8 字节），
业务字段从 raw 偏移 12 开始。

```
uint32 / int32 / float32  → 4-byte 对齐
float64 (double)          → 8-byte 对齐
sequence<T>               → uint32 length + pad-to-alignment-of-T + N × T
T[N] (fixed array)        → 直接 N × T，无 length prefix
string                    → uint32 length (含尾 '\0') + length bytes
```

## 添加新 schema

**有 pycdr2（推荐）**：什么都不用改。recorder 把 IDL 写进 mcap → `register_from_mcap_summary()` 自动解析。

**IDL 解析失败**（用了 typedef / enum / union / 嵌套 struct 等未覆盖构造）：
在 `leju_msgs.py` 的 `_IDLParser` 扩展对应语法，或在 `_HARDCODED` 字典手写 decoder。

**没装 pycdr2**：只有 `_HARDCODED` 里的 7 个 schema 可用。

## 重要规则

- 脚本路径: `scripts/mcap/<name>.py`，从项目根目录执行
- `decode()` 始终返回 plain dict，字段名对应 IDL 定义
- `--json` 输出可直接 `| jq` 或 `| python3 -c "import json,sys; ..."`
- 遇到 `UnsupportedSchema` → 建议 `pip install pycdr2` 或用 `--raw` 看字节
- **不要修改** `scripts/diagnose_roban2_mimic_mcap.py`（老脚本，有已知 JointCmd 解码 bug）
- CSV 导出 `--field` 只接受数值字段；`StringData.data` 是字符串，不能导 CSV

## 附: 官方 mcap CLI 安装

本套脚本不依赖官方 CLI，但看 topic 列表、切片、合并 bag 时它更顺手。

**坑点先说**：
- `npm i -g mcap-cli` 装的是 mwaylabs 的 app 脚手架，只是名字撞车，**不是这个**。
- `@mcap/cli` 这个 npm 包不存在，别按印象去装。
- GitHub 的 `/releases/latest` 指向 lib release（没二进制），会 404。必须用显式 tag。

推荐方式（预编译二进制，约 27 MB）：

```bash
mkdir -p ~/.local/bin
# 版本号到 https://github.com/foxglove/mcap/releases 找最新 "releases/mcap-cli/vX.Y.Z"
curl -fL "https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.62/mcap-linux-amd64" -o ~/.local/bin/mcap
chmod +x ~/.local/bin/mcap
mcap version   # -> v0.0.62
```

装好后对 Leju profile 的 mcap，CLI 只能给出 header / statistics / channel message count —— 想看消息内容仍然要回到本目录下的 Python 脚本。
