# 电机零点标定工具 - 使用说明

## 介绍

电机零点标定工具（`motor_cali_tool.sh`）是用于对机器人关节电机进行零点校准的脚本工具。

**WARN: 本工具需在 root 权限下运行。**

## 快速预览

```bash
# 基本用法
./scripts/motor_cali_tool.sh --cali_mode <1|2|3> --cali_type <1|2|3>
```

### 参数说明

**注意：当前仅支持工装校准模式（--cali_mode 1）**

- `--cali_mode <mode>`: 校准模式（必需）
  - `1`: Tooling - 工装校准，将当前位置记做零点
  - `2`: Manual - 手动校准，从当前位置 ± 圈数进行校准
  - `3`: JointLimited - 基于关节限位校准

- `--cali_type <type>`: 校准类型（必需）
  - `1`: FullBody - 全身关节校准
  - `2`: UpperBody - 上身校准
  - `3`: LowerBody - 下身校准

## 使用

### 工装校准模式 (Tooling)

该模式下，我们通过安装工装到机器人下肢各个关节预留的孔位，将关节限制在零点位置。（事实上，手动摆正到关节零点也是Ok, 只是插工装标定会更加精准。）

对于上半身即手臂和头部关节而言，需要操作员手动将对应的关节摆正到关节零点位置。

运行标定程序，会将当前的电机的位置记录为关节的零点位置。

```bash
# 工装模式进行全身校准
sudo su
./scripts/motor_cali_tool.sh --cali_mode 1 --cali_type 1

# 工装模式进行上身校准(手臂+头部)
sudo su
./scripts/motor_cali_tool.sh --cali_mode 1 --cali_type 2

# 工装模式进行下身校准(腿部+腰部)
sudo su
./scripts/motor_cali_tool.sh --cali_mode 1 --cali_type 3
```

### 手动校准模式 (Manual)

**WARN: 暂不支持, 会陆续的更新。**

### 关节限位校准模式 (JointLimited)

**WARN: 暂不支持。**