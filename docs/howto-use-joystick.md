# 遥控器按键功能说明

> 真机运行配置：`~/.config/lejuconfig/teleop_bindings.yaml`
>
> 仓库中的 `src/leju-controllers/leju-rl-controller/config/<version>/teleop_bindings.yaml`
> 是对应机器人版本的标准模板，不是现场直接修改的运行文件。

## 安装与维护运行配置

首次部署时，由实际运行 `run_rl_controller` 的用户安装标准模板：

```bash
export ROBOT_VERSION=17
bash scripts/install_teleop_bindings.sh
```

升级时可明确选择保留现场配置或恢复仓库标准配置：

```bash
# 保留现有运行配置
bash scripts/install_teleop_bindings.sh --keep

# 使用标准模板覆盖，覆盖前自动更新唯一的上一版备份
bash scripts/install_teleop_bindings.sh --overwrite
```

桌面端保存 ROBAN 手柄绑定时，会写回同一份
`~/.config/lejuconfig/teleop_bindings.yaml`，并在覆盖前生成
`teleop_bindings.yaml.bak`（仅保留上一版）。相关动作和音乐资源位于同目录下的
`action_files/` 与 `music/`，这些目录应归运行用户所有。

显式传入 `--teleop-config <path>` 时，该路径优先于上述默认运行配置；真机现场
通常不应将它指向仓库标准模板。

## 摇杆

| 按键 | 功能 |
|------|------|
| 左摇杆 上/下 | 前进/后退 |
| 左摇杆 左/右 | 左移/右移（当前禁用） |
| 右摇杆 左/右 | 左转/右转 |

## 面键

| 按键 | 功能 |
|------|------|
| A | 站立 |
| B | 无绑定（预留） |
| X | 切换 AMP 控制器 |
| Y | 无绑定（预留） |
| START | 启动/恢复 |
| BACK + START | 立即退出 |

## 组合键

| 按键 | 功能 |
|------|------|
| LB + A | 屏蔽/恢复摇杆和方向键 |
| LB + B | 切换手臂模式（auto ↔ keep_pose） |
| LB + Y | 恢复初始站立 |
| RB + A | 切换舞蹈控制器（爱情鸟） |
| RB + B | 切换舞蹈控制器（lonelydance） |
| RB + X | 播放当前舞蹈动作 + 配乐 |

## LT + 面键（松开触发）

| 按键 | 功能 |
|------|------|
| LT + A | 抱拳 |
| LT + B | 右手打招呼 |
| LT + X | 右转腰打招呼 |
| LT + Y | 右手握手 |

## RT + 面键（松开触发）

| 按键 | 功能 |
|------|------|
| RT + A | 碰拳 |
| RT + B | 飞吻 |
| RT + X | 欢呼 |
| RT + Y | 抬右手致意 |
