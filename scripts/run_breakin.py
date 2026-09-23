#!/usr/bin/env python3
"""CANFD 全身电机磨线便捷启动脚本。

封装 canfd_breakin: 自动定位二进制、按版本选配置、从部署 canbus 配置读取总线并
检查/拉起 CAN, 跑安全清单确认后启动磨线。-- 之后的参数透传给 canfd_breakin。

用法:
  sudo python3 scripts/run_breakin.py [--version 17|14] [--setup-can] [-- <canfd_breakin 参数...>]

示例:
  python3 scripts/run_breakin.py --version 17 -- --dry-run
  sudo python3 scripts/run_breakin.py --version 17 -- --duration 30
  sudo python3 scripts/run_breakin.py --version 17 --setup-can -- --rounds 100
  sudo python3 scripts/run_breakin.py --version 14 -- --leg-only

v17 默认从 ~/.config/lejuconfig/canbus_device_cofig.yaml 读腿/腰电机 id 与 kp/kd
(可用 --device-config 覆盖, --no-device-config 关闭)。完整参数见 docs/canfd-breakin-guide.md。
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent

DEFAULT_DEVICE_CONFIG = Path.home() / ".config/lejuconfig/canbus_device_cofig.yaml"

REL_BIN = ("src/leju-hardware/lejusdk-hw/src/hardware_plant/lib/"
           "motorevo_controller/examples/canfd_breakin")

VERSION_CONFIG = {
    "17": "config/canfd_breakin/leg_breakin_roban2_v17.yaml",
    "14": "config/canfd_breakin/leg_breakin_roban2_v14.yaml",
}
# 哪些版本默认启用 device_config (从部署配置读拓扑)
VERSION_USES_DEVICE_CONFIG = {"17"}


# ---------- 彩色输出 ----------
def _c(code: str, msg: str) -> str:
    return f"\033[{code}m{msg}\033[0m"


def title(m):   print(_c("1;34", f"=== {m} ==="))
def info(m):    print(_c("0;36", m))
def ok(m):      print(_c("0;32", m))
def warn(m):    print(_c("1;33", m))
def err(m):     print(_c("0;31", m), file=sys.stderr)


# ---------- 定位二进制 ----------
def find_binary() -> Path | None:
    for d in ("build_cmake", "build", "build_full"):
        p = PROJECT_DIR / d / REL_BIN
        if p.is_file() and os.access(p, os.X_OK):
            return p
    for p in PROJECT_DIR.rglob("canfd_breakin"):
        if "build_docker" in p.parts:
            continue
        if p.is_file() and os.access(p, os.X_OK):
            return p
    return None


# ---------- 解析部署 canbus 配置: canbus_interfaces -> 总线列表 ----------
def parse_device_buses(path: Path) -> list[dict]:
    """返回 [{name, nbitrate, dbitrate, fd, arm}]。
    fd: dbitrate>nbitrate (CAN FD); arm: 非 canfd_broadcast 协议 (单帧, bcan2/3)。"""
    buses: list[dict] = []
    in_if = False
    cur: dict | None = None
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        key = line.strip()
        if indent == 0:
            in_if = key.startswith("canbus_interfaces:")
            cur = None
            continue
        if not in_if:
            continue
        if indent == 2 and key.endswith(":"):
            cur = {"name": key[:-1], "nbitrate": 1_000_000, "dbitrate": 1_000_000,
                   "fd": False, "arm": True}
            buses.append(cur)
        elif cur is not None:
            m = re.match(r"(\w+):\s*(.+)", key)
            if not m:
                continue
            k, v = m.group(1), m.group(2).strip().strip('"')
            if k == "nbitrate":
                cur["nbitrate"] = int(v)
            elif k == "dbitrate":
                cur["dbitrate"] = int(v)
            elif k == "name":
                cur["name"] = v
            elif k == "protocol" and v == "canfd_broadcast":
                cur["arm"] = False
    for b in buses:
        b["fd"] = b["dbitrate"] > b["nbitrate"]
    return buses


# 旧式 (v14): 从 breakin yaml 的 canbus.left_leg/right_leg 读接口名
def parse_yaml_leg_buses(config: Path) -> list[dict]:
    text = config.read_text(encoding="utf-8")
    ifaces = {"left_leg": "bcan0", "right_leg": "bcan1"}
    sub, in_canbus = None, False
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        key = line.strip()
        if indent == 0:
            in_canbus = key.startswith("canbus:")
            sub = None
            continue
        if not in_canbus:
            continue
        if key.rstrip(":") in ("left_leg", "right_leg", "waist"):
            sub = key.rstrip(":")
            continue
        m = re.match(r'interface:\s*"?([^"\s]+)"?', key)
        if m and sub in ifaces:
            ifaces[sub] = m.group(1)
    return [{"name": ifaces["left_leg"], "nbitrate": 1_000_000, "dbitrate": 5_000_000, "fd": True, "arm": False},
            {"name": ifaces["right_leg"], "nbitrate": 1_000_000, "dbitrate": 5_000_000, "fd": True, "arm": False}]


# ---------- CAN 总线 ----------
def run(cmd: list[str]) -> int:
    return subprocess.run(cmd, capture_output=True, text=True).returncode


def setup_can(buses: list[dict]) -> bool:
    for b in buses:
        kind = "CANFD %d/%d" % (b["nbitrate"], b["dbitrate"]) if b["fd"] else "CAN %d" % b["nbitrate"]
        info(f"拉起 {b['name']} ({kind})...")
        run(["ip", "link", "set", b["name"], "down"])
        # restart-ms 100: bus-off 后自动恢复(与 launch_real 用的总线配置一致);
        # 缺省 0 会导致一次 bus-off 后接口永久 DOWN
        cmd = ["ip", "link", "set", b["name"], "type", "can",
               "bitrate", str(b["nbitrate"]), "restart-ms", "100"]
        if b["fd"]:
            cmd += ["dbitrate", str(b["dbitrate"]), "fd", "on"]
        if run(cmd) != 0:
            err(f"  配置 {b['name']} 比特率失败 (接口不存在或非 CAN 设备?)")
            return False
        if run(["ip", "link", "set", b["name"], "up"]) != 0:
            err(f"  {b['name']} up 失败")
            return False
        ok(f"  ✓ {b['name']} 已就绪")
    return True


def check_can(buses: list[dict]) -> bool:
    all_ok = True
    for b in buses:
        r = subprocess.run(["ip", "-details", "link", "show", b["name"]],
                           capture_output=True, text=True)
        if r.returncode != 0:
            warn(f"  ✗ {b['name']} 不存在"); all_ok = False
        elif "state UP" not in r.stdout and "UP," not in r.stdout and "<UP" not in r.stdout:
            warn(f"  ✗ {b['name']} 未 UP"); all_ok = False
        else:
            ok(f"  ✓ {b['name']} UP")
    return all_ok


def exec_binary(binary: Path, cmd: list[str]):
    """替换为目标进程前先刷新 stdout, 否则 wrapper 已 print 的内容会因缓冲丢失。"""
    sys.stdout.flush()
    sys.stderr.flush()
    os.execv(str(binary), cmd)


def confirm(prompt: str) -> bool:
    try:
        return input(prompt).strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def main() -> int:
    ap = argparse.ArgumentParser(
        description="CANFD 全身电机磨线便捷启动",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="未识别的参数原样透传给 canfd_breakin, 如: --duration 1800 / --dry-run / --no-arm / --leg-only (无需 -- 分隔)")
    ap.add_argument("--version", default="17", choices=sorted(VERSION_CONFIG),
                    help="机器人版本对应的磨线配置 (默认 17)")
    ap.add_argument("--config", help="直接指定 YAML 配置 (覆盖 --version)")
    ap.add_argument("--binary", help="直接指定 canfd_breakin 二进制路径")
    ap.add_argument("--device-config", help="部署 canbus 配置路径 (默认 ~/.config/lejuconfig/canbus_device_cofig.yaml)")
    ap.add_argument("--no-device-config", action="store_true", help="不使用部署配置, 用 yaml 内手写拓扑")
    ap.add_argument("--setup-can", action="store_true",
                    help="启动前用 ip link 拉起所需 CAN 总线")
    # canfd_breakin 的参数(如 --no-arm/--duration)直接写即可, 无需 -- 分隔;
    # 兼容旧写法 -- (会被过滤掉)
    args, extras = ap.parse_known_args()
    pass_args = [a for a in extras if a != "--"]

    config = Path(args.config) if args.config else PROJECT_DIR / VERSION_CONFIG[args.version]
    if not config.is_file():
        err(f"配置文件不存在: {config}")
        return 1

    if args.binary:
        binary = Path(args.binary)
        if not (binary.is_file() and os.access(binary, os.X_OK)):
            err(f"指定的二进制不可执行: {binary}")
            return 1
    else:
        binary = find_binary()
    if binary is None:
        err("未找到 canfd_breakin 二进制, 请先编译:")
        info("  cmake --build build_cmake --target canfd_breakin -j$(nproc)")
        return 1

    is_dry_run = "--dry-run" in pass_args
    no_arm = ("--no-arm" in pass_args) or ("--leg-only" in pass_args)
    arm_only = "--arm-only" in pass_args

    # ---- 解析 device_config ----
    device_config: Path | None = None
    if not args.no_device_config:
        if args.device_config:
            device_config = Path(args.device_config).expanduser()
        elif args.version in VERSION_USES_DEVICE_CONFIG:
            device_config = DEFAULT_DEVICE_CONFIG

    # ---- 确定要操作的 CAN 总线 ----
    if device_config and device_config.is_file():
        buses = parse_device_buses(device_config)
    elif device_config and not device_config.is_file():
        warn(f"device_config 不存在: {device_config} (将回退到 yaml 内拓扑)")
        device_config = None
        buses = parse_yaml_leg_buses(config)
    else:
        buses = parse_yaml_leg_buses(config)
    if arm_only:
        buses = [b for b in buses if b["arm"]]
    elif no_arm:
        buses = [b for b in buses if not b["arm"]]

    cmd = [str(binary), "--config", str(config)]
    if device_config:
        cmd += ["--device-config", str(device_config)]
    cmd += pass_args

    title("🔧 CANFD 全身电机磨线")
    info(f"版本:     v{args.version}")
    info(f"配置:     {config}")
    info(f"二进制:   {binary}")
    info(f"设备配置: {device_config if device_config else '(用 yaml 内手写拓扑)'}")
    info(f"CAN 总线: {', '.join(b['name'] + ('(FD)' if b['fd'] else '') for b in buses)}")
    print()

    if is_dry_run:
        exec_binary(binary, cmd)

    if os.geteuid() != 0:
        err(f"需要 root 权限 (CAN 访问). 请用: sudo python3 {sys.argv[0]} ...")
        return 1

    title("CAN 总线检查")
    if args.setup_can:
        if not setup_can(buses):
            err("CAN 拉起失败, 请检查接线或手动配置")
            return 1
    elif not check_can(buses):
        warn("部分 CAN 接口未就绪。可加 --setup-can 自动拉起, 或先运行 scripts/canbus_config.sh")
        if not confirm("仍要继续吗? [y/N] "):
            info("已取消")
            return 0
    print()

    title("⚠️  运行前安全清单")
    warn("  1. 机器人已吊起, 各关节处于零位")
    warn("  2. 吊架万向环已锁住, 防止旋转")
    warn("  3. 运动范围内无障碍物 / 无人")
    warn("  4. 已确认配置正确 (建议先 --dry-run, 再 --duration 30 短跑)")
    print()
    if not confirm("以上都已确认, 开始磨线? [y/N] "):
        info("已取消")
        return 0
    print()

    ok("启动磨线 (Ctrl+C 安全停止)...")
    exec_binary(binary, cmd)


if __name__ == "__main__":
    sys.exit(main())
