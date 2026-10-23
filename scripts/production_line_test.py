#!/usr/bin/env python3
"""原产线方案的 REVO2 抓握入口：Python 交互 → C++ 测试程序 → CAN 驱动。

仅保留灵巧手，默认10分钟；不含夹爪、DDS、参数读写或磨线。
先编译 revo2_grip_endurance_test，例如在已配置的 catkin 工程中：
  cmake --build build/leju-hardware --target revo2_grip_endurance_test -- -j2
纯 CMake 工程使用实际构建目录（例如 build_cmake）。
  sudo python3 scripts/production_line_test.py
  sudo python3 scripts/production_line_test.py --duration 30
  python3 scripts/production_line_test.py --dry-run
需暂停占用相同灵巧手/CAN设备的 hardware、遥操作等进程，无需启动DDS。
程序直接初始化CAN并控制末端；确认设备YAML与本机接线一致，准备硬件急停。
"""
import argparse
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
BINARY = "revo2_grip_endurance_test"
BUILD_REL = "src/leju-hardware/lejusdk-hw/src/hardware_plant/lib/dexhand_sdk"
DEFAULT_CONFIG = REPO_ROOT / "src/leju-hardware/config/canbus_device_cofig_revohand.yaml"
MAX_SECONDS = 2147483647


def positive_seconds(raw):
    try:
        value = int(raw)
    except ValueError:
        raise argparse.ArgumentTypeError("时长必须是正整数秒") from None
    if not 0 < value <= MAX_SECONDS:
        raise argparse.ArgumentTypeError("时长须在1到2147483647秒之间")
    return value


def find_tool(binary=None, build_dir=None):
    # 沿用原脚本的构建树 → installed/bin → install/bin → PATH 顺序。
    if binary is not None:
        candidates = [Path(binary).expanduser().resolve()]
    else:
        roots = ([Path(build_dir).expanduser().resolve()] if build_dir else
                 [REPO_ROOT / "build", REPO_ROOT / "build_cmake"])
        candidates = []
        for root in roots:
            candidates.extend([
                root / BUILD_REL / BINARY,
                root / "leju-hardware/lejusdk-hw/src/hardware_plant/lib/dexhand_sdk" / BINARY,
                root / "bin" / BINARY,
            ])
        candidates.extend(REPO_ROOT / prefix / BINARY for prefix in (
            "installed/bin", "install/bin", "devel/lib/leju-hardware", "devel/bin"))
        path_binary = shutil.which(BINARY)
        if path_binary:
            candidates.append(Path(path_binary))
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    raise FileNotFoundError("未找到可执行的 %s，请先编译或用 --binary 指定路径" % BINARY)


def prompt_duration():
    while True:
        raw = input("抓取时长(分钟) [回车用默认10]: ").strip()
        if not raw:
            return 600
        try:
            minutes = int(raw)
            if 0 < minutes <= MAX_SECONDS // 60:
                return minutes * 60
        except ValueError:
            pass
        print("请输入有效的正整数分钟。")


def _signal_child(proc, sig):
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        pass


def run_passthrough(command):
    # 保留原方案的终端直通及20秒收尾等待；显式转发信号避免留下运动子进程。
    starting, stop_requested = True, False
    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True
        if not starting:
            raise KeyboardInterrupt
    previous_int = signal.signal(signal.SIGINT, request_stop)
    previous_term = signal.signal(signal.SIGTERM, request_stop)
    try:
        if stop_requested:
            return 130
        proc = subprocess.Popen(command, start_new_session=True)
        try:
            starting = False
            if stop_requested:
                raise KeyboardInterrupt
            return proc.wait()
        except KeyboardInterrupt:
            print("\n请求停止，等待灵巧手程序收尾……", file=sys.stderr)
            _signal_child(proc, signal.SIGINT)
            try:
                proc.wait(timeout=20)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                print("收尾超时或再次中断，将终止程序；请检查末端状态/急停。", file=sys.stderr)
                _signal_child(proc, signal.SIGTERM)
                try:
                    proc.wait(timeout=5)
                except (subprocess.TimeoutExpired, KeyboardInterrupt):
                    _signal_child(proc, signal.SIGKILL)
                    proc.wait()
            return 130
    finally:
        signal.signal(signal.SIGTERM, previous_term)
        signal.signal(signal.SIGINT, previous_int)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=positive_seconds, help="测试时长（秒）；交互输入为分钟")
    parser.add_argument("--device-config", help="本机设备YAML，默认使用原分支REVO2配置")
    parser.add_argument("--binary", help="显式指定 revo2_grip_endurance_test 二进制")
    parser.add_argument("--build-dir", help="实际构建目录")
    parser.add_argument("--dry-run", action="store_true", help="只检查路径/显示命令，不启动后端")
    args = parser.parse_args(argv)
    try:
        binary = find_tool(args.binary, args.build_dir)
        config = (Path(args.device_config).expanduser().resolve()
                  if args.device_config is not None else DEFAULT_CONFIG)
        if not config.is_file() or not os.access(config, os.R_OK):
            raise FileNotFoundError("设备配置不存在或不可读：%s" % config)
        seconds = args.duration or (600 if args.dry_run else prompt_duration())
        command = [binary, "--duration", str(seconds), "--device-config", str(config)]
        print("灵巧手（REVO2）抓握位置精度耐久测试")
        print("命令: " + shlex.join(command), flush=True)
        if args.dry_run:
            print("路径检查完成，未启动程序、未发送CAN指令。")
            return 0
        print("请停止占用该设备的控制进程，核对配置、清空夹持区域并准备急停。")
        if input("确认开始？输入 yes: ").strip().lower() != "yes":
            print("已取消，未启动测试。")
            return 3
        result = run_passthrough(command)
        if result == 0:
            print("测试通过。")
        elif result == 130:
            print("测试已中断，不视为通过。")
        else:
            print("测试未通过（退出码%d）。" % result, file=sys.stderr)
        return result if result >= 0 else 128 - result
    except (EOFError, KeyboardInterrupt):
        print("\n已取消。", file=sys.stderr)
        return 130
    except OSError as error:
        print("错误：%s" % error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
