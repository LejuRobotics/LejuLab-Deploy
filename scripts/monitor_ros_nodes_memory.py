#!/usr/bin/env python3
"""
@file monitor_ros_nodes_memory.py
@brief 监控 ROS launch 文件启动的节点内存使用情况
@details 支持监控 load_mujoco_sim.launch 和 load_real.launch，每分钟采样一次，记录 RSS 内存总和

用法:
    # 监控 MuJoCo 模拟 (默认)
    python3 monitor_ros_nodes_memory.py

    # 监控真实机器人
    python3 monitor_ros_nodes_memory.py --launch real

    # 后台运行
    python3 monitor_ros_nodes_memory.py &

    # 清空之前的数据
    python3 monitor_ros_nodes_memory.py --clear
"""

import os
import sys
import time
import csv
import signal
import argparse
from datetime import datetime
from typing import List, Dict, Optional

try:
    import psutil
except ImportError:
    print("错误: 请先安装 psutil: pip3 install psutil")
    sys.exit(1)


# MuJoCo 模拟进程
MUJOCO_PROCESSES = [
    "leju-mujoco-sim",
    "leju-joystick",
    "run_rl_controller",
    "lejusdk_recorder",
]

# 真实机器人进程
REAL_ROBOT_PROCESSES = [
    "leju-hardware",
    "leju-joystick",
    "run_rl_controller",
    "lejusdk_recorder",
]

# 默认输出文件
DEFAULT_OUTPUT = os.path.expanduser("~/ros_nodes_memory_log.csv")


class MemoryMonitor:
    """内存监控器"""

    def __init__(self, output_file: str, interval: int = 60, launch_type: str = "mujoco"):
        """
        初始化监控器

        @param output_file 输出 CSV 文件路径
        @param interval 采样间隔（秒），默认 60 秒
        @param launch_type launch 文件类型: "mujoco" 或 "real"
        """
        self.output_file = output_file
        self.interval = interval
        self.launch_type = launch_type
        self.running = False
        self.start_time: Optional[datetime] = None

        # 根据 launch 类型选择进程列表
        if launch_type == "real":
            self.target_processes = REAL_ROBOT_PROCESSES
            self.launch_name = "load_real.launch"
        elif launch_type == "all":
            self.target_processes = list(set(MUJOCO_PROCESSES + REAL_ROBOT_PROCESSES))
            self.launch_name = "所有 launch 文件"
        else:
            self.target_processes = MUJOCO_PROCESSES
            self.launch_name = "load_mujoco_sim.launch"

        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """处理终止信号"""
        print("\n接收到终止信号，正在停止监控...")
        self.running = False

    def find_target_processes(self) -> List[psutil.Process]:
        """
        查找目标进程

        @return 匹配的进程列表
        """
        processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline'] or [])
                name = proc.info['name'] or ''

                for target in self.target_processes:
                    if target in cmdline or target in name:
                        processes.append(proc)
                        break
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        return processes

    def get_memory_info(self, processes: List[psutil.Process]) -> Dict:
        """
        获取内存信息

        @param processes 进程列表
        @return 内存统计字典
        """
        total_rss = 0  # 实际使用物理内存 (KB)
        total_vms = 0  # 虚拟内存 (KB)
        process_details = []

        for proc in processes:
            try:
                mem_info = proc.memory_info()
                total_rss += mem_info.rss
                total_vms += mem_info.vms

                # 获取进程名
                name = proc.name()
                cmdline = ' '.join(proc.cmdline()[:3])  # 只取前3个参数

                process_details.append({
                    'pid': proc.pid,
                    'name': name,
                    'cmdline': cmdline[:50] + '...' if len(cmdline) > 50 else cmdline,
                    'rss_mb': mem_info.rss / 1024 / 1024,
                    'vms_mb': mem_info.vms / 1024 / 1024,
                })
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        return {
            'timestamp': datetime.now().isoformat(),
            'total_rss_mb': total_rss / 1024 / 1024,
            'total_vms_mb': total_vms / 1024 / 1024,
            'process_count': len(processes),
            'active_processes': len(process_details),
            'process_details': process_details,
        }

    def init_csv(self):
        """初始化 CSV 文件"""
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'total_rss_mb',
                'total_vms_mb',
                'process_count',
                'active_processes',
                'elapsed_seconds',
            ])

    def append_to_csv(self, data: Dict):
        """追加数据到 CSV"""
        elapsed = 0
        if self.start_time:
            elapsed = (datetime.now() - self.start_time).total_seconds()

        with open(self.output_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                data['timestamp'],
                f"{data['total_rss_mb']:.2f}",
                f"{data['total_vms_mb']:.2f}",
                data['process_count'],
                data['active_processes'],
                f"{elapsed:.0f}",
            ])

    def print_summary(self, data: Dict):
        """打印内存摘要"""
        elapsed = 0
        if self.start_time:
            elapsed = (datetime.now() - self.start_time).total_seconds()

        print(f"\n[{data['timestamp']}] 运行时间: {elapsed/60:.1f} 分钟")
        print(f"  总 RSS 内存: {data['total_rss_mb']:.2f} MB")
        print(f"  总 VMS 内存: {data['total_vms_mb']:.2f} MB")
        print(f"  检测到进程: {data['process_count']} 个")
        print(f"  活跃进程: {data['active_processes']} 个")

        if data['process_details']:
            print("  进程详情:")
            for proc in data['process_details']:
                print(f"    - {proc['name']}({proc['pid']}): RSS={proc['rss_mb']:.1f}MB")

    def run(self):
        """运行监控循环"""
        print(f"=" * 60)
        print("ROS 节点内存监控器")
        print(f"=" * 60)
        print(f"监控目标: {self.launch_name}")
        print(f"目标进程: {', '.join(self.target_processes)}")
        print(f"采样间隔: {self.interval} 秒")
        print(f"输出文件: {self.output_file}")
        print(f"=" * 60)
        print("按 Ctrl+C 停止监控\n")

        # 初始化 CSV
        self.init_csv()
        self.start_time = datetime.now()
        self.running = True

        while self.running:
            # 查找进程
            processes = self.find_target_processes()

            if not processes:
                print(f"[{datetime.now().isoformat()}] 未检测到目标进程，等待中...")
                time.sleep(self.interval)
                continue

            # 获取内存信息
            mem_data = self.get_memory_info(processes)

            # 记录和显示
            self.append_to_csv(mem_data)
            self.print_summary(mem_data)

            # 等待下一次采样
            for _ in range(self.interval):
                if not self.running:
                    break
                time.sleep(1)

        # 结束总结
        elapsed = (datetime.now() - self.start_time).total_seconds()
        print(f"\n监控结束！")
        print(f"总运行时间: {elapsed/60:.1f} 分钟")
        print(f"数据已保存到: {self.output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='监控 ROS launch 文件启动的节点内存使用情况',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 %(prog)s                      # 监控 MuJoCo 模拟（默认）
  python3 %(prog)s --launch real        # 监控真实机器人
  python3 %(prog)s --launch all         # 监控所有已知进程
  python3 %(prog)s -o /tmp/mem.csv      # 自定义输出文件
  python3 %(prog)s -i 30                # 30秒采样间隔
  python3 %(prog)s --clear              # 清空之前的数据
        """
    )
    parser.add_argument(
        '-o', '--output',
        default=DEFAULT_OUTPUT,
        help=f'输出 CSV 文件路径 (默认: {DEFAULT_OUTPUT})'
    )
    parser.add_argument(
        '-i', '--interval',
        type=int,
        default=60,
        help='采样间隔（秒），默认 60 秒'
    )
    parser.add_argument(
        '--launch',
        choices=['mujoco', 'real', 'all'],
        default='mujoco',
        help='选择要监控的 launch 文件类型 (默认: mujoco)'
    )
    parser.add_argument(
        '--clear',
        action='store_true',
        help='清空之前的数据文件'
    )

    args = parser.parse_args()

    # 清空数据文件
    if args.clear and os.path.exists(args.output):
        os.remove(args.output)
        print(f"已清空数据文件: {args.output}")

    # 创建并运行监控器
    monitor = MemoryMonitor(
        output_file=args.output,
        interval=args.interval,
        launch_type=args.launch
    )
    monitor.run()


if __name__ == '__main__':
    main()
