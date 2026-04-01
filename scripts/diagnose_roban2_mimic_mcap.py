#!/usr/bin/env python3
"""
Roban2 Mimic MCAP 诊断脚本
用于自动化排查 roban2_mimic RL 控制器在真机运行时的问题

Usage:
    python3 diagnose_roban2_mimic_mcap.py <mcap_file> [--motion-csv <csv_file>] [--config <yaml_file>]

Example:
    python3 diagnose_roban2_mimic_mcap.py ~/桌面/roban42-hpny-0205-error.mcap
    python3 diagnose_roban2_mimic_mcap.py ~/桌面/error.mcap --motion-csv config/14/motion_data/HPNY_dance.csv
"""

import argparse
import struct
import math
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any

# ANSI 颜色代码
class Color:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    RESET = '\033[0m'

    @staticmethod
    def red(s): return f"{Color.RED}{s}{Color.RESET}"
    @staticmethod
    def green(s): return f"{Color.GREEN}{s}{Color.RESET}"
    @staticmethod
    def yellow(s): return f"{Color.YELLOW}{s}{Color.RESET}"
    @staticmethod
    def blue(s): return f"{Color.BLUE}{s}{Color.RESET}"
    @staticmethod
    def cyan(s): return f"{Color.CYAN}{s}{Color.RESET}"
    @staticmethod
    def magenta(s): return f"{Color.MAGENTA}{s}{Color.RESET}"
    @staticmethod
    def bold(s): return f"{Color.BOLD}{s}{Color.RESET}"
    @staticmethod
    def dim(s): return f"{Color.DIM}{s}{Color.RESET}"


@dataclass
class DiagnosticResult:
    """诊断结果"""
    name: str
    status: str  # "OK", "WARNING", "ERROR"
    message: str
    details: Optional[Dict[str, Any]] = None


class MCAPParser:
    """MCAP 文件解析器"""

    # 关节名称映射 (policy index -> name)
    JOINT_NAMES = [
        "waist_yaw",      # 0
        "leg_l1", "leg_l2", "leg_l3", "leg_l4", "leg_l5", "leg_l6",  # 1-6
        "leg_r1", "leg_r2", "leg_r3", "leg_r4", "leg_r5", "leg_r6",  # 7-12
        "zarm_l1", "zarm_l2", "zarm_l3", "zarm_l4",  # 13-16
        "zarm_r1", "zarm_r2", "zarm_r3", "zarm_r4",  # 17-20
    ]

    def __init__(self, mcap_path: str):
        self.mcap_path = mcap_path
        self.data = {}

    def parse(self):
        """解析 MCAP 文件"""
        try:
            from mcap.reader import make_reader
        except ImportError:
            print(Color.red("错误: 请安装 mcap 库: pip install mcap"))
            sys.exit(1)

        with open(self.mcap_path, "rb") as f:
            reader = make_reader(f)

            # 初始化数据容器
            self.data = {
                'motion_frame': [],
                'obs_joint_pos': [],
                'obs_joint_vel': [],
                'obs_actions': [],
                'obs_base_ang_vel': [],
                'obs_projected_gravity': [],
                'obs_motion_command': [],
                'q_target': [],
                'joint_state': [],
                'joint_cmd': [],
                'imu_state': [],
            }

            for schema, channel, message in reader.iter_messages():
                ts = message.log_time / 1e9
                topic = channel.topic
                data = message.data

                try:
                    if topic == "/mimic/motion_frame":
                        self.data['motion_frame'].append((ts, self._parse_float64(data)))
                    elif topic == "/mimic/obs/joint_pos":
                        self.data['obs_joint_pos'].append((ts, self._parse_float64_array(data)))
                    elif topic == "/mimic/obs/joint_vel":
                        self.data['obs_joint_vel'].append((ts, self._parse_float64_array(data)))
                    elif topic == "/mimic/obs/actions":
                        self.data['obs_actions'].append((ts, self._parse_float64_array(data)))
                    elif topic == "/mimic/obs/base_ang_vel":
                        self.data['obs_base_ang_vel'].append((ts, self._parse_float64_array(data)))
                    elif topic == "/mimic/obs/projected_gravity":
                        self.data['obs_projected_gravity'].append((ts, self._parse_float64_array(data)))
                    elif topic == "/mimic/obs/motion_command":
                        self.data['obs_motion_command'].append((ts, self._parse_float64_array(data)))
                    elif topic == "/mimic/q_target":
                        self.data['q_target'].append((ts, self._parse_float64_array(data)))
                    elif topic == "/rt/joint_state":
                        self.data['joint_state'].append((ts, self._parse_joint_state(data)))
                    elif topic == "/rt/joint_cmd":
                        self.data['joint_cmd'].append((ts, self._parse_joint_cmd(data)))
                    elif topic == "/rt/imu_state":
                        self.data['imu_state'].append((ts, self._parse_imu_state(data)))
                except Exception as e:
                    pass  # 忽略解析错误

        return self.data

    def _parse_float64(self, data: bytes) -> float:
        """解析 Float64 消息"""
        return struct.unpack_from('<d', data, 12)[0]

    def _parse_float64_array(self, data: bytes) -> List[float]:
        """解析 Float64Array 消息"""
        data_len = struct.unpack_from('<I', data, 12)[0]
        return list(struct.unpack_from(f'<{data_len}d', data, 20))

    def _parse_joint_state(self, data: bytes) -> Dict[str, List[float]]:
        """解析 JointState 消息 (带 4 字节 padding)"""
        n = 23  # 关节数
        q = list(struct.unpack_from(f'<{n}d', data, 20))
        v = list(struct.unpack_from(f'<{n}d', data, 212))
        vd = list(struct.unpack_from(f'<{n}d', data, 404))
        tau = list(struct.unpack_from(f'<{n}d', data, 596))
        return {'q': q, 'v': v, 'vd': vd, 'tau': tau}

    def _parse_joint_cmd(self, data: bytes) -> Dict[str, List[float]]:
        """解析 JointCmd 消息"""
        offset = 12
        result = {}
        for field in ['q', 'v', 'tau', 'kp', 'kd']:
            n = struct.unpack_from('<I', data, offset)[0]
            offset += 4
            offset = (offset + 7) & ~7  # 对齐到 8
            if offset + n * 8 <= len(data):
                result[field] = list(struct.unpack_from(f'<{n}d', data, offset))
                offset += n * 8
        return result

    def _parse_imu_state(self, data: bytes) -> Dict[str, List[float]]:
        """解析 ImuData 消息"""
        offset = 12
        gyro = list(struct.unpack_from('<3d', data, offset))
        acc = list(struct.unpack_from('<3d', data, offset + 24))
        free_acc = list(struct.unpack_from('<3d', data, offset + 48))
        quat = list(struct.unpack_from('<4d', data, offset + 72))
        return {'gyro': gyro, 'acc': acc, 'free_acc': free_acc, 'quat': quat}

    def get_frame_at_time(self, ts: float) -> int:
        """获取指定时间的 motion frame"""
        frame = 0
        for f_ts, f in self.data['motion_frame']:
            if f_ts <= ts:
                frame = int(f)
            else:
                break
        return frame


class Diagnostics:
    """诊断器"""

    def __init__(self, parser: MCAPParser):
        self.parser = parser
        self.data = parser.data
        self.results: List[DiagnosticResult] = []

    def run_all(self):
        """运行所有诊断"""
        self.check_data_integrity()
        self.check_motion_timeline()
        self.check_actions_divergence()
        self.check_angular_velocity()
        self.check_joint_positions()
        self.check_imu_data()
        self.check_q_target()
        self.analyze_divergence_cause()
        return self.results

    def check_data_integrity(self):
        """检查数据完整性"""
        required_topics = ['motion_frame', 'obs_joint_pos', 'obs_actions', 'joint_state']
        missing = [t for t in required_topics if not self.data.get(t)]

        if missing:
            self.results.append(DiagnosticResult(
                "数据完整性",
                "ERROR",
                f"缺少关键数据: {', '.join(missing)}"
            ))
        else:
            counts = {k: len(v) for k, v in self.data.items() if v}
            self.results.append(DiagnosticResult(
                "数据完整性",
                "OK",
                f"所有关键数据存在",
                {"消息数量": counts}
            ))

    def check_motion_timeline(self):
        """检查 Motion 时间线"""
        frames = self.data.get('motion_frame', [])
        if not frames:
            self.results.append(DiagnosticResult(
                "Motion 时间线", "ERROR", "无 motion_frame 数据"
            ))
            return

        # 找 motion 开始时间
        start_ts = None
        for ts, f in frames:
            if f > 0:
                start_ts = ts
                break

        last_frame = max(int(f) for _, f in frames)
        duration = frames[-1][0] - frames[0][0]

        status = "WARNING" if last_frame < 100 else "OK"
        self.results.append(DiagnosticResult(
            "Motion 时间线",
            status,
            f"播放了 {last_frame} 帧" + (" (异常终止)" if last_frame < 100 else ""),
            {
                "开始时间": f"{start_ts:.2f}s" if start_ts else "未开始",
                "最后帧数": last_frame,
                "录制时长": f"{duration:.2f}s"
            }
        ))

    def check_actions_divergence(self):
        """检查 Actions 发散"""
        actions_data = self.data.get('obs_actions', [])
        if not actions_data:
            self.results.append(DiagnosticResult(
                "Actions 发散检查", "ERROR", "无 actions 数据"
            ))
            return

        divergence_frames = []
        first_divergence = None
        max_action = 0
        max_action_frame = 0
        max_action_joint = 0

        for ts, actions in actions_data:
            frame = self.parser.get_frame_at_time(ts)
            max_val = max(abs(x) for x in actions)

            if max_val > max_action:
                max_action = max_val
                max_action_frame = frame
                max_action_joint = actions.index(max(actions, key=abs))

            if max_val > 5:
                divergence_frames.append(frame)
                if first_divergence is None:
                    first_divergence = {
                        'frame': frame,
                        'ts': ts,
                        'max_val': max_val,
                        'abnormal_joints': [i for i, x in enumerate(actions) if abs(x) > 5]
                    }

        if first_divergence:
            joint_names = [MCAPParser.JOINT_NAMES[i] if i < len(MCAPParser.JOINT_NAMES) else f"joint_{i}"
                          for i in first_divergence['abnormal_joints']]
            self.results.append(DiagnosticResult(
                "Actions 发散检查",
                "ERROR",
                f"Frame {first_divergence['frame']} 开始发散, max|action|={max_action:.2f}",
                {
                    "首次发散帧": first_divergence['frame'],
                    "最大 action 值": f"{max_action:.4f}",
                    "最大值关节": f"{MCAPParser.JOINT_NAMES[max_action_joint]} (idx={max_action_joint})",
                    "异常关节": joint_names,
                    "发散帧数": len(set(divergence_frames))
                }
            ))
        else:
            self.results.append(DiagnosticResult(
                "Actions 发散检查",
                "OK",
                f"Actions 正常, max|action|={max_action:.2f}"
            ))

    def check_angular_velocity(self):
        """检查角速度"""
        ang_vel_data = self.data.get('obs_base_ang_vel', [])
        if not ang_vel_data:
            self.results.append(DiagnosticResult(
                "角速度检查", "WARNING", "无 base_ang_vel 数据"
            ))
            return

        max_ang_vel = 0
        max_ang_vel_frame = 0
        max_ang_vel_axis = 0
        high_ang_vel_frames = []

        for ts, ang_vel in ang_vel_data:
            frame = self.parser.get_frame_at_time(ts)
            for axis, val in enumerate(ang_vel):
                if abs(val) > max_ang_vel:
                    max_ang_vel = abs(val)
                    max_ang_vel_frame = frame
                    max_ang_vel_axis = axis

            if max(abs(x) for x in ang_vel) > 3:
                high_ang_vel_frames.append(frame)

        axis_names = ['roll', 'pitch', 'yaw']
        status = "ERROR" if max_ang_vel > 5 else ("WARNING" if max_ang_vel > 3 else "OK")

        self.results.append(DiagnosticResult(
            "角速度检查",
            status,
            f"max|ang_vel|={max_ang_vel:.2f} rad/s @ {axis_names[max_ang_vel_axis]}",
            {
                "话题": "/mimic/obs/base_ang_vel",
                "最大角速度": f"{max_ang_vel:.4f} rad/s",
                "最大值轴": axis_names[max_ang_vel_axis],
                "发生帧": max_ang_vel_frame,
                "高角速度帧数": len(set(high_ang_vel_frames))
            }
        ))

    def check_joint_positions(self):
        """检查关节位置偏移 (obs_joint_pos = direction * q - default)"""
        jp_data = self.data.get('obs_joint_pos', [])
        if not jp_data:
            self.results.append(DiagnosticResult(
                "关节位置偏移检查", "WARNING", "无 obs_joint_pos 数据"
            ))
            return

        max_jp = 0
        max_jp_joint = 0
        max_jp_frame = 0
        abnormal_frames = []

        for ts, jp in jp_data:
            frame = self.parser.get_frame_at_time(ts)
            for i, val in enumerate(jp):
                if abs(val) > max_jp:
                    max_jp = abs(val)
                    max_jp_joint = i
                    max_jp_frame = frame

            if max(abs(x) for x in jp) > 2:  # 超过 2 rad 的偏差
                abnormal_frames.append(frame)

        status = "ERROR" if max_jp > 3 else ("WARNING" if max_jp > 2 else "OK")
        joint_name = MCAPParser.JOINT_NAMES[max_jp_joint] if max_jp_joint < len(MCAPParser.JOINT_NAMES) else f"joint_{max_jp_joint}"

        self.results.append(DiagnosticResult(
            "关节位置偏移检查",
            status,
            f"max|obs_joint_pos|={max_jp:.2f} rad @ {joint_name} (相对默认位置)",
            {
                "话题": "/mimic/obs/joint_pos",
                "最大偏移量": f"{max_jp:.4f} rad",
                "最大值关节": f"{joint_name} (idx={max_jp_joint})",
                "发生帧": max_jp_frame,
                "异常帧数": len(set(abnormal_frames))
            }
        ))

    def check_imu_data(self):
        """检查 IMU 数据"""
        imu_data = self.data.get('imu_state', [])
        if not imu_data:
            self.results.append(DiagnosticResult(
                "IMU 数据检查", "WARNING", "无 imu_state 数据"
            ))
            return

        # 检查四元数归一化
        quat_errors = 0
        max_gyro = 0

        for ts, imu in imu_data:
            quat = imu['quat']
            norm = math.sqrt(sum(q**2 for q in quat))
            if abs(norm - 1.0) > 0.01:
                quat_errors += 1

            gyro = imu['gyro']
            max_gyro = max(max_gyro, max(abs(g) for g in gyro))

        status = "ERROR" if quat_errors > 10 else "OK"
        self.results.append(DiagnosticResult(
            "IMU 数据检查",
            status,
            f"四元数归一化错误: {quat_errors}, max|gyro|={max_gyro:.2f} rad/s",
            {
                "四元数错误数": quat_errors,
                "最大陀螺仪读数": f"{max_gyro:.4f} rad/s",
                "总 IMU 消息数": len(imu_data)
            }
        ))

    def check_q_target(self):
        """检查目标关节位置 (q_target = direction * (motion_pos + action * scale))"""
        qt_data = self.data.get('q_target', [])
        if not qt_data:
            self.results.append(DiagnosticResult(
                "目标绝对位置检查", "WARNING", "无 q_target 数据"
            ))
            return

        max_qt = 0
        max_qt_joint = 0
        max_qt_frame = 0

        for ts, qt in qt_data:
            frame = self.parser.get_frame_at_time(ts)
            for i, val in enumerate(qt):
                if abs(val) > max_qt:
                    max_qt = abs(val)
                    max_qt_joint = i
                    max_qt_frame = frame

        status = "ERROR" if max_qt > 3 else ("WARNING" if max_qt > 2 else "OK")
        joint_name = MCAPParser.JOINT_NAMES[max_qt_joint] if max_qt_joint < len(MCAPParser.JOINT_NAMES) else f"joint_{max_qt_joint}"

        self.results.append(DiagnosticResult(
            "目标绝对位置检查",
            status,
            f"max|q_target|={max_qt:.2f} rad @ {joint_name} (绝对角度)",
            {
                "话题": "/mimic/q_target",
                "最大目标位置": f"{max_qt:.4f} rad",
                "最大值关节": f"{joint_name} (idx={max_qt_joint})",
                "发生帧": max_qt_frame
            }
        ))

    def analyze_divergence_cause(self):
        """分析发散原因"""
        # 找到发散开始的帧
        actions_data = self.data.get('obs_actions', [])
        ang_vel_data = self.data.get('obs_base_ang_vel', [])

        if not actions_data:
            return

        # 找第一个发散帧
        divergence_frame = None
        for ts, actions in actions_data:
            frame = self.parser.get_frame_at_time(ts)
            if max(abs(x) for x in actions) > 5:
                divergence_frame = frame
                break

        if divergence_frame is None:
            return

        # 分析发散前后的数据变化
        causes = []

        # 检查角速度是否在发散前就异常
        for ts, ang_vel in ang_vel_data:
            frame = self.parser.get_frame_at_time(ts)
            if frame < divergence_frame and max(abs(x) for x in ang_vel) > 3:
                causes.append("发散前角速度已偏高")
                break

        # 检查是否是特定关节问题
        abnormal_joints = set()
        for ts, actions in actions_data:
            frame = self.parser.get_frame_at_time(ts)
            if frame >= divergence_frame:
                for i, x in enumerate(actions):
                    if abs(x) > 5:
                        abnormal_joints.add(i)

        if len(abnormal_joints) <= 3:
            joint_names = [MCAPParser.JOINT_NAMES[i] if i < len(MCAPParser.JOINT_NAMES) else f"joint_{i}"
                          for i in abnormal_joints]
            causes.append(f"问题集中在特定关节: {', '.join(joint_names)}")

        # 检查是否是右腿问题
        right_leg_joints = {7, 8, 9, 10, 11, 12}
        if abnormal_joints & right_leg_joints:
            causes.append("右腿关节异常")

        left_leg_joints = {1, 2, 3, 4, 5, 6}
        if abnormal_joints & left_leg_joints:
            causes.append("左腿关节异常")

        if not causes:
            causes.append("需要进一步分析 motion 参考轨迹")

        self.results.append(DiagnosticResult(
            "发散原因分析",
            "WARNING",
            f"发散始于 Frame {divergence_frame}",
            {
                "可能原因": causes,
                "异常关节索引": list(abnormal_joints)
            }
        ))


def print_report(results: List[DiagnosticResult], parser: MCAPParser):
    """打印诊断报告"""
    print(f"\n{Color.bold(Color.cyan('='*70))}")
    print(f"{Color.bold(Color.cyan('  MCAP 策略发散诊断报告'))}")
    print(f"{Color.bold(Color.cyan('='*70))}\n")

    print(f"{Color.bold('📁 文件')}: {parser.mcap_path}\n")

    # 统计
    ok_count = sum(1 for r in results if r.status == "OK")
    warn_count = sum(1 for r in results if r.status == "WARNING")
    error_count = sum(1 for r in results if r.status == "ERROR")

    print(f"{Color.bold('📊 诊断摘要')}: ", end="")
    print(f"{Color.green(f'{ok_count} OK')} | ", end="")
    print(f"{Color.yellow(f'{warn_count} WARNING')} | ", end="")
    print(f"{Color.red(f'{error_count} ERROR')}\n")

    print(f"{Color.bold('─'*70)}\n")

    # 详细结果
    for result in results:
        if result.status == "OK":
            status_str = Color.green("✓ OK")
        elif result.status == "WARNING":
            status_str = Color.yellow("⚠ WARNING")
        else:
            status_str = Color.red("✗ ERROR")

        print(f"{Color.bold(result.name)}")
        print(f"  状态: {status_str}")
        print(f"  信息: {result.message}")

        if result.details:
            for key, value in result.details.items():
                if isinstance(value, list):
                    value_str = ', '.join(str(v) for v in value[:5])
                    if len(value) > 5:
                        value_str += f" ... (+{len(value)-5})"
                elif isinstance(value, dict):
                    value_str = str(value)
                else:
                    value_str = str(value)
                print(f"    • {key}: {value_str}")
        print()

    # 结论和建议
    print(f"{Color.bold(Color.cyan('='*70))}")
    print(f"{Color.bold(Color.cyan('  结论与建议'))}")
    print(f"{Color.bold(Color.cyan('='*70))}\n")

    if error_count > 0:
        print(f"{Color.red('发现问题:')}")
        for r in results:
            if r.status == "ERROR":
                print(f"  {Color.red('•')} {r.name}: {r.message}")
        print()

        print(f"{Color.yellow('建议排查步骤:')}")
        print(f"  1. 检查发散帧附近的 motion 参考轨迹是否有剧烈变化")
        print(f"  2. 对比仿真和真机的观测数据差异")
        print(f"  3. 检查异常关节的硬件/标定状态")
        print(f"  4. 考虑添加 action 限幅或更强的正则化")
    else:
        print(f"{Color.green('未发现严重问题')}")

    print()


def analyze_motion_csv(csv_path: str, divergence_frame: int):
    """分析 Motion CSV 文件"""
    print(f"\n{Color.bold(Color.cyan('='*70))}")
    print(f"{Color.bold(Color.cyan('  Motion CSV 分析'))}")
    print(f"{Color.bold(Color.cyan('='*70))}\n")

    try:
        import csv
        with open(csv_path, 'r') as f:
            # 自动检测分隔符
            sample = f.read(2048)
            f.seek(0)
            if '\t' in sample:
                delimiter = '\t'
            elif ',' in sample:
                delimiter = ','
            else:
                delimiter = ' '
            reader = csv.reader(f, delimiter=delimiter)
            rows = list(reader)

        if len(rows) < divergence_frame + 5:
            print(f"{Color.yellow('CSV 行数不足')}")
            return

        # 分析发散帧附近的数据变化
        print(f"{Color.bold(f'发散帧附近的数据 (Frame {divergence_frame-5} ~ {divergence_frame+5}):')}\n")

        header = rows[0] if rows else []
        print(f"  列数: {len(header)}")

        # CSV 列映射说明
        # 格式: body_pos(3) + body_quat(4) + joint_pos(N) + joint_vel(N)
        # 对于 21 关节: 3 + 4 + 21 + 21 = 49 列
        n_cols = len(rows[1]) if len(rows) > 1 else 0
        n_joints = (n_cols - 7) // 2 if n_cols > 7 else 0

        print(f"  总列数: {n_cols} (body_pos:3 + body_quat:4 + joint_pos:{n_joints} + joint_vel:{n_joints})")

        # 列索引映射
        def get_col_name(col_idx):
            if col_idx < 3:
                return f"body_pos[{col_idx}]"
            elif col_idx < 7:
                return f"body_quat[{col_idx-3}]"
            elif col_idx < 7 + n_joints:
                joint_idx = col_idx - 7
                joint_name = MCAPParser.JOINT_NAMES[joint_idx] if joint_idx < len(MCAPParser.JOINT_NAMES) else f"joint_{joint_idx}"
                return f"joint_pos[{joint_idx}]({joint_name})"
            else:
                joint_idx = col_idx - 7 - n_joints
                joint_name = MCAPParser.JOINT_NAMES[joint_idx] if joint_idx < len(MCAPParser.JOINT_NAMES) else f"joint_{joint_idx}"
                return f"joint_vel[{joint_idx}]({joint_name})"

        # 计算帧间差异
        print(f"\n{Color.bold('帧间最大变化:')}")
        print(f"  {'Frame':<12} {'max_diff':<12} {'列':<10} {'字段'}")
        print(f"  {'-'*60}")

        for i in range(max(1, divergence_frame-5), min(len(rows)-1, divergence_frame+5)):
            try:
                prev_row = [float(x) for x in rows[i]]
                curr_row = [float(x) for x in rows[i+1]]
            except (ValueError, IndexError):
                continue

            max_diff = 0
            max_diff_col = 0
            for j, (p, c) in enumerate(zip(prev_row, curr_row)):
                diff = abs(c - p)
                if diff > max_diff:
                    max_diff = diff
                    max_diff_col = j

            frame = i
            color = Color.red if max_diff > 0.5 else (Color.yellow if max_diff > 0.1 else Color.green)
            col_name = get_col_name(max_diff_col)
            print(f"  {frame} -> {frame+1:<6} {color(f'{max_diff:.4f}'):<20} {max_diff_col:<10} {col_name}")

        # 分析发散帧的具体数据
        print(f"\n{Color.bold(f'Frame {divergence_frame} 的 motion 数据:')}")
        if divergence_frame < len(rows):
            try:
                row = [float(x) for x in rows[divergence_frame]]
                print(f"  body_pos: [{row[0]:.4f}, {row[1]:.4f}, {row[2]:.4f}]")
                print(f"  body_quat: [{row[3]:.4f}, {row[4]:.4f}, {row[5]:.4f}, {row[6]:.4f}]")

                # 显示异常关节的参考位置
                print(f"\n  {Color.bold('关键关节参考位置:')}")
                key_joints = [0, 7, 8, 10]  # waist, leg_r1, leg_r2, leg_r4
                for j in key_joints:
                    if 7 + j < len(row):
                        pos = row[7 + j]
                        vel = row[7 + n_joints + j] if 7 + n_joints + j < len(row) else 0
                        joint_name = MCAPParser.JOINT_NAMES[j] if j < len(MCAPParser.JOINT_NAMES) else f"joint_{j}"
                        color = Color.yellow if abs(pos) > 1 else Color.green
                        print(f"    {joint_name:<12}: pos={color(f'{pos:8.4f}')} rad, vel={vel:8.4f} rad/s")
            except (ValueError, IndexError) as e:
                print(f"  {Color.yellow(f'解析失败: {e}')}")

    except Exception as e:
        print(f"{Color.red(f'分析 CSV 失败: {e}')}")


def main():
    parser = argparse.ArgumentParser(description='MCAP 策略发散诊断工具')
    parser.add_argument('mcap_file', help='MCAP 文件路径')
    parser.add_argument('--motion-csv', help='Motion CSV 文件路径')
    parser.add_argument('--config', help='控制器配置文件路径')
    parser.add_argument('--verbose', '-v', action='store_true', help='详细输出')

    args = parser.parse_args()

    # 检查文件存在
    if not Path(args.mcap_file).exists():
        print(Color.red(f"错误: 文件不存在: {args.mcap_file}"))
        sys.exit(1)

    # 解析 MCAP
    print(f"\n{Color.cyan('正在解析 MCAP 文件...')}")
    mcap_parser = MCAPParser(args.mcap_file)
    mcap_parser.parse()

    # 运行诊断
    print(f"{Color.cyan('正在运行诊断...')}")
    diagnostics = Diagnostics(mcap_parser)
    results = diagnostics.run_all()

    # 打印报告
    print_report(results, mcap_parser)

    # 如果提供了 motion CSV，分析它
    if args.motion_csv:
        # 找到异常帧：优先使用发散帧，否则使用关节位置/目标位置异常帧
        divergence_frame = None
        for r in results:
            if r.details:
                if '首次发散帧' in r.details:
                    divergence_frame = r.details['首次发散帧']
                    break
                elif divergence_frame is None and '发生帧' in r.details:
                    divergence_frame = r.details['发生帧']

        if divergence_frame is not None:
            analyze_motion_csv(args.motion_csv, divergence_frame)
        else:
            print(f"{Color.yellow('跳过 CSV 分析：未检测到异常帧')}")

    # 返回错误码
    error_count = sum(1 for r in results if r.status == "ERROR")
    sys.exit(1 if error_count > 0 else 0)


if __name__ == '__main__':
    main()
