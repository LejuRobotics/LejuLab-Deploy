#!/usr/bin/env python3
"""
mujoco_viewer.py — VR IK 结果可视化工具

读取状态文件 /tmp/vr_ik_state.json（由 test_arm_ik_node 或 quest_vr_abs_control_node 写入），
在 MuJoCo 中实时渲染 biped_s17 手臂姿态和 VR 目标标记。

状态文件格式:
  {
    "arm_joints_rad": [...],        # 8个关节角（弧度，左臂4 + 右臂4）
    "left_hand_target":  [x, y, z], # 左手目标（躯干坐标系，缩放后 → IK）
    "right_hand_target": [x, y, z],
    "left_elbow_target": [x, y, z],
    "right_elbow_target":[x, y, z],
    "ik_success": bool,
    # 以下为 quest_vr_abs_control_node 可选写入（调试用）：
    "left_hand_pre_scale" 等: 胸减+yaw+bias+肩宽后、缩放前（躯干系）
        → 小球「pre」黄/绿色，由 waist_yaw_link 变换到 MuJoCo 世界系
  }

用法:
  DISPLAY=:1 python3 mujoco_viewer.py [选项]

说明:
  若 shell 里设置了 MUJOCO_GL=osmesa，会走 PyOpenGL，在无 Mesa/OSMesa
  环境下常报错；本脚本会在导入 mujoco 前强制改为 glfw（有窗口）或 egl（无窗口），
  与独立可执行程序 simulate（GLFW）更接近。专家可用 VR_MUJOCO_GL=glx|glfw|egl 覆盖。

选项:
  --state-file PATH   状态文件路径（默认 /tmp/vr_ik_state.json）
  --no-viewer         不启动图形窗口，仅打印状态（适用于无显示器环境）
  --hz FLOAT          刷新频率（默认 30）
  --base-z FLOAT      机器人底座高度（默认 0.95m，调整以使机器人站立合理）
  --blas-threads N    BLAS/OpenMP 线程数（默认 1）。须在命令行中给出才会在 import 前生效；
                      也可在运行前 export VR_MUJOCO_BLAS_THREADS=N（仅当对应变量尚未设置时写入）。

说明（线程数）:
  htop 里大量线程多半来自 OpenBLAS/MKL 与 OMP，而非本脚本显式创建。
  默认将 OMP/OPENBLAS/MKL 等限制为 1；需要多核时再传 --blas-threads 4 等。
"""

import os
import sys


def _configure_compute_threads() -> None:
    """在 import numpy / mujoco 之前限制 BLAS/OpenMP 线程，显著减少进程内线程数。"""
    keys = (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "VECLIB_MAXIMUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    )
    n = None
    argv = sys.argv
    for i, a in enumerate(argv):
        if a == "--blas-threads" and i + 1 < len(argv):
            try:
                n = max(1, int(argv[i + 1]))
            except ValueError:
                pass
            break
    if n is not None:
        s = str(n)
        for k in keys:
            os.environ[k] = s
        return
    env = os.environ.get("VR_MUJOCO_BLAS_THREADS", "").strip()
    if env.isdigit():
        s = str(max(1, int(env)))
        for k in keys:
            os.environ.setdefault(k, s)
        return
    for k in keys:
        os.environ.setdefault(k, "1")


_configure_compute_threads()

import json
import time
import argparse
import numpy as np

# ── MUJOCO_GL 必须在 import mujoco 之前设置 ─────────────────────────────────
# 常见坑：系统/conda 里 export 了 MUJOCO_GL=osmesa → 会 import PyOpenGL 并崩溃；
# 独立程序 simulate 使用 GLFW，不走 osmesa。此处**强制**设置（除非 VR_MUJOCO_GL）。
def _setup_gl_env(no_viewer: bool) -> None:
    expert = os.environ.get("VR_MUJOCO_GL", "").strip().lower()
    if expert in ("glfw", "glx", "egl"):
        os.environ["MUJOCO_GL"] = expert
        return
    if no_viewer or not os.environ.get("DISPLAY", "").strip():
        os.environ["MUJOCO_GL"] = "egl"
    else:
        os.environ["MUJOCO_GL"] = "glfw"

# 在解析完整参数之前做快速预判（保证 import mujoco 之前 env 已设好）
_setup_gl_env("--no-viewer" in sys.argv)

try:
    import mujoco  # noqa: E402  — 必须在 MUJOCO_GL 设置之后导入
    # namespace 包（mujoco 使用 pkgutil.extend_path）不会把子模块挂到 mujoco.viewer 上，
    # 必须用显式 import，否则 AttributeError: module 'mujoco' has no attribute 'viewer'
    from mujoco import viewer as mj_viewer  # noqa: E402
except Exception as e:  # noqa: BLE001
    sys.stderr.write(
        "[mujoco_viewer] 导入 mujoco 失败: %s\n"
        "  当前 MUJOCO_GL=%r  DISPLAY=%r\n"
        "  可尝试: VR_MUJOCO_GL=egl python3 ... --no-viewer\n"
        "       或: VR_MUJOCO_GL=glfw DISPLAY=:1 python3 ...\n"
        % (e, os.environ.get("MUJOCO_GL"), os.environ.get("DISPLAY"))
    )
    raise


def _find_assets_path() -> str:
    """从环境变量或相对路径推断 leju_assets 目录"""
    env = os.environ.get("LEJU_ASSETS_PATH", "")
    if env and os.path.isdir(env):
        return env
    # 推断：本脚本位于 src/leju-vr-control/test/
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.normpath(os.path.join(script_dir, "../../../leju_assets")),
        os.path.normpath(os.path.join(script_dir, "../../leju_assets")),
        os.path.normpath(os.path.join(script_dir, "../../../../src/leju_assets")),
    ]
    for c in candidates:
        if os.path.isdir(c):
            return c
    return ""


def _load_model_with_markers(xml_path: str):
    """加载 biped_s17.xml 并注入 VR 目标标记（mocap body）"""
    with open(xml_path, "r") as f:
        xml = f.read()

    mesh_dir = os.path.abspath(os.path.join(os.path.dirname(xml_path), "../meshes"))

    # 将 meshdir 改为绝对路径（from_xml_string 时必须）
    xml = xml.replace('meshdir="../meshes/"', f'meshdir="{mesh_dir}/"')
    xml = xml.replace("meshdir='../meshes/'", f"meshdir='{mesh_dir}/'")

    # 注入 mocap marker bodies（在 </worldbody> 之前）
    markers = """
    <!-- VR 目标标记 -->
    <body name="l_hand_target" mocap="true" pos="0 0.22 1.15">
      <geom type="sphere" size="0.045" rgba="1 0.2 0.2 0.75"
            contype="0" conaffinity="0"/>
    </body>
    <body name="r_hand_target" mocap="true" pos="0 -0.22 1.15">
      <geom type="sphere" size="0.045" rgba="0.2 0.2 1 0.75"
            contype="0" conaffinity="0"/>
    </body>
    <body name="l_elbow_target" mocap="true" pos="0 0.30 1.20">
      <geom type="sphere" size="0.032" rgba="1 0.55 0.1 0.75"
            contype="0" conaffinity="0"/>
    </body>
    <body name="r_elbow_target" mocap="true" pos="0 -0.30 1.20">
      <geom type="sphere" size="0.032" rgba="0.1 0.8 0.8 0.75"
            contype="0" conaffinity="0"/>
    </body>
    <!-- 调试：缩放前躯干系目标（经 waist_yaw_link 转到世界系） -->
    <body name="l_hand_pre" mocap="true" pos="0 0.20 1.12">
      <geom type="sphere" size="0.028" rgba="0.95 0.75 0.15 0.6"
            contype="0" conaffinity="0"/>
    </body>
    <body name="r_hand_pre" mocap="true" pos="0 -0.20 1.12">
      <geom type="sphere" size="0.028" rgba="0.95 0.55 0.15 0.6"
            contype="0" conaffinity="0"/>
    </body>
    <body name="l_elbow_pre" mocap="true" pos="0 0.26 1.18">
      <geom type="sphere" size="0.022" rgba="0.85 0.9 0.2 0.55"
            contype="0" conaffinity="0"/>
    </body>
    <body name="r_elbow_pre" mocap="true" pos="0 -0.26 1.18">
      <geom type="sphere" size="0.022" rgba="0.85 0.75 0.2 0.55"
            contype="0" conaffinity="0"/>
    </body>
"""
    xml = xml.replace("</worldbody>", markers + "\n  </worldbody>")

    model = mujoco.MjModel.from_xml_string(xml)
    data  = mujoco.MjData(model)
    return model, data


def _set_standing_pose(model, data, base_z: float):
    """将机器人底座置于指定高度，腿部关节设为 0"""
    base_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "dummy_to_base_link")
    if base_jid < 0:
        return
    adr = model.jnt_qposadr[base_jid]
    data.qpos[adr + 0] = 0.0   # x
    data.qpos[adr + 1] = 0.0   # y
    data.qpos[adr + 2] = base_z # z
    data.qpos[adr + 3] = 1.0   # qw
    data.qpos[adr + 4] = 0.0   # qx
    data.qpos[adr + 5] = 0.0   # qy
    data.qpos[adr + 6] = 0.0   # qz


def _get_torso_xpos_xmat(model, data):
    """返回 waist_yaw_link（≈Drake torso）的世界位置和旋转矩阵"""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "waist_yaw_link")
    if bid < 0:
        # 回退：取 zarm_l1_link 的父节点
        l1_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "zarm_l1_link")
        bid = int(model.body_parentid[l1_bid]) if l1_bid >= 0 else 1
    pos = np.array(data.xpos[bid])
    mat = np.array(data.xmat[bid]).reshape(3, 3)
    return pos, mat


def _read_state(path: str):
    """读取状态 JSON，返回 dict 或 None"""
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return None


def _update_model(model, data, state: dict,
                  arm_l_jids, arm_r_jids,
                  mocap_ids: dict, base_z: float):
    """将 IK 关节角和 VR 目标标记写入 MjData"""
    arm_joints = state.get("arm_joints_rad", [0.0] * 8)

    # 设置底座（每帧都设，防止模拟器移动）
    _set_standing_pose(model, data, base_z)

    # 写手臂关节角
    for i, jid in enumerate(arm_l_jids):
        if jid >= 0 and i < len(arm_joints):
            data.qpos[model.jnt_qposadr[jid]] = arm_joints[i]
    for i, jid in enumerate(arm_r_jids):
        if jid >= 0 and (4 + i) < len(arm_joints):
            data.qpos[model.jnt_qposadr[jid]] = arm_joints[4 + i]

    mujoco.mj_forward(model, data)

    # 获取躯干世界坐标（Drake torso ≈ waist_yaw_link）
    torso_pos, torso_rot = _get_torso_xpos_xmat(model, data)

    # 更新 mocap 标记位置（躯干坐标系 → 世界坐标系）
    def _set_marker(key_suffix, key_state):
        target_key = key_state + "_target"
        body_key   = key_suffix + "_target"
        mid = mocap_ids.get(body_key, -1)
        if mid < 0:
            return
        p = state.get(target_key)
        if p is None or len(p) < 3:
            return
        world_pos = torso_pos + torso_rot @ np.array(p)
        data.mocap_pos[model.body_mocapid[mid]] = world_pos

    _set_marker("l_hand",  "left_hand")
    _set_marker("r_hand",  "right_hand")
    _set_marker("l_elbow", "left_elbow")
    _set_marker("r_elbow", "right_elbow")

    def _set_marker_robot_pre(body_key: str, json_key: str):
        mid = mocap_ids.get(body_key, -1)
        if mid < 0:
            return
        p = state.get(json_key)
        if p is None or len(p) < 3:
            return
        world_pos = torso_pos + torso_rot @ np.array(p[:3], dtype=float)
        data.mocap_pos[model.body_mocapid[mid]] = world_pos

    # 缩放前、与 IK target 同躯干系
    _set_marker_robot_pre("l_hand_pre", "left_hand_pre_scale")
    _set_marker_robot_pre("r_hand_pre", "right_hand_pre_scale")
    _set_marker_robot_pre("l_elbow_pre", "left_elbow_pre_scale")
    _set_marker_robot_pre("r_elbow_pre", "right_elbow_pre_scale")


def main():
    parser = argparse.ArgumentParser(description="MuJoCo VR IK Viewer")
    parser.add_argument("--state-file", default="/tmp/vr_ik_state.json",
                        help="IK 状态文件路径")
    parser.add_argument("--no-viewer",  action="store_true",
                        help="不启动图形界面（仅打印，EGL headless）")
    parser.add_argument("--hz",         type=float, default=30.0,
                        help="刷新频率 (Hz)")
    parser.add_argument("--base-z",     type=float, default=0.95,
                        help="机器人底座高度 (m)")
    parser.add_argument(
        "--blas-threads",
        type=int,
        default=None,
        metavar="N",
        help="限制 NumPy 所用 BLAS/OpenMP 线程数（在 import 前解析；默认 1，见文首说明）",
    )
    args = parser.parse_args()

    # ── 路径 ──
    assets_path = _find_assets_path()
    if not assets_path:
        sys.exit("[Viewer] ERROR: leju_assets not found. Set LEJU_ASSETS_PATH.")
    xml_path = os.path.join(assets_path, "models/biped_s17/xml/biped_s17.xml")
    if not os.path.exists(xml_path):
        sys.exit(f"[Viewer] ERROR: {xml_path} not found.")

    print(f"[Viewer] Loading model: {xml_path}")
    model, data = _load_model_with_markers(xml_path)
    print(f"[Viewer] Model loaded. nq={model.nq}, nbody={model.nbody}")

    # ── 缓存关节 ID ──
    arm_l_jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                     f"zarm_l{i}_joint") for i in range(1, 5)]
    arm_r_jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                     f"zarm_r{i}_joint") for i in range(1, 5)]

    mocap_ids = {
        "l_hand_target":  mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_hand_target"),
        "r_hand_target":  mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_hand_target"),
        "l_elbow_target": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_elbow_target"),
        "r_elbow_target": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_elbow_target"),
        "l_hand_pre": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_hand_pre"),
        "r_hand_pre": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_hand_pre"),
        "l_elbow_pre": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_elbow_pre"),
        "r_elbow_pre": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_elbow_pre"),
    }

    _set_standing_pose(model, data, args.base_z)
    mujoco.mj_forward(model, data)

    dt = 1.0 / args.hz
    last_ts = -1.0

    if args.no_viewer:
        # ── Headless 模式：仅打印 ──
        print("[Viewer] Headless mode. Reading state file...")
        while True:
            state = _read_state(args.state_file)
            if state:
                ts = state.get("timestamp", -1.0)
                if ts != last_ts:
                    last_ts = ts
                    joints = state.get("arm_joints_rad", [])
                    ok     = state.get("ik_success", False)
                    jstr = " ".join(f"{j*180/np.pi:+6.1f}" for j in joints)
                    print(f"[Viewer] ok={ok}  joints(deg): {jstr}")
                    _update_model(model, data, state,
                                  arm_l_jids, arm_r_jids, mocap_ids, args.base_z)
            time.sleep(dt)

    else:
        # ── 图形模式 ──
        print(f"[Viewer] Launching MuJoCo viewer (DISPLAY={os.environ.get('DISPLAY','?')})")

        with mj_viewer.launch_passive(model, data) as viewer:
            viewer.cam.azimuth  = 130
            viewer.cam.elevation = -18
            viewer.cam.distance  = 2.8
            viewer.cam.lookat[:] = [0.05, 0.0, 1.0]

            print("[Viewer] Window opened. Waiting for state file...")
            while viewer.is_running():
                t0 = time.time()

                state = _read_state(args.state_file)
                if state:
                    ts = state.get("timestamp", -1.0)
                    if ts != last_ts:
                        last_ts = ts
                        _update_model(model, data, state,
                                      arm_l_jids, arm_r_jids, mocap_ids, args.base_z)

                viewer.sync()

                elapsed = time.time() - t0
                if elapsed < dt:
                    time.sleep(dt - elapsed)

        print("[Viewer] Window closed.")


if __name__ == "__main__":
    main()
