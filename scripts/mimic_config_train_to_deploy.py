#!/usr/bin/env python3
"""Convert an Isaac Lab training `env.yaml` (exported by
`leju_whole_body_tracking/scripts/rsl_rl/train.py`) into a deploy-side
mimic controller YAML consumable by `leju-rl-controller`.

Only **auto-derivable** fields are filled from env.yaml. Hardware-specific or
deploy-only fields fall back to standalone Roban 2.2/17 deploy defaults, with
CLI overrides available.  `--template` is optional and only kept as a migration
helper for legacy hand-authored fields.

The deploy-side `env.robot.joint_names` order is treated as canonical.  Training
env.yaml may use a different joint order; per-joint fields are resolved by joint
name and emitted in the deploy order.

See `.claude/training-to-mimic-yaml-mapping.md` for the full field-by-field
correspondence.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any

import yaml


# ---------------------------------------------------------------------------
# YAML loader tolerating Isaac Lab's !!python/tuple, !!python/object/apply,
# !!python/name tags without needing the runtime classes.
# ---------------------------------------------------------------------------
class EnvYamlLoader(yaml.SafeLoader):
    pass


def _python_tuple_constructor(loader: yaml.Loader, node: yaml.Node) -> list:
    return loader.construct_sequence(node, deep=True)


def _python_any_constructor(loader: yaml.Loader, tag_suffix: str, node: yaml.Node) -> Any:
    if isinstance(node, yaml.ScalarNode):
        return loader.construct_scalar(node)
    if isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node, deep=True)
    if isinstance(node, yaml.MappingNode):
        return loader.construct_mapping(node, deep=True)
    return None


EnvYamlLoader.add_constructor("tag:yaml.org,2002:python/tuple", _python_tuple_constructor)
EnvYamlLoader.add_multi_constructor("tag:yaml.org,2002:python/", _python_any_constructor)


# ---------------------------------------------------------------------------
# Mapping constants
# ---------------------------------------------------------------------------
OBS_NAME_MAP = {
    "command": "motion_command",
    "motion_target_pos": "motion_target_height",
}

OBS_META_KEYS = {
    "concatenate_terms",
    "enable_corruption",
    "history_length",
    "flatten_history_dim",
}

RESIDUAL_MARKERS = ("Residual",)

DEFAULT_DANCE_NAME = "leju_deploy"
DEFAULT_MOTION_FILE = "trajectory.csv"
DEFAULT_POLICY_PATH = "model.onnx"
DEFAULT_INFERENCE_ENGINE = "openvino"
ALLOWED_EXPLICIT_INFERENCE_ENGINE = "onnxruntime"

ROBAN2_2_MIMIC_JOINT_NAMES = [
    "waist_yaw_joint",
    "leg_l1_joint",
    "leg_l2_joint",
    "leg_l3_joint",
    "leg_l4_joint",
    "leg_l5_joint",
    "leg_l6_joint",
    "leg_r1_joint",
    "leg_r2_joint",
    "leg_r3_joint",
    "leg_r4_joint",
    "leg_r5_joint",
    "leg_r6_joint",
    "zarm_l1_joint",
    "zarm_l2_joint",
    "zarm_l3_joint",
    "zarm_l4_joint",
    "zarm_r1_joint",
    "zarm_r2_joint",
    "zarm_r3_joint",
    "zarm_r4_joint",
]

ROBAN2_2_MIMIC_JOINT_DIRECTION = [
    -1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
]

ROBAN2_2_MIMIC_ACTUATOR_CONTROL_MODE = [
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    2,
    2,
    2,
    2,
    2,
    2,
    2,
    2,
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _regex_key_to_python(key: str) -> str:
    """Isaac Lab uses `[l,r]` as a char-class; Python regex needs `[lr]`."""
    return re.sub(r"\[([^\]]*)\]", lambda m: "[" + m.group(1).replace(",", "") + "]", key)


def resolve_per_joint(
    src_cfg: dict | list,
    output_joint_names: list[str],
    source_joint_names: list[str] | None = None,
) -> list[float]:
    """Expand a per-joint config into `output_joint_names` order.

    `src_cfg` is normally an exact-name/regex-keyed dict from Isaac Lab.  Lists
    are also supported when `source_joint_names` is provided: the list is first
    bound to source names, then reordered by name.
    """
    if isinstance(src_cfg, list):
        if source_joint_names is None:
            raise TypeError("list per-joint cfg requires source_joint_names")
        if len(src_cfg) != len(source_joint_names):
            raise ValueError(
                f"list per-joint cfg has length {len(src_cfg)}, "
                f"expected {len(source_joint_names)} to match source_joint_names"
            )
        by_name = dict(zip(source_joint_names, src_cfg))
        try:
            return [float(by_name[jn]) for jn in output_joint_names]
        except KeyError as e:
            raise KeyError(f"joint {e.args[0]!r} missing from source joint order") from e

    if not isinstance(src_cfg, dict):
        raise TypeError(f"expected dict/list for per-joint cfg, got {type(src_cfg)!r}")

    compiled: list[tuple[re.Pattern, float]] = []
    exact: dict[str, float] = {}
    for raw_key, val in src_cfg.items():
        if raw_key in output_joint_names or (source_joint_names and raw_key in source_joint_names):
            exact[raw_key] = val
        else:
            py_key = _regex_key_to_python(raw_key)
            try:
                compiled.append((re.compile(py_key), val))
            except re.error:
                exact[raw_key] = val

    out: list[float] = []
    for jn in output_joint_names:
        if jn in exact:
            out.append(float(exact[jn]))
            continue
        hit = next((v for pat, v in compiled if pat.fullmatch(jn)), None)
        if hit is None:
            raise KeyError(f"no entry matches joint {jn!r} in cfg keys {list(src_cfg.keys())!r}")
        out.append(float(hit))
    return out


def pick_motor_actuator(actuators: dict) -> dict:
    if "motor" in actuators:
        return actuators["motor"]
    if len(actuators) == 1:
        return next(iter(actuators.values()))
    raise KeyError(
        f"expected a single actuator group or a 'motor' key, got {list(actuators.keys())!r}"
    )


def is_residual_action(actions_cfg: dict) -> bool:
    class_type = str(actions_cfg.get("joint_pos", {}).get("class_type", ""))
    return any(m in class_type for m in RESIDUAL_MARKERS)


def obs_policy_terms_ordered(policy_cfg: dict) -> list[tuple[str, str]]:
    """Return [(deploy_name, training_name)] in training concat order."""
    out = []
    for k in policy_cfg.keys():  # dict preserves insertion order since 3.7
        if k in OBS_META_KEYS:
            continue
        out.append((OBS_NAME_MAP.get(k, k), k))
    return out


def load_template(template_path: Path | None) -> dict:
    if template_path is None:
        return {}
    with open(template_path) as f:
        return yaml.safe_load(f) or {}


def pick_from_template(tpl: dict, dotted: str, default: Any) -> Any:
    cur: Any = tpl
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def pick_deploy_joint_names(tpl: dict, args: argparse.Namespace) -> list[str]:
    if args.joint_order:
        return list(args.joint_order)
    return list(ROBAN2_2_MIMIC_JOINT_NAMES)


def resolve_human_authored_array(
    name: str,
    cli_value: list[float] | list[int] | None,
    tpl: dict,
    output_joint_names: list[str],
    default: list | float | int,
) -> list:
    if cli_value is not None:
        if len(cli_value) != len(output_joint_names):
            raise ValueError(
                f"{name} has length {len(cli_value)}, expected {len(output_joint_names)} "
                "(to match deploy joint_names)"
            )
        return list(cli_value)

    tpl_value = pick_from_template(tpl, f"HumanoidRobotCfg.env.robot.{name}", None)
    if tpl_value is None:
        if isinstance(default, list):
            if len(default) != len(output_joint_names):
                raise ValueError(
                    f"built-in {name} has length {len(default)}, expected {len(output_joint_names)} "
                    "(to match deploy joint_names)"
                )
            return list(default)
        return [default] * len(output_joint_names)

    tpl_joint_names = pick_from_template(tpl, "HumanoidRobotCfg.env.robot.joint_names", None)
    if tpl_joint_names and list(tpl_joint_names) != output_joint_names:
        return resolve_per_joint(list(tpl_value), output_joint_names, list(tpl_joint_names))

    if len(tpl_value) != len(output_joint_names):
        raise ValueError(
            f"template {name} has length {len(tpl_value)}, expected {len(output_joint_names)} "
            "(to match deploy joint_names)"
        )
    return list(tpl_value)


def validate_joint_sets(training_joint_names: list[str], deploy_joint_names: list[str]) -> None:
    missing = [jn for jn in deploy_joint_names if jn not in training_joint_names]
    if missing:
        raise ValueError(
            "deploy joint_names contains joints missing from training actions.joint_pos.joint_names: "
            + ", ".join(missing)
        )

    duplicates = sorted({jn for jn in deploy_joint_names if deploy_joint_names.count(jn) > 1})
    if duplicates:
        raise ValueError("deploy joint_names contains duplicate joints: " + ", ".join(duplicates))


# ---------------------------------------------------------------------------
# Core conversion
# ---------------------------------------------------------------------------
def convert(env_cfg: dict, tpl: dict, args: argparse.Namespace) -> dict:
    # ---- scalars / residual ------------------------------------------------
    sim_dt = float(env_cfg["sim"]["dt"])
    decimation = int(env_cfg["decimation"])
    policy_dt = sim_dt * decimation

    actions_cfg = env_cfg["actions"]
    residual = is_residual_action(actions_cfg)

    # ---- joints ------------------------------------------------------------
    joint_pos_action = actions_cfg["joint_pos"]
    training_joint_names: list[str] = list(joint_pos_action["joint_names"])
    joint_names: list[str] = pick_deploy_joint_names(tpl, args)
    validate_joint_sets(training_joint_names, joint_names)

    # ---- per-joint arrays --------------------------------------------------
    motor = pick_motor_actuator(env_cfg["scene"]["robot"]["actuators"])
    init_joint_pos = env_cfg["scene"]["robot"]["init_state"]["joint_pos"]

    action_scale = resolve_per_joint(joint_pos_action["scale"], joint_names, training_joint_names)
    actuator_kp = resolve_per_joint(motor["stiffness"], joint_names)
    actuator_kd = resolve_per_joint(motor["damping"], joint_names)
    joint_torque_limit = resolve_per_joint(motor["effort_limit_sim"], joint_names)
    joint_default_pos = resolve_per_joint(init_joint_pos, joint_names)

    # ---- observation term ordering & names --------------------------------
    obs_terms_pairs = obs_policy_terms_ordered(env_cfg["observations"]["policy"])

    # ---- human-authored fields --------------------------------------------
    joint_direction = resolve_human_authored_array(
        "joint_direction", args.joint_direction, tpl, joint_names, ROBAN2_2_MIMIC_JOINT_DIRECTION
    )
    if args.actuator_control_mode is not None:
        if len(args.actuator_control_mode) != len(joint_names):
            raise ValueError(
                f"actuator_control_mode has length {len(args.actuator_control_mode)}, "
                f"expected {len(joint_names)} (to match deploy joint_names)"
            )
        actuator_control_mode = list(args.actuator_control_mode)
    else:
        actuator_control_mode = list(ROBAN2_2_MIMIC_ACTUATOR_CONTROL_MODE)
    inference_engine = args.inference_engine or DEFAULT_INFERENCE_ENGINE
    policy_path = args.policy_path or DEFAULT_POLICY_PATH
    dance_name = args.dance_name or args.motion_name or DEFAULT_DANCE_NAME
    motion_file = args.motion_file or DEFAULT_MOTION_FILE

    # ---- assemble output ---------------------------------------------------
    obs_terms: dict[str, dict] = {}
    for deploy_name, _training_name in obs_terms_pairs:
        obs_terms[deploy_name] = {
            "scale": 1.0,
            "clip": [-5000.0, 5000.0],
        }

    return {
        "HumanoidRobotCfg": {
            "inference_engine": inference_engine,
            "loop_dt": 0.001,
            "policy_path": policy_path,
            "motion": {
                "default_motion": dance_name,
                "motions": [
                    {"name": dance_name, "file": motion_file},
                ],
            },
            "env": {
                "policy_dt": policy_dt,
                "robot": {
                    "residual_action": residual,
                    "joint_names": joint_names,
                    "joint_direction": [float(x) for x in joint_direction],
                    "joint_default_pos": joint_default_pos,
                    "joint_torque_limit": joint_torque_limit,
                    "actuator_kp": actuator_kp,
                    "actuator_kd": actuator_kd,
                    "actuator_control_mode": [int(x) for x in actuator_control_mode],
                    "action_scale": action_scale,
                },
                "observations": {
                    "history_length": 1,
                    "stack_order": "classic",
                    "terms": obs_terms,
                },
            },
        }
    }


# ---------------------------------------------------------------------------
# YAML output formatting (keep the file diff-friendly and readable)
# ---------------------------------------------------------------------------
class _IndentDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):  # noqa: D401
        return super().increase_indent(flow=flow, indentless=False)


def _represent_list(dumper: yaml.Dumper, data: list) -> yaml.Node:
    if data and all(isinstance(x, (str, int, float, bool)) for x in data):
        return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=False)


_IndentDumper.add_representer(list, _represent_list)


def dump_yaml(obj: dict) -> str:
    return yaml.dump(
        obj,
        Dumper=_IndentDumper,
        sort_keys=False,
        default_flow_style=False,
        allow_unicode=True,
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse_float_list(s: str) -> list[float]:
    return [float(x) for x in s.replace(",", " ").split() if x]


def _parse_int_list(s: str) -> list[int]:
    return [int(x) for x in s.replace(",", " ").split() if x]


def _parse_str_list(s: str) -> list[str]:
    return [x for x in s.replace(",", " ").split() if x]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--env-yaml", required=True, type=Path, help="Training env.yaml (input)")
    ap.add_argument("--out", required=True, type=Path, help="Deploy mimic YAML (output)")
    ap.add_argument(
        "--template",
        type=Path,
        default=None,
        help="Optional legacy mimic YAML to source hand-authored fields from "
        "(joint_direction). "
        "The deploy joint order and flat deploy paths are built in by default.",
    )
    ap.add_argument(
        "--policy-path",
        default=None,
        help="Override HumanoidRobotCfg.policy_path (default: model.onnx)",
    )
    ap.add_argument(
        "--inference-engine",
        choices=[ALLOWED_EXPLICIT_INFERENCE_ENGINE],
        default=None,
        help="Override HumanoidRobotCfg.inference_engine. If provided, must be exactly 'onnxruntime'. "
        "Default is openvino.",
    )
    ap.add_argument(
        "--dance-name",
        default=None,
        help="Override motion.default_motion and motions[0].name (default: leju_deploy)",
    )
    ap.add_argument(
        "--motion-name",
        default=None,
        help="Deprecated alias of --dance-name.",
    )
    ap.add_argument(
        "--motion-file",
        default=None,
        help="Override motions[0].file (default: trajectory.csv)",
    )
    ap.add_argument(
        "--joint-order",
        type=_parse_str_list,
        default=None,
        help="Override deploy joint_names order, comma/space separated. "
        "Defaults to the built-in Roban 2.2/17 mimic order.",
    )
    ap.add_argument(
        "--joint-direction",
        type=_parse_float_list,
        default=None,
        help="Override joint_direction array, comma/space separated (length must match joint_names).",
    )
    ap.add_argument(
        "--actuator-control-mode",
        type=_parse_int_list,
        default=None,
        help="Override actuator_control_mode array, comma/space separated (0=CST, 2=CSP).",
    )
    args = ap.parse_args(argv)

    if not args.env_yaml.is_file():
        print(f"env.yaml not found: {args.env_yaml}", file=sys.stderr)
        return 2

    with open(args.env_yaml) as f:
        env_cfg = yaml.load(f, Loader=EnvYamlLoader)

    tpl = load_template(args.template)

    mimic = convert(env_cfg, tpl, args)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(dump_yaml(mimic))
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
