import ast
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "dds_topic_hz.py"


def _topic_type_for_script():
    tree = ast.parse(SCRIPT.read_text())
    function = next(
        (
            node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "_topic_type_for"
        ),
        None,
    )
    assert function is not None, "dds_topic_hz.py must define _topic_type_for"
    namespace = {"Float64Array": object(), "UInt16Array": object()}
    exec(compile(ast.Module([function], type_ignores=[]), str(SCRIPT), "exec"), namespace)
    return namespace["_topic_type_for"], namespace


CONFIG = (
    Path(__file__).parents[3]
    / "leju-controllers/leju-rl-controller/config/17/controllers/config_depth_walk.yaml"
)


def test_depth_walk_uses_normalized_upper_host_frame():
    text = CONFIG.read_text()

    expected = {
        "input_topic": 'input_topic: "rt/depth_camera/frame_normalized_36x64"',
        "input_data_type": 'input_data_type: "float64"',
        "input_width": "input_width: 64",
        "input_height": "input_height: 36",
        "input_unit": 'input_unit: "meters"',
        "input_preprocessed": "input_preprocessed: true",
        "input_frame_decimation": "input_frame_decimation: 1",
        "history_frames": "history_frames: 8",
    }

    for name, line in expected.items():
        assert line in text, f"depth config missing {name}: {line}"


def test_dds_topic_type_matches_normalized_and_raw_topics():
    topic_type_for, types = _topic_type_for_script()
    assert topic_type_for("rt/depth_camera/frame_normalized_36x64") is types["Float64Array"]
    assert topic_type_for("rt/depth_camera/frame_meters_36x64") is types["Float64Array"]
    assert topic_type_for("rt/depth_camera/frame_u16_mm_270x480") is types["UInt16Array"]
