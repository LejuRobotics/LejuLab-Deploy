import hashlib
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "install_teleop_bindings.sh"
STANDARD = ROOT / "src/leju-controllers/leju-rl-controller/config/17/teleop_bindings.yaml"


class InstallTeleopBindingsTest(unittest.TestCase):
    def run_script(self, home, *args, source=STANDARD, check=True):
        env = os.environ.copy()
        env.update({
            "HOME": str(home),
            "ROBOT_VERSION": "17",
            "TELEOP_STANDARD_SOURCE": str(source),
            "TELEOP_PYTHON": sys.executable,
        })
        return subprocess.run(
            [str(SCRIPT), *args], env=env, text=True, input="", capture_output=True, check=check
        )

    @staticmethod
    def target(home):
        return Path(home) / ".config/lejuconfig/teleop_bindings.yaml"

    def test_installs_when_target_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            self.run_script(directory)
            self.assertEqual(STANDARD.read_bytes(), self.target(directory).read_bytes())

    def test_keep_preserves_bytes_and_sha256(self):
        with tempfile.TemporaryDirectory() as directory:
            target = self.target(directory)
            target.parent.mkdir(parents=True)
            target.write_text("joy_bindings: []\n#现场配置\n", encoding="utf-8")
            before = hashlib.sha256(target.read_bytes()).hexdigest()
            result = self.run_script(directory, "--keep")
            self.assertEqual(before, hashlib.sha256(target.read_bytes()).hexdigest())
            self.assertIn(before, result.stdout)

    def test_overwrite_creates_backup_and_replaces_target(self):
        with tempfile.TemporaryDirectory() as directory:
            target = self.target(directory)
            target.parent.mkdir(parents=True)
            original = b"joy_bindings: []\n#old\n"
            target.write_bytes(original)
            self.run_script(directory, "--overwrite")
            backup = target.with_name(target.name + ".bak")
            self.assertEqual(original, backup.read_bytes())
            self.assertEqual(STANDARD.read_bytes(), target.read_bytes())

            second_original = b"joy_bindings: []\n#second\n"
            target.write_bytes(second_original)
            self.run_script(directory, "--overwrite")
            self.assertEqual(second_original, backup.read_bytes())
            self.assertEqual([], list(target.parent.glob("teleop_bindings.yaml.bak.*")))

            result = self.run_script(directory, "--overwrite")
            self.assertEqual(second_original, backup.read_bytes())
            self.assertIn("unchanged", result.stdout)

    def test_existing_target_requires_policy_when_noninteractive(self):
        with tempfile.TemporaryDirectory() as directory:
            target = self.target(directory)
            target.parent.mkdir(parents=True)
            target.write_text("joy_bindings: []\n", encoding="utf-8")
            result = self.run_script(directory, check=False)
            self.assertEqual(2, result.returncode)
            self.assertIn("--overwrite or --keep", result.stderr)

    def test_rejects_source_without_joy_bindings(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "invalid.yaml"
            source.write_text("velocity_limits: {}\n", encoding="utf-8")
            result = self.run_script(directory, source=source, check=False)
            self.assertNotEqual(0, result.returncode)
            self.assertIn("joy_bindings", result.stderr)

    def test_installed_yaml_contains_eight_standard_combinations(self):
        with tempfile.TemporaryDirectory() as directory:
            self.run_script(directory)
            config = yaml.safe_load(self.target(directory).read_text(encoding="utf-8"))
            actual = {tuple(item.get("buttons", [])) for item in config["joy_bindings"]}
            expected = {(trigger, face) for trigger in ("LT", "RT") for face in ("A", "B", "X", "Y")}
            self.assertTrue(expected.issubset(actual))


if __name__ == "__main__":
    unittest.main()
