import os
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
HELPER = ROOT / "src/leju_launch/scripts/_teleop_config_path.sh"


class LaunchRealTeleopPathTest(unittest.TestCase):
    def select(self, home, repository, explicit="", runtime_override=""):
        env = os.environ.copy()
        env.update({"HOME": str(home), "TELEOP_CONFIG": explicit})
        if runtime_override:
            env["TELEOP_RUNTIME_CONFIG"] = runtime_override
        else:
            env.pop("TELEOP_RUNTIME_CONFIG", None)
        command = f'source "{HELPER}"; select_real_teleop_config "$REPOSITORY"; printf "%s" "$TELEOP_CONFIG"'
        return subprocess.run(
            ["bash", "-c", command], env={**env, "REPOSITORY": str(repository)},
            text=True, capture_output=True, check=True,
        )

    def test_explicit_cli_path_has_highest_priority(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.select(directory, "/repo/default.yaml", explicit="/explicit/custom.yaml")
            self.assertEqual("/explicit/custom.yaml", result.stdout)

    def test_runtime_lejuconfig_is_used_when_present(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / ".config/lejuconfig/teleop_bindings.yaml"
            runtime.parent.mkdir(parents=True)
            runtime.write_text("joy_bindings: []\n")
            repository = Path(directory) / "repository.yaml"
            repository.write_text("joy_bindings: []\n")
            result = self.select(directory, repository)
            self.assertEqual(str(runtime), result.stdout)

    def test_repository_config_is_fallback_when_runtime_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            repository = Path(directory) / "repository.yaml"
            repository.write_text("joy_bindings: []\n")
            result = self.select(directory, repository)
            self.assertEqual(str(repository), result.stdout)
            self.assertIn("repository fallback", result.stderr)

    def test_runtime_override_is_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / "runtime.yaml"
            runtime.write_text("joy_bindings: []\n")
            result = self.select(directory, "/missing/repository.yaml", runtime_override=str(runtime))
            self.assertEqual(str(runtime), result.stdout)

    def test_missing_both_keeps_argument_empty(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.select(directory, "/missing/repository.yaml")
            self.assertEqual("", result.stdout)


if __name__ == "__main__":
    unittest.main()
