import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "src/leju-joystick/services/deploy_joy_autostart.sh"


class DeployJoyAutostartCliTest(unittest.TestCase):
    def run_script(self, *args):
        return subprocess.run(
            ["bash", str(SCRIPT), *args],
            text=True,
            capture_output=True,
            check=False,
        )

    def test_help_documents_teleop_config_policies(self):
        result = self.run_script("--help")

        self.assertEqual(0, result.returncode)
        self.assertIn("--keep-teleop-config", result.stdout)
        self.assertIn("--overwrite-teleop-config", result.stdout)

    def test_unknown_option_fails_before_deployment(self):
        result = self.run_script("--unknown")

        self.assertEqual(2, result.returncode)
        self.assertIn("未知参数", result.stderr)

    def test_conflicting_teleop_policies_are_rejected(self):
        result = self.run_script(
            "--keep-teleop-config", "--overwrite-teleop-config"
        )

        self.assertEqual(2, result.returncode)
        self.assertIn("不能同时使用", result.stderr)

    def test_deploy_calls_shared_teleop_installer(self):
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertIn('install_teleop_bindings "${robot_version}" "${default_profile_dir}"', source)
        self.assertIn('scripts/install_teleop_bindings.sh', source)


if __name__ == "__main__":
    unittest.main()
