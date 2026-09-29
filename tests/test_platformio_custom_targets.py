import os
import sys
import unittest
from unittest.mock import patch

# Ensure platforms/esp32 is importable
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ESP32_DIR = os.path.join(REPO_ROOT, "platforms", "esp32")
if ESP32_DIR not in sys.path:
    sys.path.insert(0, ESP32_DIR)


class MockEnv:
    def __init__(self, values):
        self._values = values

    def subst(self, key):
        return self._values.get(key, "")


class TestPlatformioCustomTargets(unittest.TestCase):
    def test_scons_keyword_calling_convention(self):
        """SCons calls actions with target=target, source=rsources, env=env."""
        import platformio_custom_targets

        mock_env = MockEnv({
            "$PROJECT_DIR": "/tmp/remus",
            "$PIOENV": "remus-blade-dev",
            "$UPLOAD_PORT": "none",
        })

        with patch("subprocess.call", return_value=0) as mock_call:
            # SCons 4.x Action call convention
            result = platformio_custom_targets.monitor_after_upload(
                target=["install_and_monitor"],
                source=["upload"],
                env=mock_env,
            )
            self.assertEqual(result, 0)
            mock_call.assert_called_once()
            called_cmd = mock_call.call_args[0][0]
            self.assertEqual(
                called_cmd,
                [
                    os.path.join("/tmp/remus", "scripts", "platformio_install_and_monitor.sh"),
                    "remus-blade-dev",
                    "Remus Blade",
                    "PlatformIO Custom task",
                    "--monitor-only",
                ],
            )
            self.assertEqual(mock_call.call_args[1].get("cwd"), "/tmp/remus")

    def test_proto1_device_name(self):
        """remus-proto1 should be named Remus Computer."""
        import platformio_custom_targets

        mock_env = MockEnv({
            "$PROJECT_DIR": "/tmp/remus",
            "$PIOENV": "remus-proto1",
            "$UPLOAD_PORT": "/dev/cu.usbmodem101",
        })

        with patch("subprocess.call", return_value=0) as mock_call:
            result = platformio_custom_targets.monitor_after_upload(
                target=["install_and_monitor"],
                source=["upload"],
                env=mock_env,
            )
            self.assertEqual(result, 0)
            mock_call.assert_called_once()
            called_cmd = mock_call.call_args[0][0]
            self.assertIn("Remus Computer", called_cmd)
            self.assertIn("--port", called_cmd)
            self.assertIn("/dev/cu.usbmodem101", called_cmd)

    def test_autonomous_profile_is_explicit_and_does_not_change_release_default(self):
        config_path = os.path.join(REPO_ROOT, "platformio.ini")
        with open(config_path, encoding="utf-8") as handle:
            config = handle.read()

        release_start = config.index("[env:remus-proto1]")
        autonomous_start = config.index("[env:remus-proto1-autonomous]")
        blade_start = config.index("[env:remus-blade-dev]")
        release_profile = config[release_start:autonomous_start]
        autonomous_profile = config[autonomous_start:blade_start]

        self.assertIn("REMUS_ENABLE_BLADE_RELAY=0", release_profile)
        self.assertNotIn("REMUS_AUTOSTART_LOCAL_CAPTURE=1", release_profile)
        self.assertIn("REMUS_ENABLE_BLADE_RELAY=1", autonomous_profile)
        self.assertIn("REMUS_AUTOSTART_LOCAL_CAPTURE=1", autonomous_profile)
        self.assertIn("REMUS_LOCAL_FLUSH_INTERVAL_MS=5000", autonomous_profile)


if __name__ == "__main__":
    unittest.main()
