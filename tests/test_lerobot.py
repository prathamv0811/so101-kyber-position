import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hardware import Bus, JOINTS


class LeRobotAdapterTests(unittest.TestCase):
    def test_real_lerobot_leader_class_with_fake_motor_bus(self):
        driver = MagicMock()
        driver.is_calibrated = True
        driver.sync_read.return_value = dict.fromkeys(JOINTS, 2048)
        data = {name: {"id": i + 1, "drive_mode": 0, "homing_offset": 0,
                       "range_min": 0, "range_max": 4095} for i, name in enumerate(JOINTS)}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test_leader.json"
            path.write_text(json.dumps(data))
            with patch("lerobot.teleoperators.so_leader.so_leader.FeetechMotorsBus", return_value=driver):
                bus = Bus("fake-port", list(range(1, 7)), path, "leader")
                self.assertEqual(bus.positions(), [2048] * 6)
                driver.sync_read.assert_called_with("Present_Position", normalize=False, num_retry=2)
                bus.send_positions([2050] * 6)
                driver.sync_write.assert_called_with("Goal_Position", dict.fromkeys(JOINTS, 2050), normalize=False, num_retry=2)
                bus.close()
            driver.connect.assert_called_once_with()
            driver.disconnect.assert_called_once_with(disable_torque=False)
            driver.enable_torque.assert_not_called()
            driver.write_calibration.assert_not_called()


if __name__ == "__main__":
    unittest.main()
