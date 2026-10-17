"""Exercise actual pinned LeRobot arm methods with only physical I/O mocked."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hardware import Bus, JOINTS, ticks_to_action


class LeRobotAdapterTests(unittest.TestCase):
    def test_public_leader_and_follower_apis_and_units(self):
        from lerobot.motors.feetech import FeetechMotorsBus
        data = {name: {"id": i + 1, "drive_mode": 0, "homing_offset": 0,
                       "range_min": 700 + i * 10, "range_max": 3500 - i * 20}
                for i, name in enumerate(JOINTS)}
        raw = [1000, 1400, 1800, 2200, 2600, 3000]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "arm.json"
            path.write_text(json.dumps(data))
            for role in ("leader", "follower"):
                with patch.object(FeetechMotorsBus, "connect"), \
                     patch.object(FeetechMotorsBus, "is_calibrated", property(lambda _: True)), \
                     patch.object(FeetechMotorsBus, "is_connected", property(lambda _: True)):
                    bus = Bus("fake-port", list(range(1, 7)), path, role)
                    with patch.object(bus.driver, "sync_read", return_value=dict.fromkeys(JOINTS, 0)) as torque:
                        bus.require_passive()
                        torque.assert_called_once_with("Torque_Enable", normalize=False, num_retry=2)
                    with patch.object(bus.driver, "sync_read", return_value=dict.fromkeys(JOINTS, 1)):
                        with self.assertRaisesRegex(RuntimeError, "Torque is enabled"):
                            bus.require_passive()
                    normalized = bus.driver._normalize(dict(zip(bus.ids, raw)))
                    reading = {name: normalized[i] for name, i in zip(JOINTS, bus.ids)}
                    with patch.object(bus.driver, "sync_read", return_value=reading) as read:
                        expected = {f"{name}.pos": reading[name] for name in JOINTS}
                        action = bus.get_action() if role == "leader" else bus.get_observation()
                        self.assertEqual(action, expected)
                        read.assert_called_once_with("Present_Position", num_retry=2)
                        self.assertEqual(ticks_to_action(raw, bus.cal), expected)
                    # Exercise actual SOFollower.send_action, not a replacement stub.
                    with patch.object(bus.driver, "sync_write") as write:
                        if role == "follower":
                            sent = bus.send_action(expected)
                            write.assert_called_once_with("Goal_Position", reading)
                            self.assertEqual(set(sent), {f"{name}.pos" for name in JOINTS})
                            decoded = bus.driver._unnormalize(normalized)
                            self.assertTrue(all(abs(decoded[i] - value) <= 1 for i, value in zip(bus.ids, raw)))
                        else:
                            with self.assertRaisesRegex(RuntimeError, "Only the follower"):
                                bus.send_action(expected)
                            write.assert_not_called()
                    # Saturation must not hide gripper register values beyond calibration.
                    reading["gripper"] = 100.0
                    with patch.object(bus.driver, "sync_read", return_value=reading), \
                         patch.object(bus.driver, "read", return_value=data["gripper"]["range_max"] + 10):
                        with self.assertRaisesRegex(RuntimeError, "calibrated range"):
                            bus.get_action() if role == "leader" else bus.get_observation()
                    with patch.object(bus.driver, "disconnect") as disconnect:
                        bus.close()
                        disconnect.assert_called_once_with(disable_torque=False)


if __name__ == "__main__":
    unittest.main()
