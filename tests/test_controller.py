import sys
import time
import unittest
import tempfile
import json
from argparse import Namespace
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hardware import Bus, JOINTS, PositionController, drive


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.cal = [{"range_min": 0, "range_max": 4095} for _ in JOINTS]
        self.config = {"limits_reviewed": True, "joints": {
            name: {"min_ticks": 100, "max_ticks": 4000, "max_ticks_per_second": 200, "invert": False}
            for name in JOINTS}}

    def sample(self, seq, positions):
        return {"version": 1, "seq": seq, "received_unix_ns": time.time_ns(), "positions": positions}

    def test_relative_anchor_bounds_and_inversion(self):
        self.config["joints"][JOINTS[1]]["invert"] = True
        with patch("hardware.time.monotonic", return_value=10):
            control = PositionController(self.config, self.cal, self.cal, [2000] * 6)
            self.assertEqual(control.target(self.sample(1, [3000] * 6), [2000] * 6), [2000] * 6)
        with patch("hardware.time.monotonic", return_value=10.04):
            target = control.target(self.sample(2, [4000] * 6), [2000] * 6)
        self.assertTrue(2000 < target[0] <= 2008)
        self.assertTrue(1992 <= target[1] < 2000)

    def test_stale_reordered_and_out_of_range_samples_fail(self):
        control = PositionController(self.config, self.cal, self.cal, [2000] * 6)
        control.target(self.sample(1, [2000] * 6), [2000] * 6)
        with self.assertRaises(ValueError):
            control.target(self.sample(1, [2000] * 6), [2000] * 6)
        stale = self.sample(2, [2000] * 6)
        stale["received_unix_ns"] -= 101_000_000
        with self.assertRaises(ValueError):
            control.target(stale, [2000] * 6)
        with self.assertRaises(ValueError):
            control.target(self.sample(2, [5000] * 6), [2000] * 6)

    def test_stop_attempts_every_motor_and_reports_unconfirmed(self):
        bus = object.__new__(Bus)
        bus.ids = list(range(1, 7))
        attempts = []
        def write(motor, *_):
            attempts.append(motor)
            if motor == 2:
                raise RuntimeError("Disconnected motor")
        bus.write = write
        bus.read = lambda motor, *_: 1 if motor == 2 else 0
        with self.assertRaisesRegex(RuntimeError, "STOP NOT CONFIRMED"):
            bus.stop()
        self.assertEqual(attempts, list(range(1, 7)))

    def test_unreviewed_limits_refused(self):
        self.config["limits_reviewed"] = False
        with self.assertRaises(ValueError):
            PositionController(self.config, self.cal, self.cal, [2000] * 6)

    def test_input_loss_after_enable_disables_all_motors(self):
        bus = object.__new__(Bus)
        bus.ids = list(range(1, 7))
        torque = {motor: 0 for motor in bus.ids}
        writes, closed = [], []
        def read(motor, register, _size):
            return {3: 777, 31: 0, 33: 0, 40: torque[motor], 56: 2000}[register]
        def write(motor, register, value, size=1):
            writes.append((motor, register, value))
            if register == 40:
                torque[motor] = value
        bus.read, bus.write = read, write
        bus.close = lambda: closed.append(True)
        cal = [{"id": i + 1, "range_min": 0, "range_max": 4095, "homing_offset": 0,
                "drive_mode": 0} for i in range(6)]
        first = self.sample(1, [2000] * 6)
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "limits.json"
            config.write_text(json.dumps(self.config))
            args = Namespace(config=config, port="fake", leader_calibration="fake",
                follower_calibration="fake", enable_motion=True, attended=True)
            with patch("hardware.Bus", return_value=bus), patch("hardware.calibration", return_value=cal), \
                 patch("hardware.read_sample", side_effect=[first, TimeoutError("Disconnected")]):
                with self.assertRaises(TimeoutError):
                    drive(args)
        self.assertTrue(all(value == 0 for value in torque.values()))
        self.assertEqual([motor for motor, reg, value in writes if reg == 40 and value == 0], list(range(1, 7)))
        self.assertEqual(closed, [True])


if __name__ == "__main__":
    unittest.main()
