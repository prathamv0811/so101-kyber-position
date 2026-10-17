import json
import logging
import os
import sys
import tempfile
import time
import unittest
from argparse import Namespace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hardware import Bus, JOINTS, LeRobotControls, SampleReader, action_to_ticks, drive, require_video, ticks_to_action


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.cal = [{"range_min": 0, "range_max": 4095} for _ in JOINTS]
        self.config = {"limits_reviewed": True, "joints": {
            name: {"min_ticks": 100, "max_ticks": 4000, "max_ticks_per_second": 200, "invert": False}
            for name in JOINTS}}
        self.initial = ticks_to_action([2000] * 6, self.cal)
        # LeRobot reports each deliberately capped goal at warning level.
        self.log_patch = patch("lerobot.robots.utils.logging.warning")
        self.log_patch.start()
        self.addCleanup(self.log_patch.stop)

    def sample(self, seq, action):
        return {"version": 3, "units": "lerobot_degrees_gripper_percent", "seq": seq,
                "received_unix_ns": time.time_ns(), "action": action}

    def test_constant_absolute_leader_action_moves_without_relative_anchor(self):
        desired = ticks_to_action([3000] * 6, self.cal)
        with patch("hardware.time.monotonic", return_value=10):
            controls = LeRobotControls(self.config, self.cal, self.initial)
            self.assertEqual(controls.target(self.sample(1, desired), self.initial), self.initial)
        with patch("hardware.time.monotonic", return_value=10.04):
            target = controls.target(self.sample(2, desired), self.initial)
        ticks = action_to_ticks(target, self.cal)
        self.assertTrue(all(2000 < value <= 2008 for value in ticks))

    def test_60hz_goals_accumulate_and_keep_per_second_speed(self):
        for limit in self.config["joints"].values():
            limit["max_ticks_per_second"] = 100
        with patch("hardware.time.monotonic", return_value=10):
            controls = LeRobotControls(self.config, self.cal, self.initial)
            controls.target(self.sample(1, self.initial), self.initial)
        measured = dict(self.initial)
        previous = 2000
        for n in range(1, 61):
            with patch("hardware.time.monotonic", return_value=10 + n / 60):
                measured = controls.target(self.sample(n + 1, ticks_to_action([3000] * 6, self.cal)), measured)
            ticks = action_to_ticks(measured, self.cal)[0]
            self.assertTrue(previous <= ticks <= previous + 2)
            previous = ticks
            controls.sent(measured)
        self.assertEqual(previous, 2100)
        with patch("hardware.time.monotonic", return_value=11.02):
            with self.assertRaisesRegex(RuntimeError, "tracking error"):
                controls.target(self.sample(62, measured), self.initial)

    def test_stale_old_protocol_reordered_and_invalid_actions_fail(self):
        controls = LeRobotControls(self.config, self.cal, self.initial)
        controls.target(self.sample(1, self.initial), self.initial)
        with self.assertRaises(ValueError):
            controls.target(self.sample(1, self.initial), self.initial)
        for change in ({"version": 2}, {"received_unix_ns": time.time_ns() - 101_000_000},
                       {"action": {**self.initial, "gripper.pos": 101}},
                       {"action": {**self.initial, "wrist_roll.pos": float("nan")}}):
            sample = self.sample(2, self.initial)
            sample.update(change)
            with self.assertRaises(ValueError):
                controls.target(sample, self.initial)

    def test_unreviewed_limits_and_custom_inversion_refused(self):
        self.config["limits_reviewed"] = False
        with self.assertRaises(ValueError):
            LeRobotControls(self.config, self.cal, self.initial)
        self.config["limits_reviewed"] = True
        self.config["joints"][JOINTS[0]]["invert"] = True
        with self.assertRaises(ValueError):
            LeRobotControls(self.config, self.cal, self.initial)

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

    def test_stale_video_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            status = Path(directory) / "wrist.json"
            status.write_text(json.dumps({"frames": 10, "updated_unix_ns": time.time_ns() - 600_000_000}))
            with self.assertRaisesRegex(RuntimeError, "Camera stream stale"):
                require_video([status])

    def test_receiver_pipe_delivers_latest_complete_record_without_replaying_backlog(self):
        read_fd, write_fd = os.pipe()
        try:
            reader = SampleReader(read_fd)
            os.write(write_fd, b'{"seq":1}\n{"seq":2}\n{"seq":')
            self.assertEqual(reader.read(.1), {"seq": 2})
            os.write(write_fd, b'3}\n')
            self.assertEqual(reader.read(.1), {"seq": 3})
            with self.assertRaisesRegex(TimeoutError, "watchdog"):
                reader.read(.01)
            os.close(write_fd)
            write_fd = None
            with self.assertRaises(EOFError):
                reader.read(.1)
        finally:
            os.close(read_fd)
            if write_fd is not None:
                os.close(write_fd)

    def test_input_loss_after_enable_disables_all_motors(self):
        self.exercise_drive_fault()

    def test_video_loss_after_enable_disables_all_motors(self):
        self.exercise_drive_fault(camera_failure=True)

    def exercise_drive_fault(self, camera_failure=False):
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
        bus.get_observation = lambda: dict(self.initial)
        bus.require_passive = lambda: self.assertTrue(all(value == 0 for value in torque.values()))
        bus.arm = SimpleNamespace(config=SimpleNamespace(max_relative_target=None))
        def send(action):
            for motor, position in zip(bus.ids, action_to_ticks(action, self.cal)):
                write(motor, 42, position, 2)
            return action
        bus.send_action = send
        bus.close = lambda: closed.append(True)
        cal = [{"id": i + 1, "range_min": 0, "range_max": 4095, "homing_offset": 0,
                "drive_mode": 0} for i in range(6)]
        first = self.sample(1, self.initial)
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "limits.json"
            config.write_text(json.dumps(self.config))
            args = Namespace(config=config, port="fake", follower_calibration="fake",
                             enable_motion=True, attended=True, video_status=["fake-video"])
            reads = [first, self.sample(2, self.initial)] if camera_failure else [first, TimeoutError("Disconnected")]
            video = [None, RuntimeError("Camera stream stale")] if camera_failure else None
            with patch("hardware.Bus", return_value=bus), patch("hardware.calibration", return_value=cal), \
                 patch("hardware.read_sample", side_effect=reads), patch("hardware.require_video", side_effect=video):
                with self.assertRaises(RuntimeError if camera_failure else TimeoutError):
                    drive(args)
        self.assertTrue(all(value == 0 for value in torque.values()))
        self.assertEqual([motor for motor, reg, value in writes if reg == 40 and value == 0], list(range(1, 7)))
        self.assertEqual(closed, [True])


if __name__ == "__main__":
    unittest.main()
