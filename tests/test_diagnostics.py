import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnostics import LoopMetrics
from video_watchdog import VideoWatchdog


class DiagnosticsTests(unittest.TestCase):
    def test_timing_windows_report_rate_percentiles_and_clamp_counts(self):
        now = [10.0]
        metrics = LoopMetrics("follower", clock=lambda: now[0])
        for i in range(1, 5):
            now[0] = 10 + i / 4
            report = metrics.record({"motor_cycle_ms": i}, {"relative_target.wrist_flex": 1})
        self.assertEqual(report["rate_hz"], 4)
        self.assertEqual(report["measurements"]["motor_cycle_ms"], {"mean": 2.5, "p95": 4, "max": 4})
        self.assertEqual(report["counters"]["relative_target.wrist_flex"], 4)
        now[0] = 12
        report = metrics.record({"motor_cycle_ms": 1})
        self.assertEqual(report["samples"], 1)
        self.assertEqual(report["counters"], {})

    def test_motor_thread_checks_cache_without_file_io_and_worker_stall_expires(self):
        now, wall = [10.0], [1_000_000_000]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "top.json"
            path.write_text(json.dumps({"frames": 1, "updated_unix_ns": wall[0]}))
            watchdog = VideoWatchdog([path], start=False, monotonic=lambda: now[0], wall_ns=lambda: wall[0])
            with patch("video_watchdog.video_deadlines", side_effect=AssertionError("I/O on motor thread")):
                for _ in range(100):
                    watchdog.check()
                now[0] = 10.501
                with self.assertRaisesRegex(RuntimeError, "stale"):
                    watchdog.check()
            watchdog.close()

    def test_camera_polls_cannot_refresh_a_frozen_frame(self):
        now, wall = [10.0], [1_000_000_000]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "wrist.json"
            path.write_text(json.dumps({"frames": 1, "updated_unix_ns": wall[0]}))
            watchdog = VideoWatchdog([path], start=False, monotonic=lambda: now[0], wall_ns=lambda: wall[0])
            now[0], wall[0] = 10.4, 1_400_000_000
            watchdog.refresh()
            watchdog.check()
            now[0] = 10.501
            with self.assertRaisesRegex(RuntimeError, "stale"):
                watchdog.check()
            watchdog.close()

    def test_only_a_new_camera_timestamp_extends_the_watchdog(self):
        now, wall = [10.0], [1_000_000_000]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "top.json"
            path.write_text(json.dumps({"frames": 1, "updated_unix_ns": wall[0]}))
            watchdog = VideoWatchdog([path], start=False, monotonic=lambda: now[0], wall_ns=lambda: wall[0])
            now[0], wall[0] = 10.1, 1_100_000_000
            path.write_text(json.dumps({"frames": 2, "updated_unix_ns": wall[0]}))
            watchdog.refresh()
            now[0] = 10.55
            watchdog.check()
            now[0] = 10.601
            with self.assertRaisesRegex(RuntimeError, "stale"):
                watchdog.check()
            watchdog.close()

    def test_missing_malformed_or_stale_status_fails_before_startup(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "wrist.json"
            for text in (None, "invalid json", json.dumps({"frames": 1, "updated_unix_ns": 0})):
                if text is not None:
                    path.write_text(text)
                with self.assertRaisesRegex(RuntimeError, "Camera"):
                    VideoWatchdog([path])


if __name__ == "__main__":
    unittest.main()
