"""Read camera status away from the motor thread without extending freshness."""
import json
import threading
import time
from pathlib import Path

VIDEO_MAX_AGE_NS = 500_000_000


def video_deadlines(paths, *, monotonic=None, wall_ns=None):
    clock = monotonic or time.monotonic
    wall = wall_ns or time.time_ns
    deadlines = []
    for path in paths:
        data = json.loads(Path(path).read_text())
        # Anchor BEFORE sampling wall time: a slow read cannot extend the expiry.
        now = clock()
        age = wall() - data["updated_unix_ns"]
        if not (data["frames"] > 0 and 0 <= age <= VIDEO_MAX_AGE_NS):
            raise RuntimeError(f"Camera stream stale or unavailable: {Path(path).name}")
        deadlines.append((now + (VIDEO_MAX_AGE_NS - age) / 1e9, Path(path).name))
    return tuple(deadlines)


class VideoWatchdog:
    def __init__(self, paths, *, interval=.05, start=True, monotonic=None, wall_ns=None):
        self.paths = tuple(paths)
        self.clock = monotonic or time.monotonic
        self.wall = wall_ns or time.time_ns
        self.interval = interval
        self.stopped = threading.Event()
        self.thread = None
        self.snapshot = ((), None)
        self.refresh()
        self.check()  # Require real status before any torque-on.
        if self.paths and start:
            self.thread = threading.Thread(target=self._run, daemon=True, name="camera-watchdog")
            self.thread.start()

    def refresh(self):
        try:
            deadlines = video_deadlines(self.paths, monotonic=self.clock, wall_ns=self.wall)
            self.snapshot = (deadlines, None)
        except Exception as error:
            self.snapshot = ((), error)

    def _run(self):
        while not self.stopped.wait(self.interval):
            self.refresh()

    def check(self):
        # Immutable snapshot assignment is atomic under CPython. No filesystem,
        # locks, or waits on the motor thread, including when the worker stalls.
        deadlines, error = self.snapshot
        if error is not None:
            raise RuntimeError(f"Camera watchdog: {error}") from error
        now = self.clock()
        for deadline, name in deadlines:
            if now >= deadline:
                raise RuntimeError(f"Camera stream stale or unavailable: {name}")

    def close(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=.1)
