"""Small, bounded timing windows for teleoperation diagnostics."""
import math
import statistics
import time
from collections import Counter, defaultdict, deque


class LoopMetrics:
    def __init__(self, role, clock=None, interval=1.0):
        self.role = role
        self.clock = clock or time.monotonic
        self.started = self.clock()
        self.interval = interval
        self.samples = 0
        self.values = defaultdict(lambda: deque(maxlen=4096))
        self.counters = Counter()

    def record(self, values, counters=None):
        self.samples += 1
        for key, value in values.items():
            self.values[key].append(value)
        self.counters.update(counters or {})
        now = self.clock()
        elapsed = now - self.started
        if elapsed < self.interval:
            return None
        measurements = {}
        for key, values in self.values.items():
            ordered = sorted(values)
            measurements[key] = {
                "mean": round(statistics.fmean(values), 3),
                "p95": round(ordered[math.ceil(.95 * len(ordered)) - 1], 3),
                "max": round(ordered[-1], 3),
            }
        report = {"role": self.role, "samples": self.samples,
                  "rate_hz": round(self.samples / elapsed, 2), "interval_s": round(elapsed, 3),
                  "measurements": measurements, "counters": dict(self.counters)}
        self.started, self.samples = now, 0
        self.values.clear()
        self.counters.clear()
        return report
