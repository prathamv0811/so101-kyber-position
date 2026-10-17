"""LeRobot SO-101 adapters with Kyber carrying all remote commands.

Joint order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.
Motor I/O uses servo ticks; network positions are calibrated fractions 0..4095.
"""
import argparse
import json
import math
import select
import signal
import sys
import time
from pathlib import Path

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def normalized_positions(raw, cal):
    return [round(4095 * (position - motor["range_min"]) / (motor["range_max"] - motor["range_min"]))
            for position, motor in zip(raw, cal)]


class Bus:
    REGISTERS = {3: "Model_Number", 31: "Homing_Offset", 33: "Operating_Mode",
                 40: "Torque_Enable", 42: "Goal_Position", 56: "Present_Position"}

    def __init__(self, port, ids, calibration_path, role):
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
        path = Path(calibration_path).resolve()
        if ids != list(range(1, 7)):
            raise ValueError("LeRobot SO-101 requires joint motor IDs 1 through 6")
        options = {"port": port, "id": path.stem, "calibration_dir": path.parent}
        self.arm = (SO101Leader(SO101LeaderConfig(**options)) if role == "leader"
                    else SO101Follower(SO101FollowerConfig(**options, cameras={})))
        self.driver = self.arm.bus
        self.ids = ids
        # Open the official bus without SOFollower.configure(), whose torque
        # context would otherwise enable torque before our explicit motion gate.
        try:
            self.driver.connect()
            if not self.driver.is_calibrated:
                raise RuntimeError("LeRobot calibration differs from hardware; run calibrate.py locally")
        except BaseException:
            self.driver.port_handler.closePort()
            raise

    def name(self, motor):
        return JOINTS[self.ids.index(motor)]

    def read(self, motor, address, size):
        return self.driver.read(self.REGISTERS[address], self.name(motor), normalize=False, num_retry=2)

    def write(self, motor, address, value, size=1):
        self.driver.write(self.REGISTERS[address], self.name(motor), value, normalize=False, num_retry=2)

    def positions(self):
        result = self.driver.sync_read("Present_Position", normalize=False, num_retry=2)
        values = [result[name] for name in JOINTS]
        if any(not 0 <= value <= 4095 for value in values):
            raise RuntimeError("Position outside single-turn range")
        return values

    def send_positions(self, positions):
        self.driver.sync_write("Goal_Position", dict(zip(JOINTS, positions)), normalize=False, num_retry=2)

    def require_passive(self):
        if any(self.read(motor, 40, 1) != 0 for motor in self.ids):
            raise RuntimeError("Torque is enabled; support the arm and disable it locally first")

    def stop(self):
        errors = []
        # Try ALL motors, including when an earlier motor fails communication.
        for motor in self.ids:
            try:
                self.write(motor, 40, 0)
            except Exception as exc:
                errors.append(str(exc))
        for motor in self.ids:
            try:
                if self.read(motor, 40, 1) != 0:
                    errors.append(f"Motor {motor} torque remains enabled")
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError("STOP NOT CONFIRMED; use local power cutoff: " + "; ".join(errors))

    def close(self):
        self.driver.disconnect(disable_torque=False)  # stop() verifies all torque registers first


def calibration(path):
    data = json.loads(Path(path).read_text())
    if set(data) != set(JOINTS):
        raise ValueError("Calibration must name exactly the six SO-101 joints")
    result = [data[name] for name in JOINTS]
    ids = []
    for item in result:
        if set(item) != {"id", "drive_mode", "homing_offset", "range_min", "range_max"}:
            raise ValueError("Unexpected calibration fields")
        if any(type(v) is not int for v in item.values()):
            raise ValueError("Calibration values must be integers")
        if not (1 <= item["id"] <= 253 and item["drive_mode"] == 0
                and -2047 <= item["homing_offset"] <= 2047
                and 0 <= item["range_min"] < item["range_max"] <= 4095):
            raise ValueError("Invalid STS3215 calibration")
        ids.append(item["id"])
    if len(set(ids)) != 6:
        raise ValueError("Duplicate motor IDs")
    return result


def verify_calibration(bus, cal):
    for motor in cal:
        # Do not change servo configuration. A mismatch requires local calibration.
        offset = bus.read(motor["id"], 31, 2)  # LeRobot decodes sign-magnitude
        if offset != motor["homing_offset"] or bus.read(motor["id"], 33, 1) != 0:
            raise RuntimeError(f"Motor {motor['id']}: saved offset/mode doesn't match hardware")
        if bus.read(motor["id"], 3, 2) != 777:
            raise RuntimeError(f"Motor {motor['id']}: expected STS3215 model 777")


def leader(args):
    cal = calibration(args.calibration)
    bus = Bus(args.port, [m["id"] for m in cal], args.calibration, "leader")
    try:
        bus.require_passive()
        verify_calibration(bus, cal)
        period = 1 / args.hz
        deadline = time.monotonic()
        while True:
            captured = time.time_ns()  # includes serial acquisition time in freshness checks
            bus.require_passive()
            positions = bus.positions()
            if any(not m["range_min"] <= p <= m["range_max"] for m, p in zip(cal, positions)):
                raise RuntimeError("Leader outside its calibrated range")
            print(json.dumps({"captured_unix_ns": captured, "positions": normalized_positions(positions, cal)}), flush=True)
            deadline = max(deadline + period, time.monotonic())
            time.sleep(max(0, deadline - time.monotonic()))
    finally:
        bus.close()  # Reader never changes torque, offsets or mode.


class PositionController:
    def __init__(self, config, leader_cal, follower_cal, measured):
        if config.get("limits_reviewed") is not True:
            raise ValueError("Review physical limits before setting limits_reviewed=true")
        self.leader_cal = leader_cal
        self.follower_cal = follower_cal
        self.limits = config["joints"]
        if set(self.limits) != set(JOINTS):
            raise ValueError("Provide limits for all six joints")
        for i, name in enumerate(JOINTS):
            limit = self.limits[name]
            lo, hi = limit["min_ticks"], limit["max_ticks"]
            speed = limit["max_ticks_per_second"]
            if not (type(lo) is int and type(hi) is int
                    and follower_cal[i]["range_min"] <= lo < hi <= follower_cal[i]["range_max"]
                    and lo <= measured[i] <= hi and type(speed) in (int, float)
                    and math.isfinite(speed) and 0 < speed <= 1000
                    and type(limit["invert"]) is bool):
                raise ValueError(f"Invalid physical limits for {name}")
        self.origin = list(measured)
        self.anchor = None
        self.previous_seq = 0
        self.last = time.monotonic()

    def target(self, sample, measured):
        now = time.monotonic()
        age = time.time_ns() - sample["received_unix_ns"]
        if sample["version"] != 2 or not 0 <= age <= 100_000_000:
            raise ValueError("Stale or unsupported receiver sample")
        if type(sample["seq"]) is not int or sample["seq"] <= self.previous_seq:
            raise ValueError("Duplicate/reordered sample")
        positions = sample["positions"]
        if len(positions) != 6 or any(type(p) is not int or not m["range_min"] <= p <= m["range_max"]
                                     for p, m in zip(positions, self.leader_cal)):
            raise ValueError("Leader sample outside calibrated limits")
        if self.anchor is None:
            self.anchor = list(positions)
        targets = []
        dt = min(now - self.last, 0.05)  # Scheduling delays never permit a large catch-up step.
        for i, name in enumerate(JOINTS):
            limit, lc, fc = self.limits[name], self.leader_cal[i], self.follower_cal[i]
            if not limit["min_ticks"] <= measured[i] <= limit["max_ticks"]:
                raise ValueError(f"Follower {name} outside reviewed limits")
            ratio = (positions[i] - self.anchor[i]) / (lc["range_max"] - lc["range_min"])
            if limit["invert"]:
                ratio = -ratio
            desired = self.origin[i] + ratio * (fc["range_max"] - fc["range_min"])
            desired = max(limit["min_ticks"], min(limit["max_ticks"], desired))
            step = limit["max_ticks_per_second"] * dt
            # floor avoids rounding a sub-tick bound up into excessive movement.
            step = math.floor(step)
            targets.append(round(max(measured[i] - step, min(measured[i] + step, desired))))
        self.previous_seq = sample["seq"]
        self.last = now
        return targets


def read_sample(timeout):
    # Use unbuffered OS reads, so select cannot miss data prefetched by TextIOWrapper.
    import os
    deadline = time.monotonic() + timeout
    line = bytearray()
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([sys.stdin.fileno()], [], [], remaining)[0]:
            raise TimeoutError("Position input watchdog expired")
        byte = os.read(sys.stdin.fileno(), 1)
        if not byte:
            raise EOFError("Receiver disconnected")
        if byte == b"\n":
            return json.loads(line)
        line.extend(byte)
        if len(line) > 2048:
            raise ValueError("Oversized receiver record")


def first_sample(timeout=30):
    """Startup happens torque-off; skip queued old records before enable."""
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("No fresh leader sample during startup")
        sample = read_sample(remaining)
        age = time.time_ns() - sample["received_unix_ns"]
        if 0 <= age <= 100_000_000:
            return sample
        if age < 0:
            raise ValueError("Receiver timestamp is in the future")


def require_video(paths):
    for path in paths:
        data = json.loads(Path(path).read_text())
        age = time.time_ns() - data["updated_unix_ns"]
        if not (data["frames"] > 0 and 0 <= age <= 500_000_000):
            raise RuntimeError(f"Camera stream stale or unavailable: {Path(path).name}")


def drive(args):
    if not (args.enable_motion and args.attended):
        raise ValueError("Physical operation requires --attended --enable-motion")
    config = json.loads(Path(args.config).read_text())
    # The leader normalizes against its own LeRobot calibration before sending.
    # The follower needs only its own physical calibration, not a file from the Mac.
    lc = [{"range_min": 0, "range_max": 4095} for _ in JOINTS]
    fc = calibration(args.follower_calibration)
    # Validate reviewed config before opening any hardware.
    if config.get("limits_reviewed") is not True:
        raise ValueError("Physical limits have not been reviewed")
    validation_pose = [(config["joints"][name]["min_ticks"] + config["joints"][name]["max_ticks"]) // 2
                       for name in JOINTS]
    PositionController(config, lc, fc, validation_pose)
    bus = Bus(args.port, [m["id"] for m in fc], args.follower_calibration, "follower")
    touched = False
    try:
        bus.require_passive()
        verify_calibration(bus, fc)
        startup = getattr(args, "startup_timeout", 30)
        print(f"Follower ready; waiting up to {startup:g} seconds for the leader (torque off)", file=sys.stderr)
        first = first_sample(startup)
        initial = bus.positions()
        controller = PositionController(config, lc, fc, initial)
        controller.target(first, initial)  # Validate fresh input and anchor before torque.
        require_video(getattr(args, "video_status", []))
        touched = True
        for motor, position in zip(bus.ids, initial):
            bus.write(motor, 42, position, 2)  # preload present position before torque-on
        for motor in bus.ids:
            bus.write(motor, 40, 1)
        if any(bus.read(motor, 40, 1) != 1 for motor in bus.ids):
            raise RuntimeError("Torque enable could not be verified")
        print("Following enabled; Ctrl+C stops and disables torque", file=sys.stderr)
        while True:
            sample = read_sample(0.2)
            targets = controller.target(sample, bus.positions())
            require_video(getattr(args, "video_status", []))
            # A sample carries a server challenge bounded to 200 ms. Each hardware
            # cycle must finish within 100 ms; a local read/write fault ends the session.
            started = time.monotonic()
            bus.send_positions(targets)
            if time.monotonic() - started > 0.1:
                raise TimeoutError("Motor write cycle exceeded 100 ms")
    finally:
        try:
            if touched:
                bus.stop()
                print("stop_confirmed=true", file=sys.stderr)
        finally:
            bus.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    read = commands.add_parser("leader", help="Read a passive leader and emit position NDJSON")
    read.add_argument("--port", required=True)
    read.add_argument("--calibration", required=True)
    read.add_argument("--hz", type=int, choices=range(10, 121), default=30, metavar="10..120")
    follow = commands.add_parser("drive", help="Consume receiver NDJSON and follow with bounded relative motion")
    follow.add_argument("--port", required=True)
    follow.add_argument("--config", required=True)
    follow.add_argument("--leader-calibration", help="Legacy argument; v2 calibration remains on the leader")
    follow.add_argument("--follower-calibration", required=True)
    follow.add_argument("--attended", action="store_true")
    follow.add_argument("--enable-motion", action="store_true")
    follow.add_argument("--video-status", type=Path, action="append", default=[],
                        help="Require each local camera status file fresh within 500 ms")
    follow.add_argument("--startup-timeout", type=float, default=30,
                        help="Torque-off wait for first input; live watchdog remains 200 ms")
    args = parser.parse_args()
    # SIGTERM follows the same torque-off cleanup as Ctrl+C.
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        (leader if args.command == "leader" else drive)(args)
    except (KeyboardInterrupt, BrokenPipeError):
        return
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
