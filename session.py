"""Run follower motors plus top/wrist Kyber video, or the MacBook operator."""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

from hardware import JOINTS, PositionController, calibration, require_video

ROOT = Path(__file__).resolve().parent
BIN = ROOT / "target/debug/so101-kyber-position"


def local(path):
    path = Path(path).expanduser()
    return path if path.is_absolute() else ROOT / path


class Children:
    def __init__(self, directory):
        self.directory = directory
        self.processes = []
        self.logs = []

    def start(self, name, command, stdin=None, pipe=False):
        log = (self.directory / f"{name}.log").open("wb")
        self.logs.append(log)
        process = subprocess.Popen([str(x) for x in command], cwd=ROOT, stdin=stdin,
            stdout=subprocess.PIPE if pipe else log, stderr=log)
        self.processes.append((name, process))
        return process

    def check(self):
        for name, process in self.processes:
            if process.poll() is not None:
                raise RuntimeError(f"{name} ended (exit {process.returncode}); see {self.directory / (name + '.log')}")

    def close(self):
        # The motor adapter gets SIGTERM first so it can verify torque-off before
        # the other processes disappear. Do not SIGKILL it or claim a confirmed stop.
        priority = {"motors": 0, "publisher": 1, "leader": 2, "commands": 3}
        ordered = sorted(self.processes, key=lambda item: priority.get(item[0], 4))
        confirmed = True
        for name, process in ordered:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                if name == "motors":
                    print("MOTOR STOP NOT CONFIRMED: use local power cutoff; motor process remains running", file=sys.stderr)
                    confirmed = False
                else:
                    process.kill()
                    process.wait()
            if name == "motors" and process.poll() is not None:
                text = (self.directory / "motors.log").read_text(errors="replace")
                if "Following enabled" in text and "stop_confirmed=true" not in text:
                    print("MOTOR STOP NOT CONFIRMED: check local power cutoff; see motors.log", file=sys.stderr)
                    confirmed = False
        for log in self.logs:
            log.close()
        return confirmed


def wait_video(paths, children, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        children.check()
        try:
            require_video(paths)
            return
        except (OSError, ValueError, KeyError, RuntimeError):
            time.sleep(0.05)
    raise TimeoutError("Both top and wrist video must be connected before command publishing starts")


def follower(args, children, runtime):
    config = json.loads(local(args.config).read_text())
    credentials = local(config.get("credentials", "credentials/server"))
    network = ["--insecure-lan"] if args.insecure_lan else []
    for name in (() if args.insecure_lan else ("ca.crt", "cert.pem", "key.pem")):
        if not (credentials / name).is_file():
            raise ValueError(f"Missing server credentials: {name}")
    cameras = config["cameras"]
    if set(cameras) != {"top", "wrist"}:
        raise ValueError("Configure both top and wrist cameras")
    ports = [config.get("command_port", 7445), *[camera["port"] for camera in cameras.values()]]
    if len(set(ports)) != len(ports):
        raise ValueError("Command, top and wrist ports must be different")
    if not args.preview_only and not args.simulate:
        if not (args.attended and args.enable_motion):
            raise ValueError("Motor operation requires --attended --enable-motion")
        lc = [{"range_min": 0, "range_max": 4095} for _ in JOINTS]
        fc = calibration(local(config["follower_calibration"]))
        limits = json.loads(local(config["limits"]).read_text())
        pose = [(limits["joints"][j]["min_ticks"] + limits["joints"][j]["max_ticks"]) // 2 for j in JOINTS]
        PositionController(limits, lc, fc, pose)
        if not Path(config["port"]).exists():
            raise ValueError("Configured follower serial path is not present")
    statuses = []
    for name, camera in cameras.items():
        device = camera.get("device")
        if not args.simulate and (not device or not Path(device).exists()):
            raise ValueError(f"Missing {name} camera device")
        capture = ["/usr/bin/python3", ROOT / "camera.py"]
        if not args.simulate:
            capture += ["--device", device]
        producer = children.start(f"{name}-capture", capture, pipe=True)
        status = runtime / f"{name}.json"
        statuses.append(status)
        address = f"{config['listen']}:{camera['port']}"
        children.start(f"{name}-server", [BIN, "camera-serve", "--listen", address,
            "--credentials", credentials, "--status-file", status] + network, stdin=producer.stdout)
        producer.stdout.close()
        print(f"{name} camera: {address}", flush=True)
    if args.preview_only:
        print("Preview only; motor device is not opened", flush=True)
        return None
    address = f"{config['listen']}:{config.get('command_port', 7445)}"
    receive = [BIN, "receive", "--listen", address, "--credentials", credentials,
        "--output", runtime / "positions.jsonl"] + network
    receiver = children.start("commands", receive, pipe=not args.simulate)
    if not args.simulate:
        command = [sys.executable, ROOT / "hardware.py", "drive", "--port", config["port"],
            "--config", local(config["limits"]), "--follower-calibration", local(config["follower_calibration"]),
            "--attended", "--enable-motion", "--startup-timeout", "3600"]
        for path in statuses:
            command += ["--video-status", path]
        children.start("motors", command, stdin=receiver.stdout)
        receiver.stdout.close()
    print(f"Command receiver: {address}; log directory: {runtime}", flush=True)
    return receiver


def operator(args, children, runtime):
    if shutil.which("gst-launch-1.0") is None:
        raise ValueError("Install GStreamer on the MacBook: brew install gstreamer")
    if not args.preview_only and not args.simulate:
        if not args.port:
            raise ValueError("Set --port to the MacBook leader USB device")
        calibration(local(args.calibration))
    credentials = local(args.credentials)
    network = ["--insecure-lan"] if args.insecure_lan else []
    statuses = []
    for name, port in (("top", args.top_port), ("wrist", args.wrist_port)):
        status = runtime / f"{name}.json"
        statuses.append(status)
        viewer = children.start(f"{name}-viewer", [BIN, "camera-view", "--remote", f"{args.host}:{port}",
            "--credentials", credentials, "--status-file", status] + network, pipe=True)
        display = ["gst-launch-1.0", "-q", "fdsrc", "!", "h264parse", "!", "avdec_h264", "!",
            "fakesink" if args.headless else "videoconvert"]
        if not args.headless:
            display += ["!", "autovideosink", f"name={name}", "sync=false"]
        children.start(f"{name}-display", display, stdin=viewer.stdout)
        viewer.stdout.close()
    print("Waiting for both top and wrist video", flush=True)
    wait_video(statuses, children)
    print("Both video streams are receiving frames", flush=True)
    if args.preview_only:
        return None, statuses
    command = [BIN, "publish", "--remote", f"{args.host}:{args.command_port}", "--credentials", credentials] + network
    if args.seconds:
        command += ["--seconds", str(args.seconds)]
    source = None
    if args.simulate:
        command += ["--simulate"]
    else:
        source = children.start("leader", [sys.executable, ROOT / "hardware.py", "leader",
            "--port", args.port, "--calibration", local(args.calibration)], pipe=True)
    publisher = children.start("publisher", command, stdin=source.stdout if source else None)
    if source:
        source.stdout.close()
    return publisher, statuses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    roles = parser.add_subparsers(dest="role", required=True)
    server = roles.add_parser("follower")
    server.add_argument("--config", default="config/follower.launch.local.json")
    server.add_argument("--attended", action="store_true")
    server.add_argument("--enable-motion", action="store_true")
    client = roles.add_parser("operator")
    client.add_argument("--host", required=True)
    client.add_argument("--credentials", default="credentials/leader")
    client.add_argument("--port")
    client.add_argument("--calibration", default="calibration/leader.json")
    client.add_argument("--top-port", type=int, default=7446)
    client.add_argument("--wrist-port", type=int, default=7447)
    client.add_argument("--command-port", type=int, default=7445)
    client.add_argument("--headless", action="store_true", help="Decode without windows for tests")
    client.add_argument("--seconds", type=float)
    for role in (server, client):
        role.add_argument("--insecure-lan", action="store_true", help="Private LAN only; no credentials or peer identity verification")
        role.add_argument("--preview-only", action="store_true")
        role.add_argument("--simulate", action="store_true", help="Synthetic devices; never opens motors")
    args = parser.parse_args()
    if args.preview_only and args.simulate:
        parser.error("Choose --preview-only or --simulate")
    if args.insecure_lan:
        print("LAN mode: encrypted but no peer identity verification; use a trusted private Wi-Fi network", flush=True)
    runtime = ROOT / "recordings" / (time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:6])
    runtime.mkdir(parents=True, mode=0o700)
    children = Children(runtime)
    def terminate(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    code = 0
    try:
        statuses = []
        if args.role == "follower":
            main_process = follower(args, children, runtime)
        else:
            main_process, statuses = operator(args, children, runtime)
        preview_start = time.monotonic()
        while True:
            if args.role == "operator" and args.preview_only and args.seconds and time.monotonic() - preview_start >= args.seconds:
                break
            if main_process and main_process.poll() == 0:
                if args.role == "follower":
                    # Motor input is already EOF. Let its verified stop finish,
                    # then retain preview briefly while the Mac drains QUIC close.
                    for name, process in children.processes:
                        if name == "motors":
                            process.wait(timeout=5)
                    time.sleep(1)
                break
            children.check()
            if statuses:
                require_video(statuses)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        code = 1
    finally:
        if not children.close():
            code = 1
    return code


if __name__ == "__main__":
    sys.exit(main())
