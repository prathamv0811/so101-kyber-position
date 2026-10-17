"""Full dual-video session test with synthetic cameras and simulated commands."""
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

INSECURE = "--insecure-lan" in sys.argv or "--tailscale" in sys.argv
HOST = os.environ.get("SESSION_TEST_HOST", "127.0.0.1")
NETWORK = ["--insecure-lan"] if INSECURE else []

RATE = int(os.environ.get("SESSION_TEST_HZ", "120"))

ROOT = Path(__file__).resolve().parents[1]


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((HOST, 0))
        return sock.getsockname()[1]


with tempfile.TemporaryDirectory(prefix="so101-dual-video-") as directory:
    temporary = Path(directory)
    credentials = temporary / "credentials"
    if not INSECURE:
        subprocess.run(["bash", "scripts/pair.sh", str(credentials)], cwd=ROOT, check=True, capture_output=True)
    top, wrist, command = free_port(), free_port(), free_port()
    config = temporary / "server.json"
    config.write_text(json.dumps({"listen": HOST, "credentials": str(credentials / "server"),
        "command_port": command, "cameras": {"top": {"port": top}, "wrist": {"port": wrist}}}))
    before = set((ROOT / "recordings").glob("*"))
    log = (temporary / "follower.log").open("w")
    follower = subprocess.Popen([sys.executable, "session.py", "follower", "--config", str(config), "--simulate", "--hz", str(RATE)] + NETWORK,
        cwd=ROOT, stdout=log, stderr=log)
    try:
        deadline = time.monotonic() + 5
        while "Command receiver" not in (temporary / "follower.log").read_text():
            if follower.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError((temporary / "follower.log").read_text())
            time.sleep(0.05)
        time.sleep(0.2)
        operator = subprocess.run([sys.executable, "session.py", "operator", "--host", HOST,
            "--credentials", str(credentials / "leader"), "--top-port", str(top), "--wrist-port", str(wrist),
            "--command-port", str(command), "--simulate", "--headless", "--seconds", "2", "--hz", str(RATE)] + NETWORK,
            cwd=ROOT, capture_output=True, text=True, timeout=20)
        assert operator.returncode == 0, operator.stdout + operator.stderr
        assert follower.wait(8) == 0, (temporary / "follower.log").read_text()
        runs = set((ROOT / "recordings").glob("*")) - before
        videos = []
        samples = 0
        metrics = []
        for run in runs:
            if not run.is_dir():
                continue
            for name in ("commands", "publisher"):
                path = run / f"{name}.log"
                if path.exists():
                    metrics.extend(json.loads(line.removeprefix("CONTROL_METRICS "))
                                   for line in path.read_text().splitlines() if line.startswith("CONTROL_METRICS "))
            for name in ("top", "wrist"):
                status = run / f"{name}.json"
                assert status.exists(), f"Missing {status}"
                count = json.loads(status.read_text())["frames"]
                assert count >= 20
                videos.append(count)
            positions = run / "actions.jsonl"
            if positions.exists():
                records = [json.loads(line) for line in positions.read_text().splitlines()]
                assert all(r["version"] == 3 and r["units"] == "lerobot_degrees_gripper_percent" and len(r["action"]) == 6 for r in records)
                samples += len(records)
                seconds = (records[-1]["received_unix_ns"] - records[0]["received_unix_ns"]) / 1e9
                measured_hz = (len(records) - 1) / seconds
                assert RATE * .9 <= measured_hz <= RATE * 1.1, measured_hz
        assert samples >= 30, samples
        assert len(videos) == 4, videos
        receiver_metrics = [m for m in metrics if m["role"] == "receiver"]
        publisher_metrics = [m for m in metrics if m["role"] == "publisher"]
        assert sum(m["samples"] for m in receiver_metrics) == samples
        assert receiver_metrics and publisher_metrics, metrics
        assert all(m["measurements"]["delivery_queue_ms"]["max"] >= 0 for m in receiver_metrics)
        assert all(m["measurements"]["source_age_ms"]["max"] <= 100 for m in publisher_metrics)
        print(json.dumps({"passed": True, "top_and_wrist_decoded": True,
            "server_and_client_video_frames": videos, "command_samples": samples, "measured_command_hz": round(measured_hz, 1),
            "timing_diagnostics_validated": True,
            "hardware_opened": False, "credential_files_required": not INSECURE}, indent=2))
    except Exception:
        print((temporary / "follower.log").read_text())
        for run in set((ROOT / "recordings").glob("*")) - before:
            if run.is_dir():
                for path in run.glob("*.log"):
                    print(str(path) + ": " + path.read_text(errors="replace")[-1800:])
        raise
    finally:
        if follower.poll() is None:
            follower.terminate()
            follower.wait(8)
        log.close()
