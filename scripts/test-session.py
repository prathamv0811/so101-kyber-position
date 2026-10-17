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
    follower = subprocess.Popen([sys.executable, "session.py", "follower", "--config", str(config), "--simulate", "--hz", "60"] + NETWORK,
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
            "--command-port", str(command), "--simulate", "--headless", "--seconds", "2", "--hz", "60"] + NETWORK,
            cwd=ROOT, capture_output=True, text=True, timeout=20)
        assert operator.returncode == 0, operator.stdout + operator.stderr
        assert follower.wait(8) == 0, (temporary / "follower.log").read_text()
        runs = set((ROOT / "recordings").glob("*")) - before
        videos = []
        samples = 0
        for run in runs:
            if not run.is_dir():
                continue
            for name in ("top", "wrist"):
                status = run / f"{name}.json"
                assert status.exists(), f"Missing {status}"
                count = json.loads(status.read_text())["frames"]
                assert count >= 20
                videos.append(count)
            positions = run / "positions.jsonl"
            if positions.exists():
                records = [json.loads(line) for line in positions.read_text().splitlines()]
                samples += len(records)
                seconds = (records[-1]["received_unix_ns"] - records[0]["received_unix_ns"]) / 1e9
                measured_hz = (len(records) - 1) / seconds
                assert 55 <= measured_hz <= 65, measured_hz
        assert samples >= 30, samples
        assert len(videos) == 4, videos
        print(json.dumps({"passed": True, "top_and_wrist_decoded": True,
            "server_and_client_video_frames": videos, "command_samples": samples, "measured_command_hz": round(measured_hz, 1),
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
