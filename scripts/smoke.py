"""Actual-process integration checks. No serial or camera devices are opened."""
import json
import os
import signal
import socket
import subprocess
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / "target/debug/so101-kyber-position"


def port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return f"127.0.0.1:{sock.getsockname()[1]}"


def main():
    processes, handles = [], []
    with tempfile.TemporaryDirectory(prefix="so101-position-test-") as temporary:
        root = Path(temporary)
        creds = root / "credentials"
        subprocess.run(["bash", "scripts/pair.sh", str(creds)], cwd=ROOT, check=True, capture_output=True)

        def start(name, args, **kwargs):
            log = (root / (name + ".log")).open("wb")
            handles.append(log)
            process = subprocess.Popen(args, cwd=ROOT, stderr=log,
                stdout=kwargs.pop("stdout", subprocess.DEVNULL), **kwargs)
            processes.append(process)
            return process

        def receiver(name):
            address, output = port(), root / (name + ".jsonl")
            process = start(name, [BIN, "receive", "--listen", address,
                "--credentials", creds / "server", "--output", output])
            deadline = time.monotonic() + 3
            while "Listening" not in (root / (name + ".log")).read_text():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError((root / (name + ".log")).read_text())
                time.sleep(0.02)
            return address, process, output

        try:
            address, server, output = receiver("normal")
            client = start("normal-client", [BIN, "publish", "--remote", address,
                "--credentials", creds / "leader", "--simulate", "--seconds", "2"])
            assert client.wait(6) == 0, (root / "normal-client.log").read_text()
            assert server.wait(3) == 0, (root / "normal.log").read_text()
            records = [json.loads(line) for line in output.read_text().splitlines()]
            assert len(records) >= 40, len(records)
            assert [r["seq"] for r in records] == list(range(1, len(records) + 1))
            assert all(len(r["positions"]) == 6 and all(0 <= p <= 4095 for p in r["positions"]) for r in records)

            address, server, output = receiver("stale")
            client = start("stale-client", [BIN, "publish", "--remote", address,
                "--credentials", creds / "leader"], stdin=subprocess.PIPE)
            sample = {"captured_unix_ns": time.time_ns() - 1_000_000_000, "positions": [2048] * 6}
            client.communicate((json.dumps(sample) + "\n").encode(), timeout=5)
            assert client.returncode != 0
            server.wait(3)
            assert output.stat().st_size == 0

            address, server, output = receiver("watchdog")
            client = start("watchdog-client", [BIN, "publish", "--remote", address,
                "--credentials", creds / "leader", "--simulate"])
            time.sleep(0.6)
            assert client.poll() is None
            os.kill(client.pid, signal.SIGSTOP)
            assert server.wait(2) != 0
            assert "watchdog expired" in (root / "watchdog.log").read_text()
            client.kill()
            client.wait()

            wrong = root / "wrong-credentials"
            subprocess.run(["bash", "scripts/pair.sh", str(wrong)], cwd=ROOT, check=True, capture_output=True)
            # Trust the real server CA while presenting a client cert signed by
            # another CA: this specifically exercises server-side client rejection.
            (wrong / "leader/ca.crt").write_bytes((creds / "leader/ca.crt").read_bytes())
            address, server, output = receiver("untrusted")
            client = start("untrusted-client", [BIN, "publish", "--remote", address,
                "--credentials", wrong / "leader", "--simulate", "--seconds", "1"])
            assert client.wait(8) != 0
            server.wait(8)
            assert output.stat().st_size == 0

            # Real GStreamer test source -> appsink -> KyNet -> GStreamer decoder.
            address = port()
            producer = start("gst-producer", ["/usr/bin/python3", "camera.py", "--frames", "45"], stdout=subprocess.PIPE)
            camera_server = start("camera-server", [BIN, "camera-serve", "--listen", address,
                "--credentials", creds / "server"], stdin=producer.stdout)
            producer.stdout.close()
            time.sleep(0.1)
            video = root / "received.mjpeg"
            with video.open("wb") as stream:
                viewer = start("camera-viewer", [BIN, "camera-view", "--remote", address,
                    "--credentials", creds / "leader"], stdout=stream)
                assert viewer.wait(8) == 0, (root / "camera-viewer.log").read_text()
            assert camera_server.wait(5) == 0, (root / "camera-server.log").read_text()
            assert producer.wait(3) == 0, (root / "gst-producer.log").read_text()
            jpeg_count = video.read_bytes().count(b"\xff\xd8")
            assert jpeg_count >= 30, jpeg_count
            decoded = subprocess.run(["gst-launch-1.0", "-q", "filesrc", f"location={video}",
                "!", "jpegparse", "!", "jpegdec", "!", "fakesink"], capture_output=True, timeout=5)
            assert decoded.returncode == 0, decoded.stderr.decode()
            print(json.dumps({"passed": True, "sdk": "0.29.0", "position_samples": len(records),
                "six_joints_atomic": True, "stale_samples_rejected": True,
                "publisher_loss_watchdog": True, "untrusted_peer_rejected": True,
                "gstreamer_video_frames": jpeg_count, "gstreamer_decode": True,
                "hardware_opened": False}, indent=2))
        except Exception:
            for log in root.glob("*.log"):
                print(f"{log.name}: {log.read_text(errors='replace')}")
            raise
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    process.kill()
                process.wait()
            for handle in handles:
                handle.close()


if __name__ == "__main__":
    main()
