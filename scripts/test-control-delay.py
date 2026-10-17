"""Check command rate through a delayed UDP proxy, without opening robot hardware."""
import heapq
import json
import selectors
import socket
import subprocess
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / "target/debug/so101-kyber-position"
RATE = 120
ONE_WAY_DELAY = .03


class DelayedLink:
    def __init__(self, server):
        self.server, self.client = server, None
        self.front = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.back = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.front.bind(("127.0.0.1", 0))
        self.back.bind(("127.0.0.1", 0))
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        pending, order = [], 0
        with selectors.DefaultSelector() as selector:
            selector.register(self.front, selectors.EVENT_READ)
            selector.register(self.back, selectors.EVENT_READ)
            while not self.stop.is_set():
                now = time.monotonic()
                while pending and pending[0][0] <= now:
                    _, _, sender, data, peer = heapq.heappop(pending)
                    sender.sendto(data, peer)
                timeout = min(.005, max(0, pending[0][0] - now)) if pending else .005
                for event, _ in selector.select(timeout):
                    data, peer = event.fileobj.recvfrom(65536)
                    if event.fileobj is self.front:
                        self.client = peer
                        sender, destination = self.back, self.server
                    else:
                        if self.client is None:
                            continue
                        sender, destination = self.front, self.client
                    order += 1
                    heapq.heappush(pending, (time.monotonic() + ONE_WAY_DELAY,
                                           order, sender, data, destination))

    def close(self):
        self.stop.set()
        self.thread.join(2)
        self.front.close()
        self.back.close()


def main():
    with tempfile.TemporaryDirectory(prefix="so101-delayed-control-") as temporary:
        directory = Path(temporary)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
            reserve.bind(("127.0.0.1", 0))
            server_address = reserve.getsockname()
        link = DelayedLink(server_address)
        server_log = (directory / "server.log").open("w")
        client_log = (directory / "client.log").open("w")
        output = directory / "actions.jsonl"
        server = subprocess.Popen([BIN, "receive", "--listen", "%s:%s" % server_address,
            "--insecure-lan", "--output", output, "--hz", str(RATE)], cwd=ROOT,
            stdout=subprocess.DEVNULL, stderr=server_log)
        client = None
        try:
            deadline = time.monotonic() + 3
            while "Listening" not in (directory / "server.log").read_text():
                if server.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError((directory / "server.log").read_text())
                time.sleep(.02)
            link.thread.start()
            client = subprocess.Popen([BIN, "publish", "--remote", "%s:%s" % link.front.getsockname(),
                "--insecure-lan", "--simulate", "--seconds", "3", "--hz", str(RATE)],
                cwd=ROOT, stdout=subprocess.DEVNULL, stderr=client_log)
            assert client.wait(10) == 0, (directory / "client.log").read_text()
            assert server.wait(4) == 0, (directory / "server.log").read_text()
            records = [json.loads(line) for line in output.read_text().splitlines()]
            duration = (records[-1]["received_unix_ns"] - records[0]["received_unix_ns"]) / 1e9
            hz = (len(records) - 1) / duration
            assert hz >= RATE * .85, f"Round-trip-limited command delivery: {hz:.1f} Hz"
            assert all(a["seq"] < b["seq"] for a, b in zip(records, records[1:]))
            assert all(r["challenge_age_ms"] <= 200 for r in records)
            print(json.dumps({"passed": True, "added_network_rtt_ms": ONE_WAY_DELAY * 2000,
                "measured_command_hz": round(hz, 1), "command_samples": len(records),
                "freshness_deadline_ms": 200, "hardware_opened": False}, indent=2))
        except Exception:
            print((directory / "client.log").read_text())
            print((directory / "server.log").read_text())
            raise
        finally:
            for process in (client, server):
                if process is not None and process.poll() is None:
                    process.kill()
                    process.wait()
            if link.thread.ident is not None:
                link.close()
            server_log.close()
            client_log.close()


if __name__ == "__main__":
    main()
