"""A failed initial camera connection must not take down the waiting listener."""
import os
import socket
import subprocess
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / 'target/debug/so101-kyber-position'
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind(('127.0.0.1', 0))
    address = f'127.0.0.1:{sock.getsockname()[1]}'
with tempfile.TemporaryDirectory(prefix='so101-camera-startup-') as directory:
    temporary = Path(directory)
    processes, handles = [], []
    def start(name, command, **kwargs):
        log = (temporary / f'{name}.log').open('wb')
        handles.append(log)
        process = subprocess.Popen([str(x) for x in command], cwd=ROOT,
            stdout=kwargs.pop('stdout', subprocess.DEVNULL), stderr=log, **kwargs)
        processes.append(process)
        return process
    try:
        producer = start('capture', ['/usr/bin/python3', 'camera.py', '--frames', '150'], stdout=subprocess.PIPE)
        server = start('server', [BIN, 'camera-serve', '--listen', address, '--tailscale'], stdin=producer.stdout)
        producer.stdout.close()
        deadline = time.monotonic() + 3
        while 'listening' not in (temporary / 'server.log').read_text():
            assert server.poll() is None, (temporary / 'server.log').read_text()
            assert time.monotonic() < deadline
            time.sleep(.02)
        # Motor-action ALPN sent to the video port must be rejected without
        # consuming the listener's one successful camera session.
        wrong = start('wrong-protocol', [BIN, 'publish', '--remote', address,
            '--tailscale', '--simulate', '--seconds', '.1'])
        assert wrong.wait(5) != 0
        assert server.poll() is None, (temporary / 'server.log').read_text()
        video = temporary / 'received.h264'
        with video.open('wb') as stream:
            viewer = start('viewer', [BIN, 'camera-view', '--remote', address, '--tailscale'], stdout=stream)
            assert viewer.wait(10) == 0, (temporary / 'viewer.log').read_text()
        assert server.wait(3) == 0, (temporary / 'server.log').read_text()
        assert producer.wait(3) == 0
        result = subprocess.run(['/usr/bin/python3', 'scripts/count-video.py', str(video)],
            cwd=ROOT, capture_output=True, text=True, timeout=5)
        assert result.returncode == 0, result.stderr
        frames = int(result.stdout)
        assert frames >= 60, frames
        assert 'waiting for another client' in (temporary / 'server.log').read_text()
        print(f'PASS: initial wrong-protocol connection rejected, listener stayed available, {frames} frames decoded')
    except Exception:
        for path in temporary.glob('*.log'):
            print(str(path) + ':\n' + path.read_text(errors='replace')[-2500:])
        raise
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(3)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        for handle in handles:
            handle.close()
