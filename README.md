# LeRobot SO-101 remote teleoperation with Kyber

LeRobot supplies SO-101 control and calibration. Kyber supplies the client/server
transport for native motor actions and two GStreamer video feeds. The leader is
on the MacBook; the follower, Logitech top camera, and Sonix wrist camera are on
the Ubuntu laptop. Control defaults to **60 Hz**, video to **30 fps**.

```text
Mac: SO101Leader.get_action()
  → Kyber client (KyNet QUIC, native .pos actions)
  → Tailscale or private LAN
  → Kyber server → SO101Follower.send_action() → follower motors

Follower cameras → GStreamer H.264 → KyProto GOP video → Mac video windows
```

The six action keys are `shoulder_pan.pos`, `shoulder_lift.pos`, `elbow_flex.pos`,
`wrist_flex.pos`, `wrist_roll.pos`, and `gripper.pos`. With LeRobot
`use_degrees=True`, the five body joints use **degrees** and the gripper uses
**0..100**. The network preserves these native values as 64-bit floats. There is
no relative leader anchor or gamepad-axis mapping. The follower approaches the
leader's absolute pose under reviewed bounds and speed limits.

## Setup

The repository is public: https://github.com/prathamv0811/so101-kyber-position.
Install `git`, Rust/Cargo, `uv`, a C compiler, and Python. On Apple Silicon macOS:

```bash
brew install rust uv pkg-config gstreamer
git clone https://github.com/prathamv0811/so101-kyber-position.git
cd so101-kyber-position
python3 scripts/setup.py
```

For an existing checkout, run `git pull` and `python3 scripts/setup.py` on both
computers. Build on each machine; a Linux executable does not run on macOS.
Setup creates a Python 3.12 environment pinned to `lerobot[feetech]==0.6.1`.
Ubuntu uses CPU PyTorch. Setup fetches the exact Kyber revisions in
[SDK.lock.json](SDK.lock.json), corresponding to components tagged 0.29.0.
It builds the official `kynet`, `kyproto`, and `kymux-types` libraries directly.

The current application does not use `kynput` gamepad emulation for robot
commands: kynput has no native SO-101 action payload. Native LeRobot actions use
Kyber's KyNet transport. Video uses the official KyProto GOP stream, not a custom
video socket or LeRobot camera transport. The full Kyber desktop app and
`kyavservice` executable are not required.

On Ubuntu, the producer uses `/usr/bin/python3` with GI/GStreamer, separately from
the LeRobot environment. It needs `python3-gi`, `gir1.2-gstreamer-1.0`,
`gstreamer1.0-tools`, plugins base/good/bad/ugly, and libav. Setup prepares a private
H.264 parser plugin when needed. On macOS, Homebrew GStreamer supplies the decoder.

## Calibration

Run LeRobot calibration locally on each supported, attended arm:

```bash
# Mac leader: replace the USB path.
.venv/bin/python calibrate.py leader --port /dev/cu.usbmodemYOUR_DEVICE \
  --id leader --directory calibration --attended

# Ubuntu follower: replace the serial path when preparing another machine.
.venv/bin/python calibrate.py follower --port /dev/serial/by-id/YOUR_FOLLOWER \
  --id follower --directory calibration --attended
```

The wrapper invokes official `lerobot-calibrate`; follow its midpoint/range
prompts. Matching existing calibration may be reused. This laptop's existing
`follower_2` calibration was verified against all six motors and is stored
privately as `calibration/follower.json`. The Mac keeps its own
`calibration/leader.json`; calibration files do not need transferring.

Copy `config/follower.launch.example.json` to
`config/follower.launch.local.json` and configure actual follower/camera paths.
Copy `config/follower.example.json` to `follower.local.json`, replace placeholder
bounds and speeds with reviewed limits inside the follower calibration, then
set `limits_reviewed=true`. Native SO-101 actions require `invert=false`.

## Start remote teleoperation

Connect both computers to the same authorized Tailscale network. This laptop's
Tailscale address is **100.79.3.30**. See the [Tailscale guide](docs/tailscale.md)
for Mac setup, reachability checks, and port access.

On the follower laptop, with the supported arm attended at its power cutoff:

```bash
source scripts/build-env.sh
.venv/bin/python session.py follower --listen-host 100.79.3.30 \
  --tailscale --hz 60 --attended --enable-motion
```

On the Mac, replace the USB path:

```bash
.venv/bin/python session.py operator --host 100.79.3.30 --tailscale --hz 60 \
  --port /dev/cu.usbmodemYOUR_DEVICE --calibration calibration/leader.json
```

Both video windows start before the client publishes leader actions. The
follower waits torque-off for fresh input and both streams, preloads its current
pose, enables torque explicitly, and approaches the absolute leader pose at the
configured speed. `SO101Follower.send_action()` performs the motor writes and
applies LeRobot's `max_relative_target` guard. A freshness/limits wrapper uses
LeRobot's `ensure_safe_goal_position` to preserve the reviewed per-second speed
limits, including sub-tick progress at 60 Hz.

UDP ports are **7445** for actions, **7446** for top video, and **7447** for wrist
video. `--tailscale` requires no application credential files: access relies on
the tailnet connection and policy. QUIC remains encrypted, but this mode does
not independently authenticate application peers. For paired mutual TLS, omit
`--tailscale` and provision credentials with `bash scripts/pair.sh credentials`;
transfer only `credentials/leader/` to the Mac. Direct Wi-Fi LAN also works using
`--insecure-lan`; see [LAN guide](docs/macbook-lan.md).

Use `--preview-only` on both roles for cameras without opening motors. Use
`--simulate` on both roles for synthetic cameras/actions without hardware.
Never connect a simulated publisher to a physical follower.

## Stop, diagnostics, and verification

Ctrl+C stops the session. Missing commands, stale video, malformed input, or a
process failure stop following and attempt torque-off on all six motors, followed
by register verification. Support the follower during torque-off; an unconfirmed
stop requires local power isolation. There is no automatic motion resume.
The listener handles one operator session; restart explicitly after it ends.

Private run directories under `recordings/` contain `motors.log`, camera/network
logs, and `actions.jsonl`. Motor logs report actual update rate, input action,
measured observation, and target once per second. Journals contain received
leader actions, not measured follower telemetry or proof of execution. Actual
rate depends on Wi-Fi/Tailscale, USB, and scheduling; 60 Hz is the requested rate.

```bash
source scripts/build-env.sh
cargo test --locked
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python scripts/smoke.py
.venv/bin/python scripts/test-camera-startup.py
.venv/bin/python scripts/test-session.py
.venv/bin/python scripts/test-session.py --insecure-lan
```

Tests cover native units, exact action transport, absolute pose behavior, 60 Hz
sampling, source freshness, authentication, connection-loss stop, camera-loss
stop, both video feeds, and real LeRobot methods with only hardware I/O mocked.
Physical movement/stop performance and the Mac-to-laptop route still need local
commissioning. See [LeRobot integration](docs/lerobot-integration.md).

## Action protocol v3

ALPN is `so101-lerobot-action-v3`; older v1/v2 clients are rejected. A 72-byte
frame contains challenge, sequence, capture Unix nanoseconds (three big-endian
`u64`s), then six big-endian IEEE-754 `f64`s in the joint order above. The receiver
reconstructs the exact `.pos` action dictionary. Nonfinite values, body values
outside -360..360, and gripper values outside 0..100 are rejected; the follower
also enforces its own stricter calibrated/reviewed bounds.

Each server challenge has a 200 ms response deadline. The client keeps only the
newest sample and rejects input over 100 ms old on its own clock. The follower
rejects receiver output over 100 ms old on its clock and stops after 200 ms
without input. Cross-host synchronized clocks are not required for these checks.
A frame echoing the current challenge with all other bytes zero ends a session.
These software checks do not establish hardware stop performance.

See [LICENSE](LICENSE) for AGPL-3.0-or-later; upstream components retain their own
license notices.

Camera setup errors now name the failed stage. Failed initial camera setup
leaves its listener waiting for a new explicit client attempt; established video
or motor sessions still stop on failure. The Mac launcher prints the failing
child process’s log tail in Terminal.
