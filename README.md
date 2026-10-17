# SO-101 Kyber position controller

A fresh project, independent of Tether. A passive SO-101 leader publishes all
six measured motor positions through **kynput 0.29.0 input types** and the current
**KyNet QUIC transport**. A server records those positions and can feed an
explicitly enabled SO-101 follower. GStreamer supplies an optional camera feed.

**Current deployment: leader on a MacBook, follower on this Ubuntu laptop,
connected over the same Wi-Fi LAN.** Start with the
[MacBook-to-Ubuntu LAN guide](docs/macbook-lan.md). The laptop's observed LAN
address is `192.168.101.50`; confirm it before connecting.

```
Leader USB → LeRobot reader → kynput six-axis packets → mTLS/KyNet QUIC
                                                        ↓
                                         server JSONL recorder
                                                        ↓ optional
                                      bounded relative position controller
                                                        ↓
                                                   Follower USB

Camera → GStreamer H.264 → KyProto GOP video channel → decoder on MacBook
```

Joint order is `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`,
`wrist_roll`, `gripper`. Values are **integer servo ticks (0–4095)**, not radians,
degrees, or Cartesian wrist poses. LeRobot acquires the six motors with a synchronized bus read;
the timestamp precedes acquisition and all six readings travel in one frame.

## SDK and installation

The SDK's current main revision was checked on 2026-10-17. Exact revisions are in
[SDK.lock.json](SDK.lock.json); the fetched components are tagged `0.29.0`.
[Kyber 0.29's release notes](https://gitlab.com/kyber/kyber/-/blob/main/blog/2026-10-09-kyber-0-29-0.md)
describe the new GStreamer capture wrapper. `vendor/kysdk` contains the current
sources; the project builds `kynput` and `kynet` directly from this checkout.

On this Ubuntu laptop, the isolated LeRobot environment and Rust executable are
installed and built. To reproduce on Ubuntu or macOS, with `git`, `cargo`, `uv`,
`pkg-config`, a C compiler and Python installed (Apple Silicon for the current macOS LeRobot/PyTorch stack):

```bash
cd /home/dev/so101-kyber-position
python3 scripts/setup.py
source scripts/build-env.sh
./target/debug/so101-kyber-position --help
```

On Ubuntu, setup downloads missing development headers/linker libraries from its
configured apt repositories into `.cache/sysroot`; it does not require sudo.
On macOS it uses native system frameworks and skips apt. Setup selects Python
3.12 through uv. The tested local compiler is Rust 1.98.1. Build separately on the
MacBook; the laptop's x86 executable is not a macOS application. The physical
MacBook has not been installed or verified from this session.

The hardware environment pins `lerobot[feetech]==0.6.1`, with CPU PyTorch on this
Ubuntu laptop. LeRobot supplies the SO-101 arm classes, calibration files and
motor-bus reads/writes. The GStreamer producer uses Ubuntu's `/usr/bin/python3`
and `gi`, separately from the LeRobot `.venv`. Required system packages are
`python3-gi`, `gir1.2-gstreamer-1.0`, `gstreamer1.0-tools`,
`gstreamer1.0-plugins-base`, `gstreamer1.0-plugins-good`,
`gstreamer1.0-plugins-bad`, `gstreamer1.0-plugins-ugly`, and
`gstreamer1.0-libav`. Setup also installs a private H.264 parser plugin when needed.

The implemented camera path is **GStreamer capture/H.264 encoding → official
Kyber `KyProto::VideoProtocol::GopStream` → H.264 decoding on the MacBook**.
KyProto manages codec/config packets and GOP video streams over authenticated
KyNet QUIC. Joint commands use `kynput` input packets over KyNet. No LeRobot
network transport or LeRobot camera stream is used.

[config/kyber-gstreamer.toml](config/kyber-gstreamer.toml) separately illustrates
the native `kyavservice` GStreamer configuration. The project directly embeds the
SDK's KyProto video channel; it does not build the full Kyber desktop/VLC app or
its `kyavservice` executable. This is an SDK application, not a stock desktop client.

## Test without hardware

```bash
source scripts/build-env.sh
cargo test --locked
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python scripts/smoke.py
```

The smoke test uses real processes, fresh temporary certificates and actual QUIC
connections. It verifies position recording, sequence order, stale source
rejection, untrusted peer rejection, publisher-loss watchdog, and GStreamer
video capture/transport/decoding. It never opens a physical serial/camera device.

For a manual demonstration, create paired certificates once:

```bash
bash scripts/pair.sh credentials
mkdir -p recordings
```

In terminal A:

```bash
source scripts/build-env.sh
./target/debug/so101-kyber-position receive \
  --listen 127.0.0.1:7445 --credentials credentials/server \
  --output recordings/demo.jsonl
```

In terminal B:

```bash
source scripts/build-env.sh
./target/debug/so101-kyber-position publish \
  --remote 127.0.0.1:7445 --credentials credentials/leader \
  --simulate --seconds 10
```

The receiver also prints accepted NDJSON samples to stdout. Output files are
created anew and are never overwritten. A receiver handles one session and exits
when its publisher disconnects; restarting a process is an explicit action.

## Calibrate with LeRobot

Run official LeRobot interactive calibration on each arm's own computer with
that arm supported and a local power cutoff available. The wrapper below invokes
`lerobot-calibrate` through its Python module; it does not implement calibration.
Follow LeRobot's midpoint and range-of-motion prompts.

On the MacBook with the leader:

```bash
.venv/bin/python calibrate.py leader --port /dev/cu.usbmodemREAL_DEVICE \
  --id leader --directory calibration --attended
```

On Ubuntu with the follower:

```bash
.venv/bin/python calibrate.py follower --port /dev/serial/by-id/REAL_FOLLOWER \
  --id follower --directory calibration --attended
```

This saves `calibration/leader.json` and `calibration/follower.json` respectively.
Use `--calibration calibration/leader.json` for the reader. Copy the leader's
calibration to the follower computer and use both files for the drive adapter.
The direct equivalent is `.venv/bin/lerobot-calibrate --teleop.type=so101_leader
--teleop.port=... --teleop.id=leader --teleop.calibration_dir=calibration` (use
`robot`, `so101_follower` and `follower` for the other arm).
See [LeRobot's SO-101 calibration guide](https://huggingface.co/docs/lerobot/so101#calibrate).
Calibration is interactive physical work; it has not been run on these arms.

## Publish a physical leader

Identify the leader's **stable serial path**, its calibration and its actual motor
IDs before opening it. On this laptop the currently connected adapters end in
`5B14030562` and `5B14115694`; their roles have not been identified. No calibration
or physical configuration has been copied from previous projects.

Use the LeRobot calibration produced above at `calibration/leader.json`.
It must contain each joint's `id`, `drive_mode`, `homing_offset`, `range_min`, and
`range_max`. LeRobot loads the saved calibration and the adapter verifies it against hardware,
including ranges and homing offsets. STS3215 single-turn mode is required. It refuses a leader with torque enabled and
never changes torque, mode, offsets or calibration.

After identifying the arm, support it and arrange passive operation locally.
Then run on the leader computer, replacing `LEADER_SERIAL_PATH` and `SERVER_IP`:

```bash
set -o pipefail
source scripts/build-env.sh
.venv/bin/python hardware.py leader \
  --port LEADER_SERIAL_PATH --calibration calibration/leader.json --hz 30 |
./target/debug/so101-kyber-position publish \
  --remote SERVER_IP:7445 --credentials credentials/leader
```

On the server, run `receive --listen PRIVATE_SERVER_IP:7445` with its own
`credentials/server` directory. Both machines need a reachable private LAN/VPN
route. This project provides no NAT traversal or Internet relay. Permit UDP 7445
only on the intended private interface. Copy only the correct role's credentials
to each computer; keep the CA signing key on the provisioning computer.

## Enable a follower after local commissioning

The follower adapter is implemented and tested with fake hardware. **It has not
been physically commissioned.** Before using it, confirm each arm's identity,
calibration, joint directions, physical limits, appropriate power and local power
cutoff with an attendant supporting the follower. Torque disable can let it fall.
First validate a no-jump enable and stop at the arm, then test short sessions.

Copy `config/follower.example.json` to `follower.local.json`. Every example bound
is a placeholder. Replace all bounds, speeds and inversion flags with reviewed
limits inside the follower's own calibration before setting `limits_reviewed`
to `true`. Keep separate leader/follower calibration files; do not reuse one
arm's calibration for the other. All structural settings are checked before
opening the follower.

On the server, with an attendant and supported follower:

```bash
set -o pipefail
source scripts/build-env.sh
./target/debug/so101-kyber-position receive \
  --listen PRIVATE_SERVER_IP:7445 --credentials credentials/server \
  --output recordings/physical-session.jsonl |
.venv/bin/python hardware.py drive \
  --port FOLLOWER_SERIAL_PATH --config follower.local.json \
  --leader-calibration calibration/leader.json --follower-calibration calibration/follower.json \
  --attended --enable-motion
```

The adapter uses the official LeRobot SO-101 classes and motor bus, connecting
the bus without enabling torque or triggering implicit calibration. The first
valid sample anchors relative leader movement to the follower's current
pose. The controller preloads the current pose before enabling torque, maps
relative calibrated movement, checks measured bounds and limits per-cycle target
changes. It attempts torque disable on all six motors on EOF, timeout, Ctrl+C,
SIGTERM, or an exception, and reads torque registers to confirm the stop.
An unconfirmed stop is an error requiring local power isolation. There is no
automatic reconnect, reset or resume. A server journal records **received leader
positions**, not proof of executed follower motion or measured follower telemetry.

Each exchange uses a server challenge with a 200 ms response deadline; old
responses cannot be carried into a new exchange. The publisher keeps only the
latest source sample and rejects samples over 100 ms old on its own clock.
The follower rejects receiver output over 100 ms old on the server's clock and
stops after 200 ms without input. This avoids requiring synchronized clocks
between computers; capture timestamps in recordings still need clock alignment
for cross-host timing analysis. These software deadlines do not replace a local
physical power cutoff or establish hardware stop performance.

## Optional GStreamer camera

On the robot computer, omit `--device` for a synthetic test signal:

```bash
set -o pipefail
source scripts/build-env.sh
/usr/bin/python3 camera.py --device CAMERA_BY_ID_PATH |
./target/debug/so101-kyber-position camera-serve \
  --listen PRIVATE_SERVER_IP:7446 --credentials credentials/server
```

On the operator:

```bash
set -o pipefail
source scripts/build-env.sh
./target/debug/so101-kyber-position camera-view \
  --remote SERVER_IP:7446 --credentials credentials/leader |
gst-launch-1.0 -q fdsrc ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

Permit UDP 7446 on the same private interface. The KyProto GOP video channel uses a separate authenticated connection. Raw
capture queues drop old buffers before encoding; stale encoded input is rejected
until a keyframe and transport stalls terminate the preview. Camera failure does not automatically stop the
separate position controller; unattended visual teleoperation is not implemented.

## Input protocol

Upstream kynput has no native SO-101 motor-position type. This project uses its
six `GamepadEvent::Axis` types as a **project-specific virtual position device**,
with device ID 101. All values are nonnegative raw ticks, preserving the full
12-bit position exactly, including both trigger axes. The custom receiver
consumes them directly; no OS virtual gamepad or standard game controller
normalization is involved. Stock Kyber desktop input injection will not drive
an SO-101 with these packets.

A frame is 78 bytes: challenge, sequence and capture Unix nanoseconds (three
big-endian `u64` fields), followed by six entries, each containing the kynput
input-type byte and its eight-byte axis payload. All joints must be present in
the defined order. The receiver checks framing, discriminants, device ID, ticks,
sequence and challenge before calling upstream deserialization. TLS 1.3 mutual
authentication protects both ends. ALPN distinguishes positions from camera
frames. This is a custom application on the Kyber SDK rather than a stock
`kynputserver` deployment.

A frame echoing the current challenge with all remaining bytes zero explicitly
ends a position session. Both read and publish rates must be 10–120 Hz to stay
within the watchdog budget; the default is 30 Hz.

See [LICENSE](LICENSE) for the project's AGPL-3.0-or-later license; upstream SDK
components retain their own license notices.
