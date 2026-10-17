# MacBook leader → Ubuntu follower over LAN

For remote access using this laptop’s **100.79.3.30** address, use the
[Tailscale guide](tailscale.md).

The public repository is https://github.com/prathamv0811/so101-kyber-position.
The follower laptop's observed Wi-Fi address is **192.168.101.50**. Both computers
must be on the same reachable Wi-Fi network. A guest network may isolate clients.

| Device | Computer | Network port (UDP) |
| --- | --- | --- |
| SO-101 follower | Ubuntu laptop | 7445 |
| Logitech Brio 100 top camera | Ubuntu laptop | 7446 |
| Sonix USB wrist camera | Ubuntu laptop | 7447 |
| SO-101 leader | MacBook | Publishes to laptop |

## Install on the Mac

The pinned LeRobot/PyTorch environment targets Apple Silicon. Install Xcode Command
Line Tools if needed (`xcode-select --install`), then with Homebrew installed:

```bash
brew install rust uv pkg-config gstreamer
git clone https://github.com/prathamv0811/so101-kyber-position.git
cd so101-kyber-position
python3 scripts/setup.py
```

If already cloned, run `git pull` followed by `python3 scripts/setup.py` to rebuild.
The Rust executable must be built on each computer. The physical MacBook has not
been tested from this session; Ubuntu and macOS builds are covered by CI.

## Prepare the Mac leader

Identify its USB path:

```bash
ls /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null
```

Reuse an existing matching LeRobot leader calibration at `calibration/leader.json`,
or calibrate the supported arm with LeRobot:

```bash
.venv/bin/python calibrate.py leader --port /dev/cu.usbmodemYOUR_DEVICE \
  --id leader --directory calibration --attended
```

Follow LeRobot's midpoint and range prompts. The passive leader reader verifies
that saved calibration matches the connected arm and refuses enabled torque.
Each computer keeps its own calibration; no calibration transfer is needed.

## Start the follower laptop

The local profile is `config/follower.launch.local.json`. This laptop uses the
existing `follower_2` calibration, verified against all six connected motors.
Its private `follower.local.json` limits must be reviewed before operation;
example configuration is provided in `config/follower.launch.example.json`.

With the follower supported and an attendant at its power cutoff:

```bash
cd /home/dev/so101-kyber-position
source scripts/build-env.sh
.venv/bin/python session.py follower --insecure-lan --hz 120 --attended --enable-motion
```

This starts both camera streams and the command receiver. The follower waits
with torque off for up to one hour. Fresh leader input and both camera streams
are required before torque is enabled. The follower approaches the leader's absolute LeRobot joint pose under reviewed
bounds and speed limits; no relative startup anchor is applied.

For video only, use `--preview-only` instead of the two motor flags. Start a
matching `--preview-only` operator. For a fully synthetic test on both sides,
use `--simulate`; never connect a simulated publisher to a physical follower.

## Connect from the Mac

In the Mac checkout, replace the USB placeholder:

```bash
.venv/bin/python session.py operator \
  --host 192.168.101.50 --insecure-lan --hz 120 \
  --port /dev/cu.usbmodemYOUR_DEVICE \
  --calibration calibration/leader.json
```

Two windows show the top and wrist cameras. Only after both receive video does
the Mac begin publishing native LeRobot actions. Ctrl+C ends the session. Command
loss, stale camera progress, or a child-process failure ends following and
attempts verified torque-off. Support the follower during torque-off. An
unconfirmed stop requires local power isolation. Restart the follower launcher
for each new session; there is no automatic reconnect or motion resume.

Logs and received actions are saved privately under `recordings/` on each
computer. Motor activity and stop confirmation appear in the follower run's
`motors.log`. The journal contains received native LeRobot actions, not measured
follower telemetry. Physical motion and stop performance still require local
commissioning; automated tests use fake motor hardware.

## Credentials are optional on a trusted LAN

`--insecure-lan` on **both** computers requires no credential files or copying.
QUIC remains encrypted, using an automatically generated server certificate,
but neither computer verifies peer identity. Other devices on the LAN could
connect or impersonate a peer. This mode accepts only explicit private,
loopback, or link-local addresses, and refuses public/wildcard bind addresses.
Tailscale IPv4 addresses in 100.64.0.0/10 are also supported. Private-address
checks are not a substitute for a trusted network.

For paired mutual TLS, omit `--insecure-lan`. Generate credentials on the laptop:

```bash
bash scripts/pair.sh credentials
```

Transfer only `credentials/leader/` to the Mac checkout; retain the CA signing key
and server keys on the laptop. Both modes use the same ports and watchdogs.
Credentials, calibration, recordings, and local profiles stay out of Git.

If connection fails, check the laptop's current address (`ip -brief -4 address`),
Wi-Fi client isolation, firewall UDP rules for 7445–7447, and the latest run logs.
The software does not change firewall settings or require router port forwarding.

Control defaults to **120 Hz** on both roles (`--hz 120`). The Mac passes this to
both the LeRobot leader reader and network publisher; the listener paces
latest actions at up to 120 Hz and the follower skips queued old samples. Video
remains 30 fps. Actual throughput depends on USB, Wi-Fi, and scheduling. The
motor log reports measured update rate, leader values, feedback, and targets
once per second. Speed limits remain in ticks/second, independent of sample rate.
`max_ticks_per_second: null` disables a joint's time-based ramp and requires an
explicit `max_tracking_error_ticks`; calibrated position/tracking guards remain.

Transport v4 carries native LeRobot body degrees and gripper 0..100. Update and
rebuild both computers; older transport clients cannot connect. See
[LeRobot controls](lerobot-integration.md).
