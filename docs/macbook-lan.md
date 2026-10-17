# MacBook leader → Ubuntu laptop follower over Wi-Fi LAN

| Computer | Connected devices | Role |
| --- | --- | --- |
| MacBook | SO-101 leader over USB | Read and publish joint positions |
| Ubuntu laptop | SO-101 follower, optional USB camera | Receive, record and optionally follow |

The Ubuntu Wi-Fi address observed on October 17, 2026 is **192.168.101.50** on
`wlp0s20f3`. Recheck with `ip -brief -4 address` if the laptop reconnects or the
router changes its DHCP lease. Set `FOLLOWER_HOST` to that address on the Mac.
Both computers must be on the same reachable LAN; a guest network may isolate
clients. Control uses UDP **7445** and optional camera preview uses UDP **7446**.
No cloud server, SSH tunnel, Tailscale or router port forwarding is required.

## 1. Install on the MacBook

Install Xcode Command Line Tools if they are missing:

```bash
xcode-select --install
```

With Homebrew installed, use [Rust](https://formulae.brew.sh/formula/rust) and
[uv](https://docs.astral.sh/uv/getting-started/installation/#homebrew):

```bash
brew install rust uv pkg-config
git clone https://github.com/prathamv0811/so101-kyber-position.git
cd so101-kyber-position
python3 scripts/setup.py
./target/debug/so101-kyber-position --help
```

The repository is private; authenticate to GitHub on the Mac first. `gh repo clone
prathamv0811/so101-kyber-position` is another option if GitHub CLI is configured.
Setup supports both Apple Silicon and Intel Macs, builds the pinned SDK natively,
and creates a Python 3.12 serial environment. It does not run Ubuntu apt commands
on macOS. The physical MacBook has not been accessed or tested from this session;
the repository includes a macOS CI build check.

The leader publisher does not need GStreamer. The optional camera viewer can send
JPEGs to GStreamer or save an MJPEG file; configure the Mac's video tools later
if using preview.

## 2. Pair once on the Ubuntu laptop

The project and dependencies are already installed at:

```bash
cd /home/dev/so101-kyber-position
source scripts/build-env.sh
bash scripts/pair.sh credentials
mkdir -p recordings
```

If credentials already exist, reuse them; the script refuses to overwrite them.
Transfer **only `credentials/leader/`** to the MacBook, placing it under the same
relative directory in the Mac's checkout. Use a trusted local transfer such as
AirDrop or SCP if SSH is already configured. Keep `credentials/server/` and the CA
signing key on the Ubuntu laptop. Credentials and local calibration/configuration
files are ignored by Git and must not be added to the repository.

## 3. Test the LAN before opening motors

On Ubuntu, start the record-only receiver:

```bash
cd /home/dev/so101-kyber-position
source scripts/build-env.sh
mkdir -p recordings
./target/debug/so101-kyber-position receive \
  --listen 192.168.101.50:7445 \
  --credentials credentials/server \
  --output "recordings/lan-$(date +%Y%m%d-%H%M%S).jsonl"
```

On the MacBook:

```bash
cd so101-kyber-position
export FOLLOWER_HOST=192.168.101.50
./target/debug/so101-kyber-position publish \
  --remote "$FOLLOWER_HOST:7445" \
  --credentials credentials/leader \
  --simulate --seconds 10
```

The laptop should print and record about 300 position samples, then end the
session. This test does not open either arm. If it times out, check the laptop's
current address, Wi-Fi client isolation and an active firewall's UDP 7445 rule
for the Mac's LAN address. Do not disable the entire firewall. No firewall rule
has been changed by the setup scripts.

## 4. Read the real leader on the Mac

Identify the USB device using:

```bash
ls /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null
```

Place that arm's existing LeRobot calibration in `leader.local.json`. It must
match the actual motor IDs, ranges and homing offsets. The reader verifies those
offsets and refuses an arm with torque on. It does not recalibrate the arm.
Start a fresh record-only receiver on Ubuntu, then on the Mac run:

```bash
set -o pipefail
export FOLLOWER_HOST=192.168.101.50
export LEADER_PORT=/dev/cu.usbmodemREPLACE_WITH_REAL_DEVICE
.venv/bin/python hardware.py leader \
  --port "$LEADER_PORT" --calibration leader.local.json --hz 30 |
./target/debug/so101-kyber-position publish \
  --remote "$FOLLOWER_HOST:7445" --credentials credentials/leader
```

Move the supported passive leader and verify that the six recorded positions
change as expected. This still does not enable follower torque.

## 5. Commission the follower on Ubuntu

Identify the follower's stable USB path with `ls -l /dev/serial/by-id/`. Place its
own calibration at `follower-calibration.local.json`. Copy `leader.local.json`
from the Mac to Ubuntu as well, so relative movement can be mapped correctly.
Copy `config/follower.example.json` to `follower.local.json`, replace all
placeholder bounds/speeds/inversion flags with reviewed physical limits and
then set `limits_reviewed` to `true`.

With an attendant supporting the follower and able to operate its power cutoff,
start the Ubuntu receiver and follower adapter:

```bash
set -o pipefail
source scripts/build-env.sh
export FOLLOWER_PORT=/dev/serial/by-id/REPLACE_WITH_FOLLOWER
./target/debug/so101-kyber-position receive \
  --listen 192.168.101.50:7445 --credentials credentials/server \
  --output "recordings/physical-$(date +%Y%m%d-%H%M%S).jsonl" |
.venv/bin/python hardware.py drive \
  --port "$FOLLOWER_PORT" --config follower.local.json \
  --leader-calibration leader.local.json \
  --follower-calibration follower-calibration.local.json \
  --attended --enable-motion
```

Run the real leader publisher from step 4. First check no-jump enable, directions,
bounded movement, Ctrl+C stop and connection-loss stop in short attended sessions.
Support the follower during torque-off. The session never resumes automatically.
Physical behavior on these arms has not yet been verified. See the main README
for watchdog behavior, recording semantics and the optional GStreamer camera path.
