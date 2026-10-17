# Diagnosing SO-101 control latency without removing protections

The remote path is `SO101Leader.get_action()` → newest-sample watch channel →
KyNet QUIC datagrams → paced newest-command receiver → draining NDJSON reader →
follower observation → reviewed target clamp → `SO101Follower.send_action()`.
Video capture and decoding run in separate processes. LeRobot follower cameras
remain empty, so `get_observation()` does not capture an image.

## What the investigation confirmed

On the follower laptop, the last captured operator session had no per-second
speed ramp (`max_ticks_per_second: null`), but retained a 10-tick tracking/relative
target bound. That is **0.879°** on body joints. A wrist leader target near 36.7°
was limited to about 60.7° while the measured follower was near 61.6°. Removing
the per-second ramp did not remove that separate protection. The target clamp is
a confirmed cause of slow catch-up. Load, servo response and motor dynamics still
need an attended hardware test; these logs do not prove the servo's maximum speed.

The receiver delivered 1,306 records with last input sequence 1,715: about
**91.35 updates/s** over the recorded interval against a 120 Hz request. Received
command intervals reached **118.6 ms**. New counters distinguish missing sequence
numbers, expired challenges, and deliberate coalescing. A separate idle Tailscale
ping measured 4.1–236.1 ms RTT; this is evidence of link jitter, not a measurement
of one-way command latency during that session. Some Mac sessions ended with
`Leader sample older than 100 ms`, which specifically means the publisher's local
source became stale. USB acquisition or Mac scheduling must be checked in the
new leader/publisher metrics rather than attributing this error solely to the network.

A passive, torque-off USB check measured median consecutive position reads of
**0.941 ms** and **0.946 ms**, with maximum paired time 2.113 ms. No motor goals
were sent. The measured P/I/D registers were 16/0/32, matching this pinned LeRobot
configuration; the measured dead zones were one tick. These measurements do not
include powered motor writes, retries under motion, or the Mac's USB adapter.
Both position reads in the live follower path are retained: the observation
validates physical/tracking bounds and LeRobot's own read enforces
`max_relative_target` immediately before its motor write.

Camera status formerly opened and parsed two files on every motor iteration.
The local median was 0.0305 ms: too small to explain seconds of slow catch-up,
but filesystem stalls were on the motor thread. The cached check measured
0.00027 ms. File polling now runs on a separate 20 Hz worker. The motor thread
checks an immutable snapshot, including its original frame-derived expiry.
A frozen timestamp or stalled worker cannot reset or extend the 500 ms watchdog.

## Reading the diagnostics

Each process emits `CONTROL_METRICS {json}` about once per second. Python windows
report mean, p95 and maximum durations; network windows report mean and maximum.
All durations below are milliseconds. Startup prints configured per-joint speed
limits and native-unit tracking bounds.

| Log | Role | Useful fields |
| --- | --- | --- |
| Mac `leader.log` | `leader` | `rate_hz`, `passive_check_ms`, `position_read_ms`, `acquisition_ms`, `source_emit_ms` |
| Mac `publisher.log` | `publisher` | `rate_hz`, `source_age_ms`, `beacon_arrival_age_ms`, `unique_source_samples` |
| Follower `commands.log` | `receiver` | delivered `rate_hz`, `delivery_queue_ms`, `challenge_age_ms`, `received_datagrams`, `sequence_gaps`, `expired_challenge`, `coalesced_delivery` |
| Follower `motors.log` | `follower` | `rate_hz`, `input_wait_ms`, `motor_read_ms`, `motor_send_ms`, `motor_cycle_ms`, `command_age_ms`, clamp counters, `coalesced_input` |

`motor_send_ms` includes LeRobot's extra protective position read and the motor
write; it is not a write-only benchmark. Read retries remain at two; elapsed I/O
timings include retries. `input_wait_ms` separates waiting for input from motor
work. Counters such as `speed_ramp.wrist_flex`, `relative_target.wrist_flex`,
`lerobot_relative_target.wrist_flex` and `physical_bounds.wrist_flex` show which
clamp actually bound the target. Coalescing intentionally skips older commands
and never replays a backlog to increase a rate counter.

`source_age_ms` is measured on the Mac and `command_age_ms` is measured on the
follower from local receiver arrival. The receiver queue age is local too.
`challenge_age_ms` is the age of an echoed server beacon, **not** one-way command
latency. Do not subtract capture and receive Unix timestamps from different
computers without independently establishing clock synchronization.

If follower `input_wait_ms` is high but its motor cycle is short, investigate
leader/publisher and network arrival rates. If read/send times approach the
requested period, USB/servo I/O and retries are limiting the achievable rate.
If timings are short but clamp counters rise, increasing network Hz cannot
remove the configured motion constraint.

## Reviewing local motion configuration

This change does not modify the ignored `follower.local.json` or deploy code to
the live follower. Review each joint locally against the actual calibration,
load and attended motion test. The example keeps `limits_reviewed: false` and
the same 150 ticks/s speed value; its explicit 15-tick tracking bound equals
the former derived default. It does not increase either limit.

- `min_ticks` / `max_ticks`: reviewed physical bounds inside that joint's calibration.
- `max_ticks_per_second`: optional time-based ramp, independent of command Hz.
- `max_tracking_error_ticks`: target offset and tracking-error protection; also
  supplies LeRobot's `max_relative_target`. Set it explicitly so changing a
  reviewed speed does not silently change this separate protection.
- `invert: false`: native LeRobot position convention.

For body joints, multiply ticks by `360 / 4095` to get degrees. The example's
150 ticks/s is about 13.19°/s; 15 ticks is about 1.32°. Gripper values instead use
`100 / (range_max - range_min)` percent per tick. A `null` speed disables only
the time ramp; it leaves physical bounds, tracking checks and
`max_relative_target` active. Existing profiles with a numeric speed and no
explicit tracking field retain their old derived bound for compatibility.
Do not use the example's placeholder position bounds as hardware calibration.

Requested control rate defaults to 120 Hz and accepts `--hz 10..500` on both
roles; video remains 30 fps. Higher Hz is an opt-in experiment, not a guarantee
of physical motor rate or a reason to raise motion limits. Check measured cycle
times before requesting 240 Hz or more. Synthetic dual-video sessions measured
120.0 and 240.0 command updates/s locally; a 60 ms artificial RTT test measured
120.0 updates/s. None of those tests actuated the robot or prove Mac-to-follower
motion latency. Network, USB and servo dynamics preclude a zero-delay guarantee.

## Safety and verification

The 100 ms command-age bound, 200 ms input watchdog, physical bounds,
`ensure_safe_goal_position`, `max_relative_target`, tracking checks, attended
motion opt-in and verified all-motor torque-off cleanup remain. Freshness is
rechecked immediately before motor I/O. The existing 100 ms motor-send timeout
is retained and the same budget now covers the complete read/control/send cycle.
Timeout checks are software checks after bounded I/O returns, not a hardware
emergency-stop guarantee.

```bash
source scripts/build-env.sh
cargo test --locked
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python scripts/smoke.py
.venv/bin/python scripts/test-control-delay.py
.venv/bin/python scripts/test-camera-startup.py
.venv/bin/python scripts/test-session.py
.venv/bin/python scripts/test-session.py --insecure-lan
SESSION_TEST_HZ=240 .venv/bin/python scripts/test-session.py --insecure-lan
```
