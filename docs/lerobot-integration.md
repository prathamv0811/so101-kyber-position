# LeRobot-based SO-101 remote teleoperation

This application builds on LeRobot's real SO-101 arm implementations. Kyber
transports the action samples and GStreamer video; Tailscale provides the remote
network route. Hardware calibration and motor communication belong to LeRobot.

| Operation | LeRobot implementation used |
| --- | --- |
| Leader calibration | Official `lerobot-calibrate` with `so101_leader` |
| Follower calibration | Official `lerobot-calibrate` with `so101_follower` |
| Leader measurement | `SO101Leader.get_action()` |
| Follower measurement | `SO101Follower.get_observation()` |
| Follower goals | `SO101Follower.send_action()` |
| Startup/stop and calibration verification | The arm's official Feetech motor bus |

`hardware.py` constructs these classes with the matching saved calibration and
`use_degrees=True`. Five body joints use degrees; the gripper uses 0..100.
The adapter converts observations to integer ticks for reviewed physical limits,
then converts bounded goals through LeRobot's own motor normalizer before
`send_action()`. A compatibility test exercises actual LeRobot arm methods and
normalization with only physical I/O mocked. Saturated gripper observations get
a raw register check so normalization cannot hide out-of-calibration positions.

The network contract remains six calibrated fractions in 0..4095 (protocol v2).
Each arm keeps its own calibration. The first leader frame anchors relative
movement to the current follower pose. Position goals follow a continuous,
speed-limited trajectory so sub-tick steps accumulate at 60 Hz. Outstanding
tracking error is bounded to 100 ms of configured travel (minimum two ticks);
excess measured error stops the session. Integer goal quantization adds up to
one tick. Existing joint bounds and speed limits remain active.

The local LeRobot source checkout was reviewed at revision
`146d8474` (which uses the newer Rustypot-backed `SerialMotorsBus`). The
reproducible application runtime remains pinned to `lerobot[feetech]==0.6.1`
in `pyproject.toml`/`uv.lock`, which uses `FeetechMotorsBus`; the public SO-101
arm methods above are shared by both implementations. The application does not
silently substitute the local checkout or modify it.

Startup opens the official bus directly, checks the saved calibration and
passive torque, and waits for fresh commands plus both camera streams. It avoids
calling full arm `connect()/configure()` before the motion gate because follower
configuration can enable torque. Current-position goals are preloaded before
explicit torque-on. The passive leader check uses one synchronized torque read
instead of six separate serial round trips. Missing/stale input, stale video,
exceptions, and shutdown attempt torque-off on all motors and verify it.

Control requests default to 60 Hz on the leader reader, publisher, and listener.
The follower updates on accepted frames; actual USB/network timing can reduce the
rate. `motors.log` reports measured update rate, input, feedback, and targets
once per second. Video stays at 30 fps. See [remote setup](tailscale.md).
