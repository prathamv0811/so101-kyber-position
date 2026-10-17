# Native LeRobot controls with Kyber client/server transport

The implementation follows the SO-101 action flow in LeRobot's teleoperation loop:

```python
# MacBook leader client
native_action = leader.get_action()
kyber_client.publish(native_action)

# Follower server, after freshness and reviewed-limit checks
observation = follower.get_observation()
follower.send_action(native_action)
```

Kyber supplies the network boundary; LeRobot supplies `SO101Leader`,
`SO101Follower`, their calibration files, motor normalization, safe-goal helper,
and motor writes. Video uses GStreamer H.264 and Kyber's KyProto datagram stream.
There is no LeRobot camera/network transport in the remote path.

Both arms use `use_degrees=True`: body actions are degrees and the gripper is
0..100, exactly as in LeRobot. Protocol v3 carries the six `.pos` values as
64-bit floats and reconstructs the same action dictionary. It replaces the old
calibrated-fraction/gamepad protocol and relative leader anchor. Both computers
must update and rebuild. A fixed leader pose now remains an absolute target;
the follower ramps toward it within reviewed bounds and speed limits.

`hardware.py` has a small safety wrapper: it verifies calibration, rejects
stale/malformed input, checks physical bounds, and uses LeRobot's own
`ensure_safe_goal_position` to limit target changes per elapsed second.
`SO101Follower.send_action()` applies `max_relative_target` relative to measured
position, then performs the actual motor write. Fractional goals accumulate so
small safe steps do not stall inside the servo dead zone. There is no custom PID
or alternative motor driver. Tick conversions are used for physical-limit checks;
the native action values retain their units across the network.

Startup opens the official bus directly, checks the saved calibration/passive
torque, and waits for fresh actions plus both camera streams. It avoids full
`connect()/configure()` before the motion gate because follower configuration
can enable torque. Current-pose goals are preloaded through `send_action()` before
explicit torque-on. Stop tries all motors and verifies torque-off. Saturated
gripper observations get a raw register check so LeRobot's normalization cannot
hide an out-of-calibration position.

The local LeRobot checkout was reviewed at `146d8474`, which uses the newer
Rustypot-backed `SerialMotorsBus`. The reproducible runtime stays pinned to
`lerobot[feetech]==0.6.1`, using `FeetechMotorsBus`; public SO-101 arm methods are
shared by both. The local checkout is not modified or silently substituted.
Tests exercise actual LeRobot get/send methods with physical I/O mocked and
verify signed fractional values across real Kyber QUIC connections.

Control requests default to 120 Hz on reader, client, and server. Video stays
30 fps. Logs report actual follower update rate. See [remote setup](tailscale.md).
