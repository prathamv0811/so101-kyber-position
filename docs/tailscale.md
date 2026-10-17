# Remote SO-101 teleoperation over Tailscale

The Ubuntu follower laptop is **100.79.3.30** (`pratham`) on Tailscale.
Commands use UDP 7445, Logitech top video 7446, and Sonix wrist video 7447.
Bind to the Tailscale address to keep these listeners off the ordinary LAN.

## Mac setup

Install and connect the [official Tailscale macOS client](https://tailscale.com/docs/install/mac)
to the same tailnet as the laptop (or arrange an authorized shared-device route).
Check that the laptop appears online in Tailscale. If the CLI is available:

```bash
tailscale status
tailscale ping 100.79.3.30
```

`tailscale ping` is a network reachability check; it does not establish that
application UDP ports are permitted. Tailnet policy must allow the Mac to reach
UDP 7445–7447 on the laptop. No policy changes are made by this repository.
Remote sessions retain the 200 ms command watchdog, so a high-latency/relayed
connection may stop rather than keep moving on stale input.

In the existing Mac project checkout, stop the old operator with Ctrl+C, then:

```bash
git pull
python3 scripts/setup.py
.venv/bin/python session.py operator \
  --host 100.79.3.30 --tailscale --hz 60 \
  --port /dev/cu.usbmodemYOUR_DEVICE \
  --calibration calibration/leader.json
```

Use the actual leader USB path. Each arm's calibration stays on its own computer.
Both video windows start before leader commands are published. Ctrl+C stops the
session. On this laptop, inspect the latest `recordings/*/motors.log` for the
actual following rate, measured positions, targets, and stop confirmation.

## Follower startup

With the follower supported and attended at its power cutoff:

```bash
source scripts/build-env.sh
.venv/bin/python session.py follower --listen-host 100.79.3.30 \
  --tailscale --hz 60 --attended --enable-motion
```

The local profile supplies the follower USB path, matching `follower_2`
calibration, reviewed limits, and the two cameras. It waits torque-off for up to
one hour for the operator. It handles one session; restart explicitly after a
stop or disconnect. Physical motion and stop behavior require local commissioning.

`--tailscale` is an alias for credential-free private-network mode. No application
keys need copying: access relies on the Tailscale connection and tailnet policy.
QUIC also encrypts traffic with ephemeral server certificates, but the application
does not independently authenticate its peer. The IP check recognizes Tailscale's
[100.64.0.0/10 range](https://tailscale.com/docs/concepts/tailscale-ip-addresses);
recognizing that range alone is not proof of tailnet membership. To use paired
application mutual TLS over Tailscale instead, omit `--tailscale` and provision
role credentials as described in the LAN guide.

The application also supports direct Wi-Fi LAN mode; see [LAN guide](macbook-lan.md).

The client and server now use native LeRobot actions (protocol v3). Stop old
clients and rebuild both sides. The follower approaches the leader’s absolute
pose under reviewed bounds and speeds, rather than anchoring relative movement.
