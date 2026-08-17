# Vicon setup

Mocap is the drone's only absolute position reference. When it degrades, the
EKF is left coasting on gyro and accelerometer data, and the drone drifts away.

## Running the bridge

The Vicon client is a **separate launch** from the main stack:

```bash
ros2 launch vicon_receiver client.launch.py
```

There's also a `client_upstairs.launch.py` for the other lab. **TODO:** document
which room each one targets, and their host IPs.

`vicon_bridge.py` is started by `real_drone.launch.py` and handles the
filtering.

### What good looks like

```bash
ros2 topic hz /poses            # steady, ~100 Hz
ros2 topic echo /vicon/cf09/cf09
ros2 run tf2_ros tf2_echo mocap cf09
```

And no `no valid Vicon pose for ...s -- flying blind` warnings in the log.

## Subject naming

Vicon subject names have to match `drone_names`. The bridge subscribes to
`/vicon/<name>/<name>` (`vicon_bridge.py:43`) and broadcasts TF as
`mocap -> <subject_name>`.

A mismatch fails silently — there's no error, you just get no data.

There's more on this in `src/ros2_fastDDS_vicon/vicon-subject-creation.md`.


## Orientation mode

`send_orientation` (set in `real_drone.launch.py:96`) chooses the fusion mode.

With **`True`, you get full pose** — position and attitude. This requires
firmware 2026.04 or newer. Vicon then corrects yaw continuously. `extQuatStdDev`
is loosened to `4.5e-2` in `crazyflies.yaml`, because Vicon attitude arrives
with some latency, and trusting it too tightly drags the EKF attitude behind
reality during takeoff transients.

With **`False`, you get position only.** The bridge publishes NaN quaternions,
so crazyswarm2 falls back to `send_extpos`. EKF yaw is then gyro-only: it
drifts, and it assumes yaw = 0 at estimator reset. This means you must **power
the drone on facing Vicon +X**, or position feedback gets applied through the
wrong yaw and the drone runs away with mirrored control. The bridge will nag you
if the boot heading is more than 15° off.

Position-only fusion was the root cause of the runaways on 2026-07-17 (bags
151857 and 152128).

## Marker placement

- **TODO:** document the marker layout and the asymmetry requirements.
- `MARKER_HEIGHT_OFFSET` (`vicon_bridge.py:13`, currently `0.0`) compensates for
  the marker centroid sitting above the floor. Set it if the drone thinks it's
  airborne while it's still on the ground.

## Gotchas

- The quaternion sign is normalised to `w >= 0` (`vicon_bridge.py:131`). Vicon
  emits `w < 0` near yaw 180°, and the antipodal quaternion reads as a roughly
  360° innovation, which diverges the EKF and sends it into a loop of
  auto-resets while the drone is sitting still.
- Occlusion at the takeoff spot is the worst case, because the EKF has no
  history built up yet.
- **TODO:** record the Vicon host IP and network config.

## See also

- [`real-flight.md`](real-flight.md)
- [`troubleshooting.md`](troubleshooting.md)
