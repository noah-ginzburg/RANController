# Flying real drones

Read [`hardware-drone.md`](hardware-drone.md) and [`vicon-setup.md`](vicon-setup.md)
first — this guide covers the run procedure only, and assumes the drone and the
mocap system are already set up correctly.

## Preflight

- [ ] Battery **fully charged** (4.2 V, not 3.7 V — see [`hardware-drone.md`](hardware-drone.md))
- [ ] Drone reachable at its configured URI (scan below)
- [ ] Vicon Tracker running, subject tracked, no occlusion at the takeoff spot
- [ ] Props intact, drone sitting level on the floor
- [ ] Workspace built and sourced
- [ ] Everyone in the room knows where the ESTOP is

Check that the radio can find the drone at the URI listed in
`src/crazyswarm2/crazyflie/config/crazyflies.yaml`:

```bash
python3 -c "
import cflib.crtp; cflib.crtp.init_drivers()
print(cflib.crtp.scan_interfaces(0xE7E7E7E7E8))"
```

A non-empty result means the drone is reachable. If it comes back empty, stop
here and work through [`hardware-drone.md`](hardware-drone.md).

## Run

Start the Vicon client first, in its own terminal. It is **not** part of the
main launch file:

```bash
ros2 launch vicon_receiver client.launch.py
```

Then bring up the stack:

```bash
ros2 launch crazyswarm_bringup real_drone.launch.py real:=true drone_names:=cf09
```

Some overrides you'll probably want at some point:

```bash
record:=false           # no rosbag
rviz:=false             # no RViz
launch_height:=0.7      # takeoff altitude (m)
drone_names:="cf01 cf09"
```

### What good looks like

- `crazyflie_server` logs a successful connect for each URI.
- No `no valid Vicon pose for ...s -- flying blind` warnings.
- No boot-heading errors (these only appear in position-only mode).
- The drone sits still with its motors off, waiting for a commanded takeoff.

Takeoff isn't automatic — you command it from the debug GUI. The interlock at
`controller_server.py:529` won't stream setpoints until a takeoff has actually
been commanded, so picking the drone up off the floor won't start it.

## Stopping

| Action | Effect |
|---|---|
| **First Ctrl+C** | Lands if airborne above `LAND_MIN_HEIGHT` (0.5 m); ESTOPs if lower |
| **Second Ctrl+C** | Emergency stop — motors cut, drone **drops** |
| **GUI ESTOP** | Same as second Ctrl+C, always responsive |

`sigterm_timeout` is set to 7 s so that `crazyflie_server` and `vicon_bridge`
outlive the 5 s landing (`real_drone.launch.py:225`). `vicon_bridge` also
deliberately ignores the first SIGINT so that mocap keeps flowing during the
descent — without that, the drone lands blind on dead reckoning.

The land duration and height thresholds are `LAND_DURATION`, `LAND_HEIGHT`, and
`LAND_MIN_HEIGHT` at `controller_server.py:27`.

## Gotchas

- **`send_orientation` depends on the firmware.** `real_drone.launch.py:96`
  sets it to `True`, which requires firmware 2026.04 or newer with the fixed
  external-attitude update. On older firmware the EKF diverges and the drone
  runs away, so verify the firmware version before trusting this.
- **`Land.height` is an absolute height, not a descent distance.** It goes
  straight through to cflib's `land(absolute_height_m, ...)`, so a negative
  value aims the planner below the floor.
- **A land command spins the motors** even when the drone is on the ground,
  because it activates the high-level commander. That's exactly what the
  `LAND_MIN_HEIGHT` guard is there to prevent.
- Bags are written to `bags/flight_<timestamp>/`, which is gitignored.

## See also

- [`troubleshooting.md`](troubleshooting.md)
- [`analysis.md`](analysis.md)
