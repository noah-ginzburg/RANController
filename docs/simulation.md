# Running in simulation

No hardware, no radio, no Vicon needed. This is the way to exercise the
controller and RAN logic when you don't have a drone in front of you.

## Prerequisites

```bash
cd ~/biodrone && colcon build --symlink-install && source install/setup.bash
```

## Run

```bash
ros2 launch crazyswarm_bringup sim_drone.launch.py real:=false
```

For multiple drones:

```bash
ros2 launch crazyswarm_bringup sim_drone.launch.py \
    real:=false drone_names:="cf01 cf02 cf03" ran_drones:="cf01 cf02 cf03"
```

### What good looks like

- RViz opens with drone models in it.
- `ros2 topic echo /cf01/cmd_full_state` starts streaming once you command a
  takeoff.
- No `takeoff service not available` messages after the first few seconds.

## How sim differs from real

These differences aren't cosmetic — the code takes genuinely different paths:

| | Sim | Real |
|---|---|---|
| Setpoint topic | `cmd_full_state` (FullState) | `cmd_velocity_world` (VelocityWorld) |
| Arm service | absent — `send_arm_req` warns and continues | present, armed at startup |
| Pose source | crazyswarm2 sim backend | Vicon → `vicon_bridge` → `/poses` |
| `go_to` | unusable while streaming setpoints | works via high-level commander |

That last row is a real constraint rather than an oversight. Streaming
`cmd_full_state` puts the sim into `MODE_LOW_FULLSTATE`, but
`crazyflie_sil.goTo()` only accepts `MODE_HIGH_POLY`. It raises *"goTo from
low-level modes not yet supported"*, which kills the service callback. The SIM
branch in `_debug_goto` (`controller_server.py:397`) is where this happens.

## Gotchas

- **Sim won't catch anything mocap-related.** EKF divergence, occlusion, yaw
  drift, and external-pose fusion problems only show up on real hardware.
- `hover_speed_sim` and `hover_speed_real` are separate arguments, and setting
  the wrong one for your run just silently does nothing.

## See also

- [`real-flight.md`](real-flight.md)
- [`troubleshooting.md`](troubleshooting.md)
