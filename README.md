# Spherical Attractor Project

This is a Crazyflie swarm control workspace. It implements a spherical attractor
network — an extension of the more common ring attractor network — to produce
collective behavior in a group of drones, flown over Vicon motion capture using
Crazyswarm2.

Everything runs on ROS 2 Humble. All of `src/` is vendored as regular files
rather than git submodules, so a plain `git clone` gives you the whole
workspace with nothing extra to initialize. ROS 2 unbag, Crazyswarm2, and
ros2-vicon-receiver were originally cloned from other repositories; links are
below.

One thing worth calling out: `vicon_bridge.py` in ros2-vicon-receiver was
written by me, not taken from upstream.

## Documentation

The task-specific guides live in [`docs/`](docs/). If you're trying to actually
do something with this workspace, start there rather than here:

| Guide | Use it when |
|---|---|
| [`docs/simulation.md`](docs/simulation.md) | Running in sim, no hardware |
| [`docs/real-flight.md`](docs/real-flight.md) | Flying actual drones |
| [`docs/hardware-drone.md`](docs/hardware-drone.md) | Radio config, firmware, batteries |
| [`docs/vicon-setup.md`](docs/vicon-setup.md) | Mocap: subjects, markers, network |
| [`docs/troubleshooting.md`](docs/troubleshooting.md) | Something is broken |
| [`docs/analysis.md`](docs/analysis.md) | Bags, plots, post-flight data |

## Build

```bash
cd ~/biodrone
colcon build --symlink-install
source install/setup.bash
```

You'll need to rebuild after editing launch files or any other non-Python asset.
Watch out for scripts installed via `install(PROGRAMS ...)` — `vicon_bridge.py`
is the main one. Those get **copied** rather than symlinked, so editing the
source file does nothing at all until you rebuild.

## Package structure

```
biodrone/
├── bags/                      # rosbag recordings (gitignored)
├── cache/                     # cflib TOC cache (gitignored)
├── docs/                      # task guides -- see above
└── src/
    ├── crazyflie_controller/       # per-drone flight controller
    │   └── controller_server.py    #   takeoff/land/goto + velocity control
    │
    ├── spherical_ran/              # spherical RAN model
    │   ├── spherical_RAN_server_lloyd.py    # Lloyd-relaxation variant (launched)
    │   ├── spherical_RAN_server.py          # continuous attractor server
    │   └── generate_kernel_cache.py         # precomputes the RAN kernel
    │
    ├── crazyswarm_bringup/         # top-level orchestration
    │   └── launch/
    │       ├── real_drone.launch.py    # hardware bringup
    │       └── sim_drone.launch.py     # simulation bringup
    │
    ├── crazyflie_debug_gui/        # Qt telemetry + manual command GUI
    ├── crazyflie_debug_interfaces/ # DebugFlags.msg (GUI -> controller)
    ├── flight_analysis/            # post-flight plotting from bags
    │
    ├── crazyswarm2/                # (vendored) Crazyflie driver + sim
    ├── ros2-vicon-receiver/        # (vendored) Vicon DataStream -> ROS 2
    │   └── vicon_bridge.py         #   filtering + /poses + TF
    ├── motion_capture_tracking/    # (vendored) mocap tracking libs
    ├── ros2_unbag/                 # (vendored) bag -> CSV export
    └── vicon_interfaces/           # Vicon message definitions
```

The packages written for this project are `crazyflie_controller`,
`spherical_ran`, `crazyswarm_bringup`, `crazyflie_debug_gui`,
`crazyflie_debug_interfaces`, `flight_analysis`, and `vicon_bridge.py`.
Everything else is vendored, and most of it has been patched locally — so don't
assume it matches upstream.

## Pipeline

State flows up from the mocap system, and commands flow back down to the radio.

```
  Vicon Tracker  (mocap cameras)
        │  DataStream SDK
        ▼
  vicon_client                        [vicon_receiver, separate launch]
        │  /vicon/<name>/<name>  (Position)
        ▼
  vicon_bridge.py                     [drops occluded/frozen frames]
        │
        ├──────────────► /poses  (NamedPoseArray) ────┐
        │                                             │
        └──────────────► TF: mocap -> <drone> ───┐    │
                                                 │    │
   ┌─────────────────────────────────────────────┘    │
   │                                                  ▼
   │                                        crazyflie_server   [crazyswarm2]
   │                                                  │  radio (cflib)
   │                                                  ▼
   │                                          Crazyflie EKF
   │                                        (external pose fusion)
   ▼
  controller_server  (one per drone)
        ▲   ▲   ▲
        │   │   └── /<drone>/debug_command   (DebugFlags, from GUI)
        │   └────── /<drone>/desired_heading (Vector3, from RAN server)
        └────────── /cmd_vel                 (Twist, from teleop)
        │
        ▼
   /<drone>/cmd_velocity_world   (real)      ──► crazyflie_server ──► radio
   /<drone>/cmd_full_state       (sim)
```

A few other things run alongside that main path:

- **`spherical_RAN_server`** runs once per RAN-enabled drone, publishing
  `<drone>/desired_heading` and `<drone>/ran_viz` (a MarkerArray).
- **`crazyflie_debug_gui`** subscribes to telemetry and publishes
  `<drone>/debug_command`. It publishes rather than calling services on
  purpose — that way a hung drone can't freeze the window, and in particular
  can't freeze the emergency stop.
- **`ros2 bag record`** writes to `bags/flight_<timestamp>/` whenever
  `record:=true`.

### Command paths

There are two ways to command the drone, and only one of them can be active at
a time:

1. **High level.** The `takeoff` / `land` / `go_to` services, which are planned
   by the drone's onboard high-level commander.
2. **Streamed setpoints.** `cmd_velocity_world` at 50 Hz, coming from
   `controller_server.update()`.

`controller_server` keeps the setpoint stream behind an interlock called
`launch_requested`. Until a takeoff has actually been commanded, no setpoints
go out — which means picking the drone up off the floor can never accidentally
start the stream.

## Launch arguments

Both launch files share these unless noted otherwise.

| Argument | Default | Meaning |
|---|---|---|
| `real` | `true` | Hardware vs simulation |
| `drone_names` | `cf09` | Space-separated drone list |
| `ran_drones` | `cf09` | Which drones run a RAN server |
| `launch_height` | `0.5` | Takeoff altitude (m) |
| `hover_speed_real` | `0.0` | Real-drone hover speed |
| `hover_speed_sim` | `0.0` | Sim hover speed |
| `target_names` | `''` | Target drones/objects |
| `target_qualities` | `''` | Per-target quality, positional |
| `use_static_targets` | `false` | Use `config/static_targets.yaml` |
| `record` | `true` | Record a rosbag (real only) |
| `rviz` | `True` | Launch RViz (real only) |

## Safety

- The **first Ctrl+C** lands the drone if it's airborne, or ESTOPs it if it's
  already on the ground.
- A **second Ctrl+C** is an emergency stop. It cuts the motors immediately, and
  the drone *drops*.
- The GUI's ESTOP button is always live and never blocks on a service call.

Please read [`docs/real-flight.md`](docs/real-flight.md) before flying real
hardware.
