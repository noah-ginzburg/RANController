# Spherical Attractor Project

This is a Crazyflie swarm control workspace. It implements a spherical attractor
network — an extension of the more common ring attractor network — to produce
collective behavior in a group of drones, flown over Vicon motion capture using
Crazyswarm2.

Everything runs on ROS 2 Humble. All of `src/` is vendored as regular files
rather than git submodules, so a plain `git clone` gives you the whole
workspace with nothing extra to initialize.

## What's mine and what isn't

Most of `src/` is other people's work. The parts written for this project are:

| Mine | What it is |
|---|---|
| `RAN/` | **The model itself** — mean-field and neural RAN notebooks, the bifurcation sweep, and the writeup. This is the research; everything else is the machinery to fly it. |
| `src/spherical_ran/` | The SAN server — the model above, running live as a ROS 2 node, publishing a heading per drone |
| `src/crazyflie_controller/` | Per-drone flight controller: takeoff/land/goto, velocity streaming, command arbitration |
| `src/crazyswarm_bringup/` | Top-level orchestration — the launch files and all the config YAML |
| `src/crazyflie_debug_gui/` | The debug window: live telemetry, manual commands, ESTOP (`crazyflie_debug_interfaces/` is just its message package) |
| `src/flight_analysis/` | Post-flight analysis path — bag → CSV → plots |
| `src/ros2-vicon-receiver/vicon_bridge.py` | **One file inside a vendored package.** The frame filtering, `/poses`, and TF publishing are mine; the rest of that package is not. |

Everything else is external. Most of it has been **patched locally**, so don't
assume any of it matches upstream:

| Vendored | Upstream |
|---|---|
| `src/crazyswarm2/` | https://github.com/IMRCLab/crazyswarm2 — the vendored copy looks like an SML-flavoured fork rather than mainline; confirm before diffing |
| `src/ros2-vicon-receiver/` | https://github.com/andreacamisa/ros2-vicon-receiver |
| `src/motion_capture_tracking/` | https://github.com/IMRCLab/motion_capture_tracking |
| `src/ros2_unbag/` | https://github.com/ika-rwth-aachen/ros2_unbag |
| `src/vicon_interfaces/` | https://github.com/einstein07/vicon_interfaces |
| `src/ros2_fastDDS_vicon/` | https://github.com/einstein07/ros2_fastDDS_vicon |
| `CollectiPy/` | https://github.com/einstein07/CollectiPy |
| `crazyflie-firmware/` | https://github.com/bitcraze/crazyflie-firmware |

Two of those sit at the top level rather than in `src/`, and neither is a ROS 2
package:

- **`CollectiPy/`** is a separate decision-making simulation framework, not
  mine, checked out here for reference. It is **gitignored** — clone it
  yourself from the link above.
- **`crazyflie-firmware/`** is stock Bitcraze firmware, also not mine. It's
  here **purely for debugging** — reading the estimator and commander source
  when the drone does something inexplicable, and flashing a known version.
  Nothing in the workspace builds against it. Also **gitignored**.

Vendored code keeps its own license; see each package.

## Documentation

The task guides live in [`docs/`](docs/). If you're trying to actually do
something with this workspace, start there rather than here — and start with
the first row:

| Guide | Use it when |
|---|---|
| [`docs/Full-Setup-and-Launch.md`](docs/Full-Setup-and-Launch.md) | **Start here.** Vicon calibration through to a flying drone, start to finish, plus what broke on the last two flight days |
| [`docs/parameter-tuning.md`](docs/parameter-tuning.md) | Which number do I change, and how do I regenerate the kernel cache? |
| [`docs/simulation.md`](docs/simulation.md) | Running in sim, no hardware |
| [`docs/real-flight.md`](docs/real-flight.md) | Flying actual drones |
| [`docs/hardware-drone-info.md`](docs/hardware-drone-info.md) | Radio config, firmware, batteries |
| [`docs/troubleshooting.md`](docs/troubleshooting.md) | Symptom → cause → fix |
| [`docs/analysis.md`](docs/analysis.md) | Bags, CSV export, plotting |

[`docs/README.md`](docs/README.md) is the index, and also lists the writing
conventions to follow when adding a guide.

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
├── RAN/                       # the model itself: notebooks, mean-field, papers, writeup
├── bags/                      # rosbag recordings (gitignored)
├── cache/                     # cflib TOC cache (gitignored)
├── data/                      # CollectiPy config folders
├── docs/                      # task guides -- see above
├── CollectiPy/                # (external, gitignored) decision-making sim framework
├── crazyflie-firmware/        # (external, gitignored) stock Bitcraze firmware, debugging reference only
└── src/
    ├── crazyflie_controller/       # per-drone flight controller
    │   └── controller_server.py    #   takeoff/land/goto + velocity control
    │
    ├── spherical_ran/              # spherical RAN model -- the SAN server
    │   ├── spherical_RAN_server.py          # continuous attractor server
    │   └── generate_kernel_cache.py         # precomputes the RAN kernel
    │
    ├── crazyswarm_bringup/         # top-level orchestration
    │   ├── config/                     # controller / ran / launch_args YAML
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
    │   └── vicon_bridge.py         #   MINE: filtering + /poses + TF
    ├── motion_capture_tracking/    # (vendored) mocap tracking libs
    ├── ros2_unbag/                 # (vendored) bag -> CSV export
    ├── ros2_fastDDS_vicon/         # (vendored) FastDDS + Vicon setup guides
    └── vicon_interfaces/           # (vendored) Vicon message definitions
```

**A note on naming.** The model is referred to as the **SAN** (spherical
attractor network) in the newer docs and GUI, but the directories, packages, and
parameters still say **RAN** (`spherical_ran`, `ran_params.yaml`,
`ran_enabled`). They're the same thing. The rename hasn't been done yet, so go
by `RAN` when you're grepping.

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
  `record:=true`. `flight_analysis` reads those back afterwards.

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
| `drone_names` | `cf09` | Comma-separated drone list |
| `ran_drones` | `cf09` | Which drones run a RAN server |
| `launch_height` | `0.5` | Takeoff altitude (m) |
| `hover_speed_sim` | `0.0` | Sim hover speed (real hardware always gets 0.0) |
| `auto_launch` | `false` | Take off at startup instead of waiting for the GUI's Takeoff button |
| `target_names` | `''` | Weight overrides: which existing targets to reweight |
| `target_qualities` | `''` | The weights themselves, zipped positionally |
| `use_static_targets` | `false` | Use `config/static_targets.yaml` |
| `record` | `true` | Record a rosbag (real only) |
| `rviz` | `True` | Launch RViz (real only) |
| `teleop` | `false` | Opens a `teleop_twist_keyboard` window, and nothing else (real only) |
| `ran_enabled` | `true` | Whether the RAN server publishes `<drone>/desired_heading`. Independent of `teleop`; the debug GUI toggles it live |
| `teleop_speed` | `0.2` | Keyboard linear speed, m/s (real only) |
| `teleop_turn` | `0.5` | Keyboard turn rate, rad/s (real only) |
| `ws_setup` | `~/biodrone/install/setup.bash` | Sourced inside the teleop window (real only) |

## Safety

- The **first Ctrl+C** lands the drone if it's airborne, or ESTOPs it if it's
  already on the ground.
- A **second Ctrl+C** is an emergency stop. It cuts the motors immediately, and
  the drone *drops*.
- The GUI's ESTOP button is always live and never blocks on a service call.

Please read
[`docs/Full-Setup-and-Launch.md`](docs/Full-Setup-and-Launch.md) before flying
real hardware — in particular the two **What Got Fixed** sections at the end,
which cover the failures that actually bit us: dropped GoTos, stale headings,
and the Vicon problems that make a drone drift or lurch with nothing logging an
error anywhere.
