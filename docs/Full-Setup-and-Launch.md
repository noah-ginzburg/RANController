# Full Drone Setup and Launch Procedures

## Vicon setup

How to setup the Z708 Vicon for this project.

1. Plug in the vicon power cable
2. Log into the computer (pwd: fish-tank) and open Vicon Tracker 3.10 (green icon)
3. Calibrate the vicon
    - Navigate to the **Calibration** tab
    - Ensure arena is clear of obstacles
    - Hit **Start** under **Create Camera Masks**. Wait two seconds, then hit stop. The vicon will create a mask over the points it detects in the empty arena, thereby filtering out initial noise.
    - Turn the monitor towards the inside of the motion capture arena so you can see the calibration status as you wave the want. Grab the vicon calibration wand and flick the switch near the top-center to turn it on. 
    - Click **Start Calibration**, and wave the wand around. Ideally, wave the wand in all the places that the drone is expected to fly, otherwise the vicon will have to extrapolate it's calibration to find the drone's position. Keep waving the wand until all the cameras physically show a green light, or the cameras tab on the computer is fully green. As you're calibrating, you should see that each camera window on the ocmputer has a coloured sticker on the bottom right of the panel. As it gets more calibrated, it transitions from red-->green, and then disappears once fully calibrated.
        - If the cameras don't pick up the wand, try turning the wand off and on again. Sometimes the wand's light intensity can decay over time.
    - Click **Start** under **Set Origin**, then place the wand (turned on) where you want your origin to be. Note that the long bar/handle of the wand represents the **Positive Y-AXIS** and the short horizontal bar represents the **X AXIS**. If holding the wand such vertically that it looks like a (T), the **Positive X-AXIS** points to the **Left** of the horizontal bar T.  
    - Once the wand is in place, click **Stop**
    - **Keep the wand on the ground, but turn the lights off**
    - Navigate to the *Views* button which is at the top-left of the inner window that currently shows the camera calibration
    - Click **3D Perspective View** to make sure your origin was set correctly and the cameras are not inside the ground
4. Create a new subject
    - Place the drone and its markers inside the vicon arena. All points should be visible. If not, head to the cameras tab, click on the highest level **Vicon Camera System** button, and look for the **Strobe Intensity** parameter. Raise or lower it until all points are visible and not flickering.
        - **IMPORTANT**: Ensure that the **Drone's X-axis aligns with the Vicon's X-axis**! If not, the crazyflie will drift mid-air and hit a wall because pitch and roll will be offset because the axes are not aligned.
        - A good practice is to push the drone's body parallel against the top of the vicon wand. Ensure that the drone's *front* marker points towards the left side of the (T) shaped wand.
    - Locate the **Subjects Section**
    - **Hold ALT + LEFT CLICK**, and **drag the mouse** to capture all subject points
    - On the bottom left tab, enter the subject name as **cfXX**, where XX is the crazyflie drone number. The vicon will automatically create a topic called */vicon/cfXX/cfXX*, which is what the code expects.
    - Right click on the new subject in the subjects pane, hit save object, and if prompted, hit "shared". 

## Activate Vicon Receiver Script

1. Run ```colcon build --symlink-install``` to build the workspace, and ```. install/setup.bash``` to source your terminal. 
2. From your root directory, run ```ros2 launch vicon-receiver client.launch.py```
3. If successful and the subject was already created, it should say something like *Creating a topic for subject on /vicon/cfXX/cfXX*
4. Run ```ros2 topic echo /vicon/cfXX/cfXX``` to verify that Vicon data is coming through. If frames drop or the receiver stops working, simply terminate it with *CTRL+C* and restart the receiver

## Filtering the Vicon Data

`vicon_bridge.py` drops frames that would otherwise poison the EKF:

| Rejected | Why |
|---|---|
| NaN quaternion | garbage attitude OR drone flipped over |
| Non-unit quaternion (>0.1 off) | invalid rotation |
| Translation exactly `(0,0,0)` | Vicon's "lost the object" output; forwarding it teleports the estimator to the origin |
| Bit-identical repeat of previous frame | Tracker re-serves the last pose when tracking is lost; real tracking always has sub-mm noise |

Short gaps aren't a problem, since the onboard Kalman filter coasts through
them. A staleness watchdog warns after 0.25 s.


## Pre-Flight Checklist

- [ ] Battery **fully charged** (4.2 V, not 3.7 V)
- [ ] Drone reachable at its configured URI (if not, CrazyRadio will not connect and an error message will be printed in the launch logs)
- [ ] Vicon Tracker running, subject tracked, no occlusion at the takeoff spot
- [ ] Propellers are intact. Spin each propeller and make sure it spins straight, and you don't see two overlapping blurring areas (that means it's bent or the motor shaft is bent, and needs to be replaced). Ensure motors do not shake or vibrate when spinning propellors.
- [ ] drone sitting level on the floor
- [ ] Workspace built and sourced
- [ ] Everyone in the room knows where the ESTOP is (it's in the GUI)

## Editing Parameters before Flight

Everything worth changing lives in three YAML files under
`src/crazyswarm_bringup/config/`:

```
controller_params.yaml    # speeds, takeoff, landing, GoTo
ran_params.yaml           # attractor model gains, timing, RViz
launch_args.yaml          # which drones, takeoff height, record/rviz on-off
```

Edit a value, save, and relaunch. **No rebuild is needed** — `colcon build
--symlink-install` symlinks the config directory, so the launch files read your
source file directly. Every parameter has a comment above it saying what it does.

The ones you are most likely to touch before a flight:

| Parameter | File | What it does |
|---|---|---|
| `max_speed` | `controller_params.yaml` | How fast the drone flies toward its target (m/s) |
| `land_min_height` | `controller_params.yaml` | Below this height, Ctrl+C cuts the motors instead of landing |
| `collision_avoidance` | `ran_params.yaml` | Fly backwards if another drone gets within 0.3 m — turn this on if your targets are real drones |
| `startup_delay` | `ran_params.yaml` | How long the network waits before starting (this is most of the ~15 s wait before flight) |
| `u`, `sigma`, `kappa` | `ran_params.yaml` | The attractor model's gains |
| `drone_names`, `launch_height`, `record` | `launch_args.yaml` | Which drones fly, how high, whether a bag is recorded |

One warning before you start: changing **`n_sub`** or **`v`** means you must
regenerate the kernel cache, or the RAN server refuses to start.

Full details, including kernel cache generation, are in
[`parameter-tuning.md`](parameter-tuning.md).

## Launching the Drone

**Note on Stopping the Launch**:
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

**Launching the drone**:
1. Ensure workspace is built and sourced
2. Run ``ros2 launch crazyswarm_bringup real_drone_teleop.launch.py``. 
    - A teleop window, GUI menu, and RViz will open up. 
    - The drone will only be moveable by teleop, i.e. the spherical attractor netowkr will not publish, and the drone will not auto-launch
    - Look at the GUI menu, and ensure **Battery Voltage** is sufficiently higher than *3.7 V*, position error is not jumping too much, and the vicon is not dropping frames too much
    - Note that the drone's onboard estimator only returns its position at **5 Hz**, so the position error graph may jump back and forth a bit, but this is normal
3. Get ready to hit the **ESTOP**. When ready, in the GUI, hit **Takeoff**, and pray that the drone launches up and stays up. 
    - Ensure that the motors are somewhat balanced (look at M1-M4, M1-M3 split, M2-M4 split, and overall split). A small split is fine, a large split means new motor and/or propellor (weak motor).
4. Move it around with ros2's teleop terminal to confirm it can fly properly
5. When you are confident that the drone can fly properly, run the full launch script ```ros2 launch crazyswarm_bringup real_drone.launch.py```
    - The drone should auto launch in the air, wait around 15 seconds for the spherical attractor network to generate, and then fly towards the target by following the heading vector that was published by the SAN. 
    - It is recommended to use invisible movable markers as targets in rviz rather than real drones. If using real drones, enable the **Collision Avoidance** parameter in the yaml file, which flies the drone backwards if it gets within 0.3m of another drone.


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


**Debugging**
See [`troubleshooting.md`](troubleshooting.md) for debugging tips, and [`analysis.md`](analysis.md) for instructions on analyzing flight data.