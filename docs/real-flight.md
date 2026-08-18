# Flying real drones

Read [`hardware-drone.md`](hardware-drone.md) and [`vicon-setup.md`](vicon-setup.md)
first — this guide covers the run procedure only, and assumes the drone and the
mocap system are already set up correctly.



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



## See also

- [`troubleshooting.md`](troubleshooting.md)
- [`analysis.md`](analysis.md)
