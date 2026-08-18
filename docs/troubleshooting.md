# Troubleshooting


## `Too many packets lost` in state [1]

```
[crazyflie_server.py-1] Got link error callback [Too many packets lost] in state [1]
```

State 1 is INITIALIZED, which means you're mid-TOC-download. That's the
highest-traffic phase of connecting, and it's where a marginal battery tends to
collapse.

**Check the battery voltage first.** Charge to 4.2 V and retry. See
[`hardware-drone-info.md`](hardware-drone-info.md#brownouts).

Other possibilities are that the drone is out of range, there's 2.4 GHz
interference, or the antenna is damaged.

---

## Drone answers on channel 80 when configured for 100

The EEPROM still holds the correct config, but the firmware couldn't read it at
boot and fell back to the compiled-in defaults.

To confirm, open cfclient → Connect → Configure 2.x while connected on channel
80. If it shows **ch 100 / `0xE7E7E7E7E8`**, nothing is corrupted — this is a
boot-time I2C read failure, which is almost always a brownout. Charge the
battery.

If it instead shows ch 80 / `E7E7E7E7E7`, the config block really did get
rewritten, and you'll need to set it back as described in
[`hardware-drone-info.md`](hardware-drone-info.md#changing-or-viewing-it).

If the dialog throws an error or shows zeros, suspect the EEPROM or I2C
hardware.

> Worth knowing: the radio config is read once in `configblockInit()` and never
> re-read, so the channel **cannot** change mid-flight. This failure only ever
> happens at boot, and it announces itself as a connection failure.

---

## `[Errno 16] Resource busy`

Another process is holding the radio, and it's usually cfclient.

```bash
ps -eo pid,cmd | grep -iE "cfclient|crazyflie_server"
```

Close whatever turns up. This isn't a USB or permissions problem.

---

## Radio not found at all

```bash
lsusb | grep 1915:7777              # dongle present?
ls -l /dev/bus/usb/<bus>/<dev>      # want root:plugdev, mode 0664
id | grep plugdev                   # you in the group?
```

The udev rules live at `/etc/udev/rules.d/99-crazyradio.rules`.

Rebooting the computer is almost never the fix.

---

## `Unable to convert call argument to Python object`

```
RuntimeError: Unable to convert call argument to Python object
  ... in _take_subscription
```

This is a message typesupport mismatch: Python message class bound against a
differently-built C library. It usually comes from a mixed-build overlay, where
an interface package was rebuilt without rebuilding the packages that depend on
it.

```bash
rm -rf build/ install/ && colcon build --symlink-install
```

Be aware that **this aborts landing.** The error surfaces out of `rclpy.spin()`
and is *not* a `KeyboardInterrupt`, so it bypasses the Ctrl+C handler entirely
and the node dies without attempting to land. It can also fire inside
`send_land_req`'s `spin_until_future_complete` and kill a land mid-descent.

The root cause hasn't been confirmed yet. `DebugFlags.msg` was checked and is in
sync with its installed copy.

---

## Drone crashed on takeoff

There are many things that can go wrong here. Hopefully this list provides some guidance

| Cause | Signature | Fix |
|---|---|---|
| **Brownout/Low Voltage** | Sudden reset, drops straight down or flips immediately; trouble booting/linking to drone over cfclient with crazyradio | Charge battery |
| **EKF yaw divergence/drone drifting and spiraling mid-air** | Mirrored control: Vicon X-axis and Drone X-axis are misaligned | Recreate the drone subject in Vicon with the crazyflie's front marker physically pointing in the same direction as the Vicon's X-Axis. 
| **ESTOP never fired** | Props keep spinning after Ctrl+C | Fixed 2026-08-13 — see below |
| **Motors/propellors are imbalanced/weak** | Drone flips immediately OR doesn't fly straight. Visible in debug GUI window under "motor split" in PWM section | Try to re-align propellors and push them back down onto their shaft, but replace motors/propellors if cracks or bent motor shafts are found.

---

## Props spin on the ground after Ctrl+C

A `Land` command activates the high-level commander, which spins up the motors
to fly a descent the drone doesn't actually need. `LAND_MIN_HEIGHT`
(`controller_server.py:29`) guards against this: below 0.5 m, the Ctrl+C path
ESTOPs instead of landing.

**Historical, fixed 2026-08-13.** The ground branch used to call
`self.get_logger()` inside `main()`, where there's no `self` at all. That raised
a `NameError` before `send_emergency_req()` could run, so the ESTOP never
fired — and the `NameError` then escaped the enclosing `except
KeyboardInterrupt`, skipping `destroy_node()` and leaving the drone armed.

---

## Drone lands blind / drifts during descent

`vicon_bridge` used to exit on the first Ctrl+C while the controller carried on
landing for another 5 seconds, so the EKF lost mocap partway through the
descent.

This is fixed: the bridge now ignores the first SIGINT and exits on SIGTERM,
and `sigterm_timeout` is 7 s, comfortably above `LAND_DURATION` (5 s).

If it comes back, check that those two values still satisfy
`sigterm_timeout > LAND_DURATION`.

---

## `no valid Vicon pose for Xs -- flying blind`

Either Vicon tracking was lost, or the upstream receiver went silent. Check for
occlusion, confirm `vicon_client` is actually running, and make sure the subject
names match `drone_names`. See [`Full-Setup-and-Launch.md`](Full-Setup-and-Launch.md).

The Wifi connection may be unstable. Try re-launching the vicon receiver client, and rebooting the cameras.

---

## Launch relaunches nodes / duplicates

`crazyflie_server` ignores SIGINT. With launch's default 5 s / 10 s timeouts it
lingers for about 10 s and then reappears as a duplicate.
`real_drone.launch.py:225` sets `sigterm_timeout` to 7 s and `sigkill_timeout`
to 2 s, so everything is dead roughly 9 s after Ctrl+C.

Before relaunching, check for survivors:

```bash
ps -eo pid,etime,cmd | grep -E "crazyflie_server|controller_server" | grep -v grep
```

## Hung Nodes that won't shut down after CTRL+C
```bash
pkill -f ros2 && ros2 daemon stop && ros2 daemon start
```
This command forcefully kills all ros2 processes and restarts the daemon. Resource all your terminals that you were using before and everything will work again

---
## Drone is Not Receiving any Info

Something is blocking packets from being sent/received, or decoded. An attempt to switch the battery may help, but reflashing the firmware is the best solution. Download `cfclient` with `sudo apt install cfclient`. Click Connect-->Bootloader and connect to the drone's URI with the CrazyRadio. Hit scan once the URI is typed in, and if its detected, hit connect. Then, underneath, click on cf, click on the 2026.04 firmware file, and hit program. Wait 30 seconds for firmware to be reflashed.