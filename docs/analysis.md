# Post-flight analysis

## Where bags live

`real_drone.launch.py` records a bag whenever `record:=true`, which is the
default. Each run writes `bags/flight_<YYYYMMDD_HHMMSS>/`, containing a `.db3`
file and a `metadata.yaml`.

It records everything under `/<drone>/`, plus `/poses`.

`bags/` is gitignored — it had reached 916 MB before it was untracked — so back
up anything you care about somewhere outside the repo.

## flight_analysis

```bash
ros2 run flight_analysis flight_data                 # most recent bag
ros2 run flight_analysis flight_data <bag>           # a specific bag folder, e.g. ~/biodrone/bags/flight_20260820_144338
ros2 run flight_analysis flight_data --help
```

The flags you're most likely to want (`flight_data.py:313`):

| Flag | Purpose |
|---|---|
| `--drone NAME` | Drone name (default: inferred from the bag) |
| `--bags-dir DIR` | Where to look for bags |
| `--dump` / `--no-dump` | Export raw data alongside plots |
| `--dump-format {json,csv,both}` | Export format |
| `--topics T [T ...]` | Restrict to specific topics |
| `--out DIR` | Output directory |

It produces position and error plots next to the bag
(`flight_<ts>_position.png` and `flight_<ts>_error.png`), plus per-topic CSVs
when you ask it to dump.

### What the error plot actually measures

The idea is that **Vicon is the ground truth and the onboard estimate is the
thing being tested**. The error is `est − vicon` (`flight_data.py:272`):

| | |
|---|---|
| **Reference** | `/vicon/<name>/<name>`, mm converted to m |
| **Measured** | `/<drone>/est` — the onboard `stateEstimate` log block, 10 Hz |
| **Error** | `est − vicon`, per axis, plus 3D magnitude `\|d\|` |
| **Sign** | positive = onboard estimate reads **higher** than Vicon on that axis |

The two topics run at different rates, so Vicon is linearly interpolated onto
the est timestamps before the subtraction. Any est samples falling outside
Vicon's own time span get dropped rather than held flat at the endpoints —
holding them would invent error that isn't really there. The console output
tells you how many were dropped.

Keep in mind that this is an **estimator-quality** metric, not a control
metric. It answers the question "does the drone know where it is?" It doesn't
compare commanded position against achieved position, so on its own it can't
tell you whether the controller is tracking well.

There's one caveat on time alignment. `vicon_receiver/msg/Position` has no
header, so there's no capture timestamp available. The tool falls back to each
message's bag receive timestamp instead (`flight_data.py:2-8`), which includes
transport latency. Treat small constant offsets with suspicion as they may be a
timing artifact rather than real estimation error.

## ros2_unbag

This is the vendored bag → CSV (and other formats) exporter, which comes with
its own GUI. See `src/ros2_unbag/README.md`. It was what we used before the
`flight_analysis` script existed.

## What to look at after a bad flight

1. **The `est − vicon` error.** This is what separates an *estimator* problem
   from a *control* problem. Large error means the drone didn't know where it
   was, so no controller could have saved it. Small error during a crash means
   the estimate was fine and the fault is somewhere downstream — control,
   power, or mechanical.
2. **Gaps in `/poses`**, which indicate tracking loss. Cross-check them against
   the `flying blind` warnings in the console log.
3. **Yaw over time.** Steady drift means gyro-only estimation
   (`send_orientation: False`), while a sudden jump near ±180° points at the
   quaternion-sign issue.
4. **Where the log ends.** An abrupt stop mid-flight with no land command
   suggests a brownout rather than a control fault. See
   [`hardware-drone-info.md`](hardware-drone-info.md#brownouts).
5. **Inverted yaw and pitch**, which come from the Vicon x axis not being
   aligned with the drone's x axis.

## Gotchas

- The console log is **not** captured in the bag. Redirect it if you want to
  keep it: `ros2 launch ... 2>&1 | tee bags/run.log`
- Bag timestamps are wall-clock at launch, not at takeoff.

## See also

- [`troubleshooting.md`](troubleshooting.md)
