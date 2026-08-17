# Parameter Tuning and Kernel Cache Generation

Everything worth changing lives in three YAML files. Edit, save, relaunch.
**No rebuild needed** — the config directory is symlinked into the build.

```
src/crazyswarm_bringup/config/
├── controller_params.yaml    # the drone: speeds, takeoff, landing, GoTo
├── ran_params.yaml           # the attractor model: gains, timing, RViz
└── launch_args.yaml          # drone names, heights, record/rviz on-off
```

Every parameter has a comment above it explaining what it does. The files are
the reference — this page is just the workflow.

## Which file do I want?

| You want to change | File |
|---|---|
| How fast the drone flies (`max_speed`) | `controller_params.yaml` |
| Takeoff or landing behaviour | `controller_params.yaml` |
| The model's gains (`u`, `sigma`, `kappa`) | `ran_params.yaml` |
| Collision avoidance on/off | `ran_params.yaml` |
| Which drones launch, takeoff height, recording | `launch_args.yaml` |

If you can't find a parameter, it's probably in `launch_args.yaml` — that's
where the things you'd normally type on the command line live.

## Checking a change took effect

```bash
ros2 param get /controller_server_cf09 max_speed
ros2 param get /spherical_RAN_server_cf09 kappa
```

The node names include the drone name. You should get back the value you typed.

## Regenerating the kernel cache

The connectivity matrix is precomputed because it's an O(N²) job over every
pair of sphere nodes. You only need to regenerate it after changing **`n_sub`**
or **`v`** in `ran_params.yaml`. Nothing else affects it.

```bash
cd ~/biodrone
python3 src/spherical_ran/spherical_ran/generate_kernel_cache.py
```

### What good looks like

```
Read from .../config/ran_params.yaml: n_sub=3, v=0.3
Generating connection matrix for 642 nodes (n_sub=3)...
Saved kernel cache to src/spherical_ran/spherical_ran/kernel_cache.npz
```

The `n_sub` and `v` it echoes back should match what you just put in the YAML.
The script reads them from that file itself, so it can't disagree with the
server, and there's no way to override them on the command line.

If you skip this step the RAN server refuses to start, with `cached kernel
parameters do not match current parameters`. That's deliberate — running the
model on a kernel built for a different mesh would give quietly wrong headings
instead of an obvious failure.

Raising `n_sub` gets expensive fast: node count is `10 · 4^n + 2`, so going
from 3 to 4 is 642 → 2562 nodes, about 16x the work both in the generator and
on every tick of the live model.

## Gotchas

- **Keep the types the same.** Writing `50` where `50.0` was will be rejected
  as an integer where a double was expected.
- **`land_height`, `land_min_height` and `takeoff_max_ground_height` are safety
  limits.** Read their comments before changing them.
- **The model gains come from `RAN/mean_field_model_3d.ipynb`**, which is kept
  in sync by hand. Change them in both places or the notebook stops predicting
  what the drone does.
- **`dt` is an integration step, not a duration.** Noise scales with
  `sigma * sqrt(dt)`, so raising it speeds the network up *and* makes it
  noisier. Not a simple speed knob.

## See also

- [`Full-Setup-and-Launch.md`](Full-Setup-and-Launch.md) — the flight procedure these feed into
- [`troubleshooting.md`](troubleshooting.md) — when something is broken
- [`analysis.md`](analysis.md) — checking whether a change actually helped
