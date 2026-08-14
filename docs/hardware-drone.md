# Drone hardware: radio, firmware, batteries

## Radio configuration

Each drone's channel and address have to match its entry in
`src/crazyswarm2/crazyflie/config/crazyflies.yaml`.

As of 2026-08-13, only `cf09` is `enabled: true`, at
`radio://0/100/2M/E7E7E7E7E8`.

### Where it's stored

The radio config is **not** in STM32 flash. It lives in an external I2C EEPROM
(a 24AA64) as a *config block*, containing the magic number `0x43427830`, a
version byte, the channel, datarate, address and trims, and a checksum.
`configblockInit()` validates that block on every boot, and if validation fails
it **writes the factory defaults** — channel 80, 2M, `0xE7E7E7E7E7`.

Two things follow from this. Reflashing the firmware doesn't normally reset the
radio config, and a drone you find sitting on channel 80 hasn't necessarily
been "reset" by anyone.

### Changing or viewing it

Install the cfclient apt package. You can connect either through the
**Crazyradio dongle** or with a **USB-C cable** straight to the drone's
microcontroller.

In cfclient, hit **Scan**, then go to **Connect → Configure 2.x**. (The menu is
called "Connect" and the entry reads "2.x" in cfclient 2026.4.) If the drone is
detected properly, the fields should populate on their own.

| Field | Value |
|---|---|
| Radio bandwidth | `2 MBit/s` |
| Radio channel | `100` |
| Radio address | `E7E7E7E7E8` (hex, no `0x` needed) |

If you change anything, click **Write** and then **power-cycle** the drone. The
dialog's "Restart needed" warning is literal — the radio keeps using the old
settings until it reboots.

### Verifying

```bash
python3 -c "
import cflib.crtp; cflib.crtp.init_drivers()
print('ch100/E8:', cflib.crtp.scan_interfaces(0xE7E7E7E7E8))
print('ch80/E7 :', cflib.crtp.scan_interfaces(0xE7E7E7E7E7))"
```

You want the first line non-empty and the second empty. Note that calling
`scan_interfaces()` with no argument probes **only** the default address, so it
won't find a correctly configured drone. Always pass the address explicitly.

## Batteries

Battery care is the single highest-value maintenance item here. For a 1S
Crazyflie pack:

| Resting voltage | State |
|---|---|
| 4.2 V | full |
| 3.8–3.9 V | comfortable |
| 3.7 V | ~40–50%, bottom half |
| 3.2 V | firmware low-battery warning (`pm.lowVoltage`) |
| ~3.0 V | cutoff |

That said, **resting voltage is the wrong number to judge a pack by.** What
actually matters is how much it sags under load:
`V_actual = V_battery − (current × internal_resistance)`. Takeoff current on a
240 mAh pack is brutal, and an aged cell whose internal resistance has grown
can drop 0.5–0.7 V instantly. Starting from 3.7 V resting, that lands you at
cutoff. A fresh pack at the same 3.7 V sags only about 0.2 V and flies fine —
and resting voltage alone can't tell those two packs apart.

The simplest rule is to keep the battery above 3.9 V. If it drops below that,
charge it.

### Brownouts

A brownout is the supply rail sagging below what the electronics need. It isn't
a loss of power so much as power that's *too low to work properly*, and the MCU
doesn't cleanly stop — it half-works. The symptoms look unrelated but all come
from the same cause:

- **At boot**, the EEPROM I2C read fails during `configblockInit()`, so the
  radio comes up on the compiled-in defaults (ch 80 / `E7E7E7E7E7`) **even
  though the EEPROM still correctly holds ch 100 / `0xE7E7E7E7E8`**. That split
  is the diagnostic signature: nothing got corrupted, the firmware just
  couldn't read it.
- **During connect**, you get `Too many packets lost` in state 1 (TOC
  download).
- **In flight**, the MCU resets and the drone falls — which looks exactly like
  a control bug.

Charge to 4.2 V before you conclude anything is wrong with the code.

This can also happen mid-flight, where the voltage drops sharply on takeoff and
the drone crashes.

## Firmware

You can read the version in cfclient's connection info, or query it directly:

```python
cf.param.get_value('firmware.revision0')  # + revision1 -> git hash
```

The drone tested while this doc was being written was on the latest firmware.
Older firmware probably still works, but that hasn't been checked.

To install the latest firmware, connect to the drone in cfclient using the
Crazyradio dongle, then go to **Connect → Bootloader**. Once connected, select
**cf2**, pick the latest firmware from the dropdown (currently 2026.04), and hit
**Program**. It takes about 30 seconds to upload.

### Estimator

`crazyflies.yaml:302` sets `stabilizer.estimator: 2` (Kalman) through
`firmware_params` at connect time. If you read the value as `1` (complementary)
*before* connecting, that's just the boot default and isn't a problem.

Make sure the drone is completely stationary when you power it on, so the IMU
can calibrate properly.

## Gotchas

- `cache/` holds the cflib TOC cache, keyed by firmware hash. It's safe to
  delete and regenerates on the next connect.
- Only one process can hold the radio at a time, so cfclient and
  `crazyflie_server` conflict. `[Errno 16] Resource busy` means something else
  already has it.

## See also

- [`troubleshooting.md`](troubleshooting.md)
- [`real-flight.md`](real-flight.md)
