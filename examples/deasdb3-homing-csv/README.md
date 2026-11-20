# Delta ASDA-B3-E: two axes in CSV mode, homing by the drive

The same machine as [deasdb3-homing-csp](../deasdb3-homing-csp), but with the drives
in CSV mode: the position loop is closed in HAL with a `pid` component per
axis and the drives get a velocity. See that example for the homing settings
and what to check before the first run, and "Homing Mode" in the
[driver documentation](../../documentation/deasda.md) for how homing works.

| File | Content |
|---|---|
| `ethercat-conf.xml` | Bus cycle, distributed clocks, E-Gear ratio, homing method, hard stop torque level |
| `deasdb3-homing-csv.hal` | Position loops, drive enable, homing speeds, homing handshake |
| `deasdb3-homing-csv.ini` | Scaling, position loop gains, homing speeds, joint limits, homing sequence |

## Differences to the CSP example

- `opmode` is `CSV`.
- `joint.N.motor-pos-cmd` goes to the PID and to `srv-pos-cmd` of the drive.
  In CSV mode the driver does not send `srv-pos-cmd` to the drive, it uses it
  for `srv-pos-fb` while the drive is homing.
- `srv-pos-fb` is the feedback of the PID and of the joint.
- The PID output is the velocity command, `srv-vel-cmd`.
- `joint.N.index-enable` is connected to `srv-home` and to the `index-enable`
  pin of the PID, so the PID ignores the position jump at the end of homing.

The position loop gains are in `[DRIVE_*]` in the INI file: `P` = 50 1/s,
`FF1` = 1 and a velocity limit of 250 mm/s. They are a starting point. With
a wrong sign or a gain that is too high the axis runs away or oscillates as
soon as the machine is switched on, so keep a hand on the E-stop on the first
run.

## Run

```sh
linuxcnc deasdb3-homing-csv.ini
```
