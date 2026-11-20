# Delta ASDA-B3-E: two axes in CSP mode, homing by the drive

LinuxCNC 2.9 configuration for two Delta ASDA-B3-E drives in CSP mode with the
[`DeASDB3`](../../documentation/deasda.md) driver. Each drive homes itself
onto a hard stop and then back to the motor Z pulse (Delta homing method 36,
written as `homeMethod` -1). LinuxCNC starts the run through
`joint.N.index-enable` and takes over the position when the drive is done.
See "Homing Mode" in the [driver documentation](../../documentation/deasda.md)
for how this works.

| File | Content |
|---|---|
| `ethercat-conf.xml` | Bus cycle, distributed clocks, E-Gear ratio, homing method, hard stop torque level |
| `deasdb3-homing-csp.hal` | Drive enable, position command/feedback, homing speeds, homing handshake |
| `deasdb3-homing-csp.ini` | Scaling, homing speeds, joint limits, following error, homing sequence |

## Before the first run

- Bus order: the first drive is X (joint 0), the second is Y (joint 1).
- The E-Gear ratio is set to 500000 PUU per motor revolution. The example
  assumes 5 mm of travel per motor revolution; change
  `[DRIVE_*]UNITS_PER_REV` for your mechanics.
- Joint limits are -600 .. 1 mm. Set them for your machine.
- Homing runs onto the hard stop at 30 rpm and back to the Z pulse at 10 rpm
  (`[DRIVE_*]HOME_SPEED_FAST` / `HOME_SPEED_SLOW`, in 0.1 rpm). The stop
  counts as reached when the torque stays above 5 % of rated torque for
  100 ms (`homeTorqueLevel_pct` / `homeTorqueTime_ms`). These values are a
  starting point: check the direction on the first run (`homeMethod` -2 is
  the other direction) and raise the torque level if the axis stops before
  it reaches the stop.
- The E-stop is a software loopback. Add a real E-stop chain before putting
  this on a machine.

## Run

```sh
linuxcnc deasdb3-homing-csp.ini
```

Release E-stop, switch the machine on, then "Home All". X homes first, then
Y (`HOME_SEQUENCE`).

While a drive is homing, `lcec.0.x.srv-homing` is true and
`lcec.0.x.srv-opmode-display` reads 6; afterwards it reads 8 again and
`lcec.0.x.srv-homed` is true.
