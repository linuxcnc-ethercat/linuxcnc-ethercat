# Driver for Delta ASDA-Ax-E Series Servos (A2 and A3-E)

The [`lcec_deasda`](../src/devices/lcec_deasda.c) driver supports Delta Automation ASDA-Ax-E series drives. 

It currently supports 2 basic families of devices:

- ASDA-A2-E series drives (type "DeASDA")
- ASDA-A3-E series of drives (type "DeASDA3")
- ASDA-B3-E Series of Drives (type "DeASDB3")

Note that to support the legacy driver, DeASDA at CSV mode is the default setting of the driver.

See the [source code](../src/devices/lcec_deasda.c) or [device
documentation](devices/) for precise details on which hardware is
currently supported. We aim to provide support for both CSV (cyclic synchronous velocity) and CSP (cyclic synchronous position) modes. 

## General Setup 
The servo setup is fairly basic.  To configure them, add a line
like this to your `ethercat.xml` file:

```xml
    <slave idx="1" type="DeASDA" name="x-axis"/>
```
type: specifies which device this is: A2 (type="DeASDA"), A3 (type="DeASDA3"), B3 (type="DeASDB3")

name: whatever you want to call this ethercat slave, e.g. "x-axis"

Example for A3 series:

```xml
    <slave idx="1" type="DeASDA3" name="A3"/>
```

To ensure that a change in operational mode does not occur during the linuxcnc runtime, mode setting is done using modParam in the configuration file. This setting is optional. The default mode is CSV (Cyclic Synchronous Velocity).

```xml
    <slave idx="1" type="DeASDA" name="x-axis">
      <modParam name="opmode" value="CSV"/>
    </slave>
```

The following `<modParam>` settings are available:

- `opmode`: controls the operational mode based on Object 6060h: Modes of operation.
  Supported modes are:
  - `CSV`: Cyclic Synchronous Velocity mode (Value for 0x6060: 9) DEFAULT when modParam not used.
  - `CSP`: Cyclic Synchronous Position mode (Value for 0x6060: 8)
- `enableDigitalOutput`: `true`/`false`, maps the digital outputs (A2 and A3 only, default `true`).
- `enableHM`: `true`/`false`, enables drive-internal homing (default `false`). See [Homing Mode](#homing-mode).
- `homeMethod`: homing method, written to Object 6098h at startup. See [Homing Mode](#homing-mode).
- `homeOffset`: home offset in PUU, written to Object 607Ch at startup. See [Homing Mode](#homing-mode).
- `homeTorqueLevel_pct`: A3 and B3 only. Torque level in % for homing on a hard stop (1 to 300), written to
  drive parameter P1.087 at startup. See [Homing Mode](#homing-mode).
- `homeTorqueTime_ms`: A3 and B3 only. Time in ms the torque has to stay above the level (2 to 2000), written
  to drive parameter P1.088 at startup. See [Homing Mode](#homing-mode).
- `eGearRatio`: E-Gear ratio, written to Object 6093h at startup. See [Setting Pulses per Rev with Egear](#setting-pulses-per-rev-with-egear).

The `opmode` values are case insensitive.

## Encoder count and Axis Movement Scale

### Motor Encoder resolution - Different between ASDA-A2 and ASDA-A3/B3 series - 

The standard motors that come with either series of drives differ slightly. A2 drives will have a default encoder resolution of 1,280,000, whereas A3 and B3 series (henceforth X3) motors have a resolution of 16,777,216. Note that X3 drives support both x2 and x3 series motors. The default value in this deasda driver is that of ASDA A2 series motors.

To set the encoder resolution use hal pin: %s.%s.%s.srv-pulses-per-rev (also know as pprev).
Example for A3:
```
lcec.0.A3.srv-pulses-per-rev 16777216
```

### Setting Pulses per Revolution with Scale

To set the correct scaling to account for ball screw pitch or gear reduction use hal pin: %s.%s.%s.pos-scale

For example: Using an A3 servo with 24 bit (16,777,216 count) encoder, directly coupled to a 5mm pitch ball screw:
```
lcec.0.A3.srv-pulses-per-rev 16777216
lcec.0.A3.pos-scale 5
```

### External encoder / full-closed loop - A2 and A3

A2 and A3 drives can use a second external encoder. Usually a linear encoder is used. The number of pulses of the external encoder per motor revolution can be set by hal pin %s.%s.%s.extenc-scale

The units are pulses per revolution and depend on:
1. The resolution of the scale (e.g. 5um, 1um, 0.1um)
2. the Linear distance moved by the axis per revolution of the motor

Example:
5mm (=5000um) pitch ball screw. 0.5um linear scale:
5000/0.5 = 10000
```
lcec.0.A3.extenc-scale 10000
```
Another example:
16mm (=16000um) pitch ball screw, 5um linear scale:
16,000/5 = 3200
```
lcec.0.A3.extenc-scale 3200
```

### Setting Pulses per Rev with Egear

ASDA drives use an electronic gear ("E-Gear") to translate between the position unit used on the bus
(PUU, "pulse of user unit") and encoder pulses:

```
encoder pulses = PUU * N / M
```

`N` is the numerator (Object 6093h sub 1, drive parameter P1.044) and `M` the denominator (Object
6093h sub 2, P1.045). All position objects of the drive (target position, actual position, home
offset) are in PUU, so one motor revolution is `encoder resolution * M / N` PUU.

In EtherCAT mode the drive resets the E-Gear ratio to 1:1 on every power cycle (unless P3.012.Z is
set to 1), which is what this driver assumes by default. To use a different ratio, set it with
the `eGearRatio` modParam, written as `numerator/denominator` (`numerator:denominator` also works):

```xml
    <slave idx="1" type="DeASDA3" name="A3">
      <modParam name="opmode" value="CSP"/>
      <modParam name="eGearRatio" value="16777216/100000"/>
    </slave>
```

This example makes an A3 motor with a 24 bit encoder use 100,000 PUU per revolution.

Notes:
- The driver adjusts the default of `srv-pulses-per-rev` to match: default encoder resolution *
  M / N (100,000 in the example above). If your motor does not have the default encoder resolution
  of the drive series, set `srv-pulses-per-rev` yourself to `encoder resolution * M / N`.
- Pick a ratio where `encoder resolution * M / N` is an integer, otherwise the position scaling
  is slightly off.
- The drive limits the ratio to 1 < N/M < 262144 on A3/B3 (N: 1 to 2^29-1, M: 1 to 2^31-1), see
  P1.044/P1.045 in the drive manual.
- The drive only accepts a new ratio while the servo is off. The driver writes it during startup.
- Changing the ratio re-defines the position system of the drive, so the axis needs to be homed
  again.
- Velocities (0.1 rpm) are not affected by the E-Gear ratio.

## Homing Mode

The drive can home itself (CiA 402 Homing mode, Object 6060h = 6), using the home switch, the
limit switches, the encoder Z pulse or a hard stop. Add `enableHM` to enable this:

```xml
    <slave idx="1" type="DeASDA3" name="A3">
      <modParam name="opmode" value="CSP"/>
      <modParam name="enableHM" value="true"/>
      <modParam name="homeMethod" value="1"/>
      <modParam name="homeOffset" value="0"/>
    </slave>
```

`enableHM` adds Objects 6060h (Modes of operation) and 6061h (Modes of operation display) to the
PDO mapping, so the driver can switch between homing mode and CSP/CSV while running, and adds
these pins:

```
bit   I/O lcec.0.A3.srv-home               | Start homing. The driver clears it when homing is complete.
bit   OUT lcec.0.A3.srv-homed              | Drive reports "homing is complete" (statusword bit 12 in homing mode)
bit   OUT lcec.0.A3.srv-homing             | Homing is running
bit   OUT lcec.0.A3.srv-homing-error       | Drive reports a homing error (statusword bit 13 in homing mode)
s32   OUT lcec.0.A3.srv-opmode-display     | Active mode of the drive (6061h): 6 = homing, 8 = CSP, 9 = CSV
float OUT lcec.0.A3.srv-pos-fb             | Position feedback for joint.N.motor-pos-fb, see below
float IN  lcec.0.A3.srv-pos-cmd            | CSV only: position command of the joint, see below
s32   IN  lcec.0.A3.srv-home-method        | Homing method (6098h)
u32   IN  lcec.0.A3.srv-home-velocity-fast | Speed during search for the switch (6099h sub 1)
u32   IN  lcec.0.A3.srv-home-velocity-slow | Speed during search for zero / the Z pulse (6099h sub 2)
u32   IN  lcec.0.A3.srv-home-accel         | Homing acceleration (609Ah)
```

The four `srv-home-*` setting pins start out with the values that are set in the drive (after
`homeMethod` has been applied). Changing a pin writes the new value to the drive. Don't change
them while homing is running.

Homing only starts once the drive has all four values of the pins. If the drive does not accept a
value, the driver keeps trying and homing does not start. `srv-home-method` has to be between -128
and 127; other values are not sent to the drive, and homing does not start with them either.

Units and values, see the description of the objects in the EtherCAT chapter of the drive manual:
- `homeMethod` / `srv-home-method`: 1 to 35 are the CiA 402 methods (15, 16, 31 and 32 are
  reserved), 35 defines the current position as the origin. A3 and B3 also accept -1 to -4 for
  homing on a hard stop. The drive default is 0, so a method has to be set before homing.
- `homeOffset`: PUU.
- `srv-home-velocity-fast` / `srv-home-velocity-slow`: 0.1 rpm by default (range 1 to 20000 and 1
  to 5000 on A3/B3).
- `srv-home-accel`: by default the time in ms to accelerate from 0 to 3000 rpm.

### Homing sequence

1. Turn the servo on as usual (`srv-switch-on`, `srv-enable-volt`, `srv-enable`).
2. Set `srv-home`. The driver requests homing mode, waits until the drive reports it in
   `srv-opmode-display` and then sets the homing start bit (controlword bit 4).
3. The drive moves on its own. `srv-homing` is on. The driver ignores `srv-pos-cmd` / `srv-vel-cmd`
   while the drive is not in CSP/CSV mode.
4. When the drive reports that homing is complete, the driver sets `srv-homed`, clears `srv-home`
   and switches back to CSP/CSV.

The driver only clears `srv-home` when homing is complete. If the drive reports a fault during
homing, the run is aborted and `srv-home` stays on; the driver does not start another run until
`srv-home` has been off. On a homing error `srv-homing-error` is set and `srv-home` stays on as
well; clear it to leave homing mode.

LinuxCNC takes a falling edge of `joint.N.index-enable` as "homing is done", so it is up to
LinuxCNC to end a failed run. Connect `srv-fault` to `joint.N.amp-fault-in`: LinuxCNC then aborts
homing on a drive fault and clears `joint.N.index-enable` itself, with the joint not homed.

Ending a homing run before it is complete, by a fault or by switching the machine off, makes
LinuxCNC report a following error for the joint: `srv-pos-fb` goes back to the actual position of
the drive, which has moved since homing started. The joint is not homed after that and has to be
homed again.

Homing changes the actual position of the drive (Object 6064h), so the position feedback jumps
when homing completes. The commanded position has to follow that jump, which is what LinuxCNC does
when it homes on an encoder index. So connect `srv-home` like an index-enable pin and use
`srv-pos-fb` as the position feedback:

```
net x-index-enable joint.0.index-enable <=> lcec.0.A3.srv-home
net x-pos-fb       lcec.0.A3.srv-pos-fb  => joint.0.motor-pos-fb
```

and set up the joint in the INI file to home on index without a switch search (`HOME_SEARCH_VEL = 0`,
a small non-zero `HOME_LATCH_VEL`, `HOME_USE_INDEX = YES`). `HOME_LATCH_VEL` is not used to move
the motor, the drive uses its own homing speeds.

`srv-pos-fb` is the actual position of the drive, in the same units as `srv-pos-cmd`. While
`srv-home` is set it reports `srv-pos-cmd` instead: the drive moves on its own during homing and
LinuxCNC would otherwise stop with a following error. `enc-pos` and `enc-pos-abs` always show the
real position.

#### CSV mode

In CSV mode the position loop is closed in HAL, usually with a `pid` component, and the drive only
gets a velocity. The driver then has no position command to report while the drive is homing, so
`enableHM` adds a `srv-pos-cmd` input in CSV mode as well. Connect the position command of the
joint to it, next to the PID, and connect `srv-home` to the `index-enable` pin of the PID as well:

```
net x-pos-cmd      joint.0.motor-pos-cmd => pid.x.command lcec.0.A3.srv-pos-cmd
net x-pos-fb       lcec.0.A3.srv-pos-fb  => pid.x.feedback joint.0.motor-pos-fb
net x-vel-cmd      pid.x.output          => lcec.0.A3.srv-vel-cmd
net x-index-enable joint.0.index-enable <=> lcec.0.A3.srv-home
net x-index-enable pid.x.index-enable
```

The driver sends a velocity of 0 instead of `srv-vel-cmd` while the drive is homing and until it
is back in CSV mode. In CSV mode `srv-pos-cmd` is not sent to the drive, it is only used for
`srv-pos-fb`.

After homing, the joint position is the position of the drive plus `HOME_OFFSET`.

Homing on a hard stop (methods -1 to -4, A3 and B3) also needs the drive parameters P1.087 and
P1.088. The drive takes the stop as reached when the motor torque has been above the level of
P1.087 (% of rated torque, drive default 1) for the time of P1.088 (ms, drive default 2000). Set
them with `homeTorqueLevel_pct` and `homeTorqueTime_ms`:

```xml
      <modParam name="homeMethod" value="-1"/>
      <modParam name="homeTorqueLevel_pct" value="5"/>
      <modParam name="homeTorqueTime_ms" value="100"/>
```

A level that is too low makes the drive take friction for the stop; start low and raise it until
the axis reliably reaches the stop.

See [examples/deasdb3-homing-csp](../examples/deasdb3-homing-csp) for a complete configuration with two
B3 drives in CSP mode, and [examples/deasdb3-homing-csv](../examples/deasdb3-homing-csv) for the
same machine in CSV mode.

## Exposed Pins Common

The following pins are exposed in both modes (CSV and CSP) and for all drives. The pin names below contain "0.A3" - this will depend on the naming in your xml file.

Encoder values (servo motor and external encoder at CN5)

_Encoder Pins_
```
u32   I/O lcec.0.A3.enc-ext-hi
u32   I/O lcec.0.A3.enc-ext-lo
bit   I/O lcec.0.A3.enc-index-ena
bit   OUT lcec.0.A3.enc-on-home-neg
bit   OUT lcec.0.A3.enc-on-home-pos
float OUT lcec.0.A3.enc-pos
float OUT lcec.0.A3.enc-pos-abs 
float OUT lcec.0.A3.enc-pos-enc
bit   IN  lcec.0.A3.enc-pos-reset
s32   OUT lcec.0.A3.enc-raw
u32   OUT lcec.0.A3.enc-ref-hi
u32   OUT lcec.0.A3.enc-ref-lo
u32   I/O lcec.0.A3.extenc-ext-hi
u32   I/O lcec.0.A3.extenc-ext-lo
bit   I/O lcec.0.A3.extenc-index-ena
bit   OUT lcec.0.A3.extenc-on-home-neg
bit   OUT lcec.0.A3.extenc-on-home-pos
float OUT lcec.0.A3.extenc-pos
float OUT lcec.0.A3.extenc-pos-abs
float OUT lcec.0.A3.extenc-pos-enc
bit   IN  lcec.0.A3.extenc-pos-reset
s32   OUT lcec.0.A3.extenc-raw
u32   OUT lcec.0.A3.extenc-ref-hi
u32   OUT lcec.0.A3.extenc-ref-lo
```
_Slave Status_
```
bit   OUT lcec.0.A3.slave-online
bit   OUT lcec.0.A3.slave-oper
bit   OUT lcec.0.A3.slave-state-init
bit   OUT lcec.0.A3.slave-state-op
bit   OUT lcec.0.A3.slave-state-preop
bit   OUT lcec.0.A3.slave-state-safeop
```
_Drive Status and Control_
```
bit   OUT lcec.0.A3.srv-at-speed
bit   IN  lcec.0.A3.srv-enable 
bit   IN  lcec.0.A3.srv-enable-volt
bit   OUT lcec.0.A3.srv-fault
bit   IN  lcec.0.A3.srv-fault-reset 
bit   IN  lcec.0.A3.srv-halt
bit   OUT lcec.0.A3.srv-on-disabled
bit   OUT lcec.0.A3.srv-limit-active
bit   OUT lcec.0.A3.srv-oper-enabled
u32   OUT lcec.0.A3.srv-operation-mode
bit   IN  lcec.0.A3.srv-quick-stop
bit   OUT lcec.0.A3.srv-quick-stoped
bit   OUT lcec.0.A3.srv-ready
bit   OUT lcec.0.A3.srv-remote
bit   IN  lcec.0.A3.srv-switch-on
bit   OUT lcec.0.A3.srv-switched-on
float OUT lcec.0.A3.srv-torque-rel
float IN  lcec.0.A3.srv-vel-cmd 
float OUT lcec.0.A3.srv-vel-fb
float OUT lcec.0.A3.srv-vel-fb-rpm
float OUT lcec.0.A3.srv-vel-fb-rpm-abs
float OUT lcec.0.A3.srv-vel-rpm
bit   OUT lcec.0.A3.srv-volt-enabled
bit   OUT lcec.0.A3.srv-warning
bit   OUT lcec.0.A3.srv-zero-speed
```
Notes:
- `srv-zero-speed` is true while the actual speed is below the zero speed detection level of the
  drive (Object 606Fh, drive parameter P1.038, default 10 rpm). The level is read from the drive at
  startup.
- `srv-limit-active` is statusword bit 11 (internal limit active). On A3 and B3 it is also true
  while the drive reports the positive or negative limit (statusword bits 14 and 15).

_Digital Inputs and Outputs at CN1_
```
bit   OUT lcec.0.A3.din-1
bit   OUT lcec.0.A3.din-2
bit   OUT lcec.0.A3.din-3
bit   OUT lcec.0.A3.din-4
bit   OUT lcec.0.A3.din-5
bit   OUT lcec.0.A3.din-6
bit   OUT lcec.0.A3.din-7
bit   OUT lcec.0.A3.din-home
bit   OUT lcec.0.A3.din-neg-lim
bit   OUT lcec.0.A3.din-pos-lim
bit   IN lcec.0.A3.dout-d01
bit   IN lcec.0.A3.dout-d02
bit   IN lcec.0.A3.dout-d03
bit   IN lcec.0.A3.dout-d04
```


## CSV Mode (Default)

When the CSV mode is chosen, the driver exposes the following additional pin:

```
float IN     lcec.0.A3.srv-vel-cmd| Connect to velocity command from either joint or PID
```

Note: 
- To support easy migration from legacy linuxcnc-ethercat, CSV mode has been made the default setting.
- x.x.x.srv-operation-mode is set to 9 by the driver 


## CSP Mode

When the CSP mode is chosen, the driver exposes the following additional pin:

```
float IN     lcec.0.A3.srv-pos-cmd| Connect to position command from either joint or PID
```

Note: 
- x.x.x.srv-operation-mode is set to 8 by the driver 
- The position of the drive is a 32 bit value in PUU. With the default E-Gear ratio of 1:1 and a
  24 bit encoder that is only +/-128 motor revolutions. Beyond that the position wraps around; the
  driver lets the command wrap the same way and extends the feedback (`enc-pos`, `enc-pos-abs`,
  `srv-pos-fb`) to 64 bit. For long axes, consider a lower resolution with `eGearRatio`.

## How to file a bug

Please file an
[issue](http://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues)
if you have any problems with this driver, including:

- hardware that should work but doesn't.
- unexpected limitations.
- features that the hardware supports but the driver does not.
