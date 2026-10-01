# Distributed Clocks

EtherCAT has a feature called "[distributed
clocks](https://infosys.beckhoff.com/english.php?content=../content/1033/ethercatsystem/2469118347.html&id=)"
that seems to confuse many people on the LinuxCNC forum.  Distributed
clocks (when enabled) allow devices to stay synchronized to each other
with very small amounts of jitter (typically a few microseconds).
The EtherCAT master needs to be told to synchronize LinuxCNC's
servo thread to the distributed clock time.
This allows LinuxCNC's motion planner to tell EtherCAT servo and
stepper drives where to be *at a specific time* around 1 millisecond,
and have them almost exactly match the plan provided by the software.

## Background

EtherCAT is designed for low-latency, low-jitter control of industrial
equipment.  To accomplish this, it pays very close attention to timing
and delays across the network.  Running `ethercat -v slaves` will show
delay times between individual EtherCAT slaves.  On my system, typical
delays are around 150ns for EL* devices, 550ns for EP* devices, and
750ns for stepper and servo drives.

Individual devices can use a [few different
schemes](https://infosys.beckhoff.com/english.php?content=../content/1033/ethercatsystem/2469122443.html&id=)
for polling and communicating PDOs across the bus.  For devices like
digital inputs, exact timing may not matter, while for servo controls
precise timing is needed to manage speeds and motion with minimal
jitter.  At least some servo and stepper drives will not work without
distributed clocks.  This includes all of the Leadshine and RTelligent
drives that I've tested.

When running in CSP (cyclic synchronous position) mode, the motor
controller expects to receive a new target position every 1 milliscond
*precisely*.  LinuxCNC can then send each axis movement plans for 1ms
in the future and expect all motors to start their next segment
precisely on cue, all in sync with each other.

To close the time chain, the LinuxCNC servo thread must be
synchronized with the distributed clock time. It is inevitable
that the distributed clock and the LinuxCNC real-time clock
run at different speeds. The lcec module can adjust the
servo thread's time to the distributed clock reference time.
Unsynchronized distributed clock and servo thread clock are known to
lead to unpleasant noises from the drives. Users report "gravel noises",
"friction noises", and rapid peaks in torque and acceleration.
This only affects drives that use distributed clock synchronization.

## Master settings

DC sync is configured on the `<master>` tag with `refClockSyncCycles`.
The sign selects the mode, the magnitude is the cycle count.

```xml
 <master idx="0" appTimePeriod="1000000" refClockSyncCycles="-1">
  ....
 </master>
```

### `refClockSyncCycles`

| Value | Mode | Meaning |
|---|---|---|
| `0` | none | DC sync disabled |
| positive (e.g. `1000`) | **R2M** | every `n` servo cycles, the master pushes its `app_time` into the DC reference slave. The master is the authoritative clock. |
| `-1` | **M2R** | LinuxCNC's servo thread is pulled toward the DC reference clock by a bang-bang PLL. The DC reference is authoritative. |

For systems with DC-sync'd drives (most servo / stepper drives,
including all Leadshine and RTelligent drives I've tested), use **M2R** -
set `refClockSyncCycles="-1"`. `-1` is the only valid negative value;
the parser rejects anything else negative.

In M2R mode the cycle count carries no information beyond "M2R" - the
master-to-reference push is not performed at all. Inter-slave clock
distribution runs every cycle unconditionally via
`ecrt_master_sync_slave_clocks()` and is not configurable here.

### `syncToRefClock` (alternate spelling)

`syncToRefClock="true|false"` is an older equivalent of the sign of
`refClockSyncCycles` - `"true"` ↔ negative (M2R), `"false"` ↔
positive (R2M). Kept for back-compat. Prefer the sign-based form;
if both are given they must agree or the parser will refuse the
config. See [#471](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/471) for a redesign discussion.

Synchronization is done with a bang-bang controller. Two hal parameters
and three hal pins are available.

Hal parameters
- `pll-step="<n>" RW`. The adjustment step in nanoseconds. Default 0.1% of appTimePeriod.
- `pll-max-error="<n>" RW`. Max allowed time difference between the servo thread and
  the reference clock in nanoseconds before a reset. Default one appTimePeriod.

Hal pins
- `pll-err="<n>" OUT`. The current time difference between the servo thread
  and the reference clock in nanoseconds.
- `pll-out="<n>" OUT`. Current output correction, will always be +/-pll-step.
- `pll-reset-count="<n>" OUT`. Number of times pll-err has been larger
  than pll-max-error.

`pll-err` varies up and down. `pll-reset-count`should be kept low.
If `pll-reset-count` increases, the difference in speed between
the clocks is large. Increasing `pll-step` might help. `pll-step` is limited
to 1% of appTimePeriod.

To verify that the slaves' clocks themselves are in sync (as opposed
to the servo thread tracking the reference clock), see the
`dc-sync-diff` / `dc-sync-converged` pins in
[Master HAL Pins](master-pins.md#dc-synchrony-monitoring).


## Slave settings

Each slave in LCEC has its own (optional) distributed clock config,
which looks like this:

```xml
  <slave idx="27" type="generic" vid="00000a88" pid="0a880002" configPdos="true" name="rt-ect60">
    <dcConf assignActivate="300" sync0Cycle="*1" sync0Shift="0"/>
    ....
  </slave>

```

The `dcConf` XML tag controls distributed clock configs for this one
slave.  It takes 5 parameters, 3 of which are used in this example:

### `assignActivate`

`assignActivate` control which DC mode the device runs in, and should
generally be considered a device-specific setting.  For new devices,
look in the manufacturer's ESI file for `AssignActivate` and copy the
value from there.

Common values are `300` and `700`.  This is a 2 byte value, and the
higher-order byte (the `3` or `7` in these cases).

The low byte is written into `0x0980` and the high byte is written
into `0x0981`.  The best documentation for these that I've found so
far is
https://www.sanyodenki.com/global/america/file/SANMOTION_R_3E_EtherCAT_Servo_M0011697C.pdf

It says that `0x0981` has 3 defined bits:

0: Active cycle operation (when 1, DC is in use?)
1: SYNC0 is active
2: SYNC1 is active.

So, `assignActivate` values of `0x3xx` only enable SYNC0, while
`0x7xx` enable SYNC0 and SYNC1.

The low order byte (`0x0980`) also has 3 defined bits:

0: SYNC out unit control.  0: master, 1: slave.  Should be 0 for
EtherCAT?
4: Latch in Unit0 (0: master-controlled, 1: slave-controlled)
5: Latch in Unit1 (0: master-controlled, 1: slave-controlled).

The only values that I've seen used for `assignActivate` so far are 0
(no DC), 0x300, 0x320, 0x700, and 0x720.  Presumably 0x330 and 0x730
exist, and maybe 0x310 and 0x710.  Those are the only defined values
for this version of the spec.

### `sync0Cycle` and `sync1Cycle`

The EtherCAT distributed clock system has 2 different timing signals
available, `sync0` and `sync1`.  This controls the cycle time for each
signal, in units of 1ns (although EtherCAT itself may round this to
the nearest 10ns).  As a shortcut, LCEC will let you say `"*1"`
to set this to the current cycle time, or `"*X"`, where X is an
integer, to set this to an integer multiple of the current cycle time. 

I'm not sure if anything other than `sync0Cycle="*1"` ever makes sense
if we're using DC, but it's not unusual to have `sync1Cycle` unset,
set to `*1`, or set to a small multiple like `*3` to have Sync1 run
every 3rd cycle.  Exactly what this means depends on the hardware.

### `sync0Shift` and `sync1Shift`

These shift the Sync0 and Sync1 interrupts by a fixed number of ns.
Presumably shifting various devices slightly could result in reduced
jitter and less contention on the network, although it's not clear
that it really matters to us.  Many examples seem to just use 0.

## Process-data Sync Units

Distributed Clock cycles and process-data exchange cycles are separate.
The optional `syncUnit` and `syncUnitCycle` slave attributes group slaves into
separate EtherCAT domains that may be queued at different integer multiples of
the master cycle:

```xml
  <master idx="0" appTimePeriod="1000000" refClockSyncCycles="-1">
    <slave idx="0" type="generic" vid="00000002" pid="00000001"
           syncUnit="slow" syncUnitCycle="*2">
      ...
    </slave>
    <slave idx="1" type="generic" vid="00000002" pid="00000002"
           syncUnit="fast" syncUnitCycle="*1">
      ...
    </slave>
  </master>
```

Here the master still runs every 1 ms, but the `slow` domain is exchanged every
2 ms and the `fast` domain every 1 ms. All domains are exchanged every master
cycle during startup until the master reaches OP once. Omitting both attributes
keeps the previous behavior: the slave belongs to the `default` domain and is
exchanged every master cycle.

For a DC-capable slave, configure `dcConf` independently and keep its hardware
cycle consistent with the Sync Unit cycle: `sync0Cycle`, or `sync0Cycle +
sync1Cycle` for oversampling terminals that run SYNC0 as the sample clock. Slaves that exchange coupled data,
including an FSoE logic device and its safety slaves, should remain in the same
Sync Unit.

### Position on the DC grid

Master cycles are counted on the DC grid: cycle 0 is the application time
handed to the master at activation, which is also the time IgH aligns every
slave's SYNC0 to.  A Sync Unit with divider `N` (`syncUnitCycle="*N"`) and
`syncUnitPhase="p"` is exchanged on the master cycles where
`(cycle - p) % N == 0`.

For a DC slave whose `sync0Cycle` equals its Sync Unit's cycle, SYNC0 fires
`sync0Shift` ns after the start of phase-0 cycles.  So with phase `0` the
unit's frame leaves at the start of the master cycle SYNC0 is aligned to, and
`sync0Shift` must cover the frame's send latency for the outputs to be
latched by the SYNC0 right after it.  With phase `N - 1` the frame leaves one
master cycle before SYNC0.

The schedule is the same on every start.  (Before, slow units counted from the
cycle the bus first reached OP, so a unit's position relative to SYNC0 changed
from run to run.)

### Running a Sync Unit in its own HAL thread

For a step-by-step setup, including the host tuning a fast bus thread
needs, see [Multi-rate systems](multi-rate.md).

The master's `read-all`/`write-all` functs run in the thread that sets the
master cycle, normally the fastest one.  By default they also run the drivers
of every Sync Unit, so all pins update in that thread.  A Sync Unit with a
divider of 2 or more can instead be serviced from a HAL thread of its own
cycle, so that the bulk of a HAL configuration runs at the rate its devices
actually use while the bus runs faster.  For example, an 8 kHz bus for a
fast analog input, with the drives and motion at 2 kHz and the I/O at 1 kHz:

```xml
  <master idx="0" appTimePeriod="125000" refClockSyncCycles="-1">
    <slave idx="0" type="EL3162" name="ain"/>  <!-- default unit, 8 kHz -->
    <slave idx="1" type="generic" ... name="x-drive"
           syncUnit="motion" syncUnitCycle="*4">
      <dcConf assignActivate="300" sync0Cycle="*4" sync0Shift="20000"/>
      ...
    </slave>
    <slave idx="2" type="EL1809" name="din"
           syncUnit="io" syncUnitCycle="*8" syncUnitPhase="2"/>
    <slave idx="3" type="generic" ... name="serial"
           syncUnit="serial" syncUnitCycle="*32">
      ...
    </slave>
  </master>
```

```
loadrt threads name1=ecat-thread period1=125000 name2=servo-thread period2=500000 name3=io-thread period3=1000000
loadusr -W lcec_conf ethercat-conf.xml
loadrt lcec
addf lcec.read-all ecat-thread
# ... 8 kHz consumers of lcec.0.ain.* ...
addf lcec.write-all ecat-thread

addf lcec.0.syncunit.motion.read servo-thread
# ... motion, drive logic ...
addf lcec.0.syncunit.motion.write servo-thread

addf lcec.0.syncunit.io.read io-thread
# ... I/O logic ...
addf lcec.0.syncunit.io.write io-thread

initf lcec.activate ecat-thread
start
```

`initf lcec.activate` belongs in the bus thread: the unit threads phase-lock
to the DC grid the master starts at activation, so a clean activation matters
here.  It needs LinuxCNC 2.10 or later (see [the README](../README.md)).

The `serial` unit has no functs added, so its drivers keep running every 32nd
cycle of `ecat-thread`.

Once a unit's `read` funct runs, the master's functs stop running that unit's
drivers and only move its process image:

```
            send       receive              send
  ecat  ----|---------|---------|---------|---------|----   one tick = appTimePeriod
           tick p    p+1                 p+N
                       |<- phase-offset
  unit                 [read .. HAL .. write]
```

The unit thread works on a private copy of the image.  The master thread
copies each received image to it, and copies the outputs the unit last
published into the domain before sending.  Neither side ever waits for the
other: if the unit thread has not published new outputs by its next send tick,
the previous ones are sent again and `late-count` increments.  After 10
consecutive unit cycles without new outputs the unit is considered stalled and
is no longer sent, so its slaves' sync manager watchdogs take their outputs to
a safe state, as they would if the master stopped.  Sending resumes with the
next outputs the unit thread publishes.

The unit's `read` funct also phase-locks its thread to the bus.  LinuxCNC
starts every thread at its own arbitrary phase, so lcec adjusts the unit
thread's period (through the same RTAPI PLL the master uses) until the thread
starts `phase-offset` ns (default a quarter of `appTimePeriod`) after the
master cycle that receives the unit's domain.  That leaves the unit thread
`N - 1` master cycles, minus `phase-offset`, to run and publish its outputs.
Lock is reported on `phase-locked`.  Like the master's own PLL this needs a
LinuxCNC with `RTAPI_TASK_PLL_SUPPORT` (uspace); without it the unit thread
still works, at whatever phase it started.

Requirements:

- The thread's period must equal the unit's cycle.  Otherwise lcec logs an
  error and does not steer the thread.
- Add both `read` and `write`, in that order, to the same thread.  A `write`
  in another thread logs an error and publishes nothing.
- The bus thread must have the higher priority, which LinuxCNC does when its
  period is the shorter.  Never add unit functs to the master's own thread.
- Several units with the same cycle may share a thread; the first one whose
  `read` runs sets the thread's phase.
- Anything netted to a unit's pins should run in the unit's thread.
- Drivers must reach the process image through `lcec_slave_pd()`; all drivers
  in this tree do.  See [Adding drivers](adding-drivers.md).

The per-unit pins (`wkc`, `fresh`, `late-count`, `phase-locked`, ...) are listed
in [Master pins](master-pins.md#sync-unit-pins).

## Drivers and DC Clocks

Some devices (like RTelligent stepper drives) *only* seem to work in
DC mode.  To make configuring them less complex, the driver in
`lcec_rtec.c` automatically enables DC mode if `<dcConf/>` isn't
provided.  Since the DC parameters are largely just derived from
information in each device's ESI file, this should be able to be done
safely.
