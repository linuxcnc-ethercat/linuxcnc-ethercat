# Wecon VD3E Servo Drives

The `wecon` driver supports Wecon's VD3E single-axis EtherCAT servo
drive.

## Setup

In your XML file, you should have an entry somewhat like this:

```xml
<masters>
  <master idx="0" appTimePeriod="1000000" refClockSyncCycles="-1">
    <slave idx="0" type="VD3E" name="s"/>
  </master>
</masters>
```

See the [CiA 402](cia402.md) documentation for additional details
about how to configure CiA 402 devices in LinuxCNC.  At a minimum, you
will need to include the `cia402` HAL component.

Note the `refClockSyncCycles="-1"`; see the distributed-clocks caveat
below before changing it.

## Devices

- [VD3E](https://docs.we-con.com.cn/bin/view/Servo/) single-axis
  EtherCAT servo drive, vendor ID `0x00000eff`, product code
  `0x0d3e0001`.  CoE only (no FoE, no EoE).

The identity was read from live hardware and matches the vendor ESI
"Wecon VD3E EtherCAT Servo V1.15.0.xml" (revision `0x00000073`).

The related VD5 and VD5L drives share one product code (`0x0d510001`)
and differ only by revision, so supporting them needs revision
matching, not just another product code.  Neither has been on our
bench, and neither is supported by this driver yet.

### Testing status

The VD3E has been tested on live hardware: driven to OP on a
five-slave bus (three Inovance IS620N axes, the VD3E, and an Omron
NX-ECC202 coupler), where it runs the spindle of a 3-axis mill in
cyclic velocity mode.  Object 0x6502 reads `0x3AD` (`pp`, `pv`, `tq`,
`hm`, `csp`, `csv`, `cst`), and position, velocity, torque and
following-error feedback are live.

### Caveats

- **The drive assigns exactly one RxPDO and one TxPDO, and every
  mapping object holds at most 10 entries.**  Objects 0x1C12 and
  0x1C13 each hold a single entry (defaulting to 0x1701 and 0x1B01),
  and the mapping objects (0x1600, 0x1701, 0x1702 for outputs; 0x1A00,
  0x1B01 for inputs) accept at most 10 entries each.  This is a
  property of the device, not a tuning knob: a longer mapping is
  rejected and the slave never reaches OP.  The driver keeps within
  these limits; keep them in mind if you enable enough optional cia402
  features to need more PDO entries, or if you write your own PDO
  config for this drive.
- **Objects 0x608F (position encoder resolution) and 0x6092 (feed
  constant) do not exist on this drive** — the SDO reply is "object
  does not exist".  A master cannot learn the encoder resolution from
  the bus; it is 2^23 counts per revolution (23-bit absolute encoder),
  confirmed by hand on the shaft.  Take it from the ESI or the motor
  datasheet when scaling.
- **SDO writes must not use complete access.**  The drive NAKs
  complete-access SDO writes.  Other drives on the same bench (e.g. a
  Mitsubishi MR-J4) accept it, so this is per vendor, not per bus.
- **Distributed clocks converge only with master-to-slaves
  distribution (`refClockSyncCycles="-1"`).**  On the test bench,
  `dc-sync-diff` settled at about 103 ns after 40 s with `-1`; with a
  positive value the clocks did not converge at all.  The driver
  enables DC by default (`assignActivate` 0x300, SYNC0 at the
  application cycle time, from the ESI DC OpMode "DC"); the drive also
  offers Free Run.  Override with a `<dcConf>` element if needed.
- **Touch probe is not enabled.**  The probe objects (0x60B8..0x60BD)
  exist, and 0x60B8 even sits in the factory RxPDO, but the driver
  does not map them: they have not been run on hardware yet, and a
  wrong mapping costs the OP transition.

## Configuration

There are no Wecon-specific `<modParam>` options; the standard
[`cia402` modParams](cia402.md) apply.  The driver enables the
following cia402 features on top of the base set: opmode selection,
CSP, CSV, CST, target torque, actual torque, actual following error,
and the error code (0x603F ships in the factory TxPDO, so exporting it
costs nothing).

To override the default distributed-clock settings, use a `<dcConf>`
element on the slave as usual.

## Pins

The driver exports the standard cia402 pin set, prefixed with
`lcec.<master>.<slave-name>.srv-`, for example:

- `srv-cia-controlword`, `srv-cia-statusword`
- `srv-opmode`, `srv-opmode-display`
- `srv-actual-position`, `srv-actual-velocity`, `srv-actual-torque`
- `srv-target-position`, `srv-target-velocity`, `srv-target-torque`
- `srv-actual-following-error`, `srv-error-code`
- `srv-supported-modes`, `srv-supports-mode-{pp,pv,tq,hm,csp,csv,cst}`

See the [CiA 402 documentation](cia402.md) for the full list and
semantics.
