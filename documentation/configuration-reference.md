# LinuxCNC Ethercat XML Configuration Reference

LinuxCNC-Ethercat needs an XML configuration file to tell it about the
EtherCAT hardware in the system.  This configuration file provides
naming information and tells it how to map specific LinuxCNC pin names
to specific hardware.

LinuxCNC-Ethercat provides a [long list of drivers](DEVICES.md) for
specific hardware, and also provides a way to talk to *generic*
EtherCAT hardware, with all of the configuration specified in the
`ethercat.xml` configuration file.

In general, compiled-in drivers are easier to work with and can have
extra error checking, but they're harder to create in the first place.
The generic driver maps specific LinuxCNC
[pins](https://linuxcnc.org/docs/html/hal/intro.html) directly to
EtherCAT
[PDOs](https://infosys.beckhoff.com/english.php?content=../content/1033/bk51x0/2519207947.html&id=),
without needing any code.  In many cases this is sufficient to get
arbitrary hardware working.

The basic structure of `ethercat.xml` looks like this:

```xml
<masters>
  <master idx="0" appTimePeriod="1000000" refClockSyncCycles="-1">
    <slave .../>
	...
  </master>
</masters>
```

There is a `<masters>` tag at the top, which contains one or more
`<master>` tags, each of which describe an EtherCAT bus.  Each
`<master>` then has one or more slaves, each of which may have some
additional configuration provided.

LinuxCNC systems with multiple Ethernet interfaces may be configured
with EtherCAT devices on multiple interfaces; in this case there will
be multiple `<master>`s listed.

## Master Configuration

The `<master>` tag has a few attributes, some of which are optional
and some of which are required:

- `idx="<number>"`: (optional, defaults to 0) the index number of the master that this refers
  to.  Systems with a single master (most common) will use `idx="0"`.
  When in doubt, run `ethercat slaves` and look at the addresses
  listed; systems with multiple masters will have sections labeled
  `Master0`, `Master1`, and so forth.
- `name="<name>"`: (optional, defaults to `idx`) the name of the master.
- `appTimePeriod="<time>"`: (optional, defaults to 1,000,000 = 1 ms) how
  frequently EtherCAT masters are updated, in nanoseconds.  This should
  match the period set on the servo thread in the LinuxCNC `.hal` files.
  If this does not match the servo thread period, an error is logged at
  the first servo cycle.  Omitting this attribute uses the 1 ms default
  and the value is replaced at runtime with the actual HAL servo
  period when they differ.
- `refClockSyncCycles="<n>"`: (required) selects the distributed-clock
  sync mode. The sign picks the direction; the magnitude is the
  number of servo cycles between syncs.

  | Value | Mode | Behavior |
  |---|---|---|
  | `0` | none | no DC sync |
  | positive (e.g. `1000`) | **R2M** | every `n` cycles, master pushes its `app_time` into the DC reference slave (master is authoritative) |
  | `-1` | **M2R** | LinuxCNC's servo thread is pulled toward the DC reference clock by a bang-bang PLL (DC is authoritative). `-1` is the only valid negative value; the parser rejects anything else negative. |

  Most systems with at least one DC-sync'd slave (servo / stepper)
  want **M2R** — set `refClockSyncCycles="-1"`. See
  [Distributed Clocks](distributed-clocks.md) for which mode to pick
  and how to tune the PLL.

- `syncToRefClock="true|false"`: (optional, alternate spelling of the
  sign of `refClockSyncCycles`). `"true"` ↔ negative cycles (M2R),
  `"false"` ↔ positive cycles (R2M). Kept for back-compat — prefer
  the sign-based form. If you specify both they must agree, otherwise
  the parser errors out. (See [#471](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/471) for a redesign discussion.)

Generally, for "normal" systems with DC-sync'd drives, this will look like

```xml
  <master idx="0" appTimePeriod="1000000" refClockSyncCycles="-1">
```

## Slave Configuration

The `<slave>` tag has a number of attributes, some of which are only
usable with generic devices, and others which are usable with any
device:

- `type="<type>"`: (required).  The device type.  This should either
  be `generic` or one of the device types from the [device
  list](DEVICES.md), such at `EL1008` or `EP2338`.
- `idx="<index>"`: (required): The index number of the slave.  This
  should match the output from `ethercat slaves`.  This is how
  LinuxCNC-Ethercat matches configs to specific, physical devices.
- `name="<name>"`: (optional but strongly recommended, defaults to the
  value of `idx`): The name of the device.  This is used in naming HAL
  pins in LinuxCNC.
-  `vid="<vid>"`: (required for generic, usable but not recommended for others): the Vendor ID for the
   device.  You can determine this via `ethercat slaves -v`.
- `pid="<pid>"`: (required for generic, usable but not recommended for others): the product ID for the
  device.  You can also get this from `ethercat slaves -v`.
- `configPdos="true|false"`: (generic-only, optional): allow
  LinuxCNC-Ethercat to configure PDOs for the generic device.
  
Non-generic devices cannot use the generic-only options, but they have
an additional configuration mechanism available to them.  You can add
additional `<modParam name="<name>" value="<value>"/>` tags.  The
exact meaning of these is driver-defined.

It is possible to specify `vid` and `pid` for non-generic devices.
This overrides the built-in VID and PID that each `type` provides;
it's really only useful with the `basic-cia402` driver.
  
Generic devices generally have `configPdos="true"`, and then define a
set of `<syncManager>`, `<pdo>`, `<pdoEntry>`, and possibly
`<complexEntry>` tags that describe the PDOs used on the device.

Here is a simple example, for a Beckhoff EL1008 digital input device.
Note that there is a compiled driver for the `EL1008`, but it's possible to use
`generic` to talk to the hardware instead:

```xml
    <slave idx="2" type="generic" vid="00000002" pid="03fa3052" name="D2" configPdos="true">
      <syncManager idx="0" dir="in">
	<pdo idx="1a00">
	  <pdoEntry idx="6000" subIdx="01" bitLen="1" halPin="din-0" halType="bit"/>
	</pdo>
	<pdo idx="1a01">
	  <pdoEntry idx="6010" subIdx="01" bitLen="1" halPin="din-1" halType="bit"/>
	</pdo>
	<pdo idx="1a02">
	  <pdoEntry idx="6020" subIdx="01" bitLen="1" halPin="din-2" halType="bit"/>
	</pdo>
	<pdo idx="1a03">
	  <pdoEntry idx="6030" subIdx="01" bitLen="1" halPin="din-3" halType="bit"/>
	</pdo>
	<pdo idx="1a04">
	  <pdoEntry idx="6040" subIdx="01" bitLen="1" halPin="din-4" halType="bit"/>
	</pdo>
	<pdo idx="1a05">
	  <pdoEntry idx="6050" subIdx="01" bitLen="1" halPin="din-5" halType="bit"/>
	</pdo>
	<pdo idx="1a06">
	  <pdoEntry idx="6060" subIdx="01" bitLen="1" halPin="din-6" halType="bit"/>
	</pdo>
	<pdo idx="1a07">
	  <pdoEntry idx="6070" subIdx="01" bitLen="1" halPin="din-7" halType="bit"/>
	</pdo>
      </syncManager>
    </slave>
```

Compare with the PDOs listed in the [LinuxCNC-Ethercat device PDO
explorer](https://linuxcnc-ethercat.github.io/esi-data/devices/EL1008),
and you'll see that there's a nearly direct mapping from the PDO
documentation to the XML file.

In order to get this correct for any given device, you'll need to
refer to the manufacturer's documentation, and some trial and error
may be required.

### `<syncManagers>`

The `<syncManager>` tag sets up an EtherCAT sync manager.  (link
needed)

The following attributes are available:

- `idx="<idx>"`: the sync manager index, from the manufactuer's
  documentation.
- `dir="in|out"`: the direction for syncing.  Must be either `in` or
  `out`.  You will need multiple sync managers if your device handles
  both input and output.

### `<pdo>`

The `<pdo>` tag has a single attribute:

- `idx="<idx>"`.  This needs to match the manufacturer's
  documentation.  Beckhoff devices generally have values in the `1xxx`
  range.

### `<pdoEntry>`

The `<pdoEntry>` tag sets up a specific PDO entry and maps it to a
LinuxCNC pin.

Attributes:

- `idx="<index>"` the PDO index; usually between `0x6000` and `0x7fff`.
  See the manufacturer documentation or our [PDO
  explorer](http://linuxcnc-ethercat.github.io/esi-data/devices).
  This is implicitly assumed to be a hexidecimal number.
- `subIdx="<subIndex>"`: the PDO subindex, usually a small number
  between 2 and 0xff.  It is assumed to be a hexidecimal number.
- `bitLen="<bit length>"`: The number of bits in the PDO.  Must match
  the manufacturer's documentation.
- `halType="<type>"`: the numeric type used for this PDO's pin in
  LinuxCNC.  Options are:
  - `bit`: a single bit.
  - `s32`: a signed 32-bit integer.
  - `u32`: an unsigned 32-bit integer.
  - `float`: the value is treated as a floating point number in
    LinuxCNC, but is communicated as a signed 32-bit integer with the
    hardware.
  - `float-unsigned`: the value is treated as a floating point number
    in LinuxCNC, but is communicated as an unsigned 32-bit integer
    with the hardware.
  - `complex`: the type is composed of multiple sub-fields defined
    with a `<complexEntry>` tag.  *Not* a complex number.
  - `float-ieee`: the value is a 32-bit floating point number.
  - `float-double-ieee`: the value is a 64-bit floating point number.
- `scale="<scale>"`: tells LinuxCNC to multiple the read value by
  `<scale>` when reading and divide by `<scale>` when setting the
  value in hardware.
- `offset="<offset>"`: tells LinuxCNC to add `<offset>` when reading
  (and subtract when writing).
- `halPin`: the name of the HAL pin in LinuxCNC.  Has `lcec.<master
  name>.<slave name>.` prepended.  Do not specify for
  `halType="complex"`.

### `<complexEntry>`

In some cases, multiple logical values are packed into a single PDO.
The `<complexEntry>` tag provides a way to extract these into
individual pins.

Attributes:

- `bitLen="<len>"`: number of bits in the field.
- `halType="<type>"`: the HAL pin type, same as above.
- `scale="<scale>"`: same as above.
- `offset="<offset>"`: same as above.
- `halPin="<name>"`: same as above.

## Other tags, not yet documented. 

In addition to the above tags, there are a handful of others available
that will be useful in specific situations:

- `<dcConf>`: controls Distributed Clocks on EtherCAT.
- `<watchdog>`: controls EtherCAT watchdog timers, used to detect
  hangs and stop operation.
- `<sdoConfig>`: sets specific configuration settings ("Service Data
  Objects") on a slave.  This is frequently used to set
  device-specific configuration parameters, like the current limit for
  a stepper driver.  See hardware documentation.
- `<sdoDataRaw>`: contains the actual data written to SDO configs.
- `<idnConfig>`: sets the IDN config for a device.
- `<idnDataRaw>`: additional IDN configuration?
- `<initCmds>`: passes in a filename with additional init commands,
  see [`examples/initcmds/`](../examples/initcmds/).
