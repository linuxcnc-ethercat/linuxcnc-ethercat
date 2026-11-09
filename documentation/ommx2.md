# Omron MX2 VFD Driver

The `lcec_ommx2` driver supports the Omron MX2 VFD with Omron's 3G3AX-MX2-ECT EtherCAT module.

Documentation:

* Omron i574 [MX2/RX Series EtherCAT Communication Unit User’s Manual](https://files.omron.eu/downloads/latest/manual/en/i574_mx2_rx_ethercat_option_board_users_manual_en.pdf?v=1739370045)

## Setup

In your XML file, you should have an entry somewhat like this:

```xml
<masters>
  <master idx="0" appTimePeriod="2000000" refClockSyncCycles="1000">
    ...
    <slave idx="1" type="OMMX2" name="spindle"/>
	...
  </master>
</masters>
```

This is the first CiA 402 VFD supported by LinuxCNC Ethercat, and
higher-level support is still lacking.  Instructions TBD.

See the [CiA 402](cia402.md) documentation for additional details
about how to configure CiA 402 devices in LinuxCNC. 

## Devices

This driver was developed with a 2.2 kW first-generation ("v1") Omron
MX2, with hardware revision `V1.00` and software `V1.10`.  It's
currently unknown if later MX2s or newer firmware will change anything
critical.

### Caveats

- The MX2 has a very weird PDO layout; it only allows 2 PDO entries
  per PDO (vs 8-12 for normal devices), but it allows up to 512 (!) RX
  and TX PDOs, vs 2-4 on many cheaper devices.
- On my hardware, I get gripes in `dmesg` about reads from 0x6046
  using the wrong size.  I'm not sure what's up; I'm using 32 bit
  reads, CiA 402 says that should be 32 bits, `ethercat sdos` for the
  device shows that it should be 32 bits, and `etherat upload -t
  uint32` works fine.

## Configuration

The minimal parameters for Ethercat control to work are:

<pre>
Parameter   Name                Setting
A001        Frequency source    04 Option (i.e. Ethercat option module)
A002        Command source      04 Option
H004        Motor poles         set to number of poles of your motor. requires vfd power-cycle to take effect.
</pre>

The number of motor poles $p$ determines how VFD output frequency $f$ maps to motor RPM set by `srv-target-vl`:

$$RPM = \frac{120 \cdot f]{p}$$

Other parameters must be set to match your VFD and motor combination.

## Inputs

* `srv-cia-controlword` (u32)
    * set controlword to 10 for spindle stop
    * set controlword to 15 for spindle run
* `srv-target-vl` (s32)
    * target velocity in RPM
    * connect to spindle.N.speed-out (float) with conv_float_s32

## Outputs

* `srv-cia-statusword` (u32)
    * Bits 0-7 and 9 follow the cia402 standard, others not used.
    * shows 592   `0b1001010000`  when spindle is not running
    * shows 567   `0b1000110111`  when spindle is running fwd/rev
* `srv-actual-vl` (s32)
    * actual RPM, positive for forward, negative for reverse. The accuracy of this RPM depends on drive mode and may be 1-3% for open-loop volts-per-Hz control mode.
