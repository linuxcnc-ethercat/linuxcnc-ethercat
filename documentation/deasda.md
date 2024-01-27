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

To ensure that a change in operational mode does not occure during the linuxcnc runtime, mode setting is done using modParam in the configuration file. This setting is optional as it defaults to mode CSV.

```xml
    <slave idx="1" type="DeASDA" name="x-axis"/>
      <modParam name="opmode" value="CSV"/>
    </slave>
```

There is 1 `<modParam>` setting:

- `mode`: controls the operational mode based on Object 6060h: Modes of operation.
  Supported modes are:
  - `CSV`: Cyclic Synchronous Velocity mode (Value for 0x6060: 9) DEFAULT when modParam not used.
  - `CSP`: Cyclic Synchronous Position mode (Value for 0x6060: 8)
    
All of these settings are case insensitive.

## Differences of ASDA-A2 and ASDA-A3/B3 series

The stnadard motors that come with either series of drives differ slightly. A2 drives will have a default encoder resolution of 1,280,000 whereas A3 and B3 series (henceforth X3) motors have a resolution of 16,777,216. Note that X3 drives support both x2 and x3 series motors. The default value in the driver is that of ASDA A2 series motors.

To set the resolution use hal pin: %s.%s.%s.srv-pulses-per-rev  (also know as pprev). The following remark for further help on the topic.

### Setting Pulses per Rev with Egear 

ASDA series drive follow the concept of using egear to translate between encoder pulses and external input. This setting relates the encoder's pprev with the pitch of the ballscrew and additional transmission factors of belt drives if these are in place.

For a ball screw pitch of p = 5mm, a gearing ratio of 1:1 the calculcation would be as follows:

TODO()


## Exposed Pins Common

The following pins are exposed in both modes and for all drives

Encoder values (servo motor and external encoder at CN5)
```
37    I/O  lcec.0.A3.enc-ext-hi
37    I/O  lcec.0.A3.enc-ext-lo
37    I/O  lcec.0.A3.enc-index-ena
bit   OUT  lcec.0.A3.enc-on-home-neg
bit   OUT  lcec.0.A3.enc-on-home-pos
float OUT  lcec.0.A3.enc-pos
float OUT  lcec.0.A3.enc-pos-abs
float OUT  lcec.0.A3.enc-pos-enc
bit   IN   lcec.0.A3.enc-pos-reset
s32   OUT  lcec.0.A3.enc-raw
u32   OUT  lcec.0.A3.enc-ref-hi
u32   OUT  lcec.0.A3.enc-ref-lo
37    I/O  lcec.0.A3.extenc-ext-hi
37    I/O  lcec.0.A3.extenc-ext-lo
37    I/O  lcec.0.A3.extenc-index-ena
bit   OUT  lcec.0.A3.extenc-on-home-neg
bit   OUT  lcec.0.A3.extenc-on-home-pos
float OUT  lcec.0.A3.extenc-pos
float OUT  lcec.0.A3.extenc-pos-abs
float OUT  lcec.0.A3.extenc-pos-enc
bit   IN   lcec.0.A3.extenc-pos-reset
s32   OUT  lcec.0.A3.extenc-raw
u32   OUT  lcec.0.A3.extenc-ref-hi
u32   OUT  lcec.0.A3.extenc-ref-lo
```
_Slave Status_
```
bit   OUT  lcec.0.A3.slave-online
bit   OUT  lcec.0.A3.slave-oper
bit   OUT  lcec.0.A3.slave-state-init
bit   OUT  lcec.0.A3.slave-state-op
bit   OUT  lcec.0.A3.slave-state-preop
bit   OUT  lcec.0.A3.slave-state-safeop
```
_Drive Status and Control_
```
bit   OUT  lcec.0.A3.srv-at-speed
bit   OUT  lcec.0.A3.srv-di1 | RO status of the DIx pin  on CN1
bit   OUT  lcec.0.A3.srv-di2
bit   OUT  lcec.0.A3.srv-di3
bit   OUT  lcec.0.A3.srv-di4
bit   OUT  lcec.0.A3.srv-di5
bit   OUT  lcec.0.A3.srv-di6
bit   OUT  lcec.0.A3.srv-di7
bit   IN   lcec.0.A3.srv-enable  | needs to be high for the servo drive to operate
bit   IN   lcec.0.A3.srv-enable-volt | needs to be high for the servo drive to operate
bit   OUT  lcec.0.A3.srv-fault
bit   IN   lcec.0.A3.srv-fault-reset 
bit   IN   lcec.0.A3.srv-halt
bit   OUT  lcec.0.A3.srv-home
bit   OUT  lcec.0.A3.srv-limit-active
bit   OUT  lcec.0.A3.srv-neg-lim
bit   OUT  lcec.0.A3.srv-on-disabled
bit   OUT  lcec.0.A3.srv-oper-enabled
u32   OUT  lcec.0.A3.srv-operation-mode | return info on the operation mode (8=CSP, 9=CSV)
bit   OUT  lcec.0.A3.srv-pos-lim
bit   IN   lcec.0.A3.srv-quick-stop
bit   OUT  lcec.0.A3.srv-quick-stoped
bit   OUT  lcec.0.A3.srv-ready
bit   OUT  lcec.0.A3.srv-remote
bit   IN   lcec.0.A3.srv-switch-on | needs to be high for the servo drive to operate
bit   OUT  lcec.0.A3.srv-switched-on
float OUT  lcec.0.A3.srv-torque-rel
float IN   lcec.0.A3.srv-vel-cmd
float OUT  lcec.0.A3.srv-vel-fb
float OUT  lcec.0.A3.srv-vel-fb-rpm
float OUT  lcec.0.A3.srv-vel-fb-rpm-abs
float OUT  lcec.0.A3.srv-vel-rpm
bit   OUT  lcec.0.A3.srv-volt-enabled
bit   OUT  lcec.0.A3.srv-warning
bit   OUT  lcec.0.A3.srv-zero-speed
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

## How to file a bug

Please file an
[issue](http://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues)
if you have any problems with this driver, including:

- hardware that should work but doesn't.
- unexpected limitations.
- features that the hardware supports but the driver does not.