# Differences from `sittner/linuxcnc-ethercat`

This tree includes drivers for many devices that the original did not
support.  Common classes of device, like digital/analog input/output
boards should have much wider device support, and a number of
commnuity-created drivers have been merged into the tree.  See the
[device list](documentation/DEVICES.md) for the full list of known
hardware.

There are several differences between this version of
LinuxCNC-Ethercat and [the
original](https://github.com/sittner/linuxcnc-ethercat) that should
make life easier, but may make transitioning from older versions of
linuxcnc-ethercat more complicated.  If you're just *using* EtherCAT
with LinuxCNC, then you can safely ignore all of this.

Developers, though, should be aware:

1. In this version, all device-specific code
([`lcec_el1xxx.c`](src/devices/lcec_el1xxx.c), for example) lives in
`src/devices`, while it used to be in `src/`.
2. The mapping between Ethercat VID/PID and device drivers now lives
   in the device source files themselves, *not* in `lcec_main.c` and
   `lcec_conf.c`.  See example below.
3. There is no need to edit `Kbuild` when adding new devices.
4. A number of details around PDO setup have changed substantially.

In short, to add a new device, you should just be able to drop source
files into `src/devices` and everything should build and work, as long
as you make one minor addition to the source.  Near the top of the
`.c` file for your driver, add a block like this to replace the code
that was in `lcnc_main.c`:

```c
static lcec_typelist_t types[]={
  { "EL1002", LCEC_EL1xxx_VID, LCEC_EL1002_PID, 0, NULL, lcec_el1xxx_init},
  { "EL1004", LCEC_EL1xxx_VID, LCEC_EL1004_PID, 0, NULL, lcec_el1xxx_init},
  ...
  { NULL },
};

ADD_TYPES(types);
```

This is from `lcec_el1xxx.c`, your names will vary, of course.  The
first field is the string that identifies the device in
`ethercat.xml`, and the other fields match up with the fields that
used to be in `lcec_main.c`.

If your driver needs `<modParam>`s in `ethercat.xml` (like the AX* and
assorted TwinSAFE devices), then you'll need to define the module
parameters in your `.c` file as well, see
[`lcec_el6900.c`](src/devices/lcec_el6900.c) for an example.

Be aware that a number of drivers have been merged together,
particularly drivers for Beckhoff EL3xxx-series analog input devices
and Beckhoff EL185x/EK18xx/EP23xx digital combo devices.  Existing
configurations should keep working just fine, as all
externally-visible names should have been kept the same.

There are also a family of libraries for supporting basic digital and
analog input and output channels with a common interface, which should
make supporting new devices less complicated.  See
[`lcec_class_din.c`](src/devices/lcec_class_din.c),
[`lcec_class_dout.c`](src/devices/lcec_class_dout.c),
[`lcec_class_ain.c`](src/devices/lcec_class_ain.c), and
[`lcec_class_aout.c`](src/devices/lcec_class_aout.c).

