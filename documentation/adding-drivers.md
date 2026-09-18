# Adding New Drivers to LinuxCNC-Ethercat

Before writing a new driver, consider opening a new issue in the
[issue
tracker](http://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/new).
Use a subject like "Add support for Fooco AB-15", and then say that
you're intending to write it.  Ideally include a link to the
manufacturer's website, and maybe a very short description of the
hardware.

## Writing Drivers

Drivers are written in C and live in `src/devices/`.  Follow the
naming scheme that already exists.  Each driver needs to live in
`src/devices` and include a `types[]` array that defines the specific
devices supported by this driver:

```C
static lcec_typelist_t types[] = {
    {"EL7041", LCEC_BECKHOFF_VID, 0x1B813052, 0, NULL, lcec_el7041_init},
    {"EL7041-1000", LCEC_BECKHOFF_VID, 0x1B813052, 0, NULL, lcec_el7041_init},
    {"EP7041", LCEC_BECKHOFF_VID, 0x1B813052, 0, NULL, lcec_el7041_init},
    {NULL},
};
ADD_TYPES(types);
```

The `ADD_TYPES(types);` line is mandatory; it does the
behind-the-scenes work to make sure that the driver is linked in and
available.

You shouldn't have to edit any other files; the `Makefile` and all of
the LinuxCNC-Ethercat support code should pick up your new driver
automatically.

When a manufacturer makes several similar devices, try to produce a
single driver that covers them all, or at least can be trivially
extended to handle them in the future.  See the
[`lcec_el3xxx.c`](../src/devices/lcec_el3xxx.c) driver for one example
of how to do this.  If you need to distinguish between different types
of devices within your code, then consider using the `flags` field in
`types[]` (again, see `lcec_el3xxx.c`).  This field is purely for
driver use, so use it however works best for you.

See the [PDOs and syncs doc](pdos-and-syncs.md) for a discussion of
the various ways of mapping PDO entries in LinuxCNC-Ethercat.

If your device forgets configuration that `_init` wrote via SDO or SII
when it loses power (module lists, PDO assignment, feature bits), add a
`proc_reinit` hook so LinuxCNC-Ethercat can re-apply it when the
device returns to the bus.  See [Runtime slave
re-initialization](runtime-reinit.md).

### HAL pin and parameter access

LinuxCNC is migrating HAL to typed getter/setter accessors (upstream
PR #4099; the old direct-dereference API disappears at the announced
API break).  LinuxCNC-Ethercat supports both APIs at once through the
compatibility layer in `src/lcec_hal_compat.h` (already included via
`lcec.h`).  New driver code must:

- Create pins with `lcec_pin_newf()` / `lcec_pin_newf_list()` and
  params with `lcec_param_newf()` / `lcec_param_newf_list()`.  Never
  call `hal_pin_*_new*()` or `hal_param_*_new*()` directly.
- Read and write pins only through the `LCEC_PIN_*` macros
  (`LCEC_PIN_U32_SET()`, `LCEC_PIN_BIT_GET()`, or the type-dispatching
  `LCEC_PIN_SET()` / `LCEC_PIN_GET()`).  Never dereference a pin
  pointer.
- Declare param fields with the `lcec_param_*_t` typedefs
  (`lcec_param_s32_t`, etc.) and access them only through the
  `LCEC_PARAM_*` macros.
- Keep pin fields declared with the classic pointer types
  (`hal_u32_t *`, etc.); the compat layer handles the opaque new-API
  reference types internally.

See any converted driver (for example
[`lcec_deasda.c`](../src/devices/lcec_deasda.c), which uses both pin
and param macros) for how this looks in practice.

### Style points

- Run `clang-format` on your code.  There's a [default
  format](../.clang-format) specifier in the tree.  Feel free to argue
  about better defaults in a new issue on Github.
- Declare all functions as `static` whenever possible.  Most drivers
  don't need to export anything into the global C namespace for other
  code to link against.

## Contributing Drivers

The best way to contribute a new driver is to sent a Github pull
request with your new driver, along with any information needed to use
the driver.  A few things that would be good to add in the pull request:

- An explanation of which hardware the new driver is for, ideally with
  links to the manufacturer's site.
- Documentation in the `documentation/` directory that explains how to
  use the new driver, including any parameters or configuration
  needed.  Some devices are so trivial that this doesn't matter
  (digital in/out boards, for example), while others are probably
  unusable without documentation.
- An entry (or entries) in `documentation/devices/` that say which
  devices the driver supports.  These can be created automatically by
  `scripts/update-devicelist.sh`, but you'll want to edit the
  resulting file(s).  Please include your Github and/or forum handles
  so people can contact you if they have issues or questions in the
  future.


