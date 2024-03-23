# PDOs and Syncs

*Disclaimer: I'm still somewhat new to EtherCAT, so I may be
mis-understanding something here.  There may be a performance impact
of some of these changes that I'm not aware of, but from a correctness
standpoint I'm fairly confident that this is accurate.*

If you look through the drivers in the LinuxCNC-Ethercat codebase and
how they handle PDOs, then you'll notice that different drivers have 3
different ways of doing the same basic operation: registering which
CoE objects need to be set up as PDO entries.  Some devices devote
almost no code to managing this, while others have multiple screenfuls
of PDO setup.  Why do they differ?  And what is the advantage of any
particular approach?

Hopefully, this should explain the current state of things.

To make these examples a bit more concrete, let's assume that we have
a device where we want to read an 8-bit value from objects `0x6000:01`
and `0x6010:01` at runtime.

## Style 1: just use `lcec_pdo_init()`

In this case, the code just calls `lcec_pdo_init()` to register our
interest in the PDO, and we're done.  It looks something like this:

```c
int foo_init(int id, struct lcec_slave *s) {
  ...
  lcec_pdo_init(slave, 0x6000, 0x01, &hal_data->channel1_os, NULL);
  lcec_pdo_init(slave, 0x6010, 0x01, &hal_data->channel2_os, NULL);
  ...
}
```

Specifically, we don't bother creating any sync objects or setting `slave->sync_info`.

If this style works for your use, then use this style.  It's the least
complicated, and for most devices it'll Just Work.  I'll explain why
in a minute.

## Style 2: manually create syncs using `ec_sync_info_t`, and then use `LCEC_PDO_INIT()`

The second case is a superset of style #1, but it adds a
manually-created sync object.  It looks like this:

```c
static ec_pdo_entry_info_t foo_out1[] = {
  // not part of this example
};

static ec_pdo_entry_info_t foo_in1[] = {
    {0x6000, 0x01, 8},  // Channel 1
    {0x6010, 0x01, 8},  // Channel 2
};

static ec_pdo_info_t foo_pdos_out[] = {
    {0x1600, 0, foo_out1},
};

static ec_pdo_info_t foo_pdos_in[] = {
    {0x1a00, 2, foo_in1},
};

static ec_sync_info_t foo_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT, 0, NULL},
    {2, EC_DIR_OUTPUT, 1, foo_pdos_out},
    {3, EC_DIR_INPUT, 1, foo_pdos_in},
    {0xff},
};

int foo_init(int id, struct lcec_slave *s) {
  ...
  lcec_pdo_init(slave, 0x6000, 0x01, &hal_data->channel1_os, NULL);
  lcec_pdo_init(slave, 0x6010, 0x01, &hal_data->channel2_os, NULL);
  
  s->sync_info = foo_syncs;
  ...
}
```

There are a *lot* of drivers like this in the LCEC tree.  This isn't
actually needed for many devices, and just makes the driver larger and
more complicated.

The point of setting `sync_info` is that it changes the device's
built-in PDO map, changing which entries are available to be synced
automatically to the master.  Most devices have a default PDO setup,
and for basic devices it almost always contains all of the objects
that we care about.  In these cases, there's no point in manually
setting PDOs.  It just complicates the driver for no real gain.

For any given device, you can check its default PDO mappings via
`ethercat pdos`, but you may need to power-cycle the device to see
what the defaults were before we overwrote them.  Or you can look at
[http://linuxcnc-ethercat.github.io/esi-data/devices/](http://linuxcnc-ethercat.github.io/esi-data/devices/),
which has the default maps for nearly all devices that we support.

If the default PDOs cover everything that you need without too much
extra data, then you probably don't want to bother with creating
syncs.  If you're modifying an existing driver, try commenting out the
`sync_info=...` line, power-cycle your Ethercat devices, and see if it
still works.

Failure is pretty obvious -- LCEC will fail at startup with an error
about being unable to map a PDO entry.

If you *need* to change PDO mappings, then feel free to use this
style, but don't just cargo-cult it blindly.

## Style 3: create syncs using `lcec_syncs_init()`, and then use `LCEC_PDO_INIT()`

The third style is similar to the second, but uses a wrapper library
to actually populate the sync structures:

```c
int foo_init(int id, struct lcec_slave *s) {
  ...
  lcec_pdo_init(slave, 0x6000, 0x01, &hal_data->channel1_os, NULL);
  lcec_pdo_init(slave, 0x6010, 0x01, &hal_data->channel2_os, NULL);
  
  lcec_syncs_init(&hal_data->syncs);
  lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
  lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_INPUT, EC_WD_DEFAULT);
  
  // Output, to match the example above.
  lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
  lcec_syncs_add_pdo_info(&hal_data->syncs, 0x1600);

  // Input
  lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_INPUT, EC_WD_DEFAULT);
  lcec_syncs_add_pdo_info(&hal_data->syncs, 0x1a00);
  lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x6000, 0x01, 8); // Channel 1
  lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x6010, 0x01, 8); // Channel 2

  s->sync_info = &hal_data->syncs.syncs[0];
  ...
}
```

There are three advantages to this style:

- It's shorter.
- It's easy to use logic to decide if you want to map (or not map)
  additional entries.
- It's harder to get wrong, as you don't need to manually count the
  number of PDOs and PDO entries.  All of the calculatable settings
  are managed automatically.

If you were trying to set up mappings for devices with variable
numbers of channels in a single driver, *and* the default mapping
didn't cover the objects that you wanted, then you'd want to use this
style, because it makes it easy to add additional entries in a loop.
Under the hood, it ends up creating the same structs as style #2.

This style is also used in `lcec_class_cia402`, because (a) CiA 402
defines a huge number of PDOs (b) which PDOs are mapped is kind of
random from device to device, and (c) we can't just map everything
because it'll fail at the first attempt to map an object that doesn't
exist on this specific device.  So we need to manually handle
mappings, *and* there are a lot of conditionals to control which
objects get mapped.

## Performance

If we don't explicitly set `sync_info`, then the Etherlab EtherCAT
master library that we use will simply sync all of the PDOs listed in
the device's default sync mappings.  If the device's default sync
setup lists 50 objects in its PDO config and we only want 1, then
we're going to end up doing extra work and slowing things down
slightly.  In this case, we *might* want to set PDOs ourselves anyway.
However, I don't think I've ever seen a device that fits this; if
anything, most seem to sync too few fields, not too many.

Does it matter if we sync too much data?  IMO, not usually, within
reason. Adding one extra byte to an existing frame traveling over 100
Mbps Ethernet takes an extra (8 bits / 100 Mbits/second) = 80ns.
Modern CPUs will probably only take a cycle or two extra, if even
that, depending on alignment and what else is being copied at the same
time.  So a few extra bytes probably don't matter at all.  Even a
hundred extra bytes would probably not make a difference in the vast
majority of cases in systems with a 1ms cycle time.  But, there's no
reason to be *overtly* wasteful here.

## Recommendations

If the device's default PDO mapping contains everything that we need
and not *too* much else, then just use it.  Don't set `sync_info`,
don't define our own syncs, just keep things simple.  If the code
tries to access something that isn't mapped, then it'll fail fast with
a mostly-readable error.

If the driver needs something that *isn't* part of the default PDO
mapping, *or* we need to pare down the number of mapped entries for
some reason, then prefer the third style.  It's less work overall and
harder to get wrong.

There's nothing *wrong* with the second style, but it's probably best
avoided in new code.
