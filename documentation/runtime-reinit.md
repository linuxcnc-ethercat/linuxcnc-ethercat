# Runtime slave re-initialization

Some EtherCAT devices keep part of their configuration only in RAM: it
is written by the master after every power-up and is gone after every
power cycle.  The first such device in LinuxCNC-Ethercat is the
Leadshine R3EC/R2EC modular coupler, whose module list (`0xF030`), PDO
assignment (`0x1C12`/`0x1C13`) *and* the two SII feature bits that
allow the master to write that assignment ("Enable PDO Assignment",
"Enable PDO Configuration") are all reset on every boot.

The IgH master already handles a slave that drops off the bus and
returns: it re-scans the bus, walks the slave through INIT, PREOP,
SAFEOP and OP again and re-applies everything *it* knows about
(sync managers, FMMUs, PDO mapping, DC settings, startup SDOs
registered with `ecrt_slave_config_sdo`).  What it cannot re-apply is
the configuration a LinuxCNC-Ethercat *driver* wrote in its `_init`
function with plain SDO or SII writes.  Before this feature that
configuration was one-shot: a coupler power-cycled while LinuxCNC was
running came back to OP with missing or stale inputs, and nothing
short of a LinuxCNC restart fixed it.

Runtime re-initialization closes that gap.  It needs the
linuxcnc-ethercat EtherCAT master fork (libethercat with
`EC_HAVE_REINIT_HOLD`, packaged from
<https://github.com/linuxcnc-ethercat/ethercat>); with a stock IgH
libethercat LinuxCNC-Ethercat still builds and runs, logs one warning
per affected slave at startup, and behaves as before.

## What happens

For every slave whose driver provides a `proc_reinit` hook, LinuxCNC-Ethercat
sets the master's per-slave `ReinitHold` feature flag at startup.  From
then on:

1. The slave loses power (or its cable).  The master's
   `slaves-responding` count drops, the slave's `slave-online`,
   `slave-oper` and `slave-state-*` pins go to 0, and the domain
   working counter (`lcec.<m>.wkc-state`) drops to "incomplete".
   Other slaves keep running.
2. The slave comes back.  The master re-scans the bus and starts the
   slave's configuration: INIT, mailbox setup, PREOP.  Because the flag
   is set and the slave is a *new instance* to the master, the
   configuration stops there.  The slave sits in PREOP with
   `slave-state-preop` = 1, and the new `slave-reconfig` pin goes to 1.
   The master logs `Holding in PREOP until the application confirms
   its re-initialization`.
3. LinuxCNC-Ethercat's re-initialization thread (a plain non-realtime
   thread inside the `lcec` component; the servo thread never blocks)
   notices the hold, re-applies the XML `<sdoConfig>` entries of the
   slave and then calls the driver's `proc_reinit`.  The log shows
   `slave <m>.<s> returned to the bus and is held in PREOP;
   re-initializing`.
4. On success LinuxCNC-Ethercat confirms to the master, which resumes
   the configuration (startup SDOs, PDO assignment, DC, SAFEOP, OP).
   `slave-reconfig` returns to 0, `slave-reconfig-count` increments,
   `slave-oper` and `slave-state-op` go back to 1 and process data is
   live again.
5. On failure the slave stays held.  `slave-reconfig-error` goes to 1,
   the failure is logged, and the attempt is retried every 2 seconds.

A slave is never released to SAFEOP/OP with an unconfirmed
configuration.  If nothing confirms within the master's hold timeout
(30 s by default, `ReinitHoldTimeoutMs` feature flag) the master logs
`Re-initialization hold timed out`, sets the slave's error flag
(`ethercat slaves` shows `E`) and leaves it in PREOP.
`slave-reconfig-error` stays at 1.  LinuxCNC-Ethercat keeps retrying;
a later successful re-initialization clears the error, and the master
reconfigures the slave from scratch (it is held once more and
re-initialized once more, which is why `proc_reinit` must be
idempotent).

The first configuration at LinuxCNC startup is not affected: the
driver's `_init` runs before the master is activated, and
LinuxCNC-Ethercat confirms it before activation, so the slave passes
straight through PREOP as it always has.

## HAL pins

Every slave exports three additional pins, whether or not its driver
supports re-initialization (they stay at 0 otherwise):

| Pin | Type | Meaning |
|---|---|---|
| `lcec.<m>.<s>.slave-reconfig` | bit | The master holds the slave in PREOP, or `proc_reinit` is running |
| `lcec.<m>.<s>.slave-reconfig-error` | bit | The last re-initialization failed, or the master's hold timed out |
| `lcec.<m>.<s>.slave-reconfig-count` | u32 | Successful runtime re-initializations since LinuxCNC started |

Existing pins keep their meaning through the dip: `slave-online` /
`slave-oper` / `slave-state-*` follow the slave's real AL state,
`lcec.<m>.slaves-responding` follows the bus.  Configurations that gate
motion on `all-op` or a slave's `slave-oper` behave correctly: the
machine is not enabled while the slave is being re-initialized.

## Scenario: power-cycling a drive bus

An R3EC coupler with an encoder module sits on a 24 V bus that is
independent from the PC.  While LinuxCNC runs, that bus dips (thermal
trip, maintenance switch, someone re-plugging the coupler):

```
LCEC: slave 0.io returned to the bus and is held in PREOP; re-initializing
LCEC: slave 0.io: updating SII CoE details 0x03 -> 0x0f (word 0x0044)
EtherCAT 0-0: SII general category re-parsed after write: PDO assign enabled, PDO configuration enabled.
LCEC: slave 0.io re-initialized (1 so far); released to the master
EtherCAT 0-0: Re-initialization confirmed; resuming configuration.
```

A few seconds after power returns the coupler is back in OP, the
encoder pins update again, and `slave-reconfig-count` reads 1.  A rapid
double power cycle simply produces two rounds of the same sequence; a
re-initialization that was in flight when the coupler died fails on
the SDO writes and is retried on the new instance.

## Which drivers support it

| Driver | Devices | What `proc_reinit` re-applies |
|---|---|---|
| `lcec_leadshine_ec` | R2EC, R3EC couplers | SII CoE feature bits, `0xF030` module list, per-slot modparams, `0x1C12`/`0x1C13` PDO assignment; warns if `0xF050` (detected modules) differs from the configured list |

For every supported slave the XML `<sdoConfig>` entries (except
complete-access ones, which the master owns) are re-applied as well.

## For driver authors

Add a `proc_reinit` function to the driver's `types[]` entry (it is
the last field, after `sourcefile`, so set it by name):

```C
static int lcec_foo_reinit(lcec_slave_t *slave);

static lcec_typelist_t types[] = {
    {.name = "FOO", .vid = LCEC_FOO_VID, .pid = 0x1234, .proc_init = lcec_foo_init, .proc_reinit = lcec_foo_reinit},
    {NULL},
};
```

The hook runs in a non-realtime thread while the slave is in PREOP
with its mailbox up.  It must:

- re-issue every SDO / SII write `_init` performed (module lists, PDO
  assignment, feature bits, mode registers ...), typically by calling
  the same helper `_init` uses;
- be idempotent: it can run several times for one power cycle;
- not create HAL pins, call `lcec_pdo_init()`, or change the sync
  manager layout: the process image is fixed after activation;
- return 0 on success, or a negative value to keep the slave held.

SII feature bits are handled with `lcec_sii_update_coe_details()`
(read-modify-write of the CoE details byte in the SII general
category, locating the category by walking the SII rather than
assuming a fixed word offset), and `lcec_sii_read16()` /
`lcec_sii_write16()` for anything else.  See
[`lcec_leadshine_ec.c`](../src/devices/lcec_leadshine_ec.c) for a
complete example.
