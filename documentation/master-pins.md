# Master HAL Pins

Each master exports a set of HAL pins named `lcec.<master>.<pin>`
(e.g. `lcec.0.wkc`) that report the health of the EtherCAT bus as a
whole.  These complement the per-slave `slave-online` / `slave-oper` /
`slave-state-*` pins and the global `lcec.*` pins.

## State pins

| Pin | Type | Dir | Meaning |
|---|---|---|---|
| `lcec.<m>.slaves-responding` | u32 | OUT | Number of slaves responding on the bus |
| `lcec.<m>.state-init` / `state-preop` / `state-safeop` / `state-op` | bit | OUT | At least one slave is in this AL state |
| `lcec.<m>.all-op` | bit | OUT | Every slave is in OP |
| `lcec.<m>.link-up` | bit | OUT | Ethernet link is up |

## Working counter monitoring

Every cyclic exchange carries a working counter (WKC) that each slave
increments as it processes the datagram.  A WKC below the expected
value means one or more slaves did not exchange process data that
cycle.  Before these pins existed, WKC problems were only visible as
kernel log messages (`Domain 0: Working counter changed ...`).

| Pin | Type | Dir | Meaning |
|---|---|---|---|
| `lcec.<m>.wkc` | u32 | OUT | Working counter of the last domain exchange |
| `lcec.<m>.wkc-state` | s32 | OUT | Interpretation of the WKC: 0 = no data exchanged, 1 = some slaves exchanged, 2 = complete exchange (`ec_wc_state_t`) |
| `lcec.<m>.wkc-min` | u32 | OUT | Lowest WKC seen since the domain first reached a complete exchange |
| `lcec.<m>.wkc-change-count` | u32 | OUT | Number of times the WKC changed since the domain first reached a complete exchange |
| `lcec.<m>.wkc-reset` | bit | IO | Set to 1 to clear `wkc-min` / `wkc-change-count`; self-clears on the next cycle |

`wkc-min` and `wkc-change-count` only start tracking once the domain
first reaches a complete exchange (`wkc-state` = 2), so the normal
ramp-up during bring-up does not pollute them.  On a healthy bus,
`wkc` is constant, `wkc-min` equals `wkc`, and `wkc-change-count`
stays 0.  A rising change count with `wkc-min` dipping below the
steady-state value means a slave is intermittently dropping out of
the exchange: a marginal cable, connector, or an overloaded slave.
These transients last a cycle or two and are easy to miss by polling;
the counter catches them between samples, and netting `wkc` into
halscope or a recorder shows exactly when they happen.

Nothing is logged from the realtime thread when the WKC changes (a
flapping bus would log at cycle rate); the change counter is the log.

Setting `wkc-reset` to 1 clears both stats and re-arms the
first-complete-exchange gate, so `wkc-min` re-anchors on the next
complete exchange.  The pin clears itself once the reset is applied
(the same idiom as `encoder.N.reset`).

## DC synchrony monitoring

These pins report how tightly the slaves' distributed clocks agree
with each other, using the same mechanism as the `ethercat master`
CLI: a broadcast read of the "system time difference" register
(0x092C), which yields an upper estimate of the largest deviation
between any slave clock and the reference clock.  See [Distributed
Clocks](distributed-clocks.md) for what DC is and how to configure it.

| Pin/Param | Type | Kind | Meaning |
|---|---|---|---|
| `lcec.<m>.dc-sync-diff` | u32 | pin OUT | Upper estimate of the worst slave clock deviation, in ns.  Reads `0xffffffff` while no monitor responses are arriving |
| `lcec.<m>.dc-sync-converged` | bit | pin OUT | TRUE while `dc-sync-diff` is below `dc-sync-max`.  Forced FALSE after ~10 consecutive missing monitor responses (communication loss) |
| `lcec.<m>.dc-sync-max` | u32 | param RW | Convergence threshold in ns.  Default: `appTimePeriod / 25` (4% of the period) |
| `lcec.<m>.dc-sync-monitor` | bit | param RW | Enables the monitor (default 1).  Set to 0 to skip the per-cycle broadcast datagram entirely |

After a cold start, `dc-sync-diff` is large while the master
distributes the reference time, then converges; a DC-synchronized bus
typically settles in the tens-of-nanoseconds range.  If
`dc-sync-converged` never turns TRUE, check that your slaves actually
have DC enabled (`<dcConf/>`, see [Distributed
Clocks](distributed-clocks.md)) and that the servo thread PLL is
locked (`pll-err` small, `pll-reset-count` stable).

On communication loss (cable pulled, all slaves dead), the monitor
datagram stops returning.  A few consecutive misses are tolerated
(single datagram timeouts happen), after which `dc-sync-converged` is
forced FALSE and `dc-sync-diff` reads `0xffffffff` until responses
resume, so a dead bus can never keep showing a stale "converged"
state.

The monitor costs one broadcast datagram per cycle.  If you do not
use these pins, `setp lcec.0.dc-sync-monitor 0` reduces the cost to a
single branch (the dc-sync pins then hold their last values and
should be ignored).

## Time correlation

These pins let an external process map timestamps taken with
`clock_gettime(CLOCK_MONOTONIC)` into the DC time domain, for
example, to correlate a camera frame or an external sensor reading
(timestamped in OS time) with the cycle-sampled position of a
DC-synchronized drive.

| Pin | Type | Dir | Meaning |
|---|---|---|---|
| `lcec.<m>.app-time-lo` / `app-time-hi` | u32 | OUT | DC application time of the current cycle (ns, 64-bit split into low/high words) |
| `lcec.<m>.mono-time-lo` / `mono-time-hi` | u32 | OUT | `CLOCK_MONOTONIC` time sampled back-to-back with the app time (ns, 64-bit split) |

Both values are sampled adjacently in the write path each cycle, so
the pair is consistent to within a few tens of nanoseconds.  To map
an external monotonic timestamp `T` into DC time:

```
dc(T) = app_time + (T - mono_time)
```

The pair refreshes every cycle, so `T - mono_time` never needs to
span more than one period and clock drift over that interval is
negligible (measured host-vs-DC drift on a real bus is a few ppm,
i.e. single-digit nanoseconds across one 250 us period).  Do not
cache one pair and extrapolate for long spans: at a few ppm the
error grows by several microseconds per second.

Reading a 64-bit value split across two u32 pins from another
process is not atomic: read `hi`, then `lo`, then `hi` again, and
retry if `hi` changed (the low word rolls over every ~4.3 s).  For
the full four-pin set, re-read until two consecutive reads agree;
the values only change once per cycle, so one retry suffices.

## Sync Unit pins

A master with more than one [Sync
Unit](distributed-clocks.md#process-data-sync-units), or with a unit
slower than the master cycle, exports pins per unit named
`lcec.<m>.syncunit.<unit>.<pin>`.  The master-level `wkc` pins above
keep describing the whole process image; these describe one domain.
They update on the master cycles the unit is exchanged on.

| Pin | Type | Dir | Meaning |
|---|---|---|---|
| `...wkc` / `wkc-state` / `wkc-min` / `wkc-change-count` / `wkc-reset` | | | As the master `wkc` pins, for this unit's domain |
| `...fresh` | bit | OUT | TRUE in the cycles new input data for this unit was read, FALSE in the cycles in between |

A unit with a divider of 2 or more also exports `read` and `write`
functs (`lcec.<m>.syncunit.<unit>.read` / `.write`) to run it from a
HAL thread of its own cycle, and these pins and params for that mode:

| Pin/Param | Type | Kind | Meaning |
|---|---|---|---|
| `...threaded` | bit | pin OUT | TRUE once the unit's `read` funct runs; from then on its drivers run in that thread |
| `...late-count` | u32 | pin OUT | Send ticks on which no new outputs from the unit thread could be taken (none published, or it kept publishing during every copy attempt), so the previous outputs went out again.  After 10 in a row the unit stops being sent until its thread publishes again |
| `...stale-count` | u32 | pin OUT | Unit thread cycles that found no new input image |
| `...phase-err` | s32 | pin OUT | Start of the unit thread relative to its target, in ns; positive is late |
| `...pll-out` | s32 | pin OUT | Period correction applied to the unit thread, in ns |
| `...phase-locked` | bit | pin OUT | The unit thread has held its slot (within `appTimePeriod / 16`) for 100 cycles; drops beyond `appTimePeriod / 8` |
| `...phase-offset` | u32 | param RW | Target start of the unit thread after the master cycle that receives the unit's domain, in ns.  Default `appTimePeriod / 4` |

`late-count` and `stale-count` also count while the thread is still
locking after start-up.  Once `phase-locked` is TRUE both should stay
constant; a rising `late-count` means the unit thread's functions take
longer than the `N - 1` master cycles minus `phase-offset` they have.
`phase-err`, `pll-out` and `phase-locked` only exist with RTAPI PLL
support, and only the unit that sets its thread's phase updates them.
