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
the exchange — a marginal cable, connector, or an overloaded slave.
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
