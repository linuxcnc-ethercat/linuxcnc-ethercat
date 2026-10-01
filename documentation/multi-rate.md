# Multi-Rate Systems

One EtherCAT bus can serve devices that need very different rates: a
fast analog input at 8 kHz, servo drives at 2 kHz, digital I/O at
1 kHz, a serial gateway at 250 Hz.  Without help, the whole HAL then
has to run at the fastest rate.  [Sync Units](distributed-clocks.md#process-data-sync-units)
fix that: each group of slaves is its own EtherCAT domain, exchanged
every Nth bus cycle, and can be serviced from a HAL thread of its own
rate.  The bus thread stays small, and the drives and the rest of the
HAL run at the rate they actually use, phase-locked to the bus and
its distributed clocks.

This page walks through setting one up.  The reference material is in
[Distributed Clocks](distributed-clocks.md#process-data-sync-units),
[Master pins](master-pins.md#sync-unit-pins) and the
[configuration reference](configuration-reference.md).  A complete
example is in `examples/sync-units/multirate.hal` and
`multirate-conf.xml`.

## 1. Plan the rates

- **The bus runs at the fastest rate.**  `appTimePeriod` on `<master>`
  is the period of the thread that runs `lcec.read-all` and
  `lcec.write-all`.  Only devices that need that rate belong in its
  (default) Sync Unit.
- **Every other rate is a whole multiple of the bus period.**  With an
  8 kHz bus (125 us), 4 kHz, 2 kHz, 1 kHz and 250 Hz all work; 3 kHz does
  not.
- **Check the DC cycles your drives accept.**  Drives often only accept
  some SYNC0 cycles, and the rest show up as a drive fault after the
  slave reaches OP.  For example, Inovance SV660N servos run at 250 us and
  500 us but fault with error `0x6320` (parameter error) at 400 us, so
  a 5 kHz bus with 2.5 kHz drives cannot work with them.  Pick a bus rate
  whose multiples include a cycle every DC device accepts.

A typical layout:

| Sync Unit | Cycle | Divider | Serviced by | Contents |
|---|---|---|---|---|
| `default` | 125 us (8 kHz) | 1 | `ecat-thread` | fast analog input |
| `motion` | 500 us (2 kHz) | `*4` | `servo-thread` | DC servo drives |
| `io` | 1 ms (1 kHz) | `*8` | `io-thread` | digital I/O |
| `serial` | 4 ms (250 Hz) | `*32` | `ecat-thread` | serial gateway |

## 2. Assign slaves to Sync Units

Give every slave that does not belong in the default unit a
`syncUnit` and `syncUnitCycle`, and optionally a `syncUnitPhase`:

```xml
<master idx="0" appTimePeriod="125000" refClockSyncCycles="-1">
  <slave idx="0" type="EL3162" name="ain"/>
  <slave idx="1" type="generic" ... name="x-drive"
         syncUnit="motion" syncUnitCycle="*4">
    <dcConf assignActivate="300" sync0Cycle="*4" sync0Shift="20000"/>
    ...
  </slave>
  <slave idx="2" type="EL1809" name="din"
         syncUnit="io" syncUnitCycle="*8" syncUnitPhase="2"/>
</master>
```

- **Keep each DC slave's SYNC0 cycle equal to its unit's cycle.**  Here
  that is `sync0Cycle="*4"` for a `*4` unit.  For oversampling terminals
  that use SYNC0 as the sample clock, `sync0Cycle + sync1Cycle` must
  equal the unit cycle instead.
- **Choose the phase together with `sync0Shift`.**  A unit is exchanged
  on the bus cycles where `(cycle - syncUnitPhase) % divider == 0`, counted
  on the same grid IgH aligns SYNC0 to.  With phase 0 the frame leaves at
  the start of the cycle SYNC0 is aligned to, so `sync0Shift` must cover
  the frame's send latency (tens of us) for outputs to be latched by the
  SYNC0 right after it.
- **Spread slow units with the phase.**  Giving slow units different
  phases spreads their datagrams across bus cycles instead of stacking
  them on cycle 0.
- **Keep slaves that exchange coupled data together.**  For example, an
  FSoE logic device and its safety slaves must be in the same unit.

## 3. Build the HAL threads

Create one HAL thread per rate.  Each unit thread's period must equal
its unit's cycle exactly.  LinuxCNC gives faster threads higher
priority, so the bus thread preempts the others, which is required.

```
loadrt threads name1=ecat-thread period1=125000 name2=servo-thread period2=500000 name3=io-thread period3=1000000
loadusr -W lcec_conf ethercat-conf.xml
loadrt lcec

addf lcec.read-all ecat-thread
# ... consumers of the 8 kHz pins ...
addf lcec.write-all ecat-thread

addf lcec.0.syncunit.motion.read servo-thread
# ... motion and drive logic ...
addf lcec.0.syncunit.motion.write servo-thread

addf lcec.0.syncunit.io.read io-thread
# ... I/O logic ...
addf lcec.0.syncunit.io.write io-thread

initf lcec.activate ecat-thread
start
```

**Activate the master with `initf` in the bus thread.**  `initf lcec.activate
ecat-thread` runs the master activation once, in realtime context, right
before the bus thread's first cycle.  That fixes the DC reference time the
grid (tick 0) and every SYNC0 start from, and the unit threads lock to that
grid, so a clean activation is what keeps them in phase from the first cycle.
`initf` needs LinuxCNC 2.10 or later.  Without the line (or on 2.9, where
halcmd does not know `initf`), lcec activates inline in the bus thread's
first `write-all`, logs a warning on 2.10+, and DC starts with a dirty phase
that the PLL then trims.

With LinuxCNC's motion controller, `base_period_nsec` gives the bus
thread (`base-thread`) and `servo_period_nsec` the servo thread.

Rules:

- Add both of a unit's functs, `read` first and `write` last, to the
  same thread.  A unit with no functs added keeps running from the bus
  thread at its divider; the `serial` unit above does.
- Never add unit functs to the bus thread.
- Anything netted to a unit's pins should run in that unit's thread.
- Out-of-tree drivers must reach their PDOs through `lcec_slave_pd()`,
  not `master->process_data` (see [Adding drivers](adding-drivers.md)).
  A second after a threaded unit's slaves are operational, lcec logs an
  error naming any slave in it whose driver never called
  `lcec_slave_pd()`.

## 4. Tune the host

A bus thread at 4 kHz or more leaves little room for wakeup latency.
On top of the usual realtime setup (PREEMPT_RT kernel, RT cores taken out
of general scheduling with `isolcpus`/`nohz_full`/`rcu_nocbs`, IRQs kept
off them with `irqaffinity`, the `performance` governor), check:

- **`kernel.timer_migration` must be 0.**  With the default of 1, the
  kernel moves the hrtimers that wake the RT threads off `nohz_full`
  cores onto housekeeping cores.  The wakeups then arrive 100-250 us late
  every few seconds, whatever the RT priority.  On one Core Ultra 7 265K
  host, `cyclictest -i 125` measured a 227 us worst case with
  `timer_migration=1` and 2 us with 0.  Make it permanent with a sysctl
  file:

  ```
  # /etc/sysctl.d/90-rt-timer-migration.conf
  kernel.timer_migration = 0
  ```

  ```bash
  sudo sysctl -p /etc/sysctl.d/90-rt-timer-migration.conf
  ```

- **Measure before blaming the bus.**  Stop HAL and run
  `sudo cyclictest -m -q -p 98 -a <rt-cpu> -t 1 -i <bus period in us> -D 60`.
  The maximum must be well under the bus period, and a few us is
  achievable.  Don't run `latency-histogram --nox` while a HAL is up:
  in that mode it loads its threads into the running HAL and runs
  `halrun -U` when it exits.

## 5. Check it

Once all slaves are in OP:

| Pin | Expect |
|---|---|
| `lcec.0.all-op`, `lcec.0.dc-phased` | TRUE |
| lcec log | no "not activated via initf" warning |
| `lcec.0.syncunit.<unit>.threaded` | TRUE for every unit whose functs you added |
| `lcec.0.syncunit.<unit>.phase-locked` | TRUE within a fraction of a second; `phase-err` within a few hundred ns |
| `lcec.0.syncunit.<unit>.late-count`, `stale-count` | constant after lock |
| `lcec.0.syncunit.<unit>.wkc-state` | 2; `wkc-change-count` constant |
| drive status / error words | no fault (catches DC cycles a drive rejects) |

A short soak with `cycle_jitter`-style components first in each thread
and these counters sampled at the start and end is a good acceptance
test.

## 6. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `datagrams UNMATCHED` / `SKIPPED` in the kernel log, WKC drops on the fastest unit every few seconds | Wakeup latency above the bus period; check `kernel.timer_migration` and `cyclictest` (section 4) |
| A burst of `UNMATCHED` only at start-up | Master activation inside the RT thread; harmless |
| `phase-locked` never TRUE | Thread period differs from the unit cycle (an error is logged).  If several units share a thread, only the one that sets the thread's phase updates its phase pins |
| `late-count` keeps rising | The unit thread's work does not fit in its window of `divider - 1` bus cycles minus `phase-offset`; lower `phase-offset`, move work out, or use a longer unit cycle |
| Unit stops being sent, error "no outputs from its thread" | The unit's `write` funct is not running: missing, in another thread, or the thread stalled |
| Drives fault right after OP | SYNC0 cycle not supported by the drive (section 1) |
