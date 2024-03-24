# Breaking Changes in LinuxCNC Ethercat

This isn't precisely a set of release notes or a changelog file; we
have a lot of detail in git logs, GitHub PRs, and GitHub release notes
that cover that.

Instead, this focuses on changes that *may break existing
configurations* or existing source code.  In general, we'd like to
avoid breaking things, but there are times where we find bugs in
drivers that don't have a clean fix, or (more likely) suggest that no
one ever actually *used* the feature in question.

Entries tagged with `[API]` will impact developers and drivers that
aren't checked into the LinuxCNC Ethercat tree, but shouldn't be
visible to users.

This list should only include deliberate breakage, not typical bugs.

## Pending

- Beckhoff EL5101 (and EL5102) calculate frequency incorrectly. See
  [#380](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/380)
- [API] `lcec_param*` now use param-specific types instead of
  overloading pin-specific types.  See
  [#375](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/375).
- [API] Memory allocation across LCEC has changed to use a set of
  `LCEC*ALLOCATE*` macros for improved error checking, instead of
  calling `hal_malloc` directly.  As part of this, failed memory
  allocations are now treated as immediately fatal, rather than
  triggering a cleanup attempt.  See
  [#376](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/376).

## v1.16.0 (2024-02-10)

- [API] PDO registration and initilization changed substantially.  We
  no longer need to pre-declare how many PDO entries we need as part
  of the device setup process.  As part of this, the `LCEC_PDO_INIT()`
  macro that was used by nearly every driver was removed.  See
  [#302](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/302).

## v1.4.1 (2024-01-10)

- Update EpoCAT to support FR4000 and break FR1000 support.  The
  manufacturer belives that there are no FR1000 users left.  See
  [#132](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/132)
  and
  [#137](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/137).
  
## v1.4.0 (2024-01-08)

- [API] Makefiles rewritten, will break patching.  We shouldn't
  generally need per-driver Makefile changes anymore.

## v1.1.0 (2023-12-31)

- [API] Device-specific code moved into `src/devices/`.  Device name
  to driver mapping completely revamped.  `LCEC_SLAVE_TYPE_T` removed.
  All drivers updated.  See
  [#75](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/75),
  [#76](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/76),
  [#88](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/88),
  [#89](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/89),
  [#90](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/90),
  [#91](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/91),
  [#92](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/92),
  [#93](https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/pull/93).

## Other (date unknown, but before February 2024)

- EL30xx analog devices now return results between 0.0 and 1.0, rather
  than 0.0 and 16.0, matching other Beckhoff analog devices.
