//
//    Copyright (C) 2026 linuxcnc-ethercat contributors
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

/// @file
/// @brief Sync Unit scheduling and phase arithmetic.
///
/// Pure functions with no HAL or EtherCAT dependencies, so they can be unit
/// tested directly.  All times are in ns.
///
/// The master cycle defines a tick grid anchored at the DC reference time
/// handed to the master at activation (`dc_ref_time`); tick n is the master
/// cycle scheduled n periods after it.  lcec counts ticks rather than
/// deriving them from the app time, which also carries execution delay.  IgH starts every
/// slave's SYNC0 on that same grid, so a Sync Unit with divider N and phase p
/// is exchanged on the ticks where `(tick - p) % N == 0`; with p = 0 those are
/// exactly the ticks a SYNC0 of period N * master cycle is aligned to.

#ifndef _LCEC_SYNCUNIT_H_
#define _LCEC_SYNCUNIT_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/// @brief Euclidean modulo, result in [0, m) for m > 0.
static inline int64_t lcec_su_mod(int64_t a, int64_t m) {
  int64_t r = a % m;
  return (r < 0) ? r + m : r;
}

/// @brief Division rounded to the nearest integer, for d > 0.
static inline int64_t lcec_su_round_div(int64_t a, int64_t d) { return (a >= 0) ? (a + d / 2) / d : -((-a + d / 2) / d); }

/// @brief Is a Sync Unit with this divider and phase exchanged on `tick`?
static inline int lcec_su_due(int64_t tick, unsigned int divider, unsigned int phase) {
  if (divider <= 1) {
    return 1;
  }
  return lcec_su_mod(tick - (int64_t)phase, divider) == 0;
}

#endif
