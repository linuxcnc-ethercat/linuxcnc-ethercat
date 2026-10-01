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
///
/// A Sync Unit sent on tick t is received at the start of tick t + 1.  A Sync
/// Unit serviced from its own HAL thread therefore has the window from tick
/// t + 1 to the next send tick t + N to run its drivers and publish outputs.

#ifndef _LCEC_SYNCUNIT_H_
#define _LCEC_SYNCUNIT_H_

#ifdef __KERNEL__
#include <linux/string.h>
#include <linux/types.h>
#else
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#endif

/// @brief Euclidean modulo, result in [0, m) for m > 0.
static inline int64_t lcec_su_mod(int64_t a, int64_t m) {
  int64_t r = a % m;
  return (r < 0) ? r + m : r;
}

/// @brief Fold a value into (-m/2, m/2].
static inline int64_t lcec_su_fold(int64_t a, int64_t m) {
  int64_t r = lcec_su_mod(a, m);
  return (r > m / 2) ? r - m : r;
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

/// @brief Phase error of a Sync Unit HAL thread.
///
/// `unit_start` is the scheduled start of the unit thread's current cycle,
/// `bus_start` the scheduled start of the most recent bus tick and `bus_tick`
/// that tick's grid index, all from the same monotonic clock.  The unit
/// thread should start `offset` ns after the receive tick (send tick + 1)
/// begins.  Returns the error folded into one unit cycle; positive means the
/// unit thread runs late.
static inline int64_t lcec_su_phase_err(
    int64_t unit_start, int64_t bus_start, int64_t bus_tick, unsigned int divider, unsigned int phase, uint32_t period, int64_t offset) {
  int64_t cycle = (int64_t)divider * period;
  int64_t ticks_since_rx = lcec_su_mod(bus_tick - (int64_t)phase - 1, divider);
  int64_t pos = ticks_since_rx * period + (unit_start - bus_start);
  return lcec_su_fold(pos - offset, cycle);
}

/// @brief Period correction for a Sync Unit thread from its phase error.
///
/// Proportional with a gain of 1/4: the measurement uses scheduled start
/// times, so it carries no wakeup jitter and needs no filtering.  Positive
/// corrections lengthen the next period.  `limit` bounds the step; rtapi
/// clamps to period / 100 on its own as well.
static inline int32_t lcec_su_pll_correction(int64_t err, int32_t limit) {
  int64_t corr = -err / 4;
  if (corr > limit) {
    return limit;
  }
  if (corr < -limit) {
    return -limit;
  }
  return (int32_t)corr;
}

// ---------------------------------------------------------------------------
// Handoff between the bus thread and a Sync Unit thread.
//
// The bus thread runs at the highest priority and must never wait, so:
//
// - Data it writes (received images, the latest tick) goes through a seqlock.
//   Only readers retry, at most LCEC_SU_SEQ_READ_TRIES times.  A read that
//   never sees a stable sequence is reported as torn and the caller skips
//   that cycle.  When the writer can preempt the reader (all RT threads on
//   one CPU, as uspace pins them by default) the first retry succeeds; on
//   separate CPUs the bound keeps the reader from spinning, since nothing
//   enforces the pinning.
// - Data the unit thread writes (outputs) is double buffered: the unit fills
//   the buffer the bus is not reading, then publishes its index and a
//   generation.  The bus copies the published buffer and re-checks the
//   generation, so an image the unit started rewriting during the copy (only
//   possible with the threads on separate CPUs) is retried, never sent torn.
// ---------------------------------------------------------------------------

#define LCEC_SU_SEQ_READ_TRIES 8

static inline void lcec_su_seq_write_begin(uint32_t *seq) {
  __atomic_store_n(seq, *seq + 1, __ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_RELEASE);
}

static inline void lcec_su_seq_write_end(uint32_t *seq) { __atomic_store_n(seq, *seq + 1, __ATOMIC_RELEASE); }

static inline uint32_t lcec_su_seq_read_begin(const uint32_t *seq) { return __atomic_load_n(seq, __ATOMIC_ACQUIRE); }

static inline int lcec_su_seq_read_retry(const uint32_t *seq, uint32_t start) {
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  return (start & 1) || __atomic_load_n(seq, __ATOMIC_RELAXED) != start;
}

/// @brief Process image handoff of one Sync Unit.
typedef struct {
  size_t len;       ///< Image size in bytes.
  uint8_t *rx;      ///< Latest received image, written by the bus thread.
  uint32_t rx_seq;  ///< rx seqlock sequence, odd while writing.
  uint32_t rx_gen;  ///< Incremented for every received image; 0 = none yet.
  uint8_t *tx[2];   ///< Output images, written by the unit thread.
  int tx_idx;       ///< Buffer holding the latest outputs.
  uint32_t tx_gen;  ///< Incremented for every published output image.
} lcec_su_xfer_t;

/// @brief Bus thread: publish a received image.
static inline void lcec_su_rx_publish(lcec_su_xfer_t *x, const uint8_t *img) {
  lcec_su_seq_write_begin(&x->rx_seq);
  memcpy(x->rx, img, x->len);
  x->rx_gen++;
  lcec_su_seq_write_end(&x->rx_seq);
}

/// @brief Unit thread: copy the latest received image to `dst`.
/// @returns 0 with its generation in `*gen`, or -1 if no consistent copy
/// could be taken (`dst` then holds a torn image).
static inline int lcec_su_rx_fetch(lcec_su_xfer_t *x, uint8_t *dst, uint32_t *gen) {
  uint32_t seq;
  int tries;

  for (tries = 0; tries < LCEC_SU_SEQ_READ_TRIES; tries++) {
    seq = lcec_su_seq_read_begin(&x->rx_seq);
    memcpy(dst, x->rx, x->len);
    *gen = x->rx_gen;
    if (!lcec_su_seq_read_retry(&x->rx_seq, seq)) {
      return 0;
    }
  }
  return -1;
}

/// @brief Unit thread: publish an output image.
static inline void lcec_su_tx_publish(lcec_su_xfer_t *x, const uint8_t *img) {
  int idx = 1 - x->tx_idx;  // only this thread writes tx_idx

  // order the previous publish before these writes, as a seqlock writer does
  __atomic_thread_fence(__ATOMIC_RELEASE);
  memcpy(x->tx[idx], img, x->len);
  __atomic_store_n(&x->tx_idx, idx, __ATOMIC_RELEASE);
  __atomic_store_n(&x->tx_gen, x->tx_gen + 1, __ATOMIC_RELEASE);
}

/// @brief Bus thread: copy outputs published since `*sent_gen` to `dst`.
///
/// The unit stores the buffer index before the generation, so a take that
/// lands between the two copies the next complete image under the previous
/// generation and sends it once more on the next take.  The reverse order
/// could record outputs as sent that never were.
/// @returns 1 if new outputs were copied (and `*sent_gen` updated), 0 if the
/// unit has published nothing new, or kept publishing during every copy
/// attempt (`dst` may then hold a torn image and must not be sent).
static inline int lcec_su_tx_take(lcec_su_xfer_t *x, uint8_t *dst, uint32_t *sent_gen) {
  uint32_t gen, again;
  int tries;

  for (tries = 0; tries < LCEC_SU_SEQ_READ_TRIES; tries++) {
    gen = __atomic_load_n(&x->tx_gen, __ATOMIC_ACQUIRE);
    if (gen == *sent_gen) {
      return 0;
    }
    memcpy(dst, x->tx[__atomic_load_n(&x->tx_idx, __ATOMIC_ACQUIRE)], x->len);
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    again = __atomic_load_n(&x->tx_gen, __ATOMIC_RELAXED);
    if (again == gen) {
      *sent_gen = gen;
      return 1;
    }
  }
  return -1;
}

/// @brief The latest bus tick, published by the bus thread for the unit
/// threads phase-locking to it.
typedef struct {
  uint32_t seq;
  int64_t index;    ///< Grid index of the tick.
  long long start;  ///< Scheduled start of the tick, monotonic ns.
} lcec_su_tick_t;

static inline void lcec_su_tick_publish(lcec_su_tick_t *t, int64_t index, long long start) {
  lcec_su_seq_write_begin(&t->seq);
  t->index = index;
  t->start = start;
  lcec_su_seq_write_end(&t->seq);
}

/// @returns 0 on success, -1 if no consistent copy could be taken.
static inline int lcec_su_tick_fetch(lcec_su_tick_t *t, int64_t *index, long long *start) {
  uint32_t seq;
  int tries;

  for (tries = 0; tries < LCEC_SU_SEQ_READ_TRIES; tries++) {
    seq = lcec_su_seq_read_begin(&t->seq);
    *index = t->index;
    *start = t->start;
    if (!lcec_su_seq_read_retry(&t->seq, seq)) {
      return 0;
    }
  }
  return -1;
}

#endif
