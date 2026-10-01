#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../../src/lcec_syncunit.h"
#include "tests.h"

TESTGLOBALSETUP;

#define T 125000  // master cycle, 8 kHz

TESTFUNC(test_su_mod_fold) {
  TESTSETUP;

  TESTINT((int)lcec_su_mod(5, 4), 1);
  TESTINT((int)lcec_su_mod(-1, 4), 3);
  TESTINT((int)lcec_su_mod(-4, 4), 0);
  TESTINT((int)lcec_su_fold(3, 4), -1);
  TESTINT((int)lcec_su_fold(2, 4), 2);
  TESTINT((int)lcec_su_fold(-2, 4), 2);
  TESTINT((int)lcec_su_fold(-3, 4), 1);

  TESTRESULTS;
}

TESTFUNC(test_su_round_div) {
  TESTSETUP;

  // M2R resyncs move app_time by whole periods
  TESTINT((int)lcec_su_round_div(3 * T, T), 3);
  TESTINT((int)lcec_su_round_div(-2 * T, T), -2);
  // a sub-period resync does not move the grid
  TESTINT((int)lcec_su_round_div(T / 3, T), 0);
  TESTINT((int)lcec_su_round_div(-T / 3, T), 0);
  TESTINT((int)lcec_su_round_div(T - 1000, T), 1);

  TESTRESULTS;
}

TESTFUNC(test_su_due) {
  TESTSETUP;

  TESTINT(lcec_su_due(7, 1, 0), 1);
  TESTINT(lcec_su_due(0, 4, 0), 1);
  TESTINT(lcec_su_due(4, 4, 0), 1);
  TESTINT(lcec_su_due(1, 4, 0), 0);
  TESTINT(lcec_su_due(-4, 4, 0), 1);
  TESTINT(lcec_su_due(3, 4, 3), 1);
  TESTINT(lcec_su_due(-1, 4, 3), 1);
  TESTINT(lcec_su_due(0, 4, 3), 0);

  TESTRESULTS;
}

TESTFUNC(test_su_phase_err) {
  TESTSETUP;
  int64_t bus = 1000000000;

  // sent on tick 0, received on tick 1: on target a quarter tick into tick 1
  TESTINT((int)lcec_su_phase_err(bus + T / 4, bus, 1, 4, 0, T, T / 4), 0);
  // 10 us late
  TESTINT((int)lcec_su_phase_err(bus + T / 4 + 10000, bus, 1, 4, 0, T, T / 4), 10000);
  // the bus already published tick 2 when the unit thread looked
  TESTINT((int)lcec_su_phase_err(bus + T / 4 + 10000, bus + T, 2, 4, 0, T, T / 4), 10000);
  // one tick early: started a quarter tick into the send tick
  TESTINT((int)lcec_su_phase_err(bus + T / 4, bus, 0, 4, 0, T, T / 4), -T);
  // phase 2: sent on tick 2, received on tick 3
  TESTINT((int)lcec_su_phase_err(bus + T / 4, bus, 3, 4, 2, T, T / 4), 0);
  TESTINT((int)lcec_su_phase_err(bus + T / 4, bus, 5, 4, 2, T, T / 4), 2 * T);
  TESTINT((int)lcec_su_phase_err(bus + T / 4, bus, 6, 4, 2, T, T / 4), -T);

  TESTRESULTS;
}

TESTFUNC(test_su_pll_correction) {
  TESTSETUP;

  TESTINT(lcec_su_pll_correction(0, 5000), 0);
  // late -> shorten the next period
  TESTINT(lcec_su_pll_correction(4000, 5000), -1000);
  TESTINT(lcec_su_pll_correction(-4000, 5000), 1000);
  TESTINT(lcec_su_pll_correction(400000, 5000), -5000);
  TESTINT(lcec_su_pll_correction(-400000, 5000), 5000);

  TESTRESULTS;
}

// Closed loop: a 2 kHz unit thread against an 8 kHz bus whose period is
// stretched by a bang-bang PLL (+/- T/1000) plus a 50 ppm crystal offset, as
// in M2R mode.  From every starting phase the unit thread has to lock within
// one second and then hold its slot to well inside a quarter master cycle.
static int simulate_lock(int64_t start_offset, int64_t *max_err_locked, int *lock_cycles) {
  const unsigned int divider = 4;
  const int64_t unit_period = (int64_t)divider * T;
  const int64_t limit = unit_period / 100;
  int64_t bus_start = 0, bus_tick = 0;
  int64_t unit_start = start_offset;
  int32_t corr = 0;
  int64_t err = 0;
  int cycle, bang = 1;

  *max_err_locked = 0;
  *lock_cycles = -1;
  for (cycle = 0; cycle < 4000; cycle++) {
    // advance the bus to the latest tick that started before the unit thread
    while (bus_start + T + T / 1000 * bang + T / 20000 <= unit_start) {
      bus_start += T + T / 1000 * bang + T / 20000;
      bus_tick++;
      bang = -bang;
    }
    err = lcec_su_phase_err(unit_start, bus_start, bus_tick, divider, 0, T, T / 4);
    if (*lock_cycles < 0 && llabs(err) < T / 16) {
      *lock_cycles = cycle;
    }
    if (*lock_cycles >= 0 && cycle > *lock_cycles + 10 && llabs(err) > *max_err_locked) {
      *max_err_locked = llabs(err);
    }
    corr = lcec_su_pll_correction(err, (int32_t)limit);
    unit_start += unit_period + corr;
  }
  return *lock_cycles >= 0 && *lock_cycles < 2000;
}

TESTFUNC(test_su_pll_lock) {
  TESTSETUP;
  int64_t offset, max_err;
  int cycles, worst_cycles = 0;
  int64_t worst_err = 0;

  for (offset = 0; offset < 4 * T; offset += 1777) {
    TESTINT(simulate_lock(offset, &max_err, &cycles), 1);
    if (cycles > worst_cycles) {
      worst_cycles = cycles;
    }
    if (max_err > worst_err) {
      worst_err = max_err;
    }
  }
  fprintf(stderr, "test_su_pll_lock: worst lock %d cycles, worst locked error %lld ns\n", worst_cycles, (long long)worst_err);
  TESTINT(worst_err < T / 32, 1);

  TESTRESULTS;
}

// Stress the image handoff with real threads.  The writer fills each image
// with its generation (every byte the low byte, the first word the whole
// value); the reader checks that nothing it accepts is torn and that
// generations only move forward.  Unthrottled writers on another CPU are far
// harsher than lcec (a unit publishes once per unit cycle): the test covers
// the multi-CPU case the single-CPU pinning would otherwise hide.
#define XFER_LEN     256
#define XFER_SECONDS 0.5

typedef struct {
  lcec_su_xfer_t x;
  volatile int stop;
  long writes;
} xfer_ctx_t;

static void fill(uint8_t *img, uint32_t gen) {
  memset(img, (int)(gen & 0xff), XFER_LEN);
  memcpy(img, &gen, sizeof(gen));
}

static int intact(const uint8_t *img, uint32_t *gen) {
  int i;
  memcpy(gen, img, sizeof(*gen));
  for (i = sizeof(*gen); i < XFER_LEN; i++) {
    if (img[i] != (uint8_t)(*gen & 0xff)) {
      return 0;
    }
  }
  return 1;
}

static double now_s(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void *rx_writer(void *arg) {
  xfer_ctx_t *c = arg;
  uint8_t img[XFER_LEN];
  while (!c->stop) {
    fill(img, c->x.rx_gen + 1);  // the image carries the generation it becomes
    lcec_su_rx_publish(&c->x, img);
    c->writes++;
  }
  return NULL;
}

static void *tx_writer(void *arg) {
  xfer_ctx_t *c = arg;
  uint8_t img[XFER_LEN];
  while (!c->stop) {
    fill(img, c->x.tx_gen + 1);
    lcec_su_tx_publish(&c->x, img);
    c->writes++;
  }
  return NULL;
}

static void xfer_init(xfer_ctx_t *c, uint8_t *bufs) {
  memset(c, 0, sizeof(*c));
  c->x.len = XFER_LEN;
  c->x.rx = bufs;
  c->x.tx[0] = bufs + XFER_LEN;
  c->x.tx[1] = bufs + 2 * XFER_LEN;
  fill(c->x.tx[0], 0);
  fill(c->x.tx[1], 0);
}

TESTFUNC(test_su_xfer_rx_stress) {
  TESTSETUP;
  static uint8_t bufs[3 * XFER_LEN];
  xfer_ctx_t c;
  pthread_t th;
  uint8_t img[XFER_LEN];
  uint32_t gen, img_gen, last = 0;
  long reads = 0, torn = 0, refused = 0, backwards = 0;
  double end;

  xfer_init(&c, bufs);
  pthread_create(&th, NULL, rx_writer, &c);
  end = now_s() + XFER_SECONDS;
  while (now_s() < end) {
    if (lcec_su_rx_fetch(&c.x, img, &gen) != 0) {
      refused++;
      continue;
    }
    reads++;
    if (gen == 0) {
      continue;
    }
    if (!intact(img, &img_gen) || img_gen != gen) {
      torn++;
    }
    if (gen < last) {
      backwards++;
    }
    last = gen;
  }
  c.stop = 1;
  pthread_join(th, NULL);
  fprintf(stderr, "test_su_xfer_rx_stress: %ld writes, %ld reads, %ld refused, %ld torn accepted\n", c.writes, reads, refused, torn);
  TESTINT((int)torn, 0);
  TESTINT((int)backwards, 0);
  TESTINT(reads > 100, 1);

  TESTRESULTS;
}

TESTFUNC(test_su_xfer_tx_stress) {
  TESTSETUP;
  static uint8_t bufs[3 * XFER_LEN];
  xfer_ctx_t c;
  pthread_t th;
  uint8_t img[XFER_LEN];
  uint32_t sent = 0, img_gen, last = 0;
  long takes = 0, torn = 0, refused = 0, backwards = 0, mismatch = 0, early = 0;
  double end;
  int r;

  xfer_init(&c, bufs);
  pthread_create(&th, NULL, tx_writer, &c);
  end = now_s() + XFER_SECONDS;
  while (now_s() < end) {
    r = lcec_su_tx_take(&c.x, img, &sent);
    if (r < 0) {
      refused++;  // the unit kept publishing: nothing is sent, never a torn image
      continue;
    }
    if (r == 0) {
      continue;
    }
    takes++;
    if (!intact(img, &img_gen)) {
      torn++;
    } else if (img_gen == sent + 1) {
      early++;  // next image, published between the index and generation stores
    } else if (img_gen != sent) {
      mismatch++;
    }
    if (sent < last) {
      backwards++;
    }
    last = sent;
  }
  c.stop = 1;
  pthread_join(th, NULL);
  fprintf(stderr, "test_su_xfer_tx_stress: %ld writes, %ld takes, %ld refused, %ld early, %ld torn accepted\n", c.writes, takes, refused,
      early, torn);
  TESTINT((int)torn, 0);
  TESTINT((int)mismatch, 0);
  TESTINT((int)backwards, 0);
  TESTINT(takes > 100, 1);

  TESTRESULTS;
}

TESTMAIN
