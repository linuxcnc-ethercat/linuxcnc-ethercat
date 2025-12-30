//
//    Copyright (C) 2015 Sascha Ittner <sascha.ittner@modusoft.de>
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

#ifndef _ECAT_RTAPI_USER_H_
#define _ECAT_RTAPI_USER_H_

#include <sched.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

static inline void *ecat_zalloc(size_t size) {
  void *p = malloc(size);
  if (p) memset(p, 0, size);
  return p;
}
#define ecat_free(ptr) free(ptr)

#define ecat_gettimeofday(x) gettimeofday(x, NULL)

#define ECAT_MS_TO_TICKS(x) (x / 10)
static inline long ecat_get_ticks(void) {
  struct timespec tp;
  clock_gettime(CLOCK_MONOTONIC, &tp);
  return ((long)(tp.tv_sec * 100LL)) + (tp.tv_nsec / 10000000L);
}

#define ecat_schedule() sched_yield()

static inline long long ecat_mod_64(long long val, unsigned long div) { return val % div; }

#endif
