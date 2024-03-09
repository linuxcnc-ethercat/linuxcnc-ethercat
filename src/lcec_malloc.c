//
//    Copyright (C) 2024 Scott Laird <scott@sigkill.org>
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
/// @brief Memory allocation helpers for LinuxCNC-Ethercat

#include "lcec.h"
#include <stdio.h>


void *lcec_hal_malloc(size_t size, const char *file, const char *func, int line) {
  void *result = hal_malloc(size);
  if (result==NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "MEMORY ALLOCATION FAILURE, hal_malloc() returned NULL in function %s at %s:%d\n", func, file, line);
    exit(1);
  }
  memset(result, 0, size);
  return result;
}

void *lcec_malloc(size_t size, const char *file, const char *func, int line) {
  void *result = malloc(size);
  if (result==NULL) {
    fprintf(stderr, LCEC_MSG_PFX "MEMORY ALLOCATION FAILURE, hal_malloc() returned NULL in function %s at %s:%d\n", func, file, line);
    exit(1);
  }
  return result;
}
