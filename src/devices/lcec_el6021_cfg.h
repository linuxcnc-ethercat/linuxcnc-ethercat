//
//    Copyright (C) 2026 The LinuxCNC EtherCAT authors
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

/// @file
/// @brief Pure serial-configuration helpers for the EL6001/EL6021 driver
///
/// Baud/frame tables and termios<->SDO mapping, shared between
/// lcec_el6021.c and tests/test_el6021.c.  Everything is static inline
/// so there is nothing to link.

#ifndef _LCEC_EL6021_CFG_H_
#define _LCEC_EL6021_CFG_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>

// kernel UAPI struct termios (asm-generic): NCCS=19, no speed fields.
// glibc's struct termios differs (NCCS=32 + ispeed/ospeed), so do not
// memcpy between the two; only the prefix layout matches.
typedef struct {
  tcflag_t c_iflag;
  tcflag_t c_oflag;
  tcflag_t c_cflag;
  tcflag_t c_lflag;
  cc_t c_line;
  cc_t c_cc[19];
  unsigned int c_ispeed;  // termios2 only (TCGETS2 / TCSETS2)
  unsigned int c_ospeed;
} lcec_el6021_ktermios_t;

// size of the kernel's struct termios (no speed fields) and struct termios2
#define LCEC_EL6021_KTERMIOS_SIZE  offsetof(lcec_el6021_ktermios_t, c_ispeed)
#define LCEC_EL6021_KTERMIOS2_SIZE sizeof(lcec_el6021_ktermios_t)

// Kernel UAPI baud encoding in c_cflag (asm-generic/termbits.h).  glibc >= 2.42
// defines B9600 etc. as plain numbers, so they cannot be compared with what
// the kernel passes in c_cflag.
#define LCEC_EL6021_KCBAUD  0x0000100f
#define LCEC_EL6021_KBOTHER 0x00001000

typedef struct {
  uint8_t idx;
  uint32_t baud;
  speed_t speed;
} lcec_el6021_baud_t;

/// EL600x supported baud rates (SDO 0x8000:11 values)
static const lcec_el6021_baud_t lcec_el6021_baud_table[] = {
    {1, 300, 0x0007},
    {2, 600, 0x0008},
    {3, 1200, 0x0009},
    {4, 2400, 0x000b},
    {5, 4800, 0x000c},
    {6, 9600, 0x000d},
    {7, 19200, 0x000e},
    {8, 38400, 0x000f},
    {9, 57600, 0x1001},
    {10, 115200, 0x1002},
};

typedef struct {
  uint8_t idx;
  uint8_t data_bits;
  char parity;  // 'N', 'E' or 'O'
  uint8_t stop_bits;
} lcec_el6021_frame_t;

/// EL600x supported data frames (SDO 0x8000:15 values)
static const lcec_el6021_frame_t lcec_el6021_frame_table[] = {
    {0x01, 7, 'E', 1},
    {0x09, 7, 'E', 2},
    {0x02, 7, 'O', 1},
    {0x0a, 7, 'O', 2},
    {0x03, 8, 'N', 1},
    {0x0b, 8, 'N', 2},
    {0x04, 8, 'E', 1},
    {0x0c, 8, 'E', 2},
    {0x05, 8, 'O', 1},
    {0x0d, 8, 'O', 2},
};

// configuration table lookups

static inline const lcec_el6021_baud_t *lcec_el6021_baud_by_value(uint32_t baud) {
  size_t i;
  for (i = 0; i < sizeof(lcec_el6021_baud_table) / sizeof(lcec_el6021_baud_table[0]); i++) {
    if (lcec_el6021_baud_table[i].baud == baud) {
      return &lcec_el6021_baud_table[i];
    }
  }
  return NULL;
}

static inline const lcec_el6021_baud_t *lcec_el6021_baud_by_idx(uint8_t idx) {
  size_t i;
  for (i = 0; i < sizeof(lcec_el6021_baud_table) / sizeof(lcec_el6021_baud_table[0]); i++) {
    if (lcec_el6021_baud_table[i].idx == idx) {
      return &lcec_el6021_baud_table[i];
    }
  }
  return NULL;
}

static inline const lcec_el6021_frame_t *lcec_el6021_frame_by_spec(uint8_t data_bits, char parity, uint8_t stop_bits) {
  size_t i;
  for (i = 0; i < sizeof(lcec_el6021_frame_table) / sizeof(lcec_el6021_frame_table[0]); i++) {
    const lcec_el6021_frame_t *f = &lcec_el6021_frame_table[i];
    if (f->data_bits == data_bits && f->parity == parity && f->stop_bits == stop_bits) {
      return f;
    }
  }
  return NULL;
}

static inline const lcec_el6021_frame_t *lcec_el6021_frame_by_idx(uint8_t idx) {
  size_t i;
  for (i = 0; i < sizeof(lcec_el6021_frame_table) / sizeof(lcec_el6021_frame_table[0]); i++) {
    if (lcec_el6021_frame_table[i].idx == idx) {
      return &lcec_el6021_frame_table[i];
    }
  }
  return NULL;
}

// parse a frame spec string like "8N1"
static inline const lcec_el6021_frame_t *lcec_el6021_frame_by_string(const char *s) {
  uint8_t data_bits, stop_bits;
  char parity;

  if (s == NULL || strlen(s) != 3) {
    return NULL;
  }
  if (s[0] != '7' && s[0] != '8') {
    return NULL;
  }
  data_bits = s[0] - '0';
  parity = s[1];
  if (parity != 'N' && parity != 'E' && parity != 'O') {
    return NULL;
  }
  if (s[2] != '1' && s[2] != '2') {
    return NULL;
  }
  stop_bits = s[2] - '0';
  return lcec_el6021_frame_by_spec(data_bits, parity, stop_bits);
}

// map a termios c_cflag to SDO indices. Returns 0 on success.
static inline int lcec_el6021_cflag_to_config(tcflag_t cflag, unsigned int ospeed, uint8_t *baud_idx, uint8_t *frame_idx, uint8_t *rtscts) {
  uint8_t data_bits, stop_bits;
  char parity;
  const lcec_el6021_baud_t *b = NULL;
  const lcec_el6021_frame_t *f;
  tcflag_t cbaud = cflag & LCEC_EL6021_KCBAUD;
  size_t i;

  // BOTHER carries the rate in c_ospeed (termios2); match it by value
  for (i = 0; i < sizeof(lcec_el6021_baud_table) / sizeof(lcec_el6021_baud_table[0]); i++) {
    if ((cbaud == LCEC_EL6021_KBOTHER) ? (lcec_el6021_baud_table[i].baud == ospeed) : (lcec_el6021_baud_table[i].speed == cbaud)) {
      b = &lcec_el6021_baud_table[i];
      break;
    }
  }
  if (b == NULL) {
    return -1;
  }

  switch (cflag & CSIZE) {
    case CS7:
      data_bits = 7;
      break;
    case CS8:
      data_bits = 8;
      break;
    default:
      return -1;
  }
  if (cflag & PARENB) {
    parity = (cflag & PARODD) ? 'O' : 'E';
  } else {
    parity = 'N';
  }
  stop_bits = (cflag & CSTOPB) ? 2 : 1;

  f = lcec_el6021_frame_by_spec(data_bits, parity, stop_bits);
  if (f == NULL) {
    return -1;
  }

  *baud_idx = b->idx;
  *frame_idx = f->idx;
  *rtscts = (cflag & CRTSCTS) ? 1 : 0;
  return 0;
}

#endif
