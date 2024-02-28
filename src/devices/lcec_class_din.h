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
/// @brief Library for digital input devices

#ifndef _LCEC_CLASS_DIN_H_
#define _LCEC_CLASS_DIN_H_
#include "../lcec.h"

typedef struct {
  char *name;                  ///< The channel name/number, used for debugging messages.
  hal_bit_t *in;               ///< `hal_bit_t` pin for the `din-X` pin in LinuxCNC.
  hal_bit_t *in_not;           ///< `hal_bit_t` pin for the `din-X-not` pin in LinuxCNC.
  unsigned int pdo_os;         ///< This bit's offset in the master's PDO data structure.
  unsigned int pdo_bp;         ///< This bit's bit position in the master's PDO data structure.
  unsigned int pdo_bp_packed;  ///< This bit's bit position in the master's PDO data structure, for packed din.
} lcec_class_din_channel_t;

typedef struct {
  int count;                            ///< The number of channels described by this structure.
  lcec_class_din_channel_t **channels;  ///< a dynamic array of `lcec_class_din_channel_t` channels.
} lcec_class_din_channels_t;

lcec_class_din_channels_t *lcec_din_allocate_channels(int count);
lcec_class_din_channel_t *lcec_din_register_channel(struct lcec_slave *slave, int id, uint16_t idx, uint16_t sidx);
lcec_class_din_channel_t *lcec_din_register_channel_named(struct lcec_slave *slave, uint16_t idx, uint16_t sidx, char *name);
lcec_class_din_channel_t *lcec_din_register_channel_packed(struct lcec_slave *slave, uint16_t idx, uint16_t sidx, int bit, char *name);

void lcec_din_read(struct lcec_slave *slave, lcec_class_din_channel_t *data);
void lcec_din_read_all(struct lcec_slave *slave, lcec_class_din_channels_t *channels);
#endif
