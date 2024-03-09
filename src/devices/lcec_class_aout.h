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
/// @brief Library for analog output devices

#include "../lcec.h"

typedef struct {
  const char *name_prefix;               ///< Prefix for device naming, defaults to "aio".
  int max_value;                   ///< The maximum value returned for "normal" output channels.
  double default_scale;            ///< Default scale for the device.
  double default_offset;           ///< Default value offset for the device.
  uint16_t value_idx, value_sidx;  ///< PDO index and subindex for reading the value.
} lcec_class_aout_options_t;

/// @brief Data for a single analog channel.
typedef struct {
  hal_bit_t *pos;
  hal_bit_t *neg;
  hal_bit_t *enable;
  hal_bit_t *absmode;
  hal_float_t *value;
  hal_float_t *scale;
  hal_float_t *offset;
  double old_scale;
  double scale_recip;
  hal_float_t *min_dc;
  hal_float_t *max_dc;
  hal_float_t *curr_dc;
  hal_s32_t *raw_val;  ///< The raw value read from the device.
  unsigned int val_pdo_os;
  lcec_class_aout_options_t *options;  ///< The options used to create this device.
} lcec_class_aout_channel_t;

/// @brief Data for an analog input device.
typedef struct {
  int count;                             ///< The number of channels in use with this device.
  lcec_class_aout_channel_t **channels;  ///< Dynamic array holding pin data for each channel.
} lcec_class_aout_channels_t;

lcec_class_aout_channels_t *lcec_aout_allocate_channels(int count);
lcec_class_aout_channel_t *lcec_aout_register_channel(struct lcec_slave *slave, int id, uint16_t idx, lcec_class_aout_options_t *opt);
void lcec_aout_write(struct lcec_slave *slave, lcec_class_aout_channel_t *data);
void lcec_aout_write_all(struct lcec_slave *slave, lcec_class_aout_channels_t *channels);
lcec_class_aout_options_t *lcec_aout_options(void);
