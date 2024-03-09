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
/// @brief Library for analog input devices

#include "../lcec.h"

typedef struct {
  const char *name_prefix;               ///< Prefix for device naming, defaults to "aio".
  int has_sync;                    ///< Device supports the sync_err PDO.
  int valueonly;                   ///< Only read the value from the sensor, do not read over/under range or error PDOs.
  int is_temperature;              ///< Device is a temperature sensor.
  int is_pressure;                 ///< Device is a pressure sensor.
  int is_unsigned;                 ///< Device readings should be treated as unsigned.
  int max_value;                   ///< The maximum value returned for "normal" input channels.  Temperature sensors do not use this.
  double default_scale;            ///< Default scale for the device.
  double default_bias;             ///< Default bias for the device.
  uint16_t value_idx, value_sidx;  ///< PDO index and subindex for reading the value.
  uint16_t underrange_idx, underrange_sidx;  ///< PDO index/subindex for reading underrange status.
  uint16_t overrange_idx, overrange_sidx;    ///< PDO index/subindex for reading overrange status.
  uint16_t error_idx, error_sidx;            ///< PDO index/subindex for reading error status.
  uint16_t syncerror_idx, syncerror_sidx;    ///< PDO index/subindex for reading sync error status.
} lcec_class_ain_options_t;

/// @brief Data for a single analog channel.
typedef struct {
  hal_bit_t *overrange;   ///< Device reading is over-range.
  hal_bit_t *underrange;  ///< Device reading is under-range.
  hal_bit_t *error;       ///< Device is in an error state.
  hal_bit_t *sync_err;    ///< Device has a sync error.
  hal_s32_t *raw_val;     ///< The raw value read from the device.
  hal_float_t *scale;     ///< The scale used to convert `raw_val` into `val`.
  hal_float_t *bias;      ///< The offset used to convert `raw_val` into `val`.
  hal_float_t *val;       ///< The final result returned to LinuxCNC.
  unsigned int ovr_pdo_os;
  unsigned int ovr_pdo_bp;
  unsigned int udr_pdo_os;
  unsigned int udr_pdo_bp;
  unsigned int error_pdo_os;
  unsigned int error_pdo_bp;
  unsigned int sync_err_pdo_os;
  unsigned int sync_err_pdo_bp;
  unsigned int val_pdo_os;
  int is_unsigned;
  lcec_class_ain_options_t *options;  ///< The options used to create this device.
} lcec_class_ain_channel_t;

/// @brief Data for an analog input device.
typedef struct {
  int count;                            ///< The number of channels in use with this device.
  lcec_class_ain_channel_t **channels;  ///< Dynamic array holding pin data for each channel.
} lcec_class_ain_channels_t;

lcec_class_ain_channels_t *lcec_ain_allocate_channels(int count);
lcec_class_ain_channel_t *lcec_ain_register_channel(struct lcec_slave *slave, int id, uint16_t idx, lcec_class_ain_options_t *opt);
void lcec_ain_read(struct lcec_slave *slave, lcec_class_ain_channel_t *data);
void lcec_ain_read_all(struct lcec_slave *slave, lcec_class_ain_channels_t *channels);
lcec_class_ain_options_t *lcec_ain_options(void);
