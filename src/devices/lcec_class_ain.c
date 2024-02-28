//
//    Copyright (C) 2011 Sascha Ittner <sascha.ittner@modusoft.de>
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
/// @brief Library for analog input devices.

#include "lcec_class_ain.h"

#include "../lcec.h"

/// @brief Basic pins common to all analog in devices.
static const lcec_pindesc_t slave_pins_basic[] = {
    {HAL_S32, HAL_OUT, offsetof(lcec_class_ain_channel_t, raw_val), "%s.%s.%s.%s-%d-raw"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_class_ain_channel_t, val), "%s.%s.%s.%s-%d-val"},
    {HAL_FLOAT, HAL_IO, offsetof(lcec_class_ain_channel_t, scale), "%s.%s.%s.%s-%d-scale"},
    {HAL_FLOAT, HAL_IO, offsetof(lcec_class_ain_channel_t, bias), "%s.%s.%s.%s-%d-bias"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static const lcec_pindesc_t slave_pins_basic_temperature[] = {
    {HAL_S32, HAL_OUT, offsetof(lcec_class_ain_channel_t, raw_val), "%s.%s.%s.%s-%d-raw"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_class_ain_channel_t, val), "%s.%s.%s.%s-%d-temperature"},
    {HAL_FLOAT, HAL_IO, offsetof(lcec_class_ain_channel_t, scale), "%s.%s.%s.%s-%d-scale"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/// @brief List of HAL pins for analog devices with sync support.
static const lcec_pindesc_t slave_pins_sync[] = {
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_ain_channel_t, sync_err), "%s.%s.%s.%s-%d-sync-err"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/// @brief List of HAL pins for pressure sensors.
static const lcec_pindesc_t slave_pins_basic_pressure[] = {
    {HAL_S32, HAL_OUT, offsetof(lcec_class_ain_channel_t, raw_val), "%s.%s.%s.press-%d-raw"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_class_ain_channel_t, val), "%s.%s.%s.press-%d-pressure"},
    {HAL_FLOAT, HAL_IO, offsetof(lcec_class_ain_channel_t, scale), "%s.%s.%s.press-%d-scale"},
    {HAL_FLOAT, HAL_IO, offsetof(lcec_class_ain_channel_t, bias), "%s.%s.%s.press-%d-bias"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/// @brief Additional HAL pins for reporting status.
static const lcec_pindesc_t slave_pins_status[] = {
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_ain_channel_t, error), "%s.%s.%s.%s-%d-error"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_ain_channel_t, overrange), "%s.%s.%s.%s-%d-overrange"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_ain_channel_t, underrange), "%s.%s.%s.%s-%d-underrange"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/// @brief Allocate a block of memory for holding the results from
/// `count` calls to `lcec_ain_register_device() and friends.
///
/// It is the caller's responsibility to verify that the result is not NULL.
///
/// @param count The number of input pins to allocate memory for.
/// @return A `lcec_class_ain_channels_t` for holding future results
/// from `lcec_ain_register_channel()`.
lcec_class_ain_channels_t *lcec_ain_allocate_channels(int count) {
  lcec_class_ain_channels_t *channels;

  channels = hal_malloc(sizeof(lcec_class_ain_channels_t));
  if (channels == NULL) {
    return NULL;
  }
  channels->count = count;
  channels->channels = hal_malloc(sizeof(lcec_class_ain_channel_t *) * count);
  if (channels->channels == NULL) {
    return NULL;
  }
  return channels;
}

/// @brief Allocates a `lcec_class_aio_options_t` and initializes it.
///
/// Most of the defaults are actually coded in
/// `lcec_ain_register_channel`, so we can safely just memset
/// everything to 0 here.
lcec_class_ain_options_t *lcec_ain_options(void) {
  lcec_class_ain_options_t *opts = hal_malloc(sizeof(lcec_class_ain_options_t));
  if (opts == NULL) {
    return NULL;
  }
  memset(opts, 0, sizeof(lcec_class_ain_options_t));

  return opts;
}

/// @brief registers a single analog-input channel and publishes it as a LinuxCNC HAL pin.
///
/// @param slave The slave, from `_init`.
/// @param id  The pin ID.  Used for naming.  Should generally start at 0 and increment once per digital in pin.
/// @param idx The PDO index for the digital input.
/// @param opt A `lcec_class_ain_options_t` structure that contains optional settings.  This includes port naming,
///            PDO overrides, temperature mode, and others.  You can pass `NULL` for defaults, or you can pass a
///            `lcec_class_ain_options_t`.  Unset fields in `opt` are treated as a request for the default.  See
///            `lcec_ain_options()` for a helper function to allocate and zero a default options struct.
/// @return A `lcec_class_ain_channel_t` that contains all per-channel data and can be used with `lcec_ain_read()`.
///
/// See lcec_el3xxx.c for an example of use.
lcec_class_ain_channel_t *lcec_ain_register_channel(lcec_slave_t *slave, int id, uint16_t idx, lcec_class_ain_options_t *opt) {
  lcec_class_ain_channel_t *data;
  int err;

  // Overrideable defaults.  These should be the default values if they're not overridden by `opt`.
  int has_sync = 0, is_temperature = 0, is_pressure = 0, is_unsigned = 0, valueonly = 0, max_value = 0x7fff;
  uint16_t value_idx = idx, value_sidx = 0x11;
  uint16_t underrange_idx = idx, underrange_sidx = 0x01;
  uint16_t overrange_idx = idx, overrange_sidx = 0x02;
  uint16_t error_idx = idx, error_sidx = 0x07;
  uint16_t syncerror_idx = idx, syncerror_sidx = 0x0e;

  // Handle options in `opt`.  Remember that `opt` can be `NULL`, and
  // any unset values should retain their values from above.
  if (opt && opt->has_sync) has_sync = 1;
  if (opt && opt->valueonly) valueonly = 1;
  if (opt && opt->is_pressure) is_pressure = 1;
  if (opt && opt->is_temperature) is_temperature = 1;
  if (opt && opt->is_unsigned) is_unsigned = 1;
  if (opt && opt->max_value) max_value = opt->max_value;
  if (opt && opt->value_idx) value_idx = opt->value_idx;
  if (opt && opt->value_sidx) value_sidx = opt->value_sidx;
  if (opt && opt->underrange_idx) underrange_idx = opt->underrange_idx;
  if (opt && opt->underrange_sidx) underrange_sidx = opt->underrange_sidx;
  if (opt && opt->overrange_idx) overrange_idx = opt->overrange_idx;
  if (opt && opt->overrange_sidx) overrange_sidx = opt->overrange_sidx;
  if (opt && opt->error_idx) error_idx = opt->error_idx;
  if (opt && opt->error_sidx) error_sidx = opt->error_sidx;
  if (opt && opt->syncerror_idx) syncerror_idx = opt->syncerror_idx;
  if (opt && opt->syncerror_sidx) syncerror_sidx = opt->syncerror_sidx;

  // The default name depends on the port type.
  char *name_prefix = "ain";
  if (is_temperature) name_prefix = "temp";
  if (is_pressure) name_prefix = "pressure";
  if (opt && opt->name_prefix) name_prefix = opt->name_prefix;

  // If we were passed a NULL opt, then create a new
  // `lcec_class_ain_options_t` and write the defaults back into it,
  // so we don't need to repeat the above default code downstream.
  if (!opt) {
    opt = lcec_ain_options();
    if (opt == NULL) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s pin %d failed\n", slave->master->name, slave->name, id);
      return NULL;
    }
  }

  // Allocate memory for per-channel data.
  data = hal_malloc(sizeof(lcec_class_ain_channel_t));
  if (data == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s pin %d failed\n", slave->master->name, slave->name, id);
    return NULL;
  }
  memset(data, 0, sizeof(lcec_class_ain_channel_t));

  // Save important options for later use.  None of the _idx/_sidx will be needed outside of this function.
  data->options = opt;
  opt->name_prefix = name_prefix;
  opt->has_sync = has_sync;
  opt->valueonly = valueonly;
  opt->is_pressure = is_pressure;
  opt->is_temperature = is_temperature;
  opt->max_value = max_value;

  // This is written into `data`, not `opt`, because there are cases
  // where we want drivers to be able to override it at runtime.  For
  // instance, with EL32xx devices in *resistance* mode, we need to
  // handle results as `unsigned`, otherwise we lose half of the
  // range.  But when they're in temperature mode, we need signed
  // values or we can't represent anything below freezing.  I'd rather
  // treat `opt` as read-only after this point.
  data->is_unsigned = is_unsigned;

  // Register basic PDO pins
  lcec_pdo_init(slave, value_idx, value_sidx, &data->val_pdo_os, NULL);

  // Register sync error PDO, if used.
  if (has_sync) lcec_pdo_init(slave, syncerror_idx, syncerror_sidx, &data->sync_err_pdo_os, &data->sync_err_pdo_bp);

  // Register error reporting PDOs, if used.
  if (!valueonly) {
    lcec_pdo_init(slave, underrange_idx, underrange_sidx, &data->udr_pdo_os, &data->udr_pdo_bp);
    lcec_pdo_init(slave, overrange_idx, overrange_sidx, &data->ovr_pdo_os, &data->ovr_pdo_bp);
    lcec_pdo_init(slave, error_idx, error_sidx, &data->error_pdo_os, &data->error_pdo_bp);
  }

  // Register basic pins
  if (is_temperature) {
    err = lcec_pin_newf_list(data, slave_pins_basic_temperature, LCEC_MODULE_NAME, slave->master->name, slave->name, name_prefix, id);
  } else if (is_pressure) {
    err = lcec_pin_newf_list(data, slave_pins_basic_pressure, LCEC_MODULE_NAME, slave->master->name, slave->name, name_prefix, id);
  } else {
    err = lcec_pin_newf_list(data, slave_pins_basic, LCEC_MODULE_NAME, slave->master->name, slave->name, name_prefix, id);
  }
  if (err != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_pin_newf_list for slave %s.%s pin %d failed\n", slave->master->name, slave->name, id);
    return NULL;
  }

  // Register sync error pin, if used
  if (has_sync) {
    err = lcec_pin_newf_list(data, slave_pins_sync, LCEC_MODULE_NAME, slave->master->name, slave->name, name_prefix, id);

    if (err != 0) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_pin_newf_list for slave %s.%s pin %d failed\n", slave->master->name, slave->name, id);
      return NULL;
    }
  }

  // Register status pins, if used
  if (!valueonly) {
    err = lcec_pin_newf_list(data, slave_pins_status, LCEC_MODULE_NAME, slave->master->name, slave->name, name_prefix, id);

    if (err != 0) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_pin_newf_list for slave %s.%s pin %d failed\n", slave->master->name, slave->name, id);
      return NULL;
    }
  }

  // Set default values for scale and bias.
  *(data->scale) = 1.0;
  if (is_temperature) *(data->scale) = 0.1;
  if (opt->default_scale != 0) *(data->scale) = opt->default_scale;
  if (opt->default_bias != 0) *(data->bias) = opt->default_bias;

  return data;
}

/// @brief Reads data from a single analog in port.
///
/// @param slave The `slave`, passed from the per-device `_read`.
/// @param data  Which channel to read; a `lcec_class_ain_channel_t *`, as returned by lcec_ain_register_channel.
///
/// Call this once per channel registered, from inside of your device's
/// read function.  Use `lcec_ain_read_all` to read all pins.
void lcec_ain_read(lcec_slave_t *slave, lcec_class_ain_channel_t *data) {
  uint8_t *pd = slave->master->process_data;
  int value;  // Needs to be large enough to hold either a uint16_t or an sint16_t without loss.
  int max_value = data->options->max_value;

  // Update status bits, if enabled
  if (!data->options->valueonly) {
    *(data->overrange) = EC_READ_BIT(&pd[data->ovr_pdo_os], data->ovr_pdo_bp);
    *(data->underrange) = EC_READ_BIT(&pd[data->udr_pdo_os], data->udr_pdo_bp);
    *(data->error) = EC_READ_BIT(&pd[data->error_pdo_os], data->error_pdo_bp);
  }

  // Update sync error, if present
  if (data->options->has_sync) {
    *(data->sync_err) = EC_READ_BIT(&pd[data->sync_err_pdo_os], data->sync_err_pdo_bp);
  }

  // update value
  if (data->is_unsigned) {
    value = EC_READ_U16(&pd[data->val_pdo_os]);
  } else {
    value = EC_READ_S16(&pd[data->val_pdo_os]);
  }
  *(data->raw_val) = value;
  if (data->options->is_temperature) {
    // Temperature uses different value calculations than regular analog sensors.
    *(data->val) = *(data->scale) * (double)value;
  } else {
    // Normal analog sensors return a value between -1.0 and 1.0 (or 0
    // and 1.0, depending on the sensor type), where 1.0 is the
    // largest possible input value.
    //
    // Then, the result is multipled by `scale` (default: 1.0) and `bias` is added (default 0).
    *(data->val) = *(data->bias) + *(data->scale) * (double)value * ((double)1 / (double)max_value);
  }
}

/// @brief Reads data from all analog in ports.
///
/// @param slave The `slave`, passed from the per-device `_read`.
/// @param channels An `lcec_class_ain_channel_t *`, as returned by lcec_ain_register_channel.
void lcec_ain_read_all(lcec_slave_t *slave, lcec_class_ain_channels_t *channels) {
  for (int i = 0; i < channels->count; i++) {
    lcec_class_ain_channel_t *channel = channels->channels[i];

    lcec_ain_read(slave, channel);
  }
}
