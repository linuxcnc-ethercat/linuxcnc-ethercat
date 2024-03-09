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
/// @brief Library for analog output devices.

#include "lcec_class_aout.h"

#include "../lcec.h"

/// @brief Basic pins common to all analog in devices.
static const lcec_pindesc_t slave_pins_basic[] = {
    {HAL_FLOAT, HAL_IO, offsetof(lcec_class_aout_channel_t, scale), "%s.%s.%s.%s-%d-scale"},
    {HAL_FLOAT, HAL_IO, offsetof(lcec_class_aout_channel_t, offset), "%s.%s.%s.%s-%d-offset"},
    {HAL_FLOAT, HAL_IO, offsetof(lcec_class_aout_channel_t, min_dc), "%s.%s.%s.%s-%d-min-dc"},
    {HAL_FLOAT, HAL_IO, offsetof(lcec_class_aout_channel_t, max_dc), "%s.%s.%s.%s-%d-max-dc"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_class_aout_channel_t, curr_dc), "%s.%s.%s.%s-%d-curr-dc"},
    {HAL_BIT, HAL_IN, offsetof(lcec_class_aout_channel_t, enable), "%s.%s.%s.%s-%d-enable"},
    {HAL_BIT, HAL_IN, offsetof(lcec_class_aout_channel_t, absmode), "%s.%s.%s.%s-%d-absmode"},
    {HAL_FLOAT, HAL_IN, offsetof(lcec_class_aout_channel_t, value), "%s.%s.%s.%s-%d-value"},
    {HAL_S32, HAL_OUT, offsetof(lcec_class_aout_channel_t, raw_val), "%s.%s.%s.%s-%d-raw"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_aout_channel_t, pos), "%s.%s.%s.%s-%d-pos"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_aout_channel_t, neg), "%s.%s.%s.%s-%d-neg"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/// @brief Allocate a block of memory for holding the results from
/// `count` calls to `lcec_aout_register_device() and friends.
///
/// It is the caller's responsibility to verify that the result is not NULL.
///
/// @param count The number of input pins to allocate memory for.
/// @return A `lcec_class_aout_channels_t` for holding future results
/// from `lcec_aout_register_channel()`.
lcec_class_aout_channels_t *lcec_aout_allocate_channels(int count) {
  lcec_class_aout_channels_t *channels;

  channels = LCEC_HAL_ALLOCATE(lcec_class_aout_channels_t);
  channels->count = count;
  channels->channels = LCEC_HAL_ALLOCATE_ARRAY(lcec_class_aout_channel_t *, count);
  return channels;
}

/// @brief Allocates a `lcec_class_aio_options_t` and initializes it.
///
/// Most of the defaults are actually coded in
/// `lcec_aout_register_channel`, so we can safely just memset
/// everything to 0 here.
lcec_class_aout_options_t *lcec_aout_options(void) {
  return LCEC_HAL_ALLOCATE(lcec_class_aout_options_t);
}

/// @brief registers a single analog-output channel and publishes it as a set of LinuxCNC HAL pins.
///
/// @param slave The slave, from `_init`.
/// @param id  The channel ID.  Used for naming.  Should generally start at 0 and increment once per analog out pin.
/// @param idx The PDO index for the analog output.  The subindex is controlled via `opt`.
/// @param opt A `lcec_class_aout_options_t` structure that contains optional settings.  This includes port naming,
///            PDO overrides, and others.  You can pass `NULL` for defaults, or you can pass a
///            `lcec_class_aout_options_t`.  Unset fields in `opt` are treated as a request for the default.  See
///            `lcec_aout_options()` for a helper function to allocate and zero a default options struct.
/// @return A `lcec_class_aout_channel_t` that contains all per-channel data and can be used with `lcec_aout_write()`.
///
/// See lcec_el3xxx.c for an example of use.
lcec_class_aout_channel_t *lcec_aout_register_channel(lcec_slave_t *slave, int id, uint16_t idx, lcec_class_aout_options_t *opt) {
  lcec_class_aout_channel_t *data;
  int err;

  // Overrideable defaults.  These should be the default values if they're not overridden by `opt`.
  int max_value = 0x7fff;
  uint16_t value_idx = idx, value_sidx = 0x1;

  // Handle options in `opt`.  Remember that `opt` can be `NULL`, and
  // any unset values should retaout their values from above.
  if (opt && opt->max_value) max_value = opt->max_value;
  if (opt && opt->value_idx) value_idx = opt->value_idx;
  if (opt && opt->value_sidx) value_sidx = opt->value_sidx;

  // The default name depends on the port type.
  const char *name_prefix = "aout";
  if (opt && opt->name_prefix) name_prefix = opt->name_prefix;

  // If we were passed a NULL opt, then create a new
  // `lcec_class_aout_options_t` and write the defaults back into it,
  // so we don't need to repeat the above default code downstream.
  if (!opt) {
    opt = lcec_aout_options();
  }

  // Allocate memory for per-channel data.
  data = LCEC_HAL_ALLOCATE(lcec_class_aout_channel_t);

  // Save important options for later use.  None of the _idx/_sidx will be needed outside of this function.
  data->options = opt;
  opt->name_prefix = name_prefix;
  opt->max_value = max_value;

  // Register PDO pin
  lcec_pdo_init(slave, value_idx, value_sidx, &data->val_pdo_os, NULL);

  // Register basic pins
  err = lcec_pin_newf_list(data, slave_pins_basic, LCEC_MODULE_NAME, slave->master->name, slave->name, name_prefix, id);

  if (err != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_pin_newf_list for slave %s.%s pin %d failed\n", slave->master->name, slave->name, id);
    return NULL;
  }

  // Set default values for scale and bias.
  *(data->scale) = 1.0;
  if (opt->default_scale != 0) *(data->scale) = opt->default_scale;
  if (opt->default_offset != 0) *(data->offset) = opt->default_offset;
  *(data->max_dc) = 1.0;
  *(data->min_dc) = -1.0;
  data->old_scale = *(data->scale) + 1.0;
  data->scale_recip = 1.0 / *(data->scale);

  return data;
}

/// @brief Writes data from a single analog out port.
///
/// @param slave The `slave`, passed from the per-device `_read`.
/// @param data  Which channel to read; a `lcec_class_aout_channel_t *`, as returned by lcec_aout_register_channel.
///
/// Call this once per channel registered, from inside of your device's
/// read function.  Use `lcec_aout_write_all` to read all pins.
void lcec_aout_write(lcec_slave_t *slave, lcec_class_aout_channel_t *data) {
  uint8_t *pd = slave->master->process_data;
  int max_value = data->options->max_value;
  double tmpval, tmpdc, raw_val;

  // validate duty cycle limits, both limits must be between
  // 0.0 and 1.0 (inclusive) and max must be greater then min
  if (*(data->max_dc) > 1.0) {
    *(data->max_dc) = 1.0;
  }
  if (*(data->min_dc) > *(data->max_dc)) {
    *(data->min_dc) = *(data->max_dc);
  }
  if (*(data->min_dc) < -1.0) {
    *(data->min_dc) = -1.0;
  }
  if (*(data->max_dc) < *(data->min_dc)) {
    *(data->max_dc) = *(data->min_dc);
  }

  // do scale calcs only when scale changes
  if (*(data->scale) != data->old_scale) {
    // validate the new scale value
    if ((*(data->scale) < 1e-20) && (*(data->scale) > -1e-20)) {
      // value too small, divide by zero is a bad thing
      *(data->scale) = 1.0;
    }
    // get ready to detect future scale changes
    data->old_scale = *(data->scale);
    // we will need the reciprocal
    data->scale_recip = 1.0 / *(data->scale);
  }

  // get command
  tmpval = *(data->value);
  if (*(data->absmode) && (tmpval < 0)) {
    tmpval = -tmpval;
  }

  // convert value command to duty cycle
  tmpdc = tmpval * data->scale_recip + *(data->offset);
  if (tmpdc < *(data->min_dc)) {
    tmpdc = *(data->min_dc);
  }
  if (tmpdc > *(data->max_dc)) {
    tmpdc = *(data->max_dc);
  }

  // set output values
  if (*(data->enable) == 0) {
    raw_val = 0;
    *(data->pos) = 0;
    *(data->neg) = 0;
    *(data->curr_dc) = 0;
  } else {
    raw_val = (double)max_value * tmpdc;
    if (raw_val > (double)max_value) {
      raw_val = (double)max_value;
    }
    if (raw_val < (double)-max_value) {
      raw_val = (double)-max_value;
    }
    *(data->pos) = (*(data->value) > 0);
    *(data->neg) = (*(data->value) < 0);
    *(data->curr_dc) = tmpdc;
  }

  // update value
  EC_WRITE_S16(&pd[data->val_pdo_os], (int16_t)raw_val);
  *(data->raw_val) = (int32_t)raw_val;
}

/// @brief Writess data to all analog out ports.
///
/// @param slave The `slave`, passed from the per-device `_read`.
/// @param channels An `lcec_class_aout_channel_t *`, as returned by lcec_aout_register_channel.
void lcec_aout_write_all(lcec_slave_t *slave, lcec_class_aout_channels_t *channels) {
  for (int i = 0; i < channels->count; i++) {
    lcec_class_aout_channel_t *channel = channels->channels[i];

    lcec_aout_write(slave, channel);
  }
}
