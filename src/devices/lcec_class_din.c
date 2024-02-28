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

#include "lcec_class_din.h"

#include <stdio.h>

#include "../lcec.h"

static const lcec_pindesc_t slave_pins[] = {
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_din_channel_t, in), "%s.%s.%s.%s"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_din_channel_t, in_not), "%s.%s.%s.%s-not"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/// @brief allocates a block of memory for holding the result of
/// `count` calls to `lcec_din_register_device()`.
///
/// It is the caller's responsibility to verify that the result is not
/// NULL.
///
/// @param count The number of input pins to allocate memory for.
lcec_class_din_channels_t *lcec_din_allocate_channels(int count) {
  lcec_class_din_channels_t *channels;

  channels = hal_malloc(sizeof(lcec_class_din_channels_t));
  if (channels == NULL) {
    return NULL;
  }
  channels->count = count;
  channels->channels = hal_malloc(sizeof(lcec_class_din_channel_t *) * count);
  if (channels->channels == NULL) {
    return NULL;
  }

  return channels;
}

/// @brief registers a single digital-input channel and publishes it as a LinuxCNC HAL pin.
///
/// See lcec_el1xxx.c for an example of use.
///
/// This calls `lcec_din_register_channel_named()`, with a default name of `din-<ID>` provided.
///
/// @param slave the slave, from `_init`.
/// @param id  the pin ID.  Used for naming.  Should generally start at 0 and increment once per digital in pin.
/// @param idx the PDO index for the digital input.
/// @param sindx the PDO sub-index for the digital input.
///
lcec_class_din_channel_t *lcec_din_register_channel(lcec_slave_t *slave, int id, uint16_t idx, uint16_t sidx) {
  char name[32];

  snprintf(name, 32, "din-%d", id);

  return lcec_din_register_channel_named(slave, idx, sidx, name);
}

/// @brief registers a single digital-input channel and publishes it as a LinuxCNC HAL pin.
///
/// @param slave the slave, from `_init`.
/// @param id  the pin ID.  Used for naming.  Should generally start at 0 and increment once per digital in pin.
/// @param idx the PDO index for the digital input.
/// @param sindx the PDO sub-index for the digital input.
/// @param name The base name to use for the channel, `din-<ID>` is common.
lcec_class_din_channel_t *lcec_din_register_channel_named(lcec_slave_t *slave, uint16_t idx, uint16_t sidx, char *name) {
  lcec_class_din_channel_t *data;
  int err;

  data = hal_malloc(sizeof(lcec_class_din_channel_t));
  if (data == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s pin %s failed\n", slave->master->name, slave->name, name);
    return NULL;
  }
  memset(data, 0, sizeof(lcec_class_din_channel_t));
  data->name = name;
  data->pdo_bp_packed = 0xffff;  // Flag value, this isn't a packed channel.

  lcec_pdo_init(slave, idx, sidx, &data->pdo_os, &data->pdo_bp);
  err = lcec_pin_newf_list(data, slave_pins, LCEC_MODULE_NAME, slave->master->name, slave->name, name);
  if (err != 0) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_pin_newf_list for slave %s.%s pin %s failed\n", slave->master->name, slave->name, name);
    return NULL;
  }

  return data;
}

/// @brief registers a single digital-input channel and publishes it as a LinuxCNC HAL pin.
///
/// The `_packed` variant is for devices where several logical digital
/// input channels are packed into a single PDO.  For instance, the
/// RTelligent ECT60 has 6 digital input pins plus 3 synthetic ports,
/// all packed into a single U32.  This function is intended to be
/// called 9 times, with the same `idx` and `sdx` values, but varying
/// `bit` and `name` values.
///
/// @param slave the slave, from `_init`.
/// @param os  The offset from `LCEC_PDO_INIT()`.
/// @param bit  The bit offset for the digital in channel.
/// @param name The base name to use for the channel, `din-<ID>` is common.
lcec_class_din_channel_t *lcec_din_register_channel_packed(lcec_slave_t *slave, uint16_t idx, uint16_t sidx, int bit, char *name) {
  lcec_class_din_channel_t *data;
  int err;

  data = hal_malloc(sizeof(lcec_class_din_channel_t));
  if (data == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s pin %s failed\n", slave->master->name, slave->name, name);
    return NULL;
  }
  memset(data, 0, sizeof(lcec_class_din_channel_t));
  data->name = name;

  // Register the whole PDO, hopefully this does the sane thing if we register the same PDO repeatedly.
  lcec_pdo_init(slave, idx, sidx, &data->pdo_os, &data->pdo_bp);

  // This is kind of a terrible hack, but it should mostly work, modulo possible issues with variable-sized PDOs and endianness problems.
  data->pdo_bp_packed = bit;

  err = lcec_pin_newf_list(data, slave_pins, LCEC_MODULE_NAME, slave->master->name, slave->name, name);
  if (err != 0) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_pin_newf_list for slave %s.%s pin %s failed\n", slave->master->name, slave->name, name);
    return NULL;
  }

  return data;
}

/// @brief reads data from a single digital in port.
///
/// @param slave the slave, passed from the per-device `_read`.
/// @param data  a lcec_class_din_channel_t *, as returned by lcec_din_register_pin.
///
/// Call this once per pin registered, from inside of your device's
/// read function.  See `lcec_din_read_all` for an alternative approach.
void lcec_din_read(lcec_slave_t *slave, lcec_class_din_channel_t *data) {
  lcec_master_t *master = slave->master;
  uint8_t *pd = master->process_data;
  hal_bit_t s;
  int os = data->pdo_os;
  int bp = data->pdo_bp;

  // If this is a packed input, then we need to use the packed bit
  // offset, not the one that came back from LCEC_PDO_INIT().  We also
  // need to adjust the offset to pick the correct byte.
  if (data->pdo_bp_packed != 0xffff) {
    bp = data->pdo_bp_packed & 7;
    os += data->pdo_bp_packed >> 3;  // the data in pd[] is always little-endian.

    //    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "reading slave %s channel %s from %d:%d\n", slave->name, data->name, os, bp);
  }

  s = EC_READ_BIT(&pd[os], bp);
  *(data->in) = s;
  *(data->in_not) = !s;
}

/// @brief reads data from all digital in ports.
///
/// @param slave The slave, passed from the per-device `_read`.
/// @param channels An `lcec_class_din_channels_t *`, as returned by `lcec_din_allocate_channels()`.
void lcec_din_read_all(lcec_slave_t *slave, lcec_class_din_channels_t *channels) {
  for (int i = 0; i < channels->count; i++) {
    lcec_class_din_channel_t *channel = channels->channels[i];

    lcec_din_read(slave, channel);
  }
}
