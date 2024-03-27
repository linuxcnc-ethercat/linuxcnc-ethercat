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
/// @brief Library for CiA 402 servo/stepper controllers

#include "lcec_class_cia402.h"

#include <stdio.h>
#include <stdlib.h>

#include "../lcec.h"
#include "lcec_class_cia402_opt.h"

/// @brief Pins common to all CiA 402 devices
static const lcec_pindesc_t pins_required[] = {
    // HAL_OUT is readable, HAL_IN is writable.
    {HAL_U32, HAL_IN, offsetof(lcec_class_cia402_channel_t, controlword), "%s.%s.%s.%s-cia-controlword"},
    {HAL_U32, HAL_OUT, offsetof(lcec_class_cia402_channel_t, statusword), "%s.%s.%s.%s-cia-statusword"},
    {HAL_U32, HAL_OUT, offsetof(lcec_class_cia402_channel_t, supported_modes), "%s.%s.%s.%s-supported-modes"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_cia402_channel_t, supports_mode_pp), "%s.%s.%s.%s-supports-mode-pp"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_cia402_channel_t, supports_mode_vl), "%s.%s.%s.%s-supports-mode-vl"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_cia402_channel_t, supports_mode_pv), "%s.%s.%s.%s-supports-mode-pv"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_cia402_channel_t, supports_mode_tq), "%s.%s.%s.%s-supports-mode-tq"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_cia402_channel_t, supports_mode_hm), "%s.%s.%s.%s-supports-mode-hm"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_cia402_channel_t, supports_mode_ip), "%s.%s.%s.%s-supports-mode-ip"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_cia402_channel_t, supports_mode_csp), "%s.%s.%s.%s-supports-mode-csp"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_cia402_channel_t, supports_mode_csv), "%s.%s.%s.%s-supports-mode-csv"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_class_cia402_channel_t, supports_mode_cst), "%s.%s.%s.%s-supports-mode-cst"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/// @brief Create a new, optional pin for reading, using standardized names.
#define OPTIONAL_PIN_READ(var_name)                                                                                                \
  static const lcec_pindesc_t pins_##var_name[] = {                                                                                \
      {PDO_PIN_TYPE_##var_name, HAL_OUT, offsetof(lcec_class_cia402_channel_t, var_name), "%s.%s.%s.%s-" PDO_PIN_NAME_##var_name}, \
      {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},                                                                       \
  }

/// @brief Create a new, optional pin for writing, using standardized names.
#define OPTIONAL_PIN_WRITE(var_name)                                                                                              \
  static const lcec_pindesc_t pins_##var_name[] = {                                                                               \
      {PDO_PIN_TYPE_##var_name, HAL_IN, offsetof(lcec_class_cia402_channel_t, var_name), "%s.%s.%s.%s-" PDO_PIN_NAME_##var_name}, \
      {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},                                                                      \
  }

// These three create a whole slew of `pin_foo` variables that define
// the pins for each PDO and SDO that we want pins for.
FOR_ALL_READ_PDOS_DO(OPTIONAL_PIN_READ);
FOR_ALL_WRITE_PDOS_DO(OPTIONAL_PIN_WRITE);
FOR_ALL_WRITE_SDOS_DO(OPTIONAL_PIN_WRITE);

/// @brief Create a `lcec_class_cia402_enabled_t` from a
/// `lcec_class_cia402_channel_options_t`.
static lcec_class_cia402_enabled_t *lcec_cia402_enabled(lcec_class_cia402_channel_options_t *opt) {
  lcec_class_cia402_enabled_t *enabled;
  enabled = hal_malloc(sizeof(lcec_class_cia402_enabled_t));
  if (enabled == NULL) return NULL;

  memset(enabled, 0, sizeof(lcec_class_cia402_enabled_t));

  if (opt->enable_opmode) {
    enabled->enable_opmode = 1;
    enabled->enable_opmode_display = 1;
  }
  if (opt->enable_pp) {
    enabled->enable_actual_position = 1;
    enabled->enable_target_position = 1;
  }
  if (opt->enable_pv) {
    enabled->enable_actual_velocity = 1;
    enabled->enable_target_velocity = 1;
  }
  if (opt->enable_csp) {
    enabled->enable_actual_position = 1;
    enabled->enable_actual_velocity = 1;
    enabled->enable_target_position = 1;
  }
  if (opt->enable_csv) {
    enabled->enable_actual_velocity = 1;
    enabled->enable_target_velocity = 1;
  }
  if (opt->enable_hm) {
    enabled->enable_hm = 1;
    enabled->enable_home_method = 1;
    enabled->enable_home_velocity_fast = 1;
    enabled->enable_home_velocity_slow = 1;
  }
  if (opt->enable_ip) {
    // TODO: add interpolation pins once they're added.
  }
  if (opt->enable_vl) {
    enabled->enable_target_vl = 1;
    enabled->enable_demand_vl = 1;
    enabled->enable_actual_vl = 1;
    enabled->enable_vl_minimum = 1;
    enabled->enable_vl_maximum = 1;
  }
  if (opt->enable_cst) {
    // TODO: add cyclic synchronous torque pins once they're added.
  }

  // Set individual pins in `enabled` using values from `opt`.
#define ENABLE_OPT(pin_name) \
  if (opt->enable_##pin_name) enabled->enable_##pin_name = 1;

  // Copy all `enable_foo` settings from `opt` to `enabled`.
  FOR_ALL_OPTS_DO(ENABLE_OPT);

  return enabled;
}

/// @brief Allocate a block of memory for holding the results from
/// `count` calls to `lcec_cia402_register_device() and friends.
///
/// It is the caller's responsibility to verify that the result is not NULL.
///
/// @param count The number of input channels (axes) to allocate
///              memory for.
/// @return A `lcec_class_cia402_channels_t` for holding
///         future results from `lcec_cia402_register_channel()`.
lcec_class_cia402_channels_t *lcec_cia402_allocate_channels(int count) {
  lcec_class_cia402_channels_t *channels;

  channels = hal_malloc(sizeof(lcec_class_cia402_channels_t));
  if (channels == NULL) {
    return NULL;
  }
  channels->count = count;
  channels->channels = hal_malloc(sizeof(lcec_class_cia402_channel_t *) * count);
  if (channels->channels == NULL) {
    return NULL;
  }
  return channels;
}

/// @brief Allocates a `lcec_class_cia402_options_t` and initializes it.
lcec_class_cia402_options_t *lcec_cia402_options(void) {
  lcec_class_cia402_options_t *opts = hal_malloc(sizeof(lcec_class_cia402_options_t));
  if (opts == NULL) {
    return NULL;
  }
  memset(opts, 0, sizeof(lcec_class_cia402_options_t));
  opts->channels = 1;

  for (int channel = 0; channel < 8; channel++) {
    opts->channel[channel] = lcec_cia402_channel_options();
  }

  return opts;
}

/// @brief Allocates a `lcec_class_cia402_channel_options_t` and initializes it.
lcec_class_cia402_channel_options_t *lcec_cia402_channel_options(void) {
  lcec_class_cia402_channel_options_t *opts = hal_malloc(sizeof(lcec_class_cia402_channel_options_t));
  if (opts == NULL) {
    return NULL;
  }
  memset(opts, 0, sizeof(lcec_class_cia402_channel_options_t));
  opts->enable_opmode = 1;  // Should almost always be enabled.
  opts->digital_in_channels = 16;
  opts->digital_out_channels = 16;

  return opts;
}

/// @brief Rename pins for multi-axis devices.
///
/// For single-axis devices, pins are named like `srv-foo`.  For
/// multi-axis devices, where we have multiple `foo` pins, we need to
/// use names like `srv-1-foo` instead.
void lcec_cia402_rename_multiaxis_channels(lcec_class_cia402_options_t *opt) {
  for (int channel = 0; channel < opt->channels; channel++) {
    char *prefix = hal_malloc(16);
    snprintf(prefix, 16, "srv-%d", channel + 1);
    opt->channel[channel]->name_prefix = prefix;
  }
}

/// @brief Allocates a `lcec_syncs_t` and fills in the CiA 402 portion of it using data from `options`.
///
/// @param opt A `lcec_class_cia402_channel_options_t` structure that describes the options in use.
lcec_syncs_t *lcec_cia402_init_sync(lcec_slave_t *slave, lcec_class_cia402_options_t *options) {
  lcec_syncs_t *syncs;

  syncs = hal_malloc(sizeof(lcec_syncs_t));
  if (syncs == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for lcec_cia402_init_sync failed\n");
    return NULL;
  }
  lcec_syncs_init(slave, syncs);
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DEFAULT);

  return syncs;
}

/// @brief Add an optional PDO, if it's enabled in `enabled`.
///
/// This simply removes a bunch of boilerplate code and makes optional
/// PDOs more readable.
#define MAP_OPTIONAL_PDO(name)                                                                         \
  if (enabled->enable_##name) {                                                                        \
    lcec_syncs_add_pdo_entry(syncs, offset + PDO_IDX_OFFSET_##name, PDO_SIDX_##name, PDO_BITS_##name); \
  }

#define PRINT_OPTIONAL_PDO_NAME(name)                                                                   \
  if (enabled->enable_##name && PDO_MP_NAME_##name[0] != 0) {                                           \
    rtapi_print_msg(RTAPI_MSG_ERR, "  -  <modParam name=\"" PDO_MP_NAME_##name "\" value=\"true\">\n"); \
  }

/// @brief Sets up the first batch of output PDOs for syncing.
///
/// This should be called after `lcec_cia402_init_sync()`, but before
/// registering any device-specific PDOs.  Once this returns, then
/// call `lcec_syncs_add_pdo_info(syncs, 0x1601)` and whichever
/// `lcec_syncs_add_pdo_entry()` calls you need.
int lcec_cia402_add_output_sync(lcec_slave_t *slave, lcec_syncs_t *syncs, lcec_class_cia402_options_t *options) {
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
  for (int channel = 0; channel < options->channels; channel++) {
    unsigned int offset = 0x6000 + 0x800 * channel;
    int entrycount = syncs->pdo_entry_count;

    lcec_class_cia402_enabled_t *enabled = lcec_cia402_enabled(options->channel[channel]);
    if (enabled == NULL) return -1;

    lcec_syncs_add_pdo_info(syncs, 0x1600 + channel);
    lcec_syncs_add_pdo_entry(syncs, offset + 0x40, 0x00, 16);  // Control word

    // Map all writeable PDOs, plus `digital_output` which is special
    // but still needs to be mapped.
    FOR_ALL_WRITE_PDOS_DO(MAP_OPTIONAL_PDO);
    MAP_OPTIONAL_PDO(digital_output);

    if (options->rxpdolimit && (syncs->pdo_entry_count - entrycount) > options->rxpdolimit) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          LCEC_MSG_PFX "slave %s.%s: FAILURE: FAILURE: more output PDO entries configured than your hardware supports.\n",
          slave->master->name, slave->name);
      rtapi_print_msg(RTAPI_MSG_ERR, "Axis %d has %d PDO entries, vs a configured limit of %d.  You will need to edit your XML\n",
          channel + 1, syncs->pdo_entry_count - entrycount, options->rxpdolimit);
      rtapi_print_msg(RTAPI_MSG_ERR, "configuration and either remove some <modParam name=\"enable*\"> entries or increase \n");
      rtapi_print_msg(
          RTAPI_MSG_ERR, "<modParam name=\"ciaRxPDOEntryLimit\">. Check your CiA 402 slave's hardware manual to determine the\n");
      rtapi_print_msg(RTAPI_MSG_ERR, "correct limit.\n\n");
      rtapi_print_msg(RTAPI_MSG_ERR, "Enabled features that impact this limit are:\n\n");
      FOR_ALL_WRITE_PDOS_DO(PRINT_OPTIONAL_PDO_NAME);
      rtapi_print_msg(
          RTAPI_MSG_ERR, "\nIn addition, disabling unneeded CiA 402 modes may help, as some implicitly add additional PDO entries:\n\n");
      FOR_ALL_CIA402_MODES_DO(PRINT_OPTIONAL_PDO_NAME);

      return -1;
    }
  }

  return 0;
};

/// @brief Sets up the first batch of input PDOs for syncing.
///
/// This should be called after you're done with all output PDOs.
/// Once this returns, then call `lcec_syncs_add_pdo_info(syncs,
/// 0x1a01)` and whichever `lcec_syncs_add_pdo_entry()` calls you
/// need.
int lcec_cia402_add_input_sync(lcec_slave_t *slave, lcec_syncs_t *syncs, lcec_class_cia402_options_t *options) {
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DEFAULT);
  for (int channel = 0; channel < options->channels; channel++) {
    unsigned int offset = 0x6000 + 0x800 * channel;
    int entrycount = syncs->pdo_entry_count;

    lcec_class_cia402_enabled_t *enabled = lcec_cia402_enabled(options->channel[channel]);
    if (enabled == NULL) return -1;

    lcec_syncs_add_pdo_info(syncs, 0x1a00 + channel);
    lcec_syncs_add_pdo_entry(syncs, offset + 0x41, 0x00, 16);  // Status word

    // Map all readable PDOs, plus `digital_input` which is special
    // but still needs to be mapped.
    FOR_ALL_READ_PDOS_DO(MAP_OPTIONAL_PDO);
    MAP_OPTIONAL_PDO(digital_input);  // Special

    if (options->txpdolimit && (syncs->pdo_entry_count - entrycount) > options->txpdolimit) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          LCEC_MSG_PFX "slave %s.%s: FAILURE: FAILURE: more input PDO entries configured than your hardware supports.\n",
          slave->master->name, slave->name);
      rtapi_print_msg(RTAPI_MSG_ERR, "Axis %d has %d PDO entries, vs a configured limit of %d.  You will need to edit your XML\n",
          channel + 1, syncs->pdo_entry_count - entrycount, options->txpdolimit);
      rtapi_print_msg(RTAPI_MSG_ERR, "configuration and either remove some <modParam name=\"enable*\"> entries or increase \n");
      rtapi_print_msg(
          RTAPI_MSG_ERR, "<modParam name=\"ciaTxPDOEntryLimit\">. Check your CiA 402 slave's hardware manual to determine the\n");
      rtapi_print_msg(RTAPI_MSG_ERR, "correct limit.\n\n");
      rtapi_print_msg(RTAPI_MSG_ERR, "Enabled features that impact this limit are:\n\n");
      FOR_ALL_READ_PDOS_DO(PRINT_OPTIONAL_PDO_NAME);
      rtapi_print_msg(
          RTAPI_MSG_ERR, "\nIn addition, disabling unneeded CiA 402 modes may help, as some implicitly add additional PDO entries:\n\n");
      FOR_ALL_CIA402_MODES_DO(PRINT_OPTIONAL_PDO_NAME);

      return -1;
    }
  }

  return 0;
};

/// @brief Register a new CiA 402 channel.
///
/// This creates a new CiA 402 channel, which is basically a single
/// axis for a CiA 402-compatible stepper or servo controller.
///
/// @param slave The `slave` passed into `_init`.
/// @param base_idx The base index for PDOs for this channel.  For
///   single-axis devices, this should be 0x6000.  For multi-axis
///   devices, it should be 0x6000 for the first, then 0x6800, 0x7000,
///   0x7800, etc.  See the documentation for your servo driver for
///   details.
/// @param opt Optional settings.  `NULL` for defaults, or a
///   `lcec_class_cia402_channel_options_t` from
///   `lcec_cia402_channel_options()`.
lcec_class_cia402_channel_t *lcec_cia402_register_channel(
    lcec_slave_t *slave, uint16_t base_idx, lcec_class_cia402_channel_options_t *opt) {
  lcec_class_cia402_channel_t *data;
  int err;
  lcec_class_cia402_enabled_t *enabled;

  // The default name depends on the port type.
  char *name_prefix = "srv";
  if (opt && opt->name_prefix) name_prefix = opt->name_prefix;

  // If we were passed a NULL opt, then create a new
  // `lcec_class_cia402_channel_options_t` and write the defaults back into it,
  // so we don't need to repeat the above default code downstream.
  if (!opt) {
    opt = lcec_cia402_channel_options();
    if (opt == NULL) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", slave->master->name, slave->name);
      return NULL;
    }
  }

  // Allocate memory for per-channel data.
  data = hal_malloc(sizeof(lcec_class_cia402_channel_t));
  if (data == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", slave->master->name, slave->name);
    return NULL;
  }
  memset(data, 0, sizeof(lcec_class_cia402_channel_t));

  data->options = opt;
  data->base_idx = base_idx;

  // Set the `enabled` struct from `opt`.
  enabled = lcec_cia402_enabled(opt);
  if (enabled == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", slave->master->name, slave->name);
    return NULL;
  }
  data->enabled = enabled;

  // Register PDOs
  lcec_pdo_init(slave, base_idx + 0x40, 0, &data->controlword_os, NULL);
  lcec_pdo_init(slave, base_idx + 0x41, 0, &data->statusword_os, NULL);

#define INIT_OPTIONAL_PDO(pin_name) \
  if (enabled->enable_##pin_name)   \
  lcec_pdo_init(slave, base_idx + PDO_IDX_OFFSET_##pin_name, PDO_SIDX_##pin_name, &data->pin_name##_os, NULL)

  // Call `lcec_pdo_init()` for all readable and writeable PDOs.
  FOR_ALL_READ_PDOS_DO(INIT_OPTIONAL_PDO);
  FOR_ALL_WRITE_PDOS_DO(INIT_OPTIONAL_PDO);

#define INIT_SDO_REQUEST(pin_name) \
  data->pin_name##_sdorequest =    \
      ecrt_slave_config_create_sdo_request(slave->config, base_idx + PDO_IDX_OFFSET_##pin_name, PDO_SIDX_##pin_name, PDO_BITS_##pin_name)

  // Call `ecrt_slave_config_create_sdo_request()` for all writable
  // SDOs, so we're able to write to them after we flip to real-time
  // mode.
  FOR_ALL_WRITE_SDOS_DO(INIT_SDO_REQUEST);

  // Register pins
  err = lcec_pin_newf_list(data, pins_required, LCEC_MODULE_NAME, slave->master->name, slave->name, name_prefix);
  if (err != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_pin_newf_list for slave %s.%s failed\n", slave->master->name, slave->name);
    return NULL;
  }

  // Set up digital in/out
  if (enabled->enable_digital_input) {
    char *dname;
    data->din = lcec_din_allocate_channels(opt->digital_in_channels + 4);
    if (data->din == NULL) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_din_allocate_channels() for slave %s.%s failed\n", slave->master->name, slave->name);
      return NULL;
    }

    dname = hal_malloc(sizeof(char[20]));
    snprintf(dname, 20, "%s-din-cw-limit", name_prefix);
    data->din->channels[0] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 0, dname);  // negative limit switch
    dname = hal_malloc(sizeof(char[20]));
    snprintf(dname, 20, "%s-din-ccw-limit", name_prefix);
    data->din->channels[1] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 1, dname);  // positive limit switch
    dname = hal_malloc(sizeof(char[20]));
    snprintf(dname, 20, "%s-din-home", name_prefix);
    data->din->channels[2] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 2, dname);  // home
    dname = hal_malloc(sizeof(char[20]));
    snprintf(dname, 20, "%s-din-interlock", name_prefix);
    data->din->channels[3] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 3, dname);  // interlock?

    for (int channel = 0; channel < opt->digital_in_channels; channel++) {
      dname = hal_malloc(sizeof(char[20]));
      if (dname == NULL) return NULL;

      snprintf(dname, 20, "%s-din-%d", name_prefix, channel);
      data->din->channels[4 + channel] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 16 + channel, dname);
    }
  }

  if (enabled->enable_digital_output) {
    char *dname;
    data->dout = lcec_dout_allocate_channels(opt->digital_out_channels + 1);
    if (data->din == NULL) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_din_allocate_channels() for slave %s.%s failed\n", slave->master->name, slave->name);
      return NULL;
    }
    dname = hal_malloc(sizeof(char[20]));
    snprintf(dname, 20, "%s-dout-brake", name_prefix);
    data->dout->channels[0] = lcec_dout_register_channel_packed(slave, base_idx + 0xfe, 1, 0, dname);  // brake
    for (int channel = 0; channel < opt->digital_out_channels; channel++) {
      dname = hal_malloc(sizeof(char[20]));
      if (dname == NULL) return NULL;

      snprintf(dname, 20, "%s-dout-%d", name_prefix, channel);
      data->dout->channels[1 + channel] = lcec_dout_register_channel_packed(slave, base_idx + 0xfe, 1, 16 + channel, dname);
    }
  }

#define REGISTER_OPTIONAL_PINS(pin_name)                                                                                              \
  do {                                                                                                                                \
    if (enabled->enable_##pin_name) {                                                                                                 \
      err = lcec_pin_newf_list(data, pins_##pin_name, LCEC_MODULE_NAME, slave->master->name, slave->name, name_prefix);               \
      if (err != 0) {                                                                                                                 \
        rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_pin_newf_list for slave %s.%s failed\n", slave->master->name, slave->name); \
        return NULL;                                                                                                                  \
      }                                                                                                                               \
    }                                                                                                                                 \
  } while (0)
  //

  // Register pins for all read PDOs, write PDOs, and write SDOs.  The
  // process is identical for all three.
  FOR_ALL_READ_PDOS_DO(REGISTER_OPTIONAL_PINS);
  FOR_ALL_WRITE_PDOS_DO(REGISTER_OPTIONAL_PINS);
  FOR_ALL_WRITE_SDOS_DO(REGISTER_OPTIONAL_PINS);

  // Set default values for pins here.
  uint32_t modes;
  lcec_read_sdo32(slave, base_idx + 0x502, 0, &modes);

  *(data->supported_modes) = modes;
  *(data->supports_mode_pp) = modes & 1 << 0;
  *(data->supports_mode_vl) = modes & 1 << 1;
  *(data->supports_mode_pv) = modes & 1 << 2;
  *(data->supports_mode_tq) = modes & 1 << 3;
  *(data->supports_mode_hm) = modes & 1 << 5;
  *(data->supports_mode_ip) = modes & 1 << 6;
  *(data->supports_mode_csp) = modes & 1 << 7;
  *(data->supports_mode_csv) = modes & 1 << 8;
  *(data->supports_mode_cst) = modes & 1 << 9;

  /// @brief Initializes a pin's defaults using the current value of the backing SDO.
  ///
  /// Calling `SET_OPTIONAL_DEFAULTS_FOO` checks to see if
  /// `enabled->enable_FOO` is true, and if so it sets the pin data
  /// stores in `data->FOO` to the current value of the SDO behind it.
  ///
  /// It does this by looking at PDO_IDX_OFFSET_FOO and PDO_SIDX_FOO
  /// to find the SDO port.  Then it reads the correct number of bits
  /// and the correct signedness using `lcec_read_sdo_XX_pin_YY32`,
  /// where XX is the value of PDO_BITS_FOO and YY is value of
  /// PDO_SIGN_FOO.
  ///
  /// The upshot?  Call SET_OPTIONAL_DEFAULTS(FOO), and the right
  /// thing happens.
#define SET_OPTIONAL_DEFAULTS(pin_name)                                                                          \
  if (enabled->enable_##pin_name) SUBSTJOIN5(lcec_read_sdo, PDO_BITS_##pin_name, _pin_, PDO_SIGN_##pin_name, 32) \
  (slave, base_idx + PDO_IDX_OFFSET_##pin_name, PDO_SIDX_##pin_name, data->pin_name)

  // Read the current value of all of our writable PDOs and SDOs, so
  // (a) we have a reasonable default and (b) we don't immediately
  // overwrite the presumably-valid settings on the device.
  FOR_ALL_WRITE_PDOS_DO(SET_OPTIONAL_DEFAULTS);
  FOR_ALL_WRITE_SDOS_DO(SET_OPTIONAL_DEFAULTS);

  return data;
}

/// @brief Reads data from a single CiA 402 channel (one axis).
///
/// @param slave The `slave`, passed from the per-device `_read`.
/// @param data  Which channel to read; a `lcec_class_cia402_channel_t *`, as returned by lcec_cia402_register_channel.
///
/// Call this once per channel registered, from inside of your device's
/// read function.  Use `lcec_cia402_read_all` to read all channels.
void lcec_cia402_read(lcec_slave_t *slave, lcec_class_cia402_channel_t *data) {
  uint8_t *pd = slave->master->process_data;

#define READ_OPT(pin_name)              \
  if (data->enabled->enable_##pin_name) \
  *(data->pin_name) = (SUBSTJOIN3(EC_READ_, PDO_SIGN_##pin_name, PDO_BITS_##pin_name)(&pd[data->pin_name##_os]))

  *(data->statusword) = EC_READ_U16(&pd[data->statusword_os]);

  // Read from all readable PDOs.
  FOR_ALL_READ_PDOS_DO(READ_OPT);

  if (data->enabled->enable_digital_input) {
    lcec_din_read_all(slave, data->din);
  }
}

/// @brief Reads data from all CiA 402 input ports.
///
/// @param slave The `slave`, passed from the per-device `_read`.
/// @param channels An `lcec_class_cia402_channel_t *`, as returned by lcec_cia402_register_channel.
void lcec_cia402_read_all(lcec_slave_t *slave, lcec_class_cia402_channels_t *channels) {
  for (int i = 0; i < channels->count; i++) {
    lcec_cia402_read(slave, channels->channels[i]);
  }
}

#define WRITE_OPT(name) \
  if (data->enabled->enable_##name) SUBSTJOIN3(EC_WRITE_, PDO_SIGN_##name, PDO_BITS_##name)(&pd[data->name##_os], *(data->name))

/// OK, this is kind of a mess.  It presents the same interface as
/// WRITE_OPT(), but instead of writing to a mapped PDO entry, it
/// writes to an SDO.  Unfortunately, Etherlab's EtherCAT library
/// doesn't make this entirely trivial.  We have to have allocated an
/// SDO request before real-time mode started (which we did, it's
/// stored in `name##_sdorequest`).  Then we need to do 3 things:
///
/// 1. Make sure that another write isn't in progress for this SDO.
///    To do this, we need to check `ecrt_sdo_request_state()` and make
///    sure that it's not EC_REQUEST_BUSY.
/// 2. Next, we need to write the data into the request struct, using
///    `ecrt_sdo_request_data()` and one of the
///    `EC_WRITE_*()`. macros.
/// 3. Finally, we call `ecrt_sdo_request_write()` to start a write.
///    It may not finish for a while.
#define WRITE_OPT_SDO(name)                                                                   \
  do {                                                                                        \
    if (data->enabled->enable_##name) {                                                       \
      if (*(data->name) != data->name##_old) {                                                \
        if (ecrt_sdo_request_state(data->name##_sdorequest) != EC_REQUEST_BUSY) {             \
          data->name##_old = *(data->name);                                                   \
          uint8_t *sdo_tmp = ecrt_sdo_request_data(data->name##_sdorequest);                  \
          SUBSTJOIN3(EC_WRITE_, PDO_SIGN_##name, PDO_BITS_##name)(sdo_tmp, data->name##_old); \
          ecrt_sdo_request_write(data->name##_sdorequest);                                    \
        }                                                                                     \
      }                                                                                       \
    }                                                                                         \
  } while (0)

void lcec_cia402_write(lcec_slave_t *slave, lcec_class_cia402_channel_t *data) {
  uint8_t *pd = slave->master->process_data;

  EC_WRITE_U16(&pd[data->controlword_os], (uint16_t)(*(data->controlword)));

  // Write PDOs (mapped, auto-synced between slaves and the master)
  FOR_ALL_WRITE_PDOS_DO(WRITE_OPT);

  // Write SDOs (*not* mapped, written on demand, slower)
  FOR_ALL_WRITE_SDOS_DO(WRITE_OPT_SDO);

  if (data->enabled->enable_digital_output) {
    lcec_dout_write_all(slave, data->dout);
  }
}

/// @brief Writess data to all CiA 402 output ports.
///
/// @param slave The `slave`, passed from the per-device `_read`.
/// @param channels An `lcec_class_cia402_channel_t *`, as returned by lcec_cia402_register_channel.
void lcec_cia402_write_all(lcec_slave_t *slave, lcec_class_cia402_channels_t *channels) {
  for (int i = 0; i < channels->count; i++) {
    lcec_class_cia402_channel_t *channel = channels->channels[i];

    lcec_cia402_write(slave, channel);
  }
}

#define ENABLE_MODPARAM(name) {PDO_MP_NAME_##name, CIA402_MP_ENABLE_##name, MODPARAM_TYPE_BIT},

/// @brief Modparams settings available via XML.
static const lcec_modparam_desc_t per_channel_modparams[] = {
    // Configuration modParams
    {"positionLimitMin", CIA402_MP_POSLIMIT_MIN, MODPARAM_TYPE_S32},
    {"positionLimitMax", CIA402_MP_POSLIMIT_MAX, MODPARAM_TYPE_S32},
    {"swPositionLimitMin", CIA402_MP_SWPOSLIMIT_MIN, MODPARAM_TYPE_S32},
    {"swPositionLimitMax", CIA402_MP_SWPOSLIMIT_MIN, MODPARAM_TYPE_S32},
    {"homeOffset", CIA402_MP_HOME_OFFSET, MODPARAM_TYPE_S32},
    {"quickDecel", CIA402_MP_QUICKDECEL, MODPARAM_TYPE_U32},
    {"quickStopOptionCode", CIA402_MP_OPTCODE_QUICKSTOP, MODPARAM_TYPE_S32},
    {"positioningOptionCode", CIA402_MP_OPTCODE_POSITIONING, MODPARAM_TYPE_S32},
    {"connectionOptionCode", CIA402_MP_OPTCODE_CONNECTION, MODPARAM_TYPE_S32},
    {"shutdownOptionCode", CIA402_MP_OPTCODE_SHUTDOWN, MODPARAM_TYPE_S32},
    {"disableOptionCode", CIA402_MP_OPTCODE_DISABLE, MODPARAM_TYPE_S32},
    {"haltOptionCode", CIA402_MP_OPTCODE_HALT, MODPARAM_TYPE_S32},
    {"faultOptionCode", CIA402_MP_OPTCODE_FAULT, MODPARAM_TYPE_S32},
    {"probeFunction", CIA402_MP_PROBE_FUNCTION, MODPARAM_TYPE_U32},
    {"probe1Positive", CIA402_MP_PROBE1_POS, MODPARAM_TYPE_S32},
    {"probe1Negative", CIA402_MP_PROBE1_NEG, MODPARAM_TYPE_S32},
    {"probe2Positive", CIA402_MP_PROBE2_POS, MODPARAM_TYPE_S32},
    {"probe2Negative", CIA402_MP_PROBE2_NEG, MODPARAM_TYPE_S32},
    {"digitalInChannels", CIA402_MP_DIGITAL_IN_CHANNELS, MODPARAM_TYPE_U32},
    {"digitalOutChannels", CIA402_MP_DIGITAL_OUT_CHANNELS, MODPARAM_TYPE_U32},
    {"vlQuickStopRatio", CIA402_MP_VL_QUICKSTOP_RATIO, MODPARAM_TYPE_STRING},
    {"vlSetPoint", CIA402_MP_VL_SET_POINT, MODPARAM_TYPE_STRING},
    {"vlDimensionFactor", CIA402_MP_VL_DIMENSION_FACTOR, MODPARAM_TYPE_STRING},
    {"encoderRatio", CIA402_MP_POS_ENCODER_RATIO, MODPARAM_TYPE_STRING},
    {"velEncoderRatio", CIA402_MP_VEL_ENCODER_RATIO, MODPARAM_TYPE_STRING},
    {"gearRatio", CIA402_MP_GEAR_RATIO, MODPARAM_TYPE_STRING},
    {"feedRatio", CIA402_MP_FEED_RATIO, MODPARAM_TYPE_STRING},
    {"eGearRatio", CIA402_MP_EGEAR_RATIO, MODPARAM_TYPE_STRING},
    {"vlAccel", CIA402_MP_VL_ACCEL, MODPARAM_TYPE_STRING},
    {"vlDecel", CIA402_MP_VL_DECEL, MODPARAM_TYPE_STRING},

    // Create entries for all options using the names from
    // lcec_class_cia402_opt.h:
    FOR_ALL_OPTS_DO(ENABLE_MODPARAM)

        {NULL},
};

/// @brief Duplicate modparams with per-channel options
///
/// This reads from `lcec_class_cia402_modparams` and produces a new
/// list of modparams that includes versions of all modparams for 8
/// distinct channels, for use with multi-axis CiA 402 devices.
///
/// Specifically, this reads from `lcec_class_cia402_modparams` and
/// returns a version where each line is duplicated 9 times.  The
/// first duplicate is just a copy of the entry from
/// `lcec_class_cia402_modparams`; the other 8 are channel-specific
/// versions, with names and ID numbers modified from the original.
/// If there's an entry for `foo` with an ID of 10, then this will
/// produce 9 outputs:
///
/// - `{"foo", 10}`
/// - `{"ch1foo", 10}`
/// - `{"ch2foo", 11}`
/// - `{"ch3foo", 12}`
/// - `{"ch4foo", 13}`
/// - `{"ch5foo", 13}`
/// - `{"ch6foo", 13}`
/// - `{"ch7foo", 13}`
///
/// Note that this makes `ch1foo` equivalant to `foo`.  In all cases,
/// `foo` should set a parameter for the first channel, not all
/// channels.
///
/// If we have device-level modParams, then we should handle them via
/// a different list.
lcec_modparam_desc_t *lcec_cia402_channelized_modparams(lcec_modparam_desc_t const *orig) {
  lcec_modparam_desc_t *mp;
  int l;

  l = lcec_modparam_desc_len(orig);

  mp = malloc(sizeof(lcec_modparam_desc_t) * (l * 9 + 1));
  if (mp == NULL) {
    return NULL;
  }

  mp[l * 9] = orig[l];  // Copy terminator.

  for (l = 0; orig[l].name != NULL; l++) {
    mp[l * 9] = orig[l];
    for (int i = 1; i < 9; i++) {
      char *name;
      mp[l * 9 + i] = orig[l];

      name = malloc(strlen(orig[l].name) + 10);
      if (name == NULL) {
        free(mp);
        return NULL;
      }
      sprintf(name, "ch%d%s", i, orig[l].name);
      mp[l * 9 + i].name = name;
      mp[l * 9 + i].id += i - 1;
    }
  }

  return mp;
}

/// @brief Merge per-device modParams and channelized generic CiA 402
/// modParams into a single list.
///
/// @param device_mps a `lcec_modparam_desc_t[]` containing all of the
/// device-specific `<modParam>`settings.
lcec_modparam_desc_t *lcec_cia402_modparams(lcec_modparam_desc_t const *device_mps) {
  const lcec_modparam_desc_t *channelized_mps = lcec_cia402_channelized_modparams(per_channel_modparams);
  if (channelized_mps == NULL) return NULL;

  return lcec_modparam_desc_concat(device_mps, channelized_mps);
}

/// @brief Handle a single modparam entry
///
/// This should be called as part of the slave's modparam handling
/// code, in a loop over `slave->modparams`.  This will set SDOs on
/// the slave device as needed.
///
/// @param slave The `lcec_slave` passed to `_init`.
/// @param p The current modparam being processed.
///
/// @return 0 if the modparam was handled, 1 if it was not handled, and <0 if an error occurred.
int lcec_cia402_handle_modparam(lcec_slave_t *slave, const lcec_slave_modparam_t *p, lcec_class_cia402_options_t *opt) {
  if (p->id < CIA402_MP_BASE) {
    return 0;
  }

  // Each of these params is available in 9 forms:
  // `foo`: set foo for channel 0.  Generally used for single-axis devices.
  // `ch0foo`: set foo for channel 0.  Identical to `foo`, above.
  // `ch1foo`: set foo for channel 1.
  // ...
  // `ch8foo`: set foo for channel 8.
  //
  // The `id` for CiA402 modparams needs to be coded so that the channel is the low-order 3 bits.

  // To keep the switch statement from getting weird, this breaks the
  // channel and ID apart so we can handle them independently.
  int channel = p->id & 7;
  int id = p->id & ~7;
  int base = 0x6000 + 0x800 * channel;
  lcec_ratio ratio;

#define CASE_MP_S8(mp_name, idx, sidx) \
  case mp_name:                        \
    return lcec_write_sdo8_modparam(slave, idx, sidx, p->value.s32, p->name)
#define CASE_MP_S16(mp_name, idx, sidx) \
  case mp_name:                         \
    return lcec_write_sdo16_modparam(slave, idx, sidx, p->value.s32, p->name)
#define CASE_MP_U16(mp_name, idx, sidx) \
  case mp_name:                         \
    return lcec_write_sdo16_modparam(slave, idx, sidx, p->value.u32, p->name)
#define CASE_MP_S32(mp_name, idx, sidx) \
  case mp_name:                         \
    return lcec_write_sdo32_modparam(slave, idx, sidx, p->value.s32, p->name)
#define CASE_MP_U32(mp_name, idx, sidx) \
  case mp_name:                         \
    return lcec_write_sdo32_modparam(slave, idx, sidx, p->value.u32, p->name)
#define CASE_MP_ENABLE_BIT(pin_name)                         \
  case CIA402_MP_ENABLE_##pin_name:                          \
    opt->channel[channel]->enable_##pin_name = p->value.bit; \
    return 0;

  switch (id) {
    CASE_MP_S32(CIA402_MP_POSLIMIT_MIN, base + 0x7b, 1);
    CASE_MP_S32(CIA402_MP_POSLIMIT_MAX, base + 0x7b, 2);
    CASE_MP_S32(CIA402_MP_SWPOSLIMIT_MIN, base + 0x7d, 1);
    CASE_MP_S32(CIA402_MP_SWPOSLIMIT_MAX, base + 0x7d, 2);
    CASE_MP_S32(CIA402_MP_HOME_OFFSET, base + 0x7c, 0);
    CASE_MP_U32(CIA402_MP_QUICKDECEL, base + 0x85, 0);
    CASE_MP_S16(CIA402_MP_OPTCODE_QUICKSTOP, base + 0x5a, 0);
    CASE_MP_S16(CIA402_MP_OPTCODE_SHUTDOWN, base + 0x5b, 0);
    CASE_MP_S16(CIA402_MP_OPTCODE_DISABLE, base + 0x5c, 0);
    CASE_MP_S16(CIA402_MP_OPTCODE_HALT, base + 0x5d, 0);
    CASE_MP_S16(CIA402_MP_OPTCODE_FAULT, base + 0x5e, 0);
    CASE_MP_S16(CIA402_MP_OPTCODE_POSITIONING, base + 0xf2, 0);
    CASE_MP_S16(CIA402_MP_OPTCODE_CONNECTION, base + 0x7, 0);

    CASE_MP_U16(CIA402_MP_PROBE_FUNCTION, base + 0xb8, 0);
    CASE_MP_U32(CIA402_MP_PROBE1_POS, base + 0xba, 0);
    CASE_MP_U32(CIA402_MP_PROBE1_NEG, base + 0xbb, 0);
    CASE_MP_U32(CIA402_MP_PROBE2_POS, base + 0xbc, 0);
    CASE_MP_U32(CIA402_MP_PROBE2_NEG, base + 0xbd, 0);

    // Handle options for all enableable bits.
    FOR_ALL_OPTS_DO(CASE_MP_ENABLE_BIT)

    case CIA402_MP_DIGITAL_IN_CHANNELS:
      opt->channel[channel]->digital_in_channels = p->value.u32;
      return 0;
    case CIA402_MP_DIGITAL_OUT_CHANNELS:
      opt->channel[channel]->digital_out_channels = p->value.u32;
      return 0;
    case CIA402_MP_VL_QUICKSTOP_RATIO:
      ratio = lcec_cia402_decode_ratio_modparam(p->value.str, 1 << 14);
      lcec_write_sdo32_modparam(slave, base + 0x4a, 1, ratio.numerator, p->name);
      lcec_write_sdo16_modparam(slave, base + 0x4a, 2, ratio.denominator, p->name);
      return 0;
    case CIA402_MP_VL_SET_POINT:
      ratio = lcec_cia402_decode_ratio_modparam(p->value.str, 1 << 14);
      lcec_write_sdo16_modparam(slave, base + 0x4b, 1, ratio.numerator, p->name);
      lcec_write_sdo16_modparam(slave, base + 0x4b, 2, ratio.denominator, p->name);
      return 0;
    case CIA402_MP_VL_DIMENSION_FACTOR:
      ratio = lcec_cia402_decode_ratio_modparam(p->value.str, 1 << 30);
      lcec_write_sdo32_modparam(slave, base + 0x4c, 1, ratio.numerator, p->name);
      lcec_write_sdo32_modparam(slave, base + 0x4c, 2, ratio.denominator, p->name);
      return 0;
    case CIA402_MP_POS_ENCODER_RATIO:
      ratio = lcec_cia402_decode_ratio_modparam(p->value.str, 1 << 30);
      lcec_write_sdo32_modparam(slave, base + 0x8f, 1, ratio.numerator, p->name);
      lcec_write_sdo32_modparam(slave, base + 0x8f, 2, ratio.denominator, p->name);
      return 0;
    case CIA402_MP_VEL_ENCODER_RATIO:
      ratio = lcec_cia402_decode_ratio_modparam(p->value.str, 1 << 30);
      lcec_write_sdo32_modparam(slave, base + 0x90, 1, ratio.numerator, p->name);
      lcec_write_sdo32_modparam(slave, base + 0x90, 2, ratio.denominator, p->name);
      return 0;
    case CIA402_MP_GEAR_RATIO:
      ratio = lcec_cia402_decode_ratio_modparam(p->value.str, 1 << 30);
      lcec_write_sdo32_modparam(slave, base + 0x91, 1, ratio.numerator, p->name);
      lcec_write_sdo32_modparam(slave, base + 0x91, 2, ratio.denominator, p->name);
      return 0;
    case CIA402_MP_FEED_RATIO:
      ratio = lcec_cia402_decode_ratio_modparam(p->value.str, 1 << 30);
      lcec_write_sdo32_modparam(slave, base + 0x92, 1, ratio.numerator, p->name);
      lcec_write_sdo32_modparam(slave, base + 0x92, 2, ratio.denominator, p->name);
      return 0;
    case CIA402_MP_EGEAR_RATIO:
      ratio = lcec_cia402_decode_ratio_modparam(p->value.str, 1 << 30);
      lcec_write_sdo32_modparam(slave, base + 0x93, 1, ratio.numerator, p->name);
      lcec_write_sdo32_modparam(slave, base + 0x93, 2, ratio.denominator, p->name);
      return 0;
    case CIA402_MP_VL_ACCEL:
      ratio = lcec_cia402_decode_ratio_modparam(p->value.str, 1 << 30);
      lcec_write_sdo32_modparam(slave, base + 0x48, 1, ratio.numerator, p->name);
      lcec_write_sdo16_modparam(slave, base + 0x48, 2, ratio.denominator, p->name);
      return 0;
    case CIA402_MP_VL_DECEL:
      ratio = lcec_cia402_decode_ratio_modparam(p->value.str, 1 << 30);
      lcec_write_sdo32_modparam(slave, base + 0x49, 1, ratio.numerator, p->name);
      lcec_write_sdo16_modparam(slave, base + 0x49, 2, ratio.denominator, p->name);
      return 0;
    default:
      return 1;
  }
}

/// @brief Handle a "ratio" modparam.
///
/// CiA 402 has a number of config settings that are effectively
/// ratios; they have a numerator object (usually int32) and a
/// denominator object (usually uint16), and the ratio of the two
/// control some setting, like encoder resolution, the mapping between
/// steps and distance, and so forth.
///
/// I'm feeling a bit surly here, and want to let the user set these in 2 different ways:
///
/// 1. Provide the numerator and denominator directly.  `<modParam
///    name="encoderResolution" value="4000/3"/>`
/// 2. Provide a floating point number, which we'll try to approximate
///    as best we can.  So `<modParam name="feedConstant"
///    value="241.72"/>` In cases where actual measurement is
///    involved, this is probably better, and I'd rather just write
///    the damn measurement into the config than spend time finding an
///    approximation by hand.
///
/// @param value A string that contains either a ratio of two
///   integers with a slash ("123/45") or colon ("123:45"), or a
///   floating point number ("6.789").
/// @param max_denominator The largest denominator allowed when
///   computing a rational approximation of a FP number.  Generally,
///   you'll want to pick the largest int representable by the
///   register that you're writing it into.
lcec_ratio lcec_cia402_decode_ratio_modparam(const char *value, unsigned int max_denominator) {
  lcec_ratio result;
  char *slash = strchr(value, '/');

  // If there's no "/", then try a colon.
  if (slash == NULL) slash = strchr(value, ':');

  if (slash != NULL) {
    // It's a ratio, so decode as-is.
    result.numerator = strtol(value, NULL, 10);
    result.denominator = strtoul(slash + 1, NULL, 10);

    if (result.numerator == 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Attempting to handle modParam ratio of \"%s\" produced a numerator of 0\n", value);
    }
    if (result.denominator == 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Attempting to handle modParam ratio of \"%s\" produced a denominator of 0\n", value);
    }

    return result;
  } else {
    // Probably a floating point value.  Let's extract it.
    double fpval = atof(value);

    int matrix[2][2];
    double x = fpval;
    int error;

    // Derived from https://ics.uci.edu/~eppstein/numth/frap.c
    matrix[0][0] = matrix[1][1] = 1;
    matrix[0][1] = matrix[1][0] = 0;

    error = x;
    while ((matrix[1][0] * error + matrix[1][1]) <= max_denominator) {
      int temp;

      temp = matrix[0][0] * error + matrix[0][1];
      matrix[0][1] = matrix[0][0];
      matrix[0][0] = temp;

      temp = matrix[1][0] * error + matrix[1][1];
      matrix[1][1] = matrix[1][0];
      matrix[1][0] = temp;
      if (x == (double)error) break;  // will divide by 0
      x = 1 / (x - error);
      if (x > (double)0x7fffffff) break;  // will overflow

      error = x;
    }

    result.numerator = matrix[0][0];
    result.denominator = matrix[1][0];

    return result;
  }
}
