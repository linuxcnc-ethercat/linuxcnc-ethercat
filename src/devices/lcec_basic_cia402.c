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
/// @brief Driver for "Basic" CiA 402 devices
///
/// This driver has two purposes:
///
/// 1. It acts as an example of a simple CiA 402 driver, to be used as
///    a base for creating device-specific CiA 402 drivers.
/// 2. It can be used as-is with some devices, instead of using the
///    `generic` XML config in LinuxCNC-Ethercat.
///
/// These two purposes can conflict with each other; when this
/// happens, we'll generally favor the first purpose and try to keep
/// this simple.

#include "../lcec.h"
#include "lcec_class_cia402.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

// Constants for modparams.  The basic_cia402 driver only has one:
#define M_CHANNELS 0

/// @brief Device-specific modparam settings available via XML.
static const lcec_modparam_desc_t modparams_lcec_basic_cia402[] = {
    {"ciaChannels", M_CHANNELS, MODPARAM_TYPE_U32},
    // XXXX, add device-specific modparams here.
    {NULL},
};

static int lcec_basic_cia402_init(int comp_id, lcec_slave_t *slave);

// When extending this, you will need to modify the number of PDOs
// registered (the `8` here).  For now, the easiest way to do this is
// to set it to a fairly large number, compile the driver, and attempt
// to use it.  It will fail with an error, but the error message will
// give the correct value.
//
// In the future, we'll probably remove the number entirely and handle
// it dynamically.  Then this comment can go away.
//
// XXXX: remove `basic_cia402` and replace it with your device name,
// then change the next two parameters to match your device's VID and
// PID.  Feel free to add multiple devices here if they can share the
// same driver.
static lcec_typelist_t types[] = {
    {"basic_cia402", /* fake vid */ -1, /* fake pid */ -1, 0, NULL, lcec_basic_cia402_init, /* modparams implicitly added below */},
    {NULL},
};
ADD_TYPES_WITH_CIA402_MODPARAMS(types, modparams_lcec_basic_cia402)

static void lcec_basic_cia402_read(lcec_slave_t *slave, long period);
static void lcec_basic_cia402_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
  lcec_class_din_channels_t *din;
  lcec_class_dout_channels_t *dout;
  // XXXX: Add pins and vars for PDO offsets here.
} lcec_basic_cia402_data_t;

static const lcec_pindesc_t slave_pins[] = {
    // XXXX: add device-specific pins here.
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static int handle_modparams(lcec_slave_t *slave, lcec_class_cia402_options_t *options) {
  lcec_master_t *master = slave->master;
  lcec_slave_modparam_t *p;
  int v;

  for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
    switch (p->id) {
        // XXXX: add device-specific modparam handlers here.
      case M_CHANNELS:
        options->channels = p->value.u32;
        break;
      default:
        // Handle cia402 generic modparams
        v = lcec_cia402_handle_modparam(slave, p, options);

        // If an error occured, then return the error.
        if (v < 0) {
          return v;
        }

        // if nothing handled this modparam, then something's wrong.  Return an error:
        if (v > 0) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "unknown modparam %s for slave %s.%s\n", p->name, master->name, slave->name);
          return -1;
        }
        break;
    }
  }

  return 0;
}

static int lcec_basic_cia402_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_basic_cia402_data_t *hal_data;
  int err;

  // XXXX: you can remove this if you replace the /* fake vid */ and /* fake pid */ in `types`, above.
  if (slave->vid == -1 || slave->pid == -1) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "basic_cia402 device slave %s.%s not configured correctly, you must specify vid and pid in the XML file.\n",
        slave->master->name, slave->name);
    return -EIO;
  }

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_basic_cia402_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", slave->master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_basic_cia402_data_t));
  slave->hal_data = hal_data;

  // initialize callbacks
  slave->proc_read = lcec_basic_cia402_read;
  slave->proc_write = lcec_basic_cia402_write;

  lcec_class_cia402_options_t *options = lcec_cia402_options();
  // XXXX: set which options this device supports.  This controls
  // which pins are registered and which PDOs are mapped.  See
  // lcec_class_cia402.h for the full list of what is currently
  // available, and instructions on how to add additional CiA 402
  // features.
  options->channels = 1;
  options->channel[0]->enable_pv = 1;
  options->channel[0]->enable_pp = 1;
  options->channel[0]->enable_csv = 0;
  options->channel[0]->enable_csp = 0;
  options->channel[0]->enable_actual_torque = 0;
  options->channel[0]->enable_digital_input = 0;
  options->channel[0]->enable_digital_output = 0;

  // Handle modparams
  if (handle_modparams(slave, options) != 0) {
    return -EIO;
  }

  if (options->channels > 1) {
    lcec_cia402_rename_multiaxis_channels(options);
  }

  // XXXX: set up syncs.  This is generally needed because CiA 402
  // covers a lot of area and few (if any) devices have all of the
  // useful pins pre-mapped.  If you try to use a PDO that hasn't been
  // mapped, then you will get a runtime error about PDOs not being
  // mapped, and you'll want to come back here and fix it.
  //
  // These need to be done in the correct order, as the `lcec_syncs*`
  // code only adds new entries at the end.
  lcec_syncs_t *syncs = lcec_cia402_init_sync(slave, options);
  lcec_cia402_add_output_sync(syncs, options);

  // XXXX: ff this driver needed to set up device-specific output PDO
  // entries, then the next 2 lines should be used.  You should be
  // able to duplicate the `lcec_syncs_add_pdo_entry()` line as many
  // times as needed.
  //
  // lcec_syncs_add_pdo_info(syncs, 0x1602);
  // lcec_syncs_add_pdo_entry(syncs, 0x200e, 0x00, 16);

  lcec_cia402_add_input_sync(syncs, options);
  // XXXX: Similarly, uncomment these for input PDOs:
  //
  // lcec_syncs_add_pdo_info(syncs, 0x1a02);
  // lcec_syncs_add_pdo_entry(syncs, 0x2048, 0x00, 16);  // current voltage

  slave->sync_info = &syncs->syncs[0];

  hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);
  if (hal_data->cia402 == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_cia402_allocate_channels() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }

  for (int channel = 0; channel < options->channels; channel++) {
    hal_data->cia402->channels[channel] = lcec_cia402_register_channel(slave, 0x6000 + 0x800 * channel, options->channel[channel]);
  }

  // XXXX: The CiA 402 spec specifies how to use digital in and out ports,
  // but there doesn't seem to be a way to figure out how many ports
  // of each are available.  We *could* just map all possible ports
  // here, but instead we're going to leave them out entirely for
  // basic_cia402 devices.
  //
  // hal_data->din = lcec_din_allocate_channels(9);
  // if (hal_data->din == NULL) {
  //  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_din_allocate_channels() for slave %s.%s failed\n", master->name, slave->name);
  //  return -EIO;
  //}
  //
  // hal_data->dout = lcec_dout_allocate_channels(3);
  // if (hal_data->din == NULL) {
  //   rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_din_allocate_channels() for slave %s.%s failed\n", master->name, slave->name);
  //   return -EIO;
  // }

  // XXXX: register device-specific PDOs.
  // If you need device-specific PDO entries registered, then do that here.
  //
  // lcec_pdo_init(slave,  0x200e, 0, &hal_data->alarm_code_os, NULL);

  // XXXX: If you want din/dout ports, then uncomment these.
  //
  // hal_data->din->channels[0] =
  //    lcec_din_register_channel_packed(slave, 0x60fd, 0, 0, "din-cw-limit");  // negative limit switch
  // hal_data->din->channels[1] =
  //    lcec_din_register_channel_packed(slave, 0x60fd, 0, 1, "din-ccw-limit");                      // positive limit
  //    switch
  // hal_data->din->channels[2] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 2, "din-home");  // home
  // hal_data->din->channels[2] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 3, "din-interlock");  // interlock?
  // hal_data->din->channels[3] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 16, "din-1");
  // hal_data->din->channels[4] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 17, "din-2");
  // hal_data->din->channels[5] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 18, "din-3");
  // hal_data->din->channels[6] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 19, "din-4");
  // hal_data->din->channels[7] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 20, "din-5");
  // hal_data->din->channels[8] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 21, "din-6");
  //
  // Register dout ports
  // hal_data->dout->channels[0] = lcec_dout_register_channel_packed(slave, 0x60fe, 1, 16, "dout-brake");
  // hal_data->dout->channels[1] = lcec_dout_register_channel_packed(slave, 0x60fe, 1, 16, "dout-1");
  // hal_data->dout->channels[2] = lcec_dout_register_channel_packed(slave, 0x60fe, 1, 17, "dout-2");

  // export device-specific pins.  This shouldn't need edited, just edit `slave_pins` above.
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, slave->master->name, slave->name)) != 0) {
    return err;
  }

  return 0;
}

static void lcec_basic_cia402_read(lcec_slave_t *slave, long period) {
  lcec_basic_cia402_data_t *hal_data = (lcec_basic_cia402_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  // XXXX: If you need to read device-specific PDOs and set pins, then you should do this here.
  //
  // uint8_t *pd = slave->master->process_data;
  // *(hal_data->alarm_code) = EC_READ_U16(&pd[hal_data->alarm_code_os]);

  lcec_cia402_read_all(slave, hal_data->cia402);
  // XXXX: If you want digital in pins, then uncomment this:
  //  lcec_din_read_all(slave, hal_data->din);
}

static void lcec_basic_cia402_write(lcec_slave_t *slave, long period) {
  lcec_basic_cia402_data_t *hal_data = (lcec_basic_cia402_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  // XXXX: similarly, if you need to write device-specific PDOs from
  // pins, then do that here.

  lcec_cia402_write_all(slave, hal_data->cia402);
  // XXXX: uncomment for digital out pins:
  //  lcec_dout_write_all(slave, hal_data->dout);
}
