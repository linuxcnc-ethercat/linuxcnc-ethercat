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
/// @brief Driver for Omron MX2 VFD w/ an 3G3AX-MX2-ECT EtherCAT module installed.

#include "../lcec.h"
#include "lcec_class_cia402.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

// TODO:
//
// - Add digital inputs (0x3010:28
// - Add digital outputs (0x3010:29) (read only -- function set via C021 on front panel)
// - Add actual torgue (0x3010:2e?)
// - Add frequency (0x4010:2c)
// - Add output voltage monitor (0x3010:32)
// - Add input power monitor (0x3010:33)
// - Add temperature monitor (0x3010:3a)
// - Add DC voltage monitor (0x3010:47)
// - Add Regenerative breaking monitor (0x3010:48)
// - Add Thermal load rate monitor (0x3010:49)
// - Add output current monitor (0x3010:24)
//
// *Generally*, we're going to target using the front panel to handle
// *setup tasks, and only use EtherCAT for controlling speed, etc.  So
// *we may not end up with any modParams at all here.

/// @brief Device-specific modparam settings available via XML.
static const lcec_modparam_desc_t modparams_perchannel[] = {
    {NULL},
};

static const lcec_modparam_desc_t modparams_base[] = {
    {NULL},
};

static const lcec_modparam_doc_t chan_docs[] = {
    // XXXX, add documentation for per-channel settings here
    {NULL},
};

static const lcec_modparam_doc_t base_docs[] = {
    // XXXX, add documentation for device-specific settings here
    {NULL},
};

static int lcec_ommx2_init(int comp_id, lcec_slave_t *slave);

static lcec_typelist_t types[] = {
    {.name = "OMMX2", .vid = LCEC_OMRON_VID, .pid = 0x53, .proc_init = lcec_ommx2_init},
    {NULL},
};
ADD_TYPES_WITH_CIA402_MODPARAMS(types, 1, modparams_perchannel, modparams_base, chan_docs, base_docs)

static void lcec_ommx2_read(lcec_slave_t *slave, long period);
static void lcec_ommx2_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
  // XXXX: Add pins and vars for PDO offsets here.
} lcec_ommx2_data_t;

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

static int lcec_ommx2_init(int comp_id, lcec_slave_t *slave) {
  lcec_ommx2_data_t *hal_data;
  int err;

  // XXXX: you can remove this if you replace the /* fake vid */ and /* fake pid */ in `types`, above.
  if (slave->vid == 0xffffffff || slave->pid == 0xffffffff) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "ommx2 device slave %s.%s not configured correctly, you must specify vid and pid in the XML file.\n",
        slave->master->name, slave->name);
    return -EIO;
  }

  // alloc hal memory
  hal_data = LCEC_HAL_ALLOCATE(lcec_ommx2_data_t);
  slave->hal_data = hal_data;

  // initialize read/write
  slave->proc_read = lcec_ommx2_read;
  slave->proc_write = lcec_ommx2_write;

  lcec_class_cia402_options_t *options = lcec_cia402_options();
  // XXXX: set which options this device supports.  This controls
  // which pins are registered and which PDOs are mapped.  See
  // lcec_class_cia402.h for the full list of what is currently
  // available, and instructions on how to add additional CiA 402
  // features.

  options->channels = 1;
  options->rxpdolimit = 12;  // See https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/343
  options->txpdolimit = 12;  // See https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/343
  options->pdo_autoflow = 1;
  options->pdo_entry_limit = 2;
  options->pdo_limit = 64;  // The hardware supports it, but I don't know that we're good with more than 5 or so.
  options->pdo_increment = 1;

  for (int channel = 0; channel < options->channels; channel++) {
    options->channel[channel]->enable_vl = 1;
    options->channel[channel]->enable_error_code = 1;
    options->channel[channel]->enable_digital_input = 0;  // Seems to have the hardware, but it doesn't list CiA 402-compatible PDOs.
    options->channel[channel]->enable_digital_output = 0;
  }

  // Handle modparams
  if (handle_modparams(slave, options) != 0) {
    return -EIO;
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
  lcec_cia402_add_output_sync(slave, syncs, options);

  // XXXX: ff this driver needed to set up device-specific output PDO
  // entries, then the next 2 lines should be used.  You should be
  // able to duplicate the `lcec_syncs_add_pdo_entry()` line as many
  // times as needed, up the point where your hardware runs out of
  // available PDOs.
  //
  // lcec_syncs_add_pdo_info(slave, syncs, 0x1602);
  // lcec_syncs_add_pdo_entry(slave, syncs, 0x200e, 0x00, 16);

  lcec_cia402_add_input_sync(slave, syncs, options);
  // XXXX: Similarly, uncomment these for input PDOs:
  //
  // lcec_syncs_add_pdo_info(slave, syncs, 0x1a02);
  // lcec_syncs_add_pdo_entry(slave, syncs, 0x2048, 0x00, 16);  // current voltage

  slave->sync_info = &syncs->syncs[0];

  hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);

  for (int channel = 0; channel < options->channels; channel++) {
    hal_data->cia402->channels[channel] = lcec_cia402_register_channel(slave, 0x6000 + 0x800 * channel, options->channel[channel]);
  }

  // XXXX: register device-specific PDOs.
  // If you need device-specific PDO entries registered, then do that here.
  //
  // lcec_pdo_init(slave,  0x200e, 0, &hal_data->alarm_code_os, NULL);

  // export device-specific pins.  This shouldn't need edited, just edit `slave_pins` above.
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, slave->master->name, slave->name)) != 0) {
    return err;
  }

  return 0;
}

static void lcec_ommx2_read(lcec_slave_t *slave, long period) {
  lcec_ommx2_data_t *hal_data = (lcec_ommx2_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  // XXXX: If you need to read device-specific PDOs and set pins, then you should do this here.
  //
  // uint8_t *pd = slave->master->process_data;
  // LCEC_PIN_U32_SET(hal_data->alarm_code, EC_READ_U16(&pd[hal_data->alarm_code_os]));

  lcec_cia402_read_all(slave, hal_data->cia402);
  // XXXX: If you want digital in pins, then uncomment this:
  //  lcec_din_read_all(slave, hal_data->din);
}

static void lcec_ommx2_write(lcec_slave_t *slave, long period) {
  lcec_ommx2_data_t *hal_data = (lcec_ommx2_data_t *)slave->hal_data;

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
