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

// Constants for modparams.  The leadshine_stepper driver only has one:
#define M_CHANNELS     0
#define M_RXPDOLIMIT   1
#define M_TXPDOLIMIT   2
#define M_PDOINCREMENT 3

/// @brief Device-specific modparam settings available via XML.
static const lcec_modparam_desc_t modparams_perchannel[] = {
    // XXXX, add per-channel device-specific modparams here.
    {NULL},
};

static const lcec_modparam_desc_t modparams_base[] = {
    {"ciaChannels", M_CHANNELS, MODPARAM_TYPE_U32},
    {"ciaRxPDOEntryLimit", M_RXPDOLIMIT, MODPARAM_TYPE_U32},
    {"ciaTxPDOEntryLimit", M_TXPDOLIMIT, MODPARAM_TYPE_U32},
    // XXXX, add device-specific modparams here that aren't duplicated for multi-axis devices
    {NULL},
};

static const lcec_modparam_desc_t overrides1[] = {
    {"feedRatio", 0, 0, "10000", "Microsteps per rotation"},
    {"encoderRatio", 0, 0, "4000", "Encoder steps per rotation"},
    {NULL},
};

static const lcec_modparam_desc_t overrides2[] = {
    {"ch1feedRatio", 0, 0, "10000", "Microsteps per rotation"},
    {"ch2feedRatio", 0, 0, "10000", "Microsteps per rotation"},
    {"ch1encoderRatio", 0, 0, "4000", "Encoder steps per rotation"},
    {"ch2encoderRatio", 0, 0, "4000", "Encoder steps per rotation"},
    {NULL},
};

static int lcec_leadshine_stepper_init(int comp_id, lcec_slave_t *slave);

#define AXES(flags)  ((flags >> 60) & 0xf)
#define DIN(flags)   ((flags >> 56) & 0xf)
#define DOUT(flags)  ((flags >> 52) & 0xf)
#define F_AXES(axes) ((uint64_t)axes << 60)
#define F_DIN(din)   ((uint64_t)din << 56)
#define F_DOUT(dout) ((uint64_t)dout << 52)

static lcec_typelist_t types1[] = {
    // Single axis, closed loop
    {"CS3E-D503", 0x00004321, 0x1300, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(7) | F_DOUT(7)},
    {"CS3E-D507", 0x00004321, 0x1100, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(7) | F_DOUT(7)},
    {"CS3E-D1008", 0x00004321, 0x1200, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(7) | F_DOUT(7)},
    {"CS3E-D503E", 0x00004321, 0x700, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)},
    {"CS3E-D507E", 0x00004321, 0x500, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)},
    //{"CS3E-D503B", 0x00004321, ?, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)}, // On website, ID unknown
    //{"CS3E-D507B", 0x00004321, ?, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)}, // On website, ID unknown

    // Single axis, open loop
    {"EM3E-522E", 0x00004321, 0x8800, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)},
    {"EM3E-556E", 0x00004321, 0x8600, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)},
    {"EM3E-870E", 0x00004321, 0x8700, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)},

    //{"EM3E-522B", 0x00004321, ?, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)}, // On website, ID unknown
    //{"EM3E-556B", 0x00004321, ?, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)}, // On website, ID unknown
    //{"EM3E-870B", 0x00004321, ?, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)}, // On website, ID unknown
    //{"DM3C-EC882AC", 0x00004321, 0x00008a00, 0, NULL, lcec_leadshine_stepper_init, NULL, F_DIN(6) | F_DOUT(2)},  // Not on website

    {NULL},
};

static lcec_typelist_t types2[] = {
    // Dual axis, closed loop
    {"2CS3E-D503", 0x00004321, 0x00002200, 0, NULL, lcec_leadshine_stepper_init, NULL, F_AXES(2) | F_DIN(4) | F_DOUT(2)},
    {"2CS3E-D507", 0x00004321, 0x00002100, 0, NULL, lcec_leadshine_stepper_init, NULL, F_AXES(2) | F_DIN(4) | F_DOUT(2)},

    // Dual axis, open loop
    {"2EM3E-D522", 0x00004321, 0x0000a300, 0, NULL, lcec_leadshine_stepper_init, NULL, F_AXES(2) | F_DIN(4) | F_DOUT(2)},
    {"2EM3E-D556", 0x00004321, 0x0000a100, 0, NULL, lcec_leadshine_stepper_init, NULL, F_AXES(2) | F_DIN(4) | F_DOUT(2)},
    {"2EM3E-D870", 0x00004321, 0x0000a200, 0, NULL, lcec_leadshine_stepper_init, NULL, F_AXES(2) | F_DIN(4) | F_DOUT(2)},
    {NULL},
};

ADD_TYPES_WITH_CIA402_MODPARAMS(types1, modparams_perchannel, modparams_base, overrides1)
ADD_TYPES_WITH_CIA402_MODPARAMS(types2, modparams_perchannel, modparams_base, overrides2)

static void lcec_leadshine_stepper_read(lcec_slave_t *slave, long period);
static void lcec_leadshine_stepper_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
  // XXXX: Add pins and vars for PDO offsets here.
} lcec_leadshine_stepper_data_t;

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
      case M_RXPDOLIMIT:
        options->rxpdolimit = p->value.u32;
        break;
      case M_TXPDOLIMIT:
        options->txpdolimit = p->value.u32;
        break;
      case M_PDOINCREMENT:
        options->pdo_increment = p->value.u32;
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

static int lcec_leadshine_stepper_init(int comp_id, lcec_slave_t *slave) {
  lcec_leadshine_stepper_data_t *hal_data;
  int err;

  // alloc hal memory
  hal_data = LCEC_HAL_ALLOCATE(lcec_leadshine_stepper_data_t);
  slave->hal_data = hal_data;

  // initialize read/write
  slave->proc_read = lcec_leadshine_stepper_read;
  slave->proc_write = lcec_leadshine_stepper_write;

  lcec_class_cia402_options_t *options = lcec_cia402_options();
  // XXXX: set which options this device supports.  This controls
  // which pins are registered and which PDOs are mapped.  See
  // lcec_class_cia402.h for the full list of what is currently
  // available, and instructions on how to add additional CiA 402
  // features.
  options->channels = AXES(slave->flags);
  options->rxpdolimit = 8;  // See https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/343
  options->txpdolimit = 8;  // See https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/343

  for (int channel = 0; channel < options->channels; channel++) {
    options->channel[channel]->enable_csp = 1;
    options->channel[channel]->enable_digital_input = 1;
    options->channel[channel]->enable_digital_output = 1;
    options->channel[channel]->digital_in_channels = DIN(slave->flags);
    options->channel[channel]->digital_out_channels = DOUT(slave->flags);
  }

  // Apply default Distributed Clock settings if it's not already set.
  if (slave->dc_conf == NULL) {
    lcec_slave_dc_t *dc = LCEC_HAL_ALLOCATE(lcec_slave_dc_t);
    if (options->channels == 2) {
      dc->assignActivate = 0x700;  // 2-channel devices are all 0x700 according to LS's ESI.
      dc->sync1Cycle = slave->master->app_time_period;
    } else {
      dc->assignActivate = 0x300;  // 1-channel devices are all 0x300 according to LS's ESI.
    }

    dc->sync0Cycle = slave->master->app_time_period;

    slave->dc_conf = dc;
  }

  // Handle modparams
  if (handle_modparams(slave, options) != 0) {
    return -EIO;
  }

  if (options->channels > 1) {
    lcec_cia402_rename_multiaxis_channels(options);
  }

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

static void lcec_leadshine_stepper_read(lcec_slave_t *slave, long period) {
  lcec_leadshine_stepper_data_t *hal_data = (lcec_leadshine_stepper_data_t *)slave->hal_data;

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

static void lcec_leadshine_stepper_write(lcec_slave_t *slave, long period) {
  lcec_leadshine_stepper_data_t *hal_data = (lcec_leadshine_stepper_data_t *)slave->hal_data;

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
