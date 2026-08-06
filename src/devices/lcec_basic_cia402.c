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

// XXXX: Change this description of the driver
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

// Constants for modparams.  These should have their last hex digit be
// a 0, to make handling per-channel modparams easier.  Keep these
// under 0x1000 to avoid collisions with params from class cia402.
//
// XXXX: You can probaably remove most if not ell of these, if you
// know how many channels, etc your device has and just hard-code
// their values in the _init function.
#define M_CHANNELS      0x00
#define M_RXPDOLIMIT    0x10
#define M_TXPDOLIMIT    0x20
#define M_PDOINCREMENT  0x30
#define M_PDOAUTOFLOW   0x40
#define M_PDOLIMIT      0x50
#define M_PDOENTRYLIMIT 0x60

/// @brief Device-specific modparam settings available via XML.
static const lcec_modparam_desc_t modparams_perchannel[] = {
    // XXXX, add per-channel device-specific modparams here.
    {NULL},
};

static const lcec_modparam_desc_t modparams_base[] = {
    {"ciaChannels", M_CHANNELS, MODPARAM_TYPE_U32},
    {"ciaRxPDOEntryLimit", M_RXPDOLIMIT, MODPARAM_TYPE_U32},
    {"ciaTxPDOEntryLimit", M_TXPDOLIMIT, MODPARAM_TYPE_U32},
    {"pdoIncrement", M_PDOINCREMENT, MODPARAM_TYPE_U32},
    {"pdoAutoflow", M_PDOAUTOFLOW, MODPARAM_TYPE_BIT},
    {"pdoLimit", M_PDOLIMIT, MODPARAM_TYPE_U32},
    {"pdoEntryLimit", M_PDOENTRYLIMIT, MODPARAM_TYPE_U32},
    // XXXX, add device-specific modparams here that aren't duplicated for multi-axis devices
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

static int lcec_basic_cia402_init(int comp_id, lcec_slave_t *slave);

// XXXX: macros like these are helpful if you're planning on
// supporting devices with varying numbers of axes and I/O ports in
// your device.  See lcec_leadshine_stepper.c and lece_rtec.c for
// examples of use.
//
//#define AXES(flags)  ((flags >> 60) & 0xf)
//#define DIN(flags) ((flags >> 56) & 0xf)
//#define DOUT(flags) ((flags >> 52) & 0xf)
//#define F_AXES(axes) ((uint64_t)axes << 60)
//#define F_DIN(din) ((uint64_t)din<<56)
//#define F_DOUT(dout) ((uint64_t)dout<<52)

// XXXX: remove `basic_cia402` and replace it with your device name,
// then change the next two parameters to match your device's VID and
// PID.  Feel free to add multiple devices here if they can share the
// same driver.
static lcec_typelist_t types[] = {
    {"basic_cia402", /* fake vid */ 0xffffffff, /* fake pid */ 0xffffffff, 0, NULL, lcec_basic_cia402_init,
        /* modparams implicitly added below */},
    {NULL},
};
ADD_TYPES_WITH_CIA402_MODPARAMS(types, CIA402_MAX_CHANNELS, modparams_perchannel, modparams_base, chan_docs, base_docs)

static void lcec_basic_cia402_read(lcec_slave_t *slave, long period);
static void lcec_basic_cia402_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
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
    // int channel = p->id & 7;
    int id = p->id & ~7;
    // int base = 0x2000 + 0x800 * channel;

    switch (id) {
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
      case M_PDOAUTOFLOW:
        options->pdo_autoflow = p->value.bit;
        break;
      case M_PDOLIMIT:
        options->pdo_limit = p->value.u32;
        break;
      case M_PDOENTRYLIMIT:
        options->pdo_entry_limit = p->value.u32;
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
  lcec_basic_cia402_data_t *hal_data;
  int err;

  // XXXX: you can remove this if you replace the /* fake vid */ and /* fake pid */ in `types`, above.
  if (slave->vid == 0xffffffff || slave->pid == 0xffffffff) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "basic_cia402 device slave %s.%s not configured correctly, you must specify vid and pid in the XML file.\n",
        slave->master->name, slave->name);
    return -EIO;
  }

  // alloc hal memory
  hal_data = LCEC_HAL_ALLOCATE(lcec_basic_cia402_data_t);
  slave->hal_data = hal_data;

  // initialize read/write
  slave->proc_read = lcec_basic_cia402_read;
  slave->proc_write = lcec_basic_cia402_write;

  // XXXX: we should generally (always?) run CiA 402 devices in
  // distributed-clock mode.  Consider turning this on by default if
  // we know the correct values for all of the fields.
  //
  // Apply default Distributed Clock settings if it's not already set.
  //  if (slave->dc_conf == NULL) {
  //    lcec_slave_dc_t *dc = LCEC_HAL_ALLOCATE(lcec_slave_dc_t);
  //    dc->assignActivate = 0x300;  // See ESI and/or manufacturer docs
  //    dc->sync0Cycle = slave->master->app_time_period;
  //
  //    slave->dc_conf = dc;
  //  }

  lcec_class_cia402_options_t *options = lcec_cia402_options();
  // XXXX: set which options this device supports.  This controls
  // which pins are registered and which PDOs are mapped.  See
  // lcec_class_cia402.h for the full list of what is currently
  // available, and instructions on how to add additional CiA 402
  // features.

  options->channels = 1;
  options->rxpdolimit = 8;  // See https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/343
  options->txpdolimit = 8;  // See https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/343

  for (int channel = 0; channel < options->channels; channel++) {
    options->channel[channel]->enable_pv = 1;
    options->channel[channel]->enable_pp = 0;
    options->channel[channel]->enable_csv = 0;
    options->channel[channel]->enable_csp = 0;  // Should generally be enabled for servos and steppers
    options->channel[channel]->enable_actual_torque = 0;
    options->channel[channel]->enable_digital_input = 0;
    options->channel[channel]->enable_digital_output = 0;
  }

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

static void lcec_basic_cia402_read(lcec_slave_t *slave, long period) {
  lcec_basic_cia402_data_t *hal_data = (lcec_basic_cia402_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  // XXXX: If you need to read device-specific PDOs and set pins, then you should do this here.
  //
  // uint8_t *pd = slave->master->process_data;
  // LCEC_PIN_U32_SET(hal_data->alarm_code, EC_READ_U16(&pd[hal_data->alarm_code_os]));

  lcec_cia402_read_all(slave, hal_data->cia402);
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
}
