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
/// @brief Driver for Leadshine EtherCAT stepper drives

// TODO:
//
// Add additional device-specific pins and modParams, as needed to fully
// support reasonable use cases.
//
// - Input settings? (0x2152:0[1-4]?)  Needs restart.  FYI, 0x2155 shows actual values, 0x60fd shows functions.
//   - "probe1" -> 0x17          (.26)
//   - "probe2" -> 0x18          (.27)
//   - "home" -> 0x16            (.2)
//   - "positive-limit" -> 0x01  (.1)
//   - "negative-limit" -> 0x02  (.0)
//   - "quick-stop" -> 0x14      (.23)
//   - "gpio" -> 0x19  (Makes inputs show up as 60fd:0.[4-7]?  Seems wrong, should be .16+.  .4-.15 are reserved in spec.
// - Leadshine error code (0x3fff:1)
// - Virtual homing input (0x225c:00.2=1 + 0x5012:03)?  Allows index Z use for homing, etc.

#include <stdio.h>

#include "../lcec.h"
#include "lcec_class_cia402.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

// Constants for modparams.  The leadshine_stepper driver only has one:
#define M_PEAKCURRENT100MA 0x100
#define M_PEAKCURRENT1MA   0x110
#define M_CONTROLMODE      0x120

/// @brief Device-specific modparam settings available via XML.
static const lcec_modparam_desc_t modparams_perchannel_closed[] = {
    {"peakCurrent_amps", M_PEAKCURRENT100MA, MODPARAM_TYPE_FLOAT, "6.0", "Peak motor current, in Amps"},
    {"controlMode", M_CONTROLMODE, MODPARAM_TYPE_STRING, "closedloop", "Operation mode: openloop or closedloop"},
    {NULL},
};

static const lcec_modparam_desc_t modparams_perchannel_open[] = {
    {"peakCurrent_amps", M_PEAKCURRENT1MA, MODPARAM_TYPE_FLOAT, "5.6", "Peak motor current, in Amps"},
    {NULL},
};

static const lcec_modparam_desc_t modparams_base[] = {
    // XXXX, add device-specific modparams here that aren't duplicated for multi-axis devices
    {NULL},
};

static const lcec_modparam_doc_t docs[] = {
    {"feedRatio", "10000", "Microsteps per rotation"},
    {"encoderRatio", "4000", "Encoder steps per rotation"},
    {NULL},
};

static const lcec_lookuptable_int_t controlmode[] = {
    {"openloop", 0},
    {"closedloop", 2},
    {NULL},
};

static int lcec_leadshine_stepper_init(int comp_id, lcec_slave_t *slave);

#define AXES(flags)  ((flags >> 60) & 0xf)
#define DIN(flags)   ((flags >> 56) & 0xf)
#define DOUT(flags)  ((flags >> 52) & 0xf)
#define F_AXES(axes) ((uint64_t)axes << 60)
#define F_DIN(din)   ((uint64_t)din << 56)
#define F_DOUT(dout) ((uint64_t)dout << 52)

#define TYPEDEFAULT .vid = 0x4321, .proc_init = lcec_leadshine_stepper_init
static lcec_typelist_t types1open[] = {
    // Single axis, open loop
    {.name = "EM3E-522E", TYPEDEFAULT, .pid = 0x8800, .flags = F_DIN(6) | F_DOUT(2)},
    {.name = "EM3E-556E", TYPEDEFAULT, .pid = 0x8600, .flags = F_DIN(6) | F_DOUT(2)},
    {.name = "EM3E-870E", TYPEDEFAULT, .pid = 0x8700, .flags = F_DIN(6) | F_DOUT(2)},

    //{.name="EM3E-522B", TYPEDEFAULT, .pid=?, .flags=F_DIN(6) | F_DOUT(2)}, // On website, ID unknown
    //{.name="EM3E-556B", TYPEDEFAULT, .pid=?, .flags=F_DIN(6) | F_DOUT(2)}, // On website, ID unknown
    //{.name="EM3E-870B", TYPEDEFAULT, .pid=?, .flags=F_DIN(6) | F_DOUT(2)}, // On website, ID unknown
    //{.name="DM3C-EC882AC", TYPEDEFAULT, .pid=0xa00,  flags=F_DIN(6) | F_DOUT(2)},  // Not on website

    {NULL},
};

static lcec_typelist_t types1closed[] = {
    // Single axis, closed loop
    {.name = "CS3E-D503", TYPEDEFAULT, .pid = 0x1300, .flags = F_DIN(7) | F_DOUT(7)},
    {.name = "CS3E-D507", TYPEDEFAULT, .pid = 0x1100, .flags = F_DIN(7) | F_DOUT(7)},
    {.name = "CS3E-D1008", TYPEDEFAULT, .pid = 0x1200, .flags = F_DIN(7) | F_DOUT(7)},
    {.name = "CS3E-D503E", TYPEDEFAULT, .pid = 0x700, .flags = F_DIN(6) | F_DOUT(2)},
    {.name = "CS3E-D507E", TYPEDEFAULT, .pid = 0x500, .flags = F_DIN(6) | F_DOUT(2)},
    //{.name="CS3E-D503B", TYPEDEFAULT, .pid=?, .flags=F_DIN(6) | F_DOUT(2)}, // On website, ID unknown
    //{.name="CS3E-D507B", TYPEDEFAULT, .pid=?, .flags=F_DIN(6) | F_DOUT(2)}, // On website, ID unknown

    {NULL},
};

static lcec_typelist_t types2open[] = {
    // Dual axis, open loop
    {.name = "2EM3E-D522", TYPEDEFAULT, .pid = 0xa300, .flags = F_AXES(2) | F_DIN(4) | F_DOUT(2)},
    {.name = "2EM3E-D556", TYPEDEFAULT, .pid = 0xa100, .flags = F_AXES(2) | F_DIN(4) | F_DOUT(2)},
    {.name = "2EM3E-D870", TYPEDEFAULT, .pid = 0xa200, .flags = F_AXES(2) | F_DIN(4) | F_DOUT(2)},
    {NULL},
};

static lcec_typelist_t types2closed[] = {
    // Dual axis, closed loop
    {.name = "2CS3E-D503", TYPEDEFAULT, .pid = 0x2200, .flags = F_AXES(2) | F_DIN(4) | F_DOUT(2)},
    {.name = "2CS3E-D507", TYPEDEFAULT, .pid = 0x2100, .flags = F_AXES(2) | F_DIN(4) | F_DOUT(2)},
    {NULL},
};

ADD_TYPES_WITH_CIA402_MODPARAMS(types1open, 1, modparams_perchannel_open, modparams_base, docs, NULL)
ADD_TYPES_WITH_CIA402_MODPARAMS(types1closed, 1, modparams_perchannel_closed, modparams_base, docs, NULL)
ADD_TYPES_WITH_CIA402_MODPARAMS(types2open, 2, modparams_perchannel_open, modparams_base, docs, NULL)
ADD_TYPES_WITH_CIA402_MODPARAMS(types2closed, 2, modparams_perchannel_closed, modparams_base, docs, NULL)

static void lcec_leadshine_stepper_read(lcec_slave_t *slave, long period);
static void lcec_leadshine_stepper_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
  lcec_class_din_channels_t *din;
  lcec_class_dout_channels_t *dout;
} lcec_leadshine_stepper_data_t;

static const lcec_pindesc_t slave_pins[] = {
    // XXXX: add device-specific pins here.
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static int handle_modparams(lcec_slave_t *slave, lcec_class_cia402_options_t *options) {
  lcec_master_t *master = slave->master;
  lcec_slave_modparam_t *p;
  int v, val;
  uint32_t uval;

  for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
    int channel = p->id & 7;
    int id = p->id & ~7;
    int base = 0x2000 + 0x800 * channel;

    switch (id) {
      case M_PEAKCURRENT100MA:
        // Leadshine's closed-loop steppers want peak current set in units of 100 mA.
        uval = p->value.flt * 10.0 + 0.5;
        if (lcec_write_sdo16_modparam(slave, base + 0, 0, uval, p->name) < 0) return -1;
        break;
      case M_PEAKCURRENT1MA:
        // Leadshine's open-loop steppers want peak current set in units of 1 mA.
        uval = p->value.flt * 1000.0 + 0.5;
        if (lcec_write_sdo16_modparam(slave, base + 0, 0, uval, p->name) < 0) return -1;
        break;
      case M_CONTROLMODE:
        val = lcec_lookupint_i(controlmode, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"controlMode\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, base + 0x24, 0, val, p->name) < 0) return -1;
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
    options->channel[channel]->digital_in_channels = DIN(slave->flags);
    options->channel[channel]->digital_out_channels = DOUT(slave->flags);
    options->channel[channel]->enable_digital_output = 1;

    // Leadshine doesn't follow the spec, so we need to do this ourselves
    options->channel[channel]->enable_digital_input = 0;
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

  for (int channel = 0; channel < options->channels; channel++) {
    int base_idx = 0x6000 + 0x800 * channel;
    lcec_syncs_add_pdo_entry(syncs, base_idx + 0xfd, 0x00, 32);  // Digital input
  }

  slave->sync_info = &syncs->syncs[0];

  hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);

  for (int channel = 0; channel < options->channels; channel++) {
    hal_data->cia402->channels[channel] = lcec_cia402_register_channel(slave, 0x6000 + 0x800 * channel, options->channel[channel]);
  }

  // Set up digital in/out.  Leadshine puts these at the wrong place,
  // so we need to do this ourselves instead of relying on the base
  // CiA code.
  int i = 0;
  hal_data->din = lcec_din_allocate_channels(64);  //(options->channel[0]->digital_in_channels + 8)*options->channels);
  for (int axis = 0; axis < options->channels; axis++) {
    char *dname;
    const char *name_prefix = options->channel[axis]->name_prefix;
    int base_idx = 0x6000 + 0x800 * axis;

    dname = LCEC_HAL_ALLOCATE_STRING(30);
    snprintf(dname, 30, "%s-din-negative-limit", name_prefix);
    hal_data->din->channels[i++] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 0, dname);  // negative limit switch

    dname = LCEC_HAL_ALLOCATE_STRING(30);
    snprintf(dname, 30, "%s-din-positive-limit", name_prefix);
    hal_data->din->channels[i++] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 1, dname);  // positive limit switch

    dname = LCEC_HAL_ALLOCATE_STRING(30);
    snprintf(dname, 30, "%s-din-home", name_prefix);
    hal_data->din->channels[i++] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 2, dname);  // home

    dname = LCEC_HAL_ALLOCATE_STRING(30);
    snprintf(dname, 30, "%s-din-probe1", name_prefix);
    hal_data->din->channels[i++] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 26, dname);  // home

    dname = LCEC_HAL_ALLOCATE_STRING(30);
    snprintf(dname, 30, "%s-din-probe2", name_prefix);
    hal_data->din->channels[i++] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 27, dname);  // home

    dname = LCEC_HAL_ALLOCATE_STRING(30);
    snprintf(dname, 30, "%s-din-index-z", name_prefix);
    hal_data->din->channels[i++] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 31, dname);  // home

    dname = LCEC_HAL_ALLOCATE_STRING(30);
    snprintf(dname, 30, "%s-din-quick-stop", name_prefix);
    hal_data->din->channels[i++] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 23, dname);  // home

    for (int channel = 0; channel < options->channel[axis]->digital_in_channels; channel++) {
      dname = LCEC_HAL_ALLOCATE_STRING(30);
      snprintf(dname, 30, "%s-din-%d", name_prefix, channel + 1);
      hal_data->din->channels[i++] = lcec_din_register_channel_packed(slave, base_idx + 0xfd, 0, 4 + channel, dname);
    }
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
  // LCEC_PIN_U32_SET(hal_data->alarm_code, EC_READ_U16(&pd[hal_data->alarm_code_os]));

  lcec_cia402_read_all(slave, hal_data->cia402);
  lcec_din_read_all(slave, hal_data->din);
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
}
