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
/// @brief Driver for RTelligent Ethercat Stepper Drives
///
/// See documentation at http://www.rtelligent.net/upload/wenjian/Stepper/ECT%20Series%20User%20Manual.pdf

#include "../lcec.h"
#include "lcec_class_cia402.h"

#define M_MICROSTEPS 0x0
#define M_MOTOR_RESOLUTION   0x10
#define M_RUNCURRENT        0x20
#define M_STANDBYCURRENT     0x30
#define M_STALLGUARD_THRESHOLD         0x40
#define M_CHOPPER         0x50
#define M_COOLSTEP    0x60

/// @brief Modparams settings available via XML.
static const lcec_modparam_desc_t modparams_perchannel[] = {
  {"microsteps", M_MICROSTEPS, MODPARAM_TYPE_U32},
    {"motorResolution_pulses", M_MOTOR_RESOLUTION, MODPARAM_TYPE_U32},
  {"current_amps", M_RUNCURRENT, MODPARAM_TYPE_FLOAT},
  {"standbyCurrent_amps", M_STANDBYCURRENT, MODPARAM_TYPE_U32},
  {"stallguardThreshold", M_STALLGUARD_THRESHOLD, MODPARAM_TYPE_U32},
  {"chopperConfiguration", M_CHOPPER, MODPARAM_TYPE_U32},
  {"coolstepConfiguration", M_COOLSTEP, MODPARAM_TYPE_U32},
    {NULL},
};

static const lcec_modparam_desc_t modparams_base[] = {
    {NULL},
};

// "Normal" settings that should be applied to each channel.
//
// We don't want to list *everything* here, because that gets
// overwhelming, but we do want to include the most common things that
// users will want to change.
#define PER_CHANNEL_OVERRIDES(ch)                                                                                         \
      {#ch "current_amps", 0, 0, "2.0", "Maximum stepper Amps."},                                                     \
      {#ch "microsteps", 0, 0, "0", "Microsteps setting (default 0)"},                                                     \
      {#ch "motorResolution_pulses", 0, 0, "100000", "Steps per rotation"}                                                     

/// Override values for single-axis open-loop steppers
static const lcec_modparam_desc_t overrides[] = {
    PER_CHANNEL_OVERRIDES(ch1),
    PER_CHANNEL_OVERRIDES(ch2),
    PER_CHANNEL_OVERRIDES(ch3),
    PER_CHANNEL_OVERRIDES(ch4),
    PER_CHANNEL_OVERRIDES(ch5),
    PER_CHANNEL_OVERRIDES(ch6),
    {NULL},
};


static int lcec_rovix_init(int comp_id, lcec_slave_t *slave);

static lcec_typelist_t types[] = {
    // note that modparams_rovix is added implicitly in ADD_TYPES_WITH_CIA402_MODPARAMS.
    {"RovixESD-A6", LCEC_ROVIX_VID, 0x5, 0, NULL, lcec_rovix_init, NULL, 0},
    {NULL},
};

ADD_TYPES_WITH_CIA402_MODPARAMS(types, modparams_perchannel, modparams_base, overrides)

static void lcec_rovix_read(lcec_slave_t *slave, long period);
static void lcec_rovix_write(lcec_slave_t *slave, long period);


typedef struct {
  lcec_class_cia402_channels_t *cia402;
  hal_u32_t *status_code;
  unsigned int status_code_os;  ///<
} lcec_rovix_data_t;

static int handle_modparams(lcec_slave_t *slave, lcec_class_cia402_options_t *opt) {
  lcec_master_t *master = slave->master;
  lcec_slave_modparam_t *p;
  uint32_t uval;
  int v;


  // We'll need to byte-swap here, for big-endian systems.

  for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
    int channel = p->id&7;
    int id = p->id&~7;
    int base = 0x2000 + 0x800*channel;
    
    switch (id) {
      case M_RUNCURRENT:
        uval = p->value.flt * 10.0 + 0.5;
        if (lcec_write_sdo16_modparam(slave, base + 5, 0, uval, p->name) < 0) return -1;
        break;
      case M_MOTOR_RESOLUTION:
        if (lcec_write_sdo16_modparam(slave, base + 3, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_STANDBYCURRENT:
        uval = p->value.flt * 10.0 + 0.5;
        if (lcec_write_sdo16_modparam(slave, base + 5, 0, uval, p->name) < 0) return -1;
        break;
      default:
        v = lcec_cia402_handle_modparam(slave, p, opt);

        if (v > 0) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "unknown modparam %s for slave %s.%s\n", p->name, master->name, slave->name);
          return -1;
        }

        if (v < 0) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "unknown error %d from lcec_cia402_handle_modparam for slave %s.%s\n", v,
              master->name, slave->name);
          return v;
        }
        break;
    }
  }
  
  return 0;
}

static int lcec_rovix_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_rovix_data_t *hal_data;
  int channel;

  // alloc hal memory
  hal_data = LCEC_HAL_ALLOCATE(lcec_rovix_data_t);
  slave->hal_data = hal_data;

  // initialize callbacks
  slave->proc_read = lcec_rovix_read;
  slave->proc_write = lcec_rovix_write;

  // Apply default Distributed Clock settings if it's not already set.
  //  if (slave->dc_conf == NULL) {
  //    lcec_slave_dc_t *dc = LCEC_HAL_ALLOCATE(lcec_slave_dc_t);
  //    dc->assignActivate = 0x300;  // All known RTelligent steppers use 0x300, according to their ESI.
  //    dc->sync0Cycle = slave->master->app_time_period;
  //
  //    slave->dc_conf = dc;
  //  }

  lcec_class_cia402_options_t *options = lcec_cia402_options();
  options->channels = 6;
  lcec_cia402_rename_multiaxis_channels(options);

  // The ECT60 supports these CiA 402 features (plus a few others).
  // We'll assume that all of the EC* devices do, until we learn
  // otherwise.
  for (channel = 0; channel < options->channels; channel++) {
    options->channel[channel]->enable_csp = 1;
    options->channel[channel]->enable_error_code = 1;
    options->channel[channel]->enable_polarity = 1;
    options->channel[channel]->enable_positioning_window =1;
    options->channel[channel]->enable_profile_accel = 1;
    options->channel[channel]->enable_profile_decel = 1;
    options->channel[channel]->enable_profile_velocity = 1;
    options->channel[channel]->enable_profile_max_velocity = 1;
    options->channel[channel]->enable_digital_input = 1;
    options->channel[channel]->enable_digital_output = 1;
    options->channel[channel]->digital_in_channels = 6; // No idea if this is the right number
    options->channel[channel]->digital_out_channels = 2; // No idea if this is the right number
  }

  if (handle_modparams(slave, options) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "modparam handling failure for slave %s.%s\n", master->name, slave->name);
    return -EIO;
  }

  // *Don't* set up syncs.  The ESD-A6 don't have modifiable PDOs.

  hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);

  for (int channel = 0; channel < options->channels; channel++) {
    hal_data->cia402->channels[channel] = lcec_cia402_register_channel(slave, 0x6000 + 0x800 * channel, options->channel[channel]);
  }

  return 0;
}

static void lcec_rovix_read(lcec_slave_t *slave, long period) {
  lcec_rovix_data_t *hal_data = (lcec_rovix_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_read_all(slave, hal_data->cia402);
}

static void lcec_rovix_write(lcec_slave_t *slave, long period) {
  lcec_rovix_data_t *hal_data = (lcec_rovix_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_write_all(slave, hal_data->cia402);
}
