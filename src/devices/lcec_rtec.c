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
///
/// This driver should (but does not currently) have modParams for these settings
///
/// Useful:
/// - 0x200b:01 motor autotune enable
/// - 0x200b:02 motor Kp
/// - 0x200b:03 motor Ki
/// - 0x200b:04 motor Kc
/// - 0x200c:04 stepper resistance set
/// - 0x200c:05 stepper inductance set
/// - 0x200c:06 BEMF coefficient
/// - 0x200d run reverse

#include "../lcec.h"
#include "lcec_class_cia402.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

#define M_PEAKCURRENT        0
#define M_MOTOR_RESOLUTION   1
#define M_STANDBYTIME        2
#define M_STANDBYCURRENT     3
#define M_OUTPUT1FUNC        4
#define M_OUTPUT2FUNC        5
#define M_OUTPUT1POLARITY    6
#define M_OUTPUT2POLARITY    7
#define M_INPUT3FUNC         8
#define M_INPUT4FUNC         9
#define M_INPUT5FUNC         10
#define M_INPUT6FUNC         11
#define M_INPUT3POLARITY     12
#define M_INPUT4POLARITY     13
#define M_INPUT5POLARITY     14
#define M_INPUT6POLARITY     15
#define M_FILTERTIME         16
#define M_SHAFTLOCKTIME      17
#define M_MOTORAUTOTUNE      18
#define M_STEPPERPHASES      19
#define M_CONTROLMODE        20
#define M_ENCODER_RESOLUTION 21
#define M_POSITION_ERROR     22

/// @brief Modparams settings available via XML.
static const lcec_modparam_desc_t modparams_rtec[] = {
    {"peakCurrent_amps", M_PEAKCURRENT, MODPARAM_TYPE_FLOAT},
    {"motorResolution_pulses", M_MOTOR_RESOLUTION, MODPARAM_TYPE_U32},
    {"standbyTime_ms", M_STANDBYTIME, MODPARAM_TYPE_U32},
    {"standbyCurrent_pct", M_STANDBYCURRENT, MODPARAM_TYPE_U32},
    {"output1Func", M_OUTPUT1FUNC, MODPARAM_TYPE_STRING},
    {"output2Func", M_OUTPUT2FUNC, MODPARAM_TYPE_STRING},
    {"output1Polarity", M_OUTPUT1POLARITY, MODPARAM_TYPE_STRING},
    {"output2Polarity", M_OUTPUT1POLARITY, MODPARAM_TYPE_STRING},
    {"input3Func", M_INPUT3FUNC, MODPARAM_TYPE_STRING},
    {"input4Func", M_INPUT4FUNC, MODPARAM_TYPE_STRING},
    {"input5Func", M_INPUT5FUNC, MODPARAM_TYPE_STRING},
    {"input6Func", M_INPUT6FUNC, MODPARAM_TYPE_STRING},
    {"input3Polarity", M_INPUT3POLARITY, MODPARAM_TYPE_STRING},
    {"input4Polarity", M_INPUT4POLARITY, MODPARAM_TYPE_STRING},
    {"input5Polarity", M_INPUT5POLARITY, MODPARAM_TYPE_STRING},
    {"input6Polarity", M_INPUT6POLARITY, MODPARAM_TYPE_STRING},
    {"filterTime_us", M_FILTERTIME, MODPARAM_TYPE_U32},
    {"shaftLockTime_us", M_SHAFTLOCKTIME, MODPARAM_TYPE_U32},
    {"motorAutoTune", M_MOTORAUTOTUNE, MODPARAM_TYPE_BIT},
    {"stepperPhases", M_STEPPERPHASES, MODPARAM_TYPE_STRING},
    {"controlMode", M_CONTROLMODE, MODPARAM_TYPE_STRING},
    {"encoderResolution", M_ENCODER_RESOLUTION, MODPARAM_TYPE_U32},
    {"positionErrorLimit", M_POSITION_ERROR, MODPARAM_TYPE_U32},
    {NULL},
};

static int lcec_rtec_init(int comp_id, lcec_slave_t *slave);

static lcec_typelist_t types[] = {
    {"ECR60", LCEC_RTELLIGENT_VID, 0x0a880001, 0, NULL, lcec_rtec_init, /* modparams_rtec */},
    {"ECR60x2", LCEC_RTELLIGENT_VID, 0x0a880005, 0, NULL, lcec_rtec_init,
        /* modparams_rtec */},  // Only one channel supported right now
    {"ECT60", LCEC_RTELLIGENT_VID, 0x0a880002, 0, NULL, lcec_rtec_init, /* modparams_rtec */},
    {"ECT60x2", LCEC_RTELLIGENT_VID, 0x0a880006, 0, NULL, lcec_rtec_init,
        /* modparams_rtec */},  // Only one channel supported right now
    {"ECR86", LCEC_RTELLIGENT_VID, 0x0a880003, 0, NULL, lcec_rtec_init, /* modparams_rtec */},
    {"ECT86", LCEC_RTELLIGENT_VID, 0x0a880004, 0, NULL, lcec_rtec_init, /* modparams_rtec */},
    {NULL},
};
ADD_TYPES_WITH_CIA402_MODPARAMS(types, modparams_rtec)

static void lcec_rtec_read(lcec_slave_t *slave, long period);
static void lcec_rtec_write(lcec_slave_t *slave, long period);

static const lcec_lookuptable_int_t rtec_outputfunc[] = {
    {"custom", 0},
    {"general", 0},  // The docs say "custom", but "general" matches the input options.
    {"alarm", 1},
    {"brake", 2},
    {"in-place", 3},
    {NULL},
};

static const lcec_lookuptable_int_t rtec_polarity[] = {
    {"nc", 0},
    {"no", 1},
    {NULL},
};

static const lcec_lookuptable_int_t rtec_inputfunc[] = {
    {"general", 0},
    {"cw_limit", 1},
    {"cw-limit", 1},
    {"ccw_limit", 2},
    {"ccw-limit", 2},
    {"home", 3},
    {"clear_fault", 4},
    {"clear-fault", 4},
    {"emergency_stop", 5},
    {"emergency-stop", 5},
    {"motor_offline", 6},
    {"motor-offline", 6},
    {"probe1", 7},
    {"probe2", 8},
    {NULL},
};

static const lcec_lookuptable_int_t rtec_stepperphases[] = {
    {"2", 0},
    {"3", 1},
    {NULL},
};

static const lcec_lookuptable_int_t rtec_controlmode[] = {
    {"openloop", 0},
    {"closedloop", 1},
    {"foc", 2},
    {NULL},
};

typedef struct {
  lcec_class_cia402_channels_t *cia402;
  lcec_class_din_channels_t *din;
  lcec_class_dout_channels_t *dout;
  hal_u32_t *alarm_code;
  hal_u32_t *status_code;
  hal_u32_t *encoder_position;
  hal_s32_t *current_rpm;
  hal_float_t *voltage;
  unsigned int alarm_code_os;        ///<
  unsigned int status_code_os;       ///<
  unsigned int encoder_position_os;  ///<
  unsigned int current_rpm_os;       ///<
  unsigned int voltage_os;           ///<
  unsigned int din_os;
} lcec_rtec_data_t;

static const lcec_pindesc_t slave_pins[] = {
    {HAL_U32, HAL_OUT, offsetof(lcec_rtec_data_t, alarm_code), "%s.%s.%s.alarm-code"},
    {HAL_U32, HAL_OUT, offsetof(lcec_rtec_data_t, status_code), "%s.%s.%s.status-code"},
    {HAL_U32, HAL_OUT, offsetof(lcec_rtec_data_t, encoder_position), "%s.%s.%s.enc-pos"},
    {HAL_S32, HAL_OUT, offsetof(lcec_rtec_data_t, current_rpm), "%s.%s.%s.current-rpm"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_rtec_data_t, voltage), "%s.%s.%s.voltage"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static int handle_modparams(lcec_slave_t *slave, lcec_class_cia402_options_t *opt) {
  lcec_master_t *master = slave->master;
  lcec_slave_modparam_t *p;
  uint16_t input_polarity = 0, input_polarity_set = 0;
  uint16_t output_polarity = 0, output_polarity_set = 0;
  uint32_t uval;
  int v;

  // Read current polarity values, so we don't overwrite them all.
  lcec_read_sdo16(slave, 0x2006, 0, &output_polarity);
  lcec_read_sdo16(slave, 0x2008, 0, &input_polarity);

  // We'll need to byte-swap here, for big-endian systems.

  for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
    switch (p->id) {
      case M_PEAKCURRENT:
        uval = p->value.flt * 1000.0 + 0.5;
        if (lcec_write_sdo16_modparam(slave, 0x2000, 0, uval, p->name) < 0) return -1;
        break;
      case M_MOTOR_RESOLUTION:
        if (lcec_write_sdo16_modparam(slave, 0x2001, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_STANDBYTIME:
        if (lcec_write_sdo16_modparam(slave, 0x2002, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_STANDBYCURRENT:
        if (lcec_write_sdo16_modparam(slave, 0x2003, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_OUTPUT1FUNC:
        uval = lcec_lookupint_i(rtec_outputfunc, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"output1Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, 0x2005, 1, uval, p->name) < 0) return -1;
        break;
      case M_OUTPUT2FUNC:
        uval = lcec_lookupint_i(rtec_outputfunc, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"output2Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, 0x2005, 2, uval, p->name) < 0) return -1;
        break;
      case M_OUTPUT1POLARITY:
        uval = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"output1Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        output_polarity &= ~1;  // Clear the 0 bit.
        output_polarity |= (uval & 1);
        output_polarity_set = 1;
        break;
      case M_OUTPUT2POLARITY:
        uval = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"output2Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        output_polarity &= ~2;  // Clear the 1 bit.
        output_polarity |= (uval & 1) << 1;
        output_polarity_set = 1;
        break;
      case M_INPUT3FUNC:
        uval = lcec_lookupint_i(rtec_inputfunc, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input3Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, 0x2007, 3, uval, p->name) < 0) return -1;
        break;
      case M_INPUT4FUNC:
        uval = lcec_lookupint_i(rtec_inputfunc, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input4Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, 0x2007, 4, uval, p->name) < 0) return -1;
        break;
      case M_INPUT5FUNC:
        uval = lcec_lookupint_i(rtec_inputfunc, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input5Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, 0x2007, 5, uval, p->name) < 0) return -1;
        break;
      case M_INPUT6FUNC:
        uval = lcec_lookupint_i(rtec_inputfunc, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input6Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, 0x2007, 6, uval, p->name) < 0) return -1;
        break;
      case M_INPUT3POLARITY:
        uval = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input3Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        input_polarity &= ~(1 << 2);
        input_polarity |= (uval & 1) << 2;
        input_polarity_set = 1;
        break;
      case M_INPUT4POLARITY:
        uval = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input4Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        input_polarity &= ~(1 << 3);
        input_polarity |= (uval & 1) << 3;
        input_polarity_set = 1;
        break;
      case M_INPUT5POLARITY:
        uval = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input5Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        input_polarity &= ~(1 << 4);
        input_polarity |= (uval & 1) << 4;
        input_polarity_set = 1;
        break;
      case M_INPUT6POLARITY:
        uval = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input6Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        input_polarity &= ~(1 << 5);
        input_polarity |= (uval & 1) << 5;
        input_polarity_set = 1;
        break;
      case M_FILTERTIME:
        if (lcec_write_sdo16_modparam(slave, 0x2009, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_SHAFTLOCKTIME:
        if (lcec_write_sdo16_modparam(slave, 0x200a, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_MOTORAUTOTUNE:
        if (lcec_write_sdo16_modparam(slave, 0x200b, 1, !!p->value.bit, p->name) < 0) return -1;
        break;
      case M_STEPPERPHASES:
        uval = lcec_lookupint_i(rtec_stepperphases, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"stepperPhases\"> for slave %s.%s\n", master->name,
              slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, 0x200c, 1, uval, p->name) < 0) return -1;
        break;
      case M_CONTROLMODE:
        uval = lcec_lookupint_i(rtec_controlmode, p->value.str, -1);
        if (uval == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"controlMode\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, 0x2011, 0, uval, p->name) < 0) return -1;
        break;
      case M_ENCODER_RESOLUTION:
        if (lcec_write_sdo16_modparam(slave, 0x2020, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_POSITION_ERROR:
        if (lcec_write_sdo16_modparam(slave, 0x2022, 0, p->value.u32, p->name) < 0) return -1;
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

  if (output_polarity_set) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "Setting output polarity to 0x%x for slave %s.%s\n", output_polarity, master->name, slave->name);
    if (lcec_write_sdo16(slave, 0x2006, 0, output_polarity) != 0) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to set slave %s.%s sdo outputPolarity to %u\n", master->name, slave->name, output_polarity);
      return -1;
    }
  }
  if (input_polarity_set) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "Setting input polarity to 0x%x for slave %s.%s\n", input_polarity, master->name, slave->name);
    if (lcec_write_sdo16(slave, 0x2008, 0, input_polarity) != 0) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to set slave %s.%s sdo inputPolarity to %u\n", master->name, slave->name, input_polarity);
      return -1;
    }
  }

  return 0;
}

static int lcec_rtec_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_rtec_data_t *hal_data;
  int err;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_rtec_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", slave->master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_rtec_data_t));
  slave->hal_data = hal_data;

  // initialize callbacks
  slave->proc_read = lcec_rtec_read;
  slave->proc_write = lcec_rtec_write;

  lcec_class_cia402_options_t *options = lcec_cia402_options();
  // The ECT60 should support these CiA 402 features:
  options->channel[0]->enable_pv = 1;
  options->channel[0]->enable_pp = 1;
  options->channel[0]->enable_csp = 1;
  options->channel[0]->enable_csv = 1;
  options->channel[0]->enable_hm = 1;
  options->channel[0]->enable_actual_torque = 1;
  options->channel[0]->enable_digital_input = 1;
  options->channel[0]->enable_digital_output = 1;
  options->channel[0]->enable_profile_velocity = 1;
  options->channel[0]->enable_profile_accel = 1;
  options->channel[0]->enable_profile_decel = 1;
  options->channel[0]->enable_home_accel = 1;
  // options->channel[0]->enable_interpolation_time_period = 1;  // Should be supported but doesn't actually work.
  options->channel[0]->enable_actual_following_error = 1;

  if (handle_modparams(slave, options) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "modparam handling failure for slave %s.%s\n", master->name, slave->name);
    return -EIO;
  }

  // Set up syncs.  This is needed because the ECT60 (at least)
  // doesn't map all of the PDOs that we need, so we need to set up
  // our own mappings.
  lcec_syncs_t *syncs = lcec_cia402_init_sync(slave, options);
  lcec_cia402_add_output_sync(syncs, options);
  // No output PDOs right now.

  lcec_cia402_add_input_sync(syncs, options);
  lcec_syncs_add_pdo_info(syncs, 0x1a02);
  lcec_syncs_add_pdo_entry(syncs, 0x200e, 0x00, 16);  // alarm codes
  lcec_syncs_add_pdo_entry(syncs, 0x200f, 0x00, 16);  // status codes
  lcec_syncs_add_pdo_entry(syncs, 0x2021, 0x00, 16);  // Current encoder position
  lcec_syncs_add_pdo_entry(syncs, 0x2044, 0x00, 16);  // Current speed in RPM
  lcec_syncs_add_pdo_entry(syncs, 0x2048, 0x00, 16);  // Current bus voltage

  slave->sync_info = &syncs->syncs[0];

  hal_data->cia402 = lcec_cia402_allocate_channels(1);
  if (hal_data->cia402 == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_cia402_allocate_channels() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }

  hal_data->cia402->channels[0] = lcec_cia402_register_channel(slave, 0x6000, options->channel[0]);

  hal_data->din = lcec_din_allocate_channels(10);
  if (hal_data->din == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_din_allocate_channels() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }

  hal_data->dout = lcec_dout_allocate_channels(3);
  if (hal_data->din == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_din_allocate_channels() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }

  // Register rtec-specific PDOs.
  lcec_pdo_init(slave, 0x200e, 0, &hal_data->alarm_code_os, NULL);
  lcec_pdo_init(slave, 0x200f, 0, &hal_data->status_code_os, NULL);
  lcec_pdo_init(slave, 0x2021, 0, &hal_data->encoder_position_os, NULL);
  lcec_pdo_init(slave, 0x2044, 0, &hal_data->current_rpm_os, NULL);
  lcec_pdo_init(slave, 0x2048, 0, &hal_data->voltage_os, NULL);

  hal_data->din->channels[0] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 0, "din-cw-limit");   // negative limit switch
  hal_data->din->channels[1] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 1, "din-ccw-limit");  // positive limit switch
  hal_data->din->channels[2] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 2, "din-home");       // home
  hal_data->din->channels[3] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 3, "din-interlock");  // interlock?
  hal_data->din->channels[4] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 16, "din-1");
  hal_data->din->channels[5] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 17, "din-2");
  hal_data->din->channels[6] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 18, "din-3");
  hal_data->din->channels[7] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 19, "din-4");
  hal_data->din->channels[8] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 20, "din-5");
  hal_data->din->channels[9] = lcec_din_register_channel_packed(slave, 0x60fd, 0, 21, "din-6");

  hal_data->dout->channels[0] = lcec_dout_register_channel_packed(slave, 0x60fe, 1, 16, "dout-brake");
  hal_data->dout->channels[1] = lcec_dout_register_channel_packed(slave, 0x60fe, 1, 16, "dout-1");
  hal_data->dout->channels[2] = lcec_dout_register_channel_packed(slave, 0x60fe, 1, 17, "dout-2");

  // export rtec-specific pins
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, slave->master->name, slave->name)) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "pin registration failure for slave %s.%s\n", master->name, slave->name);
    return err;
  }

  return 0;
}

static void lcec_rtec_read(lcec_slave_t *slave, long period) {
  lcec_rtec_data_t *hal_data = (lcec_rtec_data_t *)slave->hal_data;
  uint8_t *pd = slave->master->process_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  *(hal_data->alarm_code) = EC_READ_U16(&pd[hal_data->alarm_code_os]);
  *(hal_data->status_code) = EC_READ_U16(&pd[hal_data->status_code_os]);
  *(hal_data->encoder_position) = EC_READ_U16(&pd[hal_data->encoder_position_os]);
  *(hal_data->current_rpm) = EC_READ_S16(&pd[hal_data->current_rpm_os]);
  *(hal_data->voltage) = EC_READ_U16(&pd[hal_data->voltage_os]) / 100.0;
  lcec_cia402_read_all(slave, hal_data->cia402);
  lcec_din_read_all(slave, hal_data->din);
}

static void lcec_rtec_write(lcec_slave_t *slave, long period) {
  lcec_rtec_data_t *hal_data = (lcec_rtec_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_write_all(slave, hal_data->cia402);
  lcec_dout_write_all(slave, hal_data->dout);
}

/*
  Rod's sample ECT60 XML config looks like this.  I'm not sure why it's using 204a for outputs instead of 60fe.

    <slave idx="0" type="generic" vid="00000a88" pid="0a880004" configPdos="true">
      <dcConf assignActivate="300" sync0Cycle="*1" sync0Shift="0"/>
      <sdoConfig idx="2000" subIdx="0"><sdoDataRaw data ="70 17"/></sdoConfig>  <!-- Max motor current (6.0) -->
      <sdoConfig idx="2007" subIdx="6"><sdoDataRaw data ="05"/></sdoConfig>     <!-- Input 6 - Emergency Stop -->
      <sdoConfig idx="2007" subIdx="5"><sdoDataRaw data ="05"/></sdoConfig>     <!-- Input 3 - Home Function -->
      <sdoConfig idx="2011" subIdx="0"><sdoDataRaw data ="01 00"/></sdoConfig>  <!-- Closed loop -->
      <sdoConfig idx="6098" subIdx="0"><sdoDataRaw data ="11 00"/></sdoConfig>  <!-- Home mode 17 -->
      <sdoConfig idx="607C" subIdx="0"><sdoDataRaw data ="00 00"/></sdoConfig>  <!-- Home offset 0  -->
      <sdoConfig idx="609A" subIdx="0"><sdoDataRaw data ="F4 01"/></sdoConfig>  <!-- Home accelleration 500 -->
      <sdoConfig idx="6099" subIdx="01"><sdoDataRaw data ="C4 09"/></sdoConfig>  <!-- Home fast speed 2500-->
      <sdoConfig idx="6099" subIdx="02"><sdoDataRaw data ="F4 01"/></sdoConfig>  <!-- Home slow speed 500 -->
      <syncManager idx="2" dir="out">
        <pdo idx="1600">
          <pdoEntry idx="6040" subIdx="00" bitLen="16" halPin="cia-controlword" halType="u32"/>
          <pdoEntry idx="6060" subIdx="00" bitLen="8" halPin="opmode" halType="s32"/>
          <!-- Target Position -->
          <pdoEntry idx="607A" subIdx="00" bitLen="32" halPin="target-position" halType="s32"/>
          <!-- Target Velocity -->
          <pdoEntry idx="60FF" subIdx="00" bitLen="32" halPin="target-velocity" halType="s32"/>
          <!-- Digtial Outputs (manufacturer's extension ECT86/ECT60)-->
          <pdoEntry idx="204A" subIdx="0" bitLen="16" halType="complex">
            <complexEntry bitLen="1" halPin="out-1" halType="bit"/>
            <complexEntry bitLen="1" halPin="out-2" halType="bit"/>
            <complexEntry bitLen="14"/>
          </pdoEntry>
        </pdo>
      </syncManager>
      <syncManager idx="3" dir="in">
        <pdo idx="1a00">
          <pdoEntry idx="6041" subIdx="00" bitLen="16" halPin="cia-statusword" halType="u32"/>
          <pdoEntry idx="6061" subIdx="00" bitLen="8" halPin="opmode-display" halType="s32"/>
          <pdoEntry idx="6064" subIdx="00" bitLen="32" halPin="actual-position" halType="s32"/>
          <pdoEntry idx="606C" subIdx="00" bitLen="32" halPin="actual-velocity" halType="s32"/>
          <pdoEntry idx="6077" subIdx="00" bitLen="32" halPin="actual-torque" halType="s32"/>
          <!-- Digtial_inputs (cia402 compatible) -->
          <pdoEntry idx="60FD" subIdx="0" bitLen="32" halType="complex">
            <complexEntry bitLen="1" halPin="CW-limit" halType="bit"/>
            <complexEntry bitLen="1" halPin="CCW-limit" halType="bit"/>
            <complexEntry bitLen="1" halPin="in-home" halType="bit"/>
            <complexEntry bitLen="13"/>
            <complexEntry bitLen="1" halPin="in-1" halType="bit"/>
            <complexEntry bitLen="1" halPin="in-2" halType="bit"/>
            <complexEntry bitLen="1" halPin="in-3" halType="bit"/>
            <complexEntry bitLen="1" halPin="in-4" halType="bit"/>
            <complexEntry bitLen="1" halPin="in-5" halType="bit"/>
            <complexEntry bitLen="1" halPin="in-6" halType="bit"/>
            <complexEntry bitLen="10"/>
          </pdoEntry>
        </pdo>
      </syncManager>
    </slave>
*/
