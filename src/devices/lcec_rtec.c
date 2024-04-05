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

#define M_PEAKCURRENT        0x0
#define M_MOTOR_RESOLUTION   0x10
#define M_STANDBYTIME        0x20
#define M_STANDBYCURRENT     0x30
#define M_OUTPUT1FUNC        0x40
#define M_OUTPUT2FUNC        0x50
#define M_OUTPUT1POLARITY    0x60
#define M_OUTPUT2POLARITY    0x70
#define M_INPUT3FUNC         0x80
#define M_INPUT4FUNC         0x90
#define M_INPUT5FUNC         0xa0
#define M_INPUT6FUNC         0xb0
#define M_INPUT3POLARITY     0xc0
#define M_INPUT4POLARITY     0xd0
#define M_INPUT5POLARITY     0xe0
#define M_INPUT6POLARITY     0xf0
#define M_FILTERTIME         0x100
#define M_SHAFTLOCKTIME      0x110
#define M_MOTORAUTOTUNE      0x120
#define M_STEPPERPHASES      0x130
#define M_CONTROLMODE        0x140
#define M_ENCODER_RESOLUTION 0x150
#define M_POSITION_ERROR     0x160

/// @brief Modparams settings available via XML.
static const lcec_modparam_desc_t modparams_perchannel[] = {
    {"peakCurrent_amps", M_PEAKCURRENT, MODPARAM_TYPE_FLOAT},
    {"motorResolution_pulses", M_MOTOR_RESOLUTION, MODPARAM_TYPE_U32},
    {"standbyTime_ms", M_STANDBYTIME, MODPARAM_TYPE_U32},
    {"standbyCurrent_pct", M_STANDBYCURRENT, MODPARAM_TYPE_U32},
    {"filterTime_us", M_FILTERTIME, MODPARAM_TYPE_U32},
    {"shaftLockTime_us", M_SHAFTLOCKTIME, MODPARAM_TYPE_U32},
    {"motorAutoTune", M_MOTORAUTOTUNE, MODPARAM_TYPE_BIT},
    {"stepperPhases", M_STEPPERPHASES, MODPARAM_TYPE_STRING},
    {"controlMode", M_CONTROLMODE, MODPARAM_TYPE_STRING},
    {"encoderResolution", M_ENCODER_RESOLUTION, MODPARAM_TYPE_U32},
    {"positionErrorLimit", M_POSITION_ERROR, MODPARAM_TYPE_U32},
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
  {#ch "enableCSP", 0, 0, "true", "Enable support for Cyclic Synchronous Position mode."},                                \
      {#ch "enableCSV", 0, 0, "false", "Enable support for Cyclic Synchronous Velocity mode."},                           \
      {#ch "encoderResolution", 0, 0, "4000", "Number of encoder steps per revolution."},                                 \
      {#ch "peakCurrent_amps", 0, 0, "6.0", "Maximum stepper Amps."},                                                     \
      {#ch "output1Func", 0, 0, "alarm", "Output 1 use: general, alarm, brake, in-place."},                               \
      {#ch "output2Func", 0, 0, "brake", "Output 2 use: general, alarm, brake, in-place."},                               \
      {#ch "input3Func", 0, 0, "cw-limit",                                                                                \
          "Input 3 use: general, cw-limit, ccw-limit, home, clear-fault, emergency-stop, motor-offline, probe1, probe2"}, \
      {#ch "input4Func", 0, 0, "ccw-limit",                                                                               \
          "Input 4 use: general, cw-limit, ccw-limit, home, clear-fault, emergency-stop, motor-offline, probe1, probe2"}, \
      {#ch "input5Func", 0, 0, "home",                                                                                    \
          "Input 5 use: general, cw-limit, ccw-limit, home, clear-fault, emergency-stop, motor-offline, probe1, probe2"}, \
  {                                                                                                                       \
#ch "input6Func", 0, 0, "motor-offline",                                                                              \
        "Input 6 use: general, cw-limit, ccw-limit, home, clear-fault, emergency-stop, motor-offline, probe1, probe2"     \
  }

/// Override values for single-axis open-loop steppers
static const lcec_modparam_desc_t overrides_open[] = {
    PER_CHANNEL_OVERRIDES(),
    {"motorResolution_pulses", 0, 0, "10000", "Number of stepper pulses per rotation"},
    {NULL},
};

/// Overrides for dual-axis open-loop steppers
static const lcec_modparam_desc_t overrides_open_x2[] = {
    PER_CHANNEL_OVERRIDES(ch1),
    PER_CHANNEL_OVERRIDES(ch2),
    {"ch1motorResolution_pulses", 0, 0, "10000", "Number of stepper pulses per rotation"},
    {"ch1motorResolution_pulses", 0, 0, "10000", "Number of stepper pulses per rotation"},
    {NULL},
};

/// Override values for single-axis closed-loop steppers
static const lcec_modparam_desc_t overrides_closed[] = {
    PER_CHANNEL_OVERRIDES(),
    {"controlMode", 0, 0, "closedloop", "Operation mode: openloop, closedloop, or foc."},
    {NULL},
};

/// Override values for dual-axis closed-loop steppers
static const lcec_modparam_desc_t overrides_closed_x2[] = {
    PER_CHANNEL_OVERRIDES(ch1),
    PER_CHANNEL_OVERRIDES(ch2),
    {"ch1controlMode", 0, 0, "closedloop", "Operation mode: openloop, closedloop, or foc."},
    {"ch2controlMode", 0, 0, "closedloop", "Operation mode: openloop, closedloop, or foc."},
    {NULL},
};

static int lcec_rtec_init(int comp_id, lcec_slave_t *slave);

#define AXES(flags)          ((flags >> 60) & 0xf)
#define PDOINCREMENT(flags)  ((flags >> 52) & 0xff)
#define F_AXES(axes)         ((uint64_t)axes << 60)
#define F_PDOINCREMENT(incr) ((uint64_t)incr << 52)
#define F_NOEXTRAS           1  // Don't map RTelligent-specific PDO entries

static lcec_typelist_t types_open[] = {
    // note that modparams_rtec is added implicitly in ADD_TYPES_WITH_CIA402_MODPARAMS.
    {"ECR60", LCEC_RTELLIGENT_VID, 0x0a880001, 0, NULL, lcec_rtec_init, NULL, 0},
    {"ECR86", LCEC_RTELLIGENT_VID, 0x0a880003, 0, NULL, lcec_rtec_init, NULL, 0},
    {NULL},
};

static lcec_typelist_t types_open_x2[] = {
    // note that modparams_rtec is added implicitly in ADD_TYPES_WITH_CIA402_MODPARAMS.

    // The ECR60X2 only has 2 RX/TX PDOs, at 1600/1610 and 1a00/1a10.
    {"ECR60x2", LCEC_RTELLIGENT_VID, 0x0a880005, 0, NULL, lcec_rtec_init, NULL, F_AXES(2) | F_PDOINCREMENT(0x10) | F_NOEXTRAS},
    {NULL},
};

static lcec_typelist_t types_closed[] = {
    // note that modparams_rtec is added implicitly in ADD_TYPES_WITH_CIA402_MODPARAMS.
    {"ECT60", LCEC_RTELLIGENT_VID, 0x0a880002, 0, NULL, lcec_rtec_init, NULL, 0},
    {"ECT86", LCEC_RTELLIGENT_VID, 0x0a880004, 0, NULL, lcec_rtec_init, NULL, 0},
    {"ECT60x1", LCEC_RTELLIGENT_VID, 0x0a880006, 0, NULL, lcec_rtec_init, NULL, F_PDOINCREMENT(0x10)},
    {NULL},
};

static lcec_typelist_t types_closed_x2[] = {
    // note that modparams_rtec is added implicitly in ADD_TYPES_WITH_CIA402_MODPARAMS.

    // The ECT60X2 only has 2 RX/TX PDOs, at 1600/1610 and 1a00/1a10.
  {"ECT60x2", LCEC_RTELLIGENT_VID, 0x0a880006, 0, NULL, lcec_rtec_init, NULL, F_AXES(2) | F_PDOINCREMENT(0x10) | F_NOEXTRAS},
    {NULL},
};

ADD_TYPES_WITH_CIA402_MODPARAMS(types_open, modparams_perchannel, modparams_base, overrides_open)
ADD_TYPES_WITH_CIA402_MODPARAMS(types_open_x2, modparams_perchannel, modparams_base, overrides_open_x2)
ADD_TYPES_WITH_CIA402_MODPARAMS(types_closed, modparams_perchannel, modparams_base, overrides_closed)
ADD_TYPES_WITH_CIA402_MODPARAMS(types_closed_x2, modparams_perchannel, modparams_base, overrides_closed_x2)

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
  hal_u32_t *alarm_code;
  hal_u32_t *status_code;
  hal_float_t *voltage;
  unsigned int alarm_code_os;   ///<
  unsigned int status_code_os;  ///<
  unsigned int voltage_os;      ///<
} lcec_rtec_data_t;

static const lcec_pindesc_t slave_pins[] = {
    {HAL_U32, HAL_OUT, offsetof(lcec_rtec_data_t, alarm_code), "%s.%s.%s.alarm-code"},
    {HAL_U32, HAL_OUT, offsetof(lcec_rtec_data_t, status_code), "%s.%s.%s.status-code"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_rtec_data_t, voltage), "%s.%s.%s.voltage"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static int handle_modparams(lcec_slave_t *slave, lcec_class_cia402_options_t *opt) {
  lcec_master_t *master = slave->master;
  lcec_slave_modparam_t *p;
  uint16_t input_polarity[CIA402_MAX_CHANNELS], input_polarity_set[CIA402_MAX_CHANNELS];
  uint16_t output_polarity[CIA402_MAX_CHANNELS], output_polarity_set[CIA402_MAX_CHANNELS];
  uint32_t uval;
  int val, v;

  // Read current polarity values, so we don't overwrite them all.
  for (int channel = 0; channel < opt->channels; channel++) {
    lcec_read_sdo16(slave, 0x2006 + 0x800*channel, 0, &output_polarity[0]);
    lcec_read_sdo16(slave, 0x2008 + 0x800*channel, 0, &input_polarity[0]);
  }

  // We'll need to byte-swap here, for big-endian systems.

  for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
    int channel = p->id&7;
    int id = p->id&~7;
    int base = 0x2000 + 0x800*channel;

    if (channel != 0 && channel != 1) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid channel in lcec_rtec: %d slave %s.%s\n", channel, master->name, slave->name);
    }
    
    switch (id) {
      case M_PEAKCURRENT:
        uval = p->value.flt * 1000.0 + 0.5;
        if (lcec_write_sdo16_modparam(slave, base + 0, 0, uval, p->name) < 0) return -1;
        break;
      case M_MOTOR_RESOLUTION:
        if (lcec_write_sdo16_modparam(slave, base + 1, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_STANDBYTIME:
        if (lcec_write_sdo16_modparam(slave, base + 2, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_STANDBYCURRENT:
        if (lcec_write_sdo16_modparam(slave, base + 3, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_OUTPUT1FUNC:
        val = lcec_lookupint_i(rtec_outputfunc, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"output1Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, base + 5, 1, val, p->name) < 0) return -1;
        break;
      case M_OUTPUT2FUNC:
        val = lcec_lookupint_i(rtec_outputfunc, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"output2Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, base + 5, 2, val, p->name) < 0) return -1;
        break;
      case M_OUTPUT1POLARITY:
        val = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"output1Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        output_polarity[channel] &= ~1;  // Clear the 0 bit.
        output_polarity[channel] |= (val & 1);
        output_polarity_set[channel] = 1;
        break;
      case M_OUTPUT2POLARITY:
        val = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"output2Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        output_polarity[channel] &= ~2;  // Clear the 1 bit.
        output_polarity[channel] |= (val & 1) << 1;
        output_polarity_set[channel] = 1;
        break;
      case M_INPUT3FUNC:
        val = lcec_lookupint_i(rtec_inputfunc, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input3Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, base + 7, 3, val, p->name) < 0) return -1;
        break;
      case M_INPUT4FUNC:
        val = lcec_lookupint_i(rtec_inputfunc, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input4Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, base + 7, 4, val, p->name) < 0) return -1;
        break;
      case M_INPUT5FUNC:
        val = lcec_lookupint_i(rtec_inputfunc, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input5Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, base + 7, 5, val, p->name) < 0) return -1;
        break;
      case M_INPUT6FUNC:
        val = lcec_lookupint_i(rtec_inputfunc, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input6Func\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, base + 7, 6, val, p->name) < 0) return -1;
        break;
      case M_INPUT3POLARITY:
        val = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input3Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        input_polarity[channel] &= ~(1 << 2);
        input_polarity[channel] |= (val & 1) << 2;
        input_polarity_set[channel] = 1;
        break;
      case M_INPUT4POLARITY:
        val = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input4Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        input_polarity[channel] &= ~(1 << 3);
        input_polarity[channel] |= (val & 1) << 3;
        input_polarity_set[channel] = 1;
        break;
      case M_INPUT5POLARITY:
        val = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input5Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        input_polarity[channel] &= ~(1 << 4);
        input_polarity[channel] |= (val & 1) << 4;
        input_polarity_set[channel] = 1;
        break;
      case M_INPUT6POLARITY:
        val = lcec_lookupint_i(rtec_polarity, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"input6Polarity\"> for slave %s.%s\n",
              master->name, slave->name);
          return -1;
        }
        input_polarity[channel] &= ~(1 << 5);
        input_polarity[channel] |= (val & 1) << 5;
        input_polarity_set[channel] = 1;
        break;
      case M_FILTERTIME:
        if (lcec_write_sdo16_modparam(slave, base + 9, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_SHAFTLOCKTIME:
        if (lcec_write_sdo16_modparam(slave, base + 0xa, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_MOTORAUTOTUNE:
        if (lcec_write_sdo16_modparam(slave, base + 0xb, 1, !!p->value.bit, p->name) < 0) return -1;
        break;
      case M_STEPPERPHASES:
        val = lcec_lookupint_i(rtec_stepperphases, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"stepperPhases\"> for slave %s.%s\n", master->name,
              slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, base + 0xc, 1, val, p->name) < 0) return -1;
        break;
      case M_CONTROLMODE:
        val = lcec_lookupint_i(rtec_controlmode, p->value.str, -1);
        if (val == -1) {
          rtapi_print_msg(
              RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid value for <modparam name=\"controlMode\"> for slave %s.%s\n", master->name, slave->name);
          return -1;
        }
        if (lcec_write_sdo16_modparam(slave, base + 0x11, 0, val, p->name) < 0) return -1;
        break;
      case M_ENCODER_RESOLUTION:
        if (lcec_write_sdo16_modparam(slave, base + 0x20, 0, p->value.u32, p->name) < 0) return -1;
        break;
      case M_POSITION_ERROR:
        if (lcec_write_sdo16_modparam(slave, base + 0x22, 0, p->value.u32, p->name) < 0) return -1;
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

  for (int channel=0;channel<opt->channels;channel++) {
    if (output_polarity_set[channel]) {
      if (lcec_write_sdo16(slave, 0x2006 + 0x800*channel, 0, output_polarity[channel]) != 0) {
	rtapi_print_msg(
			RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to set slave %s.%s sdo outputPolarity to %u\n", master->name, slave->name, output_polarity[channel]);
	return -1;
      }
    }
    if (input_polarity_set[channel]) {
      if (lcec_write_sdo16(slave, 0x2008 + 0x800*channel, 0, input_polarity[channel]) != 0) {
	rtapi_print_msg(
			RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to set slave %s.%s sdo inputPolarity to %u\n", master->name, slave->name, input_polarity[channel]);
	return -1;
      }
    }
  }
  
  return 0;
}

static int lcec_rtec_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_rtec_data_t *hal_data;
  int err;
  int channel;

  // alloc hal memory
  hal_data = LCEC_HAL_ALLOCATE(lcec_rtec_data_t);
  slave->hal_data = hal_data;

  // initialize callbacks
  slave->proc_read = lcec_rtec_read;
  slave->proc_write = lcec_rtec_write;

  // Apply default Distributed Clock settings if it's not already set.
  if (slave->dc_conf == NULL) {
    lcec_slave_dc_t *dc = LCEC_HAL_ALLOCATE(lcec_slave_dc_t);
    dc->assignActivate = 0x300;  // All known RTelligent steppers use 0x300, according to their ESI.
    dc->sync0Cycle = slave->master->app_time_period;

    slave->dc_conf = dc;
  }

  lcec_class_cia402_options_t *options = lcec_cia402_options();
  options->rxpdolimit = 12;  // `ethercat sdos` shows 12, it's possible that more will work.
  options->txpdolimit = 12;

  if (AXES(slave->flags) != 0) {
    options->channels = AXES(slave->flags);
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "slave %s.%s has %d channels\n", master->name, slave->name, options->channels);
  } else {
    options->channels = 1;
  }

  if (PDOINCREMENT(slave->flags) != 0) {
    options->pdo_increment = PDOINCREMENT(slave->flags);
  } else {
    options->pdo_increment = 1;
  }

  if (options->channels > 1) {
    lcec_cia402_rename_multiaxis_channels(options);
  }
  // The ECT60 supports these CiA 402 features (plus a few others).
  // We'll assume that all of the EC* devices do, until we learn
  // otherwise.
  for (channel = 0; channel < options->channels; channel++) {
    options->channel[channel]->enable_csp = 1;
    options->channel[channel]->enable_csv = 0;
    options->channel[channel]->enable_hm = 1;
    options->channel[channel]->enable_actual_following_error = 1;
    options->channel[channel]->enable_actual_torque = 1;
    options->channel[channel]->enable_digital_input = 1;
    options->channel[channel]->enable_digital_output = 1;
    options->channel[channel]->enable_error_code = 1;
    options->channel[channel]->enable_home_accel = 1;
    options->channel[channel]->digital_in_channels = 6;
    options->channel[channel]->digital_out_channels = 2;
    options->channel[channel]->enable_actual_following_error = 1;
  }

  if (handle_modparams(slave, options) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "modparam handling failure for slave %s.%s\n", master->name, slave->name);
    return -EIO;
  }

  // Set up syncs.  This is needed because the ECT60 (at least)
  // doesn't map all of the PDOs that we need, so we need to set up
  // our own mappings.
  lcec_syncs_t *syncs = lcec_cia402_init_sync(slave, options);
  lcec_cia402_add_output_sync(slave, syncs, options);
  // No output PDOs right now.

  lcec_cia402_add_input_sync(slave, syncs, options);

  // Don't add extra PDOs on the ECR60x2 or ECT60x2, as they only have
  // 2 PDOs available and needs both for core functionality.
  if (!(slave->flags & F_NOEXTRAS)) {
    // On 2-axis devices, we need to make sure that we don't overwrite
    // the second axis's PDO here.
    int address = 0x1a00 + (options->channels * options->pdo_increment);
    lcec_syncs_add_pdo_info(syncs, address);

    for (channel = 0; channel < options->channels; channel++) {
      int base = 0x2000 + 0x800 * channel;
      lcec_syncs_add_pdo_entry(syncs, base + 0x0e, 0x00, 16);  // alarm codes
      lcec_syncs_add_pdo_entry(syncs, base + 0x0f, 0x00, 16);  // status codes
      lcec_syncs_add_pdo_entry(syncs, base + 0x48, 0x00, 16);  // Current bus voltage
    }
  }

  slave->sync_info = &syncs->syncs[0];

  hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);

  for (int channel = 0; channel < options->channels; channel++) {
    hal_data->cia402->channels[channel] = lcec_cia402_register_channel(slave, 0x6000 + 0x800 * channel, options->channel[channel]);
  }

  if (!(slave->flags & F_NOEXTRAS)) {
    // Register rtec-specific PDOs.
    lcec_pdo_init(slave, 0x200e, 0, &hal_data->alarm_code_os, NULL);
    lcec_pdo_init(slave, 0x200f, 0, &hal_data->status_code_os, NULL);
    lcec_pdo_init(slave, 0x2048, 0, &hal_data->voltage_os, NULL);

    // export rtec-specific pins
    if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, slave->master->name, slave->name)) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "pin registration failure for slave %s.%s\n", master->name, slave->name);
      return err;
    }
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

  if (!(slave->flags & F_NOEXTRAS)) {
    *(hal_data->alarm_code) = EC_READ_U16(&pd[hal_data->alarm_code_os]);
    *(hal_data->status_code) = EC_READ_U16(&pd[hal_data->status_code_os]);
    *(hal_data->voltage) = EC_READ_U16(&pd[hal_data->voltage_os]) / 100.0;
  }
  lcec_cia402_read_all(slave, hal_data->cia402);
}

static void lcec_rtec_write(lcec_slave_t *slave, long period) {
  lcec_rtec_data_t *hal_data = (lcec_rtec_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_write_all(slave, hal_data->cia402);
}
