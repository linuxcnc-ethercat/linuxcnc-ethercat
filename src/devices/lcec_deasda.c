//
//    Copyright (C) 2014 Sascha Ittner <sascha.ittner@modusoft.de>
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
/// @brief Driver for Delta ASDA servo controllers

#include "lcec_deasda.h"

#include "../lcec.h"
#include "lcec_class_dout.h"
#include "lcec_class_enc.h"

#define FLAG_LOWRES_ENC  1 << 0  // Device uses low res encoder as default
#define FLAG_HIGHRES_ENC 1 << 1  // Device uses high res encoder as default x3 series

#define LCEC_DESDA_MODPARAM_OPERATIONMODE 0

#define DEASDA_PULSES_PER_REV_DEFLT_LOWRES  (1280000)   // this is the default value for A2 (default value)
#define DEASDA_PULSES_PER_REV_DEFLT_HIGHRES (16777216)  // this is the default value for A3

#define DEASDA_RPM_FACTOR (0.1)
#define DEASDA_RPM_RCPT   (1.0 / DEASDA_RPM_FACTOR)
#define DEASDA_RPM_MUL    (60.0)
#define DEASDA_RPM_DIV    (1.0 / 60.0)

#define DEASDA_FAULT_AUTORESET_CYCLES  100
#define DEASDA_FAULT_AUTORESET_RETRIES 3

#define DEASDA_OPMODE_CSP 8
#define DEASDA_OPMODE_CSV 9

static int lcec_deasda_init(int comp_id, lcec_slave_t *slave);

static const lcec_modparam_desc_t lcec_deasda_modparams[] = {
    {"opmode", LCEC_DESDA_MODPARAM_OPERATIONMODE, MODPARAM_TYPE_STRING},
    {NULL},
};

typedef struct {
  char *name;      // Mode type name
  uint16_t value;  // Which value needs to be set in 0x6060:00 to enable this mode
} drive_operationmodes_t;

static const drive_operationmodes_t drive_operationmodes[] = {
    {"CSV", DEASDA_OPMODE_CSV},
    {"CSP", DEASDA_OPMODE_CSP},
    {NULL},
};

// Note that DeASDA refers to A2-E series of drives and is deliberatly not refering to A2 in its name to ensure compatability with legace
// configurations.
static lcec_typelist_t types[] = {
    {"DeASDA", LCEC_DELTA_VID, 0x10305070, 0, NULL, lcec_deasda_init, lcec_deasda_modparams, FLAG_LOWRES_ENC},
    {"DeASDA3", LCEC_DELTA_VID, 0x00006010, 0, NULL, lcec_deasda_init, lcec_deasda_modparams, FLAG_HIGHRES_ENC},
    {"DeASDB3", LCEC_DELTA_VID, 0x00006080, 0, NULL, lcec_deasda_init, lcec_deasda_modparams, FLAG_HIGHRES_ENC},
    {NULL},
};

ADD_TYPES(types);

typedef struct {
  hal_float_t *vel_fb;
  hal_float_t *vel_fb_rpm;
  hal_float_t *vel_fb_rpm_abs;
  hal_float_t *vel_rpm;
  hal_bit_t *ready;
  hal_bit_t *switched_on;
  hal_bit_t *oper_enabled;
  hal_bit_t *fault;
  hal_bit_t *volt_enabled;
  hal_bit_t *quick_stoped;
  hal_bit_t *on_disabled;
  hal_bit_t *warning;
  hal_bit_t *remote;
  hal_bit_t *at_speed;
  hal_bit_t *limit_active;
  hal_bit_t *zero_speed;
  hal_bit_t *switch_on;
  hal_bit_t *enable_volt;
  hal_bit_t *quick_stop;
  hal_bit_t *enable;
  hal_bit_t *fault_reset;
  hal_bit_t *halt;
  hal_u32_t *operation_mode;
  hal_float_t *cmd_value;

  hal_float_t pos_scale;
  hal_float_t extenc_scale;
  hal_u32_t pprev;
  hal_u32_t fault_autoreset_cycles;
  hal_u32_t fault_autoreset_retries;

  hal_float_t *torque;
  hal_bit_t *neg_lim_switch;
  hal_bit_t *pos_lim_switch;
  hal_bit_t *home_switch;
  hal_bit_t *di_1;
  hal_bit_t *di_2;
  hal_bit_t *di_3;
  hal_bit_t *di_4;
  hal_bit_t *di_5;
  hal_bit_t *di_6;
  hal_bit_t *di_7;

  lcec_class_enc_data_t enc;
  lcec_class_enc_data_t extenc;

  hal_float_t pos_scale_old;
  double pos_scale_rcpt;

  unsigned int status_pdo_os;
  unsigned int currpos_pdo_os;
  unsigned int currvel_pdo_os;
  unsigned int extenc_pdo_os;
  unsigned int control_pdo_os;
  unsigned int cmdvalue_pdo_os;
  unsigned int divalue_pdo_os;
  unsigned int torque_pdo_os;

  hal_bit_t last_switch_on;
  hal_bit_t internal_fault;

  hal_u32_t fault_reset_retry;
  hal_u32_t fault_reset_state;
  hal_u32_t fault_reset_cycle;

  lcec_class_dout_channels_t *dout;
} lcec_deasda_data_t;

static const lcec_pindesc_t slave_pins[] = {
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_deasda_data_t, vel_fb), "%s.%s.%s.srv-vel-fb"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_deasda_data_t, vel_fb_rpm), "%s.%s.%s.srv-vel-fb-rpm"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_deasda_data_t, vel_fb_rpm_abs), "%s.%s.%s.srv-vel-fb-rpm-abs"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_deasda_data_t, vel_rpm), "%s.%s.%s.srv-vel-rpm"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, ready), "%s.%s.%s.srv-ready"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, switched_on), "%s.%s.%s.srv-switched-on"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, oper_enabled), "%s.%s.%s.srv-oper-enabled"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, fault), "%s.%s.%s.srv-fault"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, volt_enabled), "%s.%s.%s.srv-volt-enabled"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, quick_stoped), "%s.%s.%s.srv-quick-stoped"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, on_disabled), "%s.%s.%s.srv-on-disabled"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, warning), "%s.%s.%s.srv-warning"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, remote), "%s.%s.%s.srv-remote"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, at_speed), "%s.%s.%s.srv-at-speed"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, limit_active), "%s.%s.%s.srv-limit-active"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, zero_speed), "%s.%s.%s.srv-zero-speed"},
    {HAL_BIT, HAL_IN, offsetof(lcec_deasda_data_t, switch_on), "%s.%s.%s.srv-switch-on"},
    {HAL_BIT, HAL_IN, offsetof(lcec_deasda_data_t, enable_volt), "%s.%s.%s.srv-enable-volt"},
    {HAL_BIT, HAL_IN, offsetof(lcec_deasda_data_t, quick_stop), "%s.%s.%s.srv-quick-stop"},
    {HAL_BIT, HAL_IN, offsetof(lcec_deasda_data_t, enable), "%s.%s.%s.srv-enable"},
    {HAL_BIT, HAL_IN, offsetof(lcec_deasda_data_t, fault_reset), "%s.%s.%s.srv-fault-reset"},
    {HAL_BIT, HAL_IN, offsetof(lcec_deasda_data_t, halt), "%s.%s.%s.srv-halt"},
    {HAL_U32, HAL_OUT, offsetof(lcec_deasda_data_t, operation_mode), "%s.%s.%s.srv-operation-mode"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_deasda_data_t, torque),
        "%s.%s.%s.srv-torque-rel"},  // relative value (5) - hence current would be redundant
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, di_1), "%s.%s.%s.din-1"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, di_2), "%s.%s.%s.din-2"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, di_3), "%s.%s.%s.din-3"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, di_4), "%s.%s.%s.din-4"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, di_5), "%s.%s.%s.din-5"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, di_6), "%s.%s.%s.din-6"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, di_7), "%s.%s.%s.din-7"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, neg_lim_switch), "%s.%s.%s.din-neg-lim"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, pos_lim_switch), "%s.%s.%s.din-pos-lim"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, home_switch), "%s.%s.%s.din-home"},

    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static const lcec_pindesc_t slave_pins_csv[] = {
    {HAL_FLOAT, HAL_IN, offsetof(lcec_deasda_data_t, cmd_value), "%s.%s.%s.srv-vel-cmd"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static const lcec_pindesc_t slave_pins_csp[] = {
    {HAL_FLOAT, HAL_IN, offsetof(lcec_deasda_data_t, cmd_value), "%s.%s.%s.srv-pos-cmd"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

// Exposed parameters are identical for both drives and modes
static const lcec_pindesc_t slave_params[] = {
    {HAL_FLOAT, HAL_RW, offsetof(lcec_deasda_data_t, pos_scale), "%s.%s.%s.pos-scale"},
    {HAL_FLOAT, HAL_RW, offsetof(lcec_deasda_data_t, extenc_scale), "%s.%s.%s.extenc-scale"},
    {HAL_U32, HAL_RW, offsetof(lcec_deasda_data_t, pprev), "%s.%s.%s.srv-pulses-per-rev"},
    {HAL_U32, HAL_RW, offsetof(lcec_deasda_data_t, fault_autoreset_cycles), "%s.%s.%s.srv-fault-autoreset-cycles"},
    {HAL_U32, HAL_RW, offsetof(lcec_deasda_data_t, fault_autoreset_retries), "%s.%s.%s.srv-fault-autoreset-retries"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static ec_pdo_entry_info_t lcec_deasda_in[] = {
    {0x6041, 0x00, 16},  // Status Word
    {0x606C, 0x00, 32},  // Current Velocity
    {0x6064, 0x00, 32},  // Current Position
    {0x2511, 0x00, 32},  // External encoder
    {0x6077, 0x00, 16},  // Torque
    {0x60FD, 0x00, 32},  // Digital Inputs

};

static ec_pdo_entry_info_t lcec_deasda_out_csv[] = {
    {0x6040, 0x00, 16},  // Control Word
    {0x60FF, 0x00, 32},  // Target Velocity
    {0x60fe, 0x01, 32},  // digital output
};

static ec_pdo_entry_info_t lcec_deasda_out_csp[] = {
    {0x6040, 0x00, 16},  // Control Word
    {0x607A, 0x00, 32},  // Target Position
    {0x60fe, 0x01, 32},  // digital output
};

static ec_pdo_info_t lcec_deasda_pdos_out_csv[] = {
    {0x1602, 3, lcec_deasda_out_csv},
};

static ec_pdo_info_t lcec_deasda_pdos_out_csp[] = {
    {0x1602, 3, lcec_deasda_out_csp},
};

static ec_pdo_info_t lcec_deasda_pdos_in[] = {
    {0x1a02, 6, lcec_deasda_in},
};

static ec_sync_info_t lcec_deasda_syncs_csv[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT, 0, NULL},
    {2, EC_DIR_OUTPUT, 1, lcec_deasda_pdos_out_csv},
    {3, EC_DIR_INPUT, 1, lcec_deasda_pdos_in},
    {0xff},
};

static ec_sync_info_t lcec_deasda_syncs_csp[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT, 0, NULL},
    {2, EC_DIR_OUTPUT, 1, lcec_deasda_pdos_out_csp},
    {3, EC_DIR_INPUT, 1, lcec_deasda_pdos_in},
    {0xff},
};

static void lcec_deasda_check_scales(lcec_deasda_data_t *hal_data);

static void lcec_deasda_read(lcec_slave_t *slave, long period);
static void lcec_deasda_write_csv(lcec_slave_t *slave, long period);
static void lcec_deasda_write_csp(lcec_slave_t *slave, long period);

static const drive_operationmodes_t *drive_opmode(char *drivemode);

static int lcec_deasda_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_deasda_data_t *hal_data;
  int err;
  uint32_t tu;
  int8_t ti;
  drive_operationmodes_t const *driveopmode;
  static uint16_t operationmode;

  uint64_t flags;
  flags = slave->flags;

  // Determine Operation Mode (modParam opmode) as this defines everything else
  LCEC_CONF_MODPARAM_VAL_T *pval;
  rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "  - checking modparam opmode for %s \n", slave->name);
  pval = lcec_modparam_get(slave, LCEC_DESDA_MODPARAM_OPERATIONMODE);
  if (pval != NULL) {
    rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "    - found opmode param for %s \n", slave->name);

    driveopmode = drive_opmode(pval->str);

    if (driveopmode != NULL) {
      rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "    - setting mode for %s to %d\n", slave->name, driveopmode->value);
      operationmode = driveopmode->value;
    } else {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "unknown mode \"%s\" for slave %s.%s !\n", pval->str, master->name, slave->name);
      return -1;
    }
  } else {
    // This would be the case when modparam mode has not been set ==> back to CSV
    rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "    - no opmode param for %s \n found. Defaulting to CSV.", slave->name);
    operationmode = DEASDA_OPMODE_CSV;
  }

  // initialize callbacks
  slave->proc_read = lcec_deasda_read;

  if (operationmode == DEASDA_OPMODE_CSV) {
    slave->proc_write = lcec_deasda_write_csv;
  } else if (operationmode == DEASDA_OPMODE_CSP) {
    slave->proc_write = lcec_deasda_write_csp;
  }
  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_deasda_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_deasda_data_t));
  slave->hal_data = hal_data;

  // Set up digital outputs
  hal_data->dout = lcec_dout_allocate_channels(4);
  hal_data->dout->channels[0] = lcec_dout_register_channel_packed(slave, 0x60f3, 0x01, 16, "dout-d01");
  hal_data->dout->channels[1] = lcec_dout_register_channel_packed(slave, 0x60f3, 0x01, 17, "dout-d02");
  hal_data->dout->channels[2] = lcec_dout_register_channel_packed(slave, 0x60f3, 0x01, 18, "dout-d03");
  hal_data->dout->channels[3] = lcec_dout_register_channel_packed(slave, 0x60f3, 0x01, 19, "dout-d04");

  if (lcec_write_sdo32(slave, 0x60fe, 0x02, 0x000f0000) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to configure slave %s.%s sdo for enabling digital output ports 1-4\n", master->name,
        slave->name);
    return -1;
  }

  // set to 0x6060 to requested mode (CSV, CSP)
  if (lcec_write_sdo8(slave, 0x6060, 0x00, operationmode) != 0) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo to op mode %d\n", master->name, slave->name, operationmode);
    return -1;
  }

  // set interpolation time period
  tu = master->app_time_period;
  ti = -9;

  while ((tu % 10) == 0 || tu > 255) {
    tu /= 10;
    ti++;
  }
  if (lcec_write_sdo8(slave, 0x60C2, 0x01, (uint8_t)tu) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo ipol time period units\n", master->name, slave->name);
    return -1;
  }
  if (lcec_write_sdo8(slave, 0x60C2, 0x02, ti) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo ipol time period index\n", master->name, slave->name);
    return -1;
  }

  if (operationmode == DEASDA_OPMODE_CSV) {
    // initialize sync info
    slave->sync_info = lcec_deasda_syncs_csv;
    // initialize POD entries
    lcec_pdo_init(slave, 0x6041, 0x00, &hal_data->status_pdo_os, NULL);
    lcec_pdo_init(slave, 0x606C, 0x00, &hal_data->currvel_pdo_os, NULL);
    lcec_pdo_init(slave, 0x6064, 0x00, &hal_data->currpos_pdo_os, NULL);
    lcec_pdo_init(slave, 0x2511, 0x00, &hal_data->extenc_pdo_os, NULL);
    lcec_pdo_init(slave, 0x6040, 0x00, &hal_data->control_pdo_os, NULL);
    lcec_pdo_init(slave, 0x60FF, 0x00, &hal_data->cmdvalue_pdo_os, NULL);
    lcec_pdo_init(slave, 0x6077, 0x00, &hal_data->torque_pdo_os, NULL);
    lcec_pdo_init(slave, 0x60FD, 0x00, &hal_data->divalue_pdo_os, NULL);

    // export pins common
    if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;

    // export pins specific
    if ((err = lcec_pin_newf_list(hal_data, slave_pins_csv, LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;

  } else if (operationmode == DEASDA_OPMODE_CSP) {
    // initialize sync info
    slave->sync_info = lcec_deasda_syncs_csp;
    // initialize POD entries
    lcec_pdo_init(slave, 0x6041, 0x00, &hal_data->status_pdo_os, NULL);
    lcec_pdo_init(slave, 0x606C, 0x00, &hal_data->currvel_pdo_os, NULL);
    lcec_pdo_init(slave, 0x6064, 0x00, &hal_data->currpos_pdo_os, NULL);
    lcec_pdo_init(slave, 0x2511, 0x00, &hal_data->extenc_pdo_os, NULL);
    lcec_pdo_init(slave, 0x6040, 0x00, &hal_data->control_pdo_os, NULL);
    lcec_pdo_init(slave, 0x607A, 0x00, &hal_data->cmdvalue_pdo_os, NULL);
    lcec_pdo_init(slave, 0x6077, 0x00, &hal_data->torque_pdo_os, NULL);
    lcec_pdo_init(slave, 0x60FD, 0x00, &hal_data->divalue_pdo_os, NULL);

    // export pins common
    if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;
    // export pins specific
    if ((err = lcec_pin_newf_list(hal_data, slave_pins_csp, LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;
  }

  *(hal_data->operation_mode) = operationmode;
  // export parameters
  if ((err = lcec_param_newf_list(hal_data, slave_params, LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;

  // init subclasses for encoders
  if ((err = class_enc_init(slave, &hal_data->enc, 32, "enc")) != 0) return err;
  if ((err = class_enc_init(slave, &hal_data->extenc, 32, "extenc")) != 0) return err;

  // initialize variables
  hal_data->pos_scale = 1.0;
  hal_data->extenc_scale = 1.0;
  hal_data->fault_autoreset_cycles = DEASDA_FAULT_AUTORESET_CYCLES;
  hal_data->fault_autoreset_retries = DEASDA_FAULT_AUTORESET_RETRIES;
  hal_data->pos_scale_old = hal_data->pos_scale + 1.0;
  hal_data->pos_scale_rcpt = 1.0;

  // change based on FLAG_LOWRES_ENC/FLAG_HIGHRES_ENC
  if (flags & FLAG_LOWRES_ENC) {
    hal_data->pprev = DEASDA_PULSES_PER_REV_DEFLT_LOWRES;
    rtapi_print_msg(
        RTAPI_MSG_DBG, LCEC_MSG_PFX "Setting pprev to Low Res Encoder (1,280,000) for device %s.%s.\n", master->name, slave->name);
  } else if (flags & FLAG_HIGHRES_ENC) {
    hal_data->pprev = DEASDA_PULSES_PER_REV_DEFLT_HIGHRES;
    rtapi_print_msg(
        RTAPI_MSG_DBG, LCEC_MSG_PFX "Setting pprev to High Res Encoder (16,777,216) for device %s.%s.\n", master->name, slave->name);
  }

  // TODO: Add additional registers here if avialalbe: e.g. DIDO based on servo type FLAG_SERVO_X2/FLAG_SERVO_X3

  hal_data->last_switch_on = 0;
  hal_data->internal_fault = 0;

  hal_data->fault_reset_retry = 0;
  hal_data->fault_reset_state = 0;
  hal_data->fault_reset_cycle = 0;

  return 0;
}

void lcec_deasda_check_scales(lcec_deasda_data_t *hal_data) {
  // check for change in scale value
  if (hal_data->pos_scale != hal_data->pos_scale_old) {
    // scale value has changed, test and update it
    if ((hal_data->pos_scale < 1e-20) && (hal_data->pos_scale > -1e-20)) hal_data->pos_scale = 1.0;

    // save new scale to detect future changes
    hal_data->pos_scale_old = hal_data->pos_scale;
    // we actually want the reciprocal
    hal_data->pos_scale_rcpt = 1.0 / hal_data->pos_scale;
  }
}

static void lcec_deasda_read(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_deasda_data_t *hal_data = (lcec_deasda_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t status;
  uint32_t status_di;
  int32_t speed_raw;
  double rpm;
  uint32_t pos_cnt;
  // wait for slave to be operational
  if (!slave->state.operational) {
    *(hal_data->ready) = 0;
    *(hal_data->switched_on) = 0;
    *(hal_data->oper_enabled) = 0;
    *(hal_data->fault) = 1;
    *(hal_data->volt_enabled) = 0;
    *(hal_data->quick_stoped) = 0;
    *(hal_data->on_disabled) = 0;
    *(hal_data->warning) = 0;
    *(hal_data->remote) = 0;
    *(hal_data->at_speed) = 0;
    *(hal_data->limit_active) = 0;
    *(hal_data->zero_speed) = 0;
    return;
  }

  // check for change in scale value
  lcec_deasda_check_scales(hal_data);

  // read status word
  status = EC_READ_U16(&pd[hal_data->status_pdo_os]);
  *(hal_data->ready) = (status >> 0) & 0x01;
  *(hal_data->switched_on) = (status >> 1) & 0x01;
  *(hal_data->oper_enabled) = (status >> 2) & 0x01;
  hal_data->internal_fault = (status >> 3) & 0x01;
  *(hal_data->volt_enabled) = (status >> 4) & 0x01;
  *(hal_data->quick_stoped) = !((status >> 5) & 0x01);
  *(hal_data->on_disabled) = (status >> 6) & 0x01;
  *(hal_data->warning) = (status >> 7) & 0x01;
  *(hal_data->remote) = (status >> 9) & 0x01;
  *(hal_data->at_speed) = (status >> 10) & 0x01;
  *(hal_data->limit_active) = (status >> 11) & 0x01;
  *(hal_data->zero_speed) = (status >> 12) & 0x01;

  // clear pending fault reset if no fault
  if (!hal_data->internal_fault) hal_data->fault_reset_retry = 0;

  // generate gated fault
  if (hal_data->fault_reset_retry > 0) {
    if (hal_data->fault_reset_cycle < hal_data->fault_autoreset_cycles) {
      hal_data->fault_reset_cycle++;
    } else {
      hal_data->fault_reset_cycle = 0;
      hal_data->fault_reset_state = !hal_data->fault_reset_state;
      if (hal_data->fault_reset_state) {
        hal_data->fault_reset_retry--;
      }
    }
    *(hal_data->fault) = 0;
  } else {
    *(hal_data->fault) = hal_data->internal_fault;
  }

  // read current speed
  speed_raw = EC_READ_S32(&pd[hal_data->currvel_pdo_os]);
  rpm = (double)speed_raw * DEASDA_RPM_FACTOR;
  *(hal_data->vel_fb_rpm) = rpm;
  *(hal_data->vel_fb_rpm_abs) = fabs(rpm);
  *(hal_data->vel_fb) = rpm * DEASDA_RPM_DIV * hal_data->pos_scale;

  // update raw position counter
  pos_cnt = EC_READ_U32(&pd[hal_data->currpos_pdo_os]);
  class_enc_update(&hal_data->enc, hal_data->pprev, hal_data->pos_scale, pos_cnt, 0, 0);

  // update external encoder counter
  pos_cnt = EC_READ_U32(&pd[hal_data->extenc_pdo_os]);
  class_enc_update(&hal_data->extenc, 1, hal_data->extenc_scale, pos_cnt, 0, 0);

  // read current
  *(hal_data->torque) = (double)EC_READ_S16(&pd[hal_data->torque_pdo_os]) * 0.1;

  // read DI status word
  status_di = EC_READ_U32(&pd[hal_data->divalue_pdo_os]);

  *(hal_data->neg_lim_switch) = (status_di >> 0) & 0x01;
  *(hal_data->pos_lim_switch) = (status_di >> 1) & 0x01;
  *(hal_data->home_switch) = (status_di >> 2) & 0x01;
  *(hal_data->di_1) = (status_di >> 16) & 0x01;
  *(hal_data->di_2) = (status_di >> 17) & 0x01;
  *(hal_data->di_3) = (status_di >> 18) & 0x01;
  *(hal_data->di_4) = (status_di >> 19) & 0x01;
  *(hal_data->di_5) = (status_di >> 20) & 0x01;
  *(hal_data->di_6) = (status_di >> 21) & 0x01;
  *(hal_data->di_7) = (status_di >> 22) & 0x01;
}

static void lcec_deasda_write_csv(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_deasda_data_t *hal_data = (lcec_deasda_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t control;
  double speed_raw;
  int switch_on_edge;

  // do digital outputs
  lcec_dout_write_all(slave, hal_data->dout);

  // check for enable edge
  switch_on_edge = *(hal_data->switch_on) && !hal_data->last_switch_on;
  hal_data->last_switch_on = *(hal_data->switch_on);

  // check for autoreset
  if (hal_data->fault_autoreset_retries > 0 && hal_data->fault_autoreset_cycles > 0 && switch_on_edge && hal_data->internal_fault) {
    hal_data->fault_reset_retry = hal_data->fault_autoreset_retries;
    hal_data->fault_reset_state = 1;
    hal_data->fault_reset_cycle = 0;
  }

  // check for change in scale value
  lcec_deasda_check_scales(hal_data);

  // write dev ctrl
  control = 0;

  if (*(hal_data->enable_volt)) control |= (1 << 1);
  if (!*(hal_data->quick_stop)) control |= (1 << 2);
  if (*(hal_data->fault_reset)) control |= (1 << 7);
  if (*(hal_data->halt)) control |= (1 << 8);

  if (hal_data->fault_reset_retry > 0) {
    if (hal_data->fault_reset_state) control |= (1 << 7);
  } else {
    if (*(hal_data->switch_on)) control |= (1 << 0);
    if (*(hal_data->enable) && *(hal_data->switched_on)) control |= (1 << 3);
  }
  EC_WRITE_U16(&pd[hal_data->control_pdo_os], control);

  // all of this is depeding on CSV/CSP
  // calculate rpm command
  *(hal_data->vel_rpm) = *(hal_data->cmd_value) * hal_data->pos_scale_rcpt * DEASDA_RPM_MUL;

  // set RPM
  speed_raw = *(hal_data->vel_rpm) * DEASDA_RPM_RCPT;
  if (speed_raw > (double)0x7fffffff) speed_raw = (double)0x7fffffff;
  if (speed_raw < (double)-0x7fffffff) speed_raw = (double)-0x7fffffff;

  EC_WRITE_S32(&pd[hal_data->cmdvalue_pdo_os], (int32_t)speed_raw);
}

static void lcec_deasda_write_csp(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_deasda_data_t *hal_data = (lcec_deasda_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t control;
  int32_t pos_puu;
  int switch_on_edge;

  // do digital outputs
  lcec_dout_write_all(slave, hal_data->dout);

  // check for enable edge
  switch_on_edge = *(hal_data->switch_on) && !hal_data->last_switch_on;
  hal_data->last_switch_on = *(hal_data->switch_on);

  // check for autoreset
  if (hal_data->fault_autoreset_retries > 0 && hal_data->fault_autoreset_cycles > 0 && switch_on_edge && hal_data->internal_fault) {
    hal_data->fault_reset_retry = hal_data->fault_autoreset_retries;
    hal_data->fault_reset_state = 1;
    hal_data->fault_reset_cycle = 0;
  }

  // check for change in scale value
  lcec_deasda_check_scales(hal_data);

  // write dev ctrl
  control = 0;
  if (*(hal_data->enable_volt)) control |= (1 << 1);
  if (!*(hal_data->quick_stop)) control |= (1 << 2);
  if (*(hal_data->fault_reset)) control |= (1 << 7);
  if (*(hal_data->halt)) control |= (1 << 8);

  if (hal_data->fault_reset_retry > 0) {
    if (hal_data->fault_reset_state) control |= (1 << 7);
  } else {
    if (*(hal_data->switch_on)) control |= (1 << 0);
    if (*(hal_data->enable) && *(hal_data->switched_on)) control |= (1 << 3);
  }
  EC_WRITE_U16(&pd[hal_data->control_pdo_os], control);

  // ASDA Drives expect target Position in PUU (Pulse per User Unit)
  // See https://www.deltaww.com/en-US/FAQ/228
  // Calculation accordingly based on pprev and pos_scale (i.e. pitch of ball screw)
  pos_puu = (int32_t)(*(hal_data->cmd_value) * hal_data->pprev / hal_data->pos_scale);
  EC_WRITE_S32(&pd[hal_data->cmdvalue_pdo_os], pos_puu);
}

// Match the drive mode configuration in modparams and return the settings for that particular operational mode.
// the value is then used both for setting the mode and to differnetiate between CSV (0) and CSP
static const drive_operationmodes_t *drive_opmode(char *drivemode) {
  drive_operationmodes_t const *modes;

  for (modes = drive_operationmodes; modes != NULL; modes++) {
    if (!strcasecmp(drivemode, modes->name)) return modes;
  }

  return NULL;
}
