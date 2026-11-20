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
#include "lcec_class_cia402.h"
#include "lcec_class_dout.h"
#include "lcec_class_enc.h"

#define FLAG_LOWRES_ENC  1 << 0  // Device uses low res encoder as default
#define FLAG_HIGHRES_ENC 1 << 1  // Device uses high res encoder as default x3 series
#define FLAG_DOUT        1 << 2  // Device has digital outputs
#define FLAG_LIMIT_BITS  1 << 3  // Device reports the positive/negative limit in statusword bits 14 and 15

#define M_OPERATIONMODE 0
#define M_DIGITALOUT    1
#define M_ENABLEHM      2
#define M_HOMEMETHOD    3
#define M_HOMEOFFSET    4
#define M_EGEARRATIO    5
#define M_HOMETORQUELVL 6
#define M_HOMETORQUETM  7

#define DEASDA_PULSES_PER_REV_DEFLT_LOWRES  (1280000)   // this is the default value for A2 (default value)
#define DEASDA_PULSES_PER_REV_DEFLT_HIGHRES (16777216)  // this is the default value for A3

#define DEASDA_RPM_FACTOR (0.1)
#define DEASDA_RPM_RCPT   (1.0 / DEASDA_RPM_FACTOR)
#define DEASDA_RPM_MUL    (60.0)
#define DEASDA_RPM_DIV    (1.0 / 60.0)

// Zero speed detection level in 0.1 rpm, if it can't be read from the drive (0x606F, drive default).
#define DEASDA_ZERO_SPEED_DEFLT 100

#define DEASDA_FAULT_AUTORESET_CYCLES  100
#define DEASDA_FAULT_AUTORESET_RETRIES 3

#define DEASDA_OPMODE_HM  6
#define DEASDA_OPMODE_CSP 8
#define DEASDA_OPMODE_CSV 9

// Statusword (0x6041) and controlword (0x6040) bits that change meaning in homing mode.
#define DEASDA_STATUS_HOMING_COMPLETE (1 << 12)  // "Homing is complete"
#define DEASDA_STATUS_HOMING_ERROR    (1 << 13)  // "Homing error"
#define DEASDA_CONTROL_HOMING_START   (1 << 4)   // "Homing (rising-edge triggered)"

// If the drive keeps reporting "homing is complete" for this long after the start bit was sent, without ever
// reporting "not complete" in between, accept the homing run as done.  This covers methods that finish
// within a single cycle (i.e. method 35, "current position is the origin").
#define DEASDA_HOMING_DONE_TIMEOUT_NS 500000000LL

static int lcec_deasda_init(int comp_id, lcec_slave_t *slave);

// modParams of all drives
#define DEASDA_MODPARAMS_COMMON                                                                                                      \
  {"enableHM", M_ENABLEHM, MODPARAM_TYPE_BIT, "false", "Enable homing mode (maps 0x6060/0x6061 and adds homing pins)"},             \
      {"homeMethod", M_HOMEMETHOD, MODPARAM_TYPE_S32, NULL, "Homing method (0x6098)"},                                               \
      {"homeOffset", M_HOMEOFFSET, MODPARAM_TYPE_S32, NULL, "Home offset in PUU (0x607C)"},                                          \
      {"eGearRatio", M_EGEARRATIO, MODPARAM_TYPE_STRING, NULL, "E-Gear ratio (0x6093), as numerator/denominator"}

// Settings for homing on a hard stop.  The ESI file of the A2 doesn't list these two objects, so they are
// only offered for the x3 drives.
#define DEASDA_MODPARAMS_HARDSTOP                                                                                               \
  {"homeTorqueLevel_pct", M_HOMETORQUELVL, MODPARAM_TYPE_U32, NULL, "Hard stop homing: torque level in % (P1.087, 0x2157)"},       \
      {"homeTorqueTime_ms", M_HOMETORQUETM, MODPARAM_TYPE_U32, NULL, "Hard stop homing: level reached timer in ms (P1.088, 0x2158)"}

static const lcec_modparam_desc_t lcec_deasda_modparams[] = {
    {"opmode", M_OPERATIONMODE, MODPARAM_TYPE_STRING, "CSP", "Operation mode, CSV or CSP"},
    {"enableDigitalOutput", M_DIGITALOUT, MODPARAM_TYPE_BIT, "true", "Enable digital output ports"},
    DEASDA_MODPARAMS_COMMON,
    {NULL},
};

static const lcec_modparam_desc_t lcec_deasda_modparams_a3[] = {
    {"opmode", M_OPERATIONMODE, MODPARAM_TYPE_STRING, "CSP", "Operation mode, CSV or CSP"},
    {"enableDigitalOutput", M_DIGITALOUT, MODPARAM_TYPE_BIT, "true", "Enable digital output ports"},
    DEASDA_MODPARAMS_COMMON,
    DEASDA_MODPARAMS_HARDSTOP,
    {NULL},
};

static const lcec_modparam_desc_t lcec_deasda_modparams_b3[] = {
    {"opmode", M_OPERATIONMODE, MODPARAM_TYPE_STRING, "CSP", "Operation mode, CSV or CSP"},
    DEASDA_MODPARAMS_COMMON,
    DEASDA_MODPARAMS_HARDSTOP,
    {NULL},
};

typedef struct {
  const char *name;  // Mode type name
  uint16_t value;    // Which value needs to be set in 0x6060:00 to enable this mode
} drive_operationmodes_t;

static const drive_operationmodes_t drive_operationmodes[] = {
    {"CSV", DEASDA_OPMODE_CSV},
    {"CSP", DEASDA_OPMODE_CSP},
    {NULL},
};

// Note that DeASDA refers to A2-E series of drives and is deliberatly not refering to A2 in its name to ensure compatability with legace
// configurations.
static lcec_typelist_t types[] = {
    {"DeASDA", LCEC_DELTA_VID, 0x10305070, 0, NULL, lcec_deasda_init, lcec_deasda_modparams, FLAG_LOWRES_ENC | FLAG_DOUT},
    {"DeASDA3", LCEC_DELTA_VID, 0x00006010, 0, NULL, lcec_deasda_init, lcec_deasda_modparams_a3,
        FLAG_HIGHRES_ENC | FLAG_DOUT | FLAG_LIMIT_BITS},
    {"DeASDB3", LCEC_DELTA_VID, 0x00006080, 0, NULL, lcec_deasda_init, lcec_deasda_modparams_b3, FLAG_HIGHRES_ENC | FLAG_LIMIT_BITS},
    {NULL},
};

ADD_TYPES(types);

/// @brief A setting of the drive that is not mapped: it is written through an SDO request when its pin changes.
typedef struct {
  ec_sdo_request_t *request;
  int bytes;
  int32_t confirmed;  ///< The value that the drive is known to hold.
  int32_t in_flight;  ///< The value of the write that is under way.
  int pending;        ///< A write is under way.
} lcec_deasda_sdo_setting_t;

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

  // homing mode (only exported with <modParam name="enableHM" value="true"/>)
  hal_bit_t *home;
  hal_bit_t *homed;
  hal_bit_t *homing;
  hal_bit_t *homing_error;
  hal_s32_t *opmode_display;
  hal_float_t *pos_fb;
  hal_float_t *pos_cmd;  // CSV only, in CSP the position command is cmd_value
  hal_s32_t *home_method;
  hal_u32_t *home_velocity_fast;
  hal_u32_t *home_velocity_slow;
  hal_u32_t *home_accel;

  lcec_param_float_t pos_scale;
  lcec_param_float_t extenc_scale;
  lcec_param_u32_t pprev;
  lcec_param_u32_t fault_autoreset_cycles;
  lcec_param_u32_t fault_autoreset_retries;

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
  unsigned int opmode_pdo_os;
  unsigned int opmode_display_pdo_os;

  int limit_bits;       // statusword bits 14/15 are the positive/negative limit
  uint16_t zero_speed_level;  // zero speed detection level of the drive [0.1 rpm]

  int hm_enabled;         // homing mode support is mapped and exported
  int8_t opmode_cyclic;   // configured cyclic mode (CSP or CSV)
  int8_t opmode_current;  // last value of 0x6061
  int home_start_sent;    // homing start bit was set in the last write cycle
  int home_seen_unhomed;  // drive reported "not complete" since the start bit was set
  int home_aborted;       // a fault ended the homing run; srv-home has to go off before the next one
  long long home_done_time;

  lcec_deasda_sdo_setting_t home_method_set;
  lcec_deasda_sdo_setting_t home_velocity_fast_set;
  lcec_deasda_sdo_setting_t home_velocity_slow_set;
  lcec_deasda_sdo_setting_t home_accel_set;

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

static const lcec_pindesc_t slave_pins_hm[] = {
    {HAL_BIT, HAL_IO, offsetof(lcec_deasda_data_t, home), "%s.%s.%s.srv-home"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, homed), "%s.%s.%s.srv-homed"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, homing), "%s.%s.%s.srv-homing"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_deasda_data_t, homing_error), "%s.%s.%s.srv-homing-error"},
    {HAL_S32, HAL_OUT, offsetof(lcec_deasda_data_t, opmode_display), "%s.%s.%s.srv-opmode-display"},
    {HAL_S32, HAL_IN, offsetof(lcec_deasda_data_t, home_method), "%s.%s.%s.srv-home-method"},
    {HAL_U32, HAL_IN, offsetof(lcec_deasda_data_t, home_velocity_fast), "%s.%s.%s.srv-home-velocity-fast"},
    {HAL_U32, HAL_IN, offsetof(lcec_deasda_data_t, home_velocity_slow), "%s.%s.%s.srv-home-velocity-slow"},
    {HAL_U32, HAL_IN, offsetof(lcec_deasda_data_t, home_accel), "%s.%s.%s.srv-home-accel"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

// Position feedback for joint.N.motor-pos-fb, which doesn't trip a following error while the drive is homing.
static const lcec_pindesc_t slave_pins_hm_csp[] = {
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_deasda_data_t, pos_fb), "%s.%s.%s.srv-pos-fb"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

// In CSV the drive only gets a velocity, so the position command of the joint needs its own pin.  It is only
// used to generate srv-pos-fb while the drive is homing.
static const lcec_pindesc_t slave_pins_hm_csv[] = {
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_deasda_data_t, pos_fb), "%s.%s.%s.srv-pos-fb"},
    {HAL_FLOAT, HAL_IN, offsetof(lcec_deasda_data_t, pos_cmd), "%s.%s.%s.srv-pos-cmd"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

// Exposed parameters are identical for both drives and modes
static const lcec_paramdesc_t slave_params[] = {
    {HAL_FLOAT, HAL_RW, offsetof(lcec_deasda_data_t, pos_scale), "%s.%s.%s.pos-scale"},
    {HAL_FLOAT, HAL_RW, offsetof(lcec_deasda_data_t, extenc_scale), "%s.%s.%s.extenc-scale"},
    {HAL_U32, HAL_RW, offsetof(lcec_deasda_data_t, pprev), "%s.%s.%s.srv-pulses-per-rev"},
    {HAL_U32, HAL_RW, offsetof(lcec_deasda_data_t, fault_autoreset_cycles), "%s.%s.%s.srv-fault-autoreset-cycles"},
    {HAL_U32, HAL_RW, offsetof(lcec_deasda_data_t, fault_autoreset_retries), "%s.%s.%s.srv-fault-autoreset-retries"},
    {HAL_TYPE_UNSPECIFIED},
};

static void lcec_deasda_check_scales(lcec_deasda_data_t *hal_data);

static void lcec_deasda_read(lcec_slave_t *slave, long period);
static void lcec_deasda_write_csv(lcec_slave_t *slave, long period);
static void lcec_deasda_write_csp(lcec_slave_t *slave, long period);

static const drive_operationmodes_t *drive_opmode(char *drivemode);

/// @brief Set up a `lcec_deasda_sdo_setting_t`.
///
/// @param value The value that the drive holds right now.
static int lcec_deasda_sdo_setting_init(
    lcec_slave_t *slave, lcec_deasda_sdo_setting_t *setting, uint16_t index, uint8_t subindex, int bytes, int32_t value) {
  setting->request = ecrt_slave_config_create_sdo_request(slave->config, index, subindex, bytes);
  if (setting->request == NULL) return -1;

  setting->bytes = bytes;
  setting->confirmed = value;
  setting->pending = 0;
  return 0;
}

static int lcec_deasda_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_deasda_data_t *hal_data;
  int err;
  uint32_t tu;
  int8_t ti;
  drive_operationmodes_t const *driveopmode;
  static uint16_t operationmode;
  lcec_syncs_t *syncs;
  uint64_t flags;
  int enable_dout = slave->flags & FLAG_DOUT;
  int enable_hm = 0;
  lcec_ratio egear = {1, 1};
  flags = slave->flags;

  syncs = LCEC_HAL_ALLOCATE(lcec_syncs_t);

  // Determine Operation Mode (modParam opmode) as this defines everything else
  LCEC_CONF_MODPARAM_VAL_T *pval;
  rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "  - checking modparam opmode for %s \n", slave->name);
  pval = lcec_modparam_get(slave, M_OPERATIONMODE);
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

  pval = lcec_modparam_get(slave, M_DIGITALOUT);
  if (pval) {
    enable_dout = pval->bit;
  }

  pval = lcec_modparam_get(slave, M_ENABLEHM);
  if (pval) {
    enable_hm = pval->bit;
  }

  // Set up PDO sync configuration
  lcec_syncs_init(slave, syncs);
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DEFAULT);

  // Set up output PDO syncs
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
  lcec_syncs_add_pdo_info(syncs, 0x1602);
  lcec_syncs_add_pdo_entry(syncs, 0x6040, 0, 16);  // Control word

  // We could actually map both of these at the same time without
  // problems, but for compabilities's sake, I don't want to change it
  // right now.
  if (operationmode == DEASDA_OPMODE_CSV) {
    lcec_syncs_add_pdo_entry(syncs, 0x60ff, 0, 32);  // Target Velocity
  } else {
    lcec_syncs_add_pdo_entry(syncs, 0x607a, 0, 32);  // Target Position
  }

  // Homing needs to switch 0x6060 between homing mode and the cyclic mode at runtime, so it has to be mapped.
  if (enable_hm) {
    lcec_syncs_add_pdo_entry(syncs, 0x6060, 0, 8);  // Modes of operation
  }

  // Only add digital outs on models that actually have the hardware.
  if (enable_dout) {
    lcec_syncs_add_pdo_entry(syncs, 0x60fe, 1, 32);  // Digital outputs
  }

  // Set up input PDO syncs
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DEFAULT);
  lcec_syncs_add_pdo_info(syncs, 0x1a02);
  lcec_syncs_add_pdo_entry(syncs, 0x6041, 0, 16);  // Status word
  lcec_syncs_add_pdo_entry(syncs, 0x606c, 0, 32);  // Current velocity
  lcec_syncs_add_pdo_entry(syncs, 0x6064, 0, 32);  // Current position
  lcec_syncs_add_pdo_entry(syncs, 0x2511, 0, 32);  // External encoder
  lcec_syncs_add_pdo_entry(syncs, 0x6077, 0, 16);  // Current torque
  lcec_syncs_add_pdo_entry(syncs, 0x60fd, 0, 32);  // Digital inputs
  if (enable_hm) {
    lcec_syncs_add_pdo_entry(syncs, 0x6061, 0, 8);  // Modes of operation display
  }
  slave->sync_info = &syncs->syncs[0];

  // initialize callbacks
  slave->proc_read = lcec_deasda_read;

  if (operationmode == DEASDA_OPMODE_CSV) {
    slave->proc_write = lcec_deasda_write_csv;
  } else if (operationmode == DEASDA_OPMODE_CSP) {
    slave->proc_write = lcec_deasda_write_csp;
  }
  // alloc hal memory
  hal_data = LCEC_HAL_ALLOCATE(lcec_deasda_data_t);
  slave->hal_data = hal_data;
  hal_data->hm_enabled = enable_hm;
  hal_data->limit_bits = (flags & FLAG_LIMIT_BITS) != 0;

  // Zero speed detection level of the drive (0x606F, drive parameter P1.038), for srv-zero-speed.
  hal_data->zero_speed_level = DEASDA_ZERO_SPEED_DEFLT;
  if (lcec_read_sdo16(slave, 0x606f, 0x00, &hal_data->zero_speed_level) != 0) {
    hal_data->zero_speed_level = DEASDA_ZERO_SPEED_DEFLT;
  }
  hal_data->opmode_cyclic = operationmode;
  hal_data->opmode_current = operationmode;

  // Set up digital outputs.  These names should match the A3, unclear about other models.
  //
  // TODO(scottlaird): It appears that various A2 and A3 models have
  // different numbers of digital out (and in?) ports.  We'll probably
  // want to make this configurable, one way or another.
  if (enable_dout) {
    hal_data->dout = lcec_dout_allocate_channels(4);
    hal_data->dout->channels[0] = lcec_dout_register_channel_packed(slave, 0x60fe, 0x01, 16, "dout-d01");
    hal_data->dout->channels[1] = lcec_dout_register_channel_packed(slave, 0x60fe, 0x01, 17, "dout-d02");
    hal_data->dout->channels[2] = lcec_dout_register_channel_packed(slave, 0x60fe, 0x01, 18, "dout-d03");
    hal_data->dout->channels[3] = lcec_dout_register_channel_packed(slave, 0x60fe, 0x01, 19, "dout-d04");

    if (lcec_write_sdo32(slave, 0x60fe, 0x02, 0x000f0000) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to configure slave %s.%s sdo for enabling digital output ports 1-4\n",
          master->name, slave->name);
      return -1;
    }
  }

  // set to 0x6060 to requested mode (CSV, CSP)
  if (lcec_write_sdo8(slave, 0x6060, 0x00, operationmode) != 0) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo to op mode %d\n", master->name, slave->name, operationmode);
    return -1;
  }

  // E-Gear ratio (0x6093:01 numerator N / 0x6093:02 denominator M).  One motor revolution is
  // (encoder resolution * M / N) PUU.  The drive only accepts changes while the servo is off.
  pval = lcec_modparam_get(slave, M_EGEARRATIO);
  if (pval != NULL) {
    egear = lcec_cia402_decode_ratio_modparam(pval->str, 1 << 30);
    if (egear.numerator <= 0 || egear.denominator == 0) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid eGearRatio \"%s\" for slave %s.%s\n", pval->str, master->name, slave->name);
      return -1;
    }
    if (lcec_write_sdo32_modparam(slave, 0x6093, 0x01, egear.numerator, "eGearRatio") != 0) return -1;
    if (lcec_write_sdo32_modparam(slave, 0x6093, 0x02, egear.denominator, "eGearRatio") != 0) return -1;
  }

  // Homing settings.  These are plain SDOs, so they can be set even if homing mode itself is not enabled.
  pval = lcec_modparam_get(slave, M_HOMEOFFSET);
  if (pval != NULL) {
    if (lcec_write_sdo32_modparam(slave, 0x607c, 0x00, pval->s32, "homeOffset") != 0) return -1;
  }
  pval = lcec_modparam_get(slave, M_HOMEMETHOD);
  if (pval != NULL) {
    if (pval->s32 < -128 || pval->s32 > 127) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "homeMethod %d out of range for slave %s.%s\n", (int)pval->s32, master->name, slave->name);
      return -1;
    }
    if (lcec_write_sdo8_modparam(slave, 0x6098, 0x00, pval->s32, "homeMethod") != 0) return -1;
  }

  // Hard stop homing (methods -1 to -4): the drive takes the stop as reached once the torque has been above
  // the level (P1.087, 0x2157) for the time of the level reached timer (P1.088, 0x2158).
  pval = lcec_modparam_get(slave, M_HOMETORQUELVL);
  if (pval != NULL) {
    if (pval->u32 < 1 || pval->u32 > 300) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "homeTorqueLevel_pct %u of slave %s.%s is out of range (1 to 300 %%)\n",
          (unsigned)pval->u32, master->name, slave->name);
      return -1;
    }
    if (lcec_write_sdo16_modparam(slave, 0x2157, 0x00, pval->u32, "homeTorqueLevel_pct") != 0) return -1;
  }
  pval = lcec_modparam_get(slave, M_HOMETORQUETM);
  if (pval != NULL) {
    if (pval->u32 < 2 || pval->u32 > 2000) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "homeTorqueTime_ms %u of slave %s.%s is out of range (2 to 2000 ms)\n",
          (unsigned)pval->u32, master->name, slave->name);
      return -1;
    }
    if (lcec_write_sdo16_modparam(slave, 0x2158, 0x00, pval->u32, "homeTorqueTime_ms") != 0) return -1;
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

  if (enable_hm) {
    lcec_pdo_init(slave, 0x6060, 0x00, &hal_data->opmode_pdo_os, NULL);
    lcec_pdo_init(slave, 0x6061, 0x00, &hal_data->opmode_display_pdo_os, NULL);

    if ((err = lcec_pin_newf_list(hal_data, slave_pins_hm, LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;
    if (operationmode == DEASDA_OPMODE_CSP) {
      if ((err = lcec_pin_newf_list(hal_data, slave_pins_hm_csp, LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;
    } else {
      if ((err = lcec_pin_newf_list(hal_data, slave_pins_hm_csv, LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;
    }

    // Start out with whatever the drive is using right now, so we don't overwrite valid settings.  The
    // pins have to show what the drive really holds, so a read that fails is an error.
    if (lcec_read_sdo8_pin_S32(slave, 0x6098, 0x00, hal_data->home_method) != 0 ||
        lcec_read_sdo32_pin_U32(slave, 0x6099, 0x01, hal_data->home_velocity_fast) != 0 ||
        lcec_read_sdo32_pin_U32(slave, 0x6099, 0x02, hal_data->home_velocity_slow) != 0 ||
        lcec_read_sdo32_pin_U32(slave, 0x609a, 0x00, hal_data->home_accel) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to read the homing settings of slave %s.%s\n", master->name, slave->name);
      return -EIO;
    }
    // 0x6098 is an INTEGER8 (Delta uses -1 to -4 for the hard stop methods), but the helper doesn't sign-extend.
    LCEC_PIN_S32_SET(hal_data->home_method, (int8_t)LCEC_PIN_S32_GET(hal_data->home_method));

    // The homing settings are not mapped; they are written through SDO requests whenever a pin changes.
    if (lcec_deasda_sdo_setting_init(slave, &hal_data->home_method_set, 0x6098, 0x00, 1, LCEC_PIN_S32_GET(hal_data->home_method)) != 0 ||
        lcec_deasda_sdo_setting_init(
            slave, &hal_data->home_velocity_fast_set, 0x6099, 0x01, 4, (int32_t)LCEC_PIN_U32_GET(hal_data->home_velocity_fast)) != 0 ||
        lcec_deasda_sdo_setting_init(
            slave, &hal_data->home_velocity_slow_set, 0x6099, 0x02, 4, (int32_t)LCEC_PIN_U32_GET(hal_data->home_velocity_slow)) != 0 ||
        lcec_deasda_sdo_setting_init(slave, &hal_data->home_accel_set, 0x609a, 0x00, 4, (int32_t)LCEC_PIN_U32_GET(hal_data->home_accel)) != 0) {
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to create the SDO requests for the homing settings of slave %s.%s\n", master->name, slave->name);
      return -EIO;
    }

    LCEC_PIN_S32_SET(hal_data->opmode_display, operationmode);
  }

  LCEC_PIN_U32_SET(hal_data->operation_mode, operationmode);
  // export parameters
  if ((err = lcec_param_newf_list(hal_data, slave_params, LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;

  // init subclasses for encoders
  if ((err = class_enc_init(slave, &hal_data->enc, 32, "enc")) != 0) return err;
  if ((err = class_enc_init(slave, &hal_data->extenc, 32, "extenc")) != 0) return err;

  // initialize variables
  LCEC_PARAM_FLOAT_SET(hal_data->pos_scale, 1.0);
  LCEC_PARAM_FLOAT_SET(hal_data->extenc_scale, 1.0);
  LCEC_PARAM_U32_SET(hal_data->fault_autoreset_cycles, DEASDA_FAULT_AUTORESET_CYCLES);
  LCEC_PARAM_U32_SET(hal_data->fault_autoreset_retries, DEASDA_FAULT_AUTORESET_RETRIES);
  hal_data->pos_scale_old = LCEC_PARAM_FLOAT_GET(hal_data->pos_scale) + 1.0;
  hal_data->pos_scale_rcpt = 1.0;

  // change based on FLAG_LOWRES_ENC/FLAG_HIGHRES_ENC
  if (flags & FLAG_LOWRES_ENC) {
    LCEC_PARAM_U32_SET(hal_data->pprev, DEASDA_PULSES_PER_REV_DEFLT_LOWRES);
    rtapi_print_msg(
        RTAPI_MSG_DBG, LCEC_MSG_PFX "Setting pprev to Low Res Encoder (1,280,000) for device %s.%s.\n", master->name, slave->name);
  } else if (flags & FLAG_HIGHRES_ENC) {
    LCEC_PARAM_U32_SET(hal_data->pprev, DEASDA_PULSES_PER_REV_DEFLT_HIGHRES);
    rtapi_print_msg(
        RTAPI_MSG_DBG, LCEC_MSG_PFX "Setting pprev to High Res Encoder (16,777,216) for device %s.%s.\n", master->name, slave->name);
  }

  // The position values of the drive are in PUU, so the E-Gear ratio changes the number of pulses per revolution.
  if (egear.numerator != (int)egear.denominator) {
    uint64_t pprev = (uint64_t)LCEC_PARAM_U32_GET(hal_data->pprev) * egear.denominator / egear.numerator;
    if (pprev == 0 || pprev > 0xffffffffULL) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "eGearRatio %d/%u of slave %s.%s gives an unusable number of pulses per revolution\n",
          egear.numerator, egear.denominator, master->name, slave->name);
      return -1;
    }
    LCEC_PARAM_U32_SET(hal_data->pprev, (uint32_t)pprev);
    rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "Setting pprev to %u for device %s.%s due to eGearRatio %d/%u.\n", (unsigned)pprev,
        master->name, slave->name, egear.numerator, egear.denominator);
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
  if (LCEC_PARAM_FLOAT_GET(hal_data->pos_scale) != hal_data->pos_scale_old) {
    // scale value has changed, test and update it
    if ((LCEC_PARAM_FLOAT_GET(hal_data->pos_scale) < 1e-20) && (LCEC_PARAM_FLOAT_GET(hal_data->pos_scale) > -1e-20)) LCEC_PARAM_FLOAT_SET(hal_data->pos_scale, 1.0);

    // save new scale to detect future changes
    hal_data->pos_scale_old = LCEC_PARAM_FLOAT_GET(hal_data->pos_scale);
    // we actually want the reciprocal
    hal_data->pos_scale_rcpt = 1.0 / LCEC_PARAM_FLOAT_GET(hal_data->pos_scale);
  }
}

/// @brief Track the state of the drive-internal homing procedure.
///
/// In homing mode, statusword bit 12 is "homing is complete" and bit 13 is "homing error"; in CSP/CSV bit 12
/// is "mode is in effect" instead, so the bits are only evaluated while 0x6061 reports homing mode.
///
/// `srv-home` is cleared here (in the read phase) once homing is done, so LinuxCNC's motion controller sees
/// the falling edge and re-syncs its commanded position *before* the next setpoint is written to the drive.
static void lcec_deasda_read_homing(lcec_deasda_data_t *hal_data, uint8_t *pd, uint16_t status, long period) {
  int homed, error;

  hal_data->opmode_current = EC_READ_S8(&pd[hal_data->opmode_display_pdo_os]);
  LCEC_PIN_S32_SET(hal_data->opmode_display, hal_data->opmode_current);

  if (hal_data->opmode_current != DEASDA_OPMODE_HM) {
    // srv-homed keeps its last value; the bits mean something else outside of homing mode.
    LCEC_PIN_BIT_SET(hal_data->homing, 0);
    LCEC_PIN_BIT_SET(hal_data->homing_error, 0);
    hal_data->home_done_time = 0;
    return;
  }

  error = (status & DEASDA_STATUS_HOMING_ERROR) != 0;
  homed = (status & DEASDA_STATUS_HOMING_COMPLETE) && !error;

  LCEC_PIN_BIT_SET(hal_data->homing_error, error);
  LCEC_PIN_BIT_SET(hal_data->homed, homed);
  LCEC_PIN_BIT_SET(hal_data->homing, hal_data->home_start_sent && !homed && !error);

  if (!LCEC_PIN_BIT_GET(hal_data->home) || !hal_data->home_start_sent) {
    hal_data->home_done_time = 0;
    return;
  }

  if (!homed) {
    // The drive acknowledged the start request by dropping "homing is complete".
    hal_data->home_seen_unhomed = 1;
    hal_data->home_done_time = 0;
    return;
  }

  // A "complete" flag that was never seen cleared may be left over from an earlier homing run, so only
  // trust it after it has been stable for a while.
  if (!hal_data->home_seen_unhomed) {
    hal_data->home_done_time += period;
    if (hal_data->home_done_time < DEASDA_HOMING_DONE_TIMEOUT_NS) return;
  }

  // done, release the request
  LCEC_PIN_BIT_SET(hal_data->home, 0);
  hal_data->home_seen_unhomed = 0;
  hal_data->home_done_time = 0;
}

static void lcec_deasda_read(lcec_slave_t *slave, long period) {
  lcec_deasda_data_t *hal_data = (lcec_deasda_data_t *)slave->hal_data;
  uint8_t *pd = lcec_slave_pd(slave);
  uint16_t status;
  uint32_t status_di;
  int32_t speed_raw;
  double rpm;
  uint32_t pos_cnt;
  // wait for slave to be operational
  if (!slave->state.operational) {
    LCEC_PIN_BIT_SET(hal_data->ready, 0);
    LCEC_PIN_BIT_SET(hal_data->switched_on, 0);
    LCEC_PIN_BIT_SET(hal_data->oper_enabled, 0);
    LCEC_PIN_BIT_SET(hal_data->fault, 1);
    LCEC_PIN_BIT_SET(hal_data->volt_enabled, 0);
    LCEC_PIN_BIT_SET(hal_data->quick_stoped, 0);
    LCEC_PIN_BIT_SET(hal_data->on_disabled, 0);
    LCEC_PIN_BIT_SET(hal_data->warning, 0);
    LCEC_PIN_BIT_SET(hal_data->remote, 0);
    LCEC_PIN_BIT_SET(hal_data->at_speed, 0);
    LCEC_PIN_BIT_SET(hal_data->limit_active, 0);
    LCEC_PIN_BIT_SET(hal_data->zero_speed, 0);
    if (hal_data->hm_enabled) {
      LCEC_PIN_BIT_SET(hal_data->homing, 0);
      LCEC_PIN_BIT_SET(hal_data->homing_error, 0);
    }
    return;
  }

  // check for change in scale value
  lcec_deasda_check_scales(hal_data);

  // read status word
  status = EC_READ_U16(&pd[hal_data->status_pdo_os]);
  LCEC_PIN_BIT_SET(hal_data->ready, (status >> 0) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->switched_on, (status >> 1) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->oper_enabled, (status >> 2) & 0x01);
  hal_data->internal_fault = (status >> 3) & 0x01;
  LCEC_PIN_BIT_SET(hal_data->volt_enabled, (status >> 4) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->quick_stoped, !((status >> 5) & 0x01));
  LCEC_PIN_BIT_SET(hal_data->on_disabled, (status >> 6) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->warning, (status >> 7) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->remote, (status >> 9) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->at_speed, (status >> 10) & 0x01);
  // Bit 11 is "internal limit active" in CiA 402.  The x3 drives don't use it (reserved), they report the
  // positive and negative limit in bits 14 and 15.
  if (hal_data->limit_bits) {
    LCEC_PIN_BIT_SET(hal_data->limit_active, (status & ((1 << 11) | (1 << 14) | (1 << 15))) != 0);
  } else {
    LCEC_PIN_BIT_SET(hal_data->limit_active, (status >> 11) & 0x01);
  }

  if (hal_data->hm_enabled) lcec_deasda_read_homing(hal_data, pd, status, period);

  // clear pending fault reset if no fault
  if (!hal_data->internal_fault) hal_data->fault_reset_retry = 0;

  // generate gated fault
  if (hal_data->fault_reset_retry > 0) {
    if (hal_data->fault_reset_cycle < LCEC_PARAM_U32_GET(hal_data->fault_autoreset_cycles)) {
      hal_data->fault_reset_cycle++;
    } else {
      hal_data->fault_reset_cycle = 0;
      hal_data->fault_reset_state = !hal_data->fault_reset_state;
      if (hal_data->fault_reset_state) {
        hal_data->fault_reset_retry--;
      }
    }
    LCEC_PIN_BIT_SET(hal_data->fault, 0);
  } else {
    LCEC_PIN_BIT_SET(hal_data->fault, hal_data->internal_fault);
  }

  // read current speed
  speed_raw = EC_READ_S32(&pd[hal_data->currvel_pdo_os]);
  // Statusword bit 12 is only "zero speed" in profile velocity mode; in CSP and CSV it is "mode is in
  // effect".  So compare the actual speed with the zero speed detection level of the drive instead.
  LCEC_PIN_BIT_SET(hal_data->zero_speed, llabs((long long)speed_raw) < hal_data->zero_speed_level);
  rpm = (double)speed_raw * DEASDA_RPM_FACTOR;
  LCEC_PIN_FLOAT_SET(hal_data->vel_fb_rpm, rpm);
  LCEC_PIN_FLOAT_SET(hal_data->vel_fb_rpm_abs, fabs(rpm));
  LCEC_PIN_FLOAT_SET(hal_data->vel_fb, rpm * DEASDA_RPM_DIV * LCEC_PARAM_FLOAT_GET(hal_data->pos_scale));

  // update raw position counter
  pos_cnt = EC_READ_U32(&pd[hal_data->currpos_pdo_os]);
  class_enc_update(&hal_data->enc, LCEC_PARAM_U32_GET(hal_data->pprev), LCEC_PARAM_FLOAT_GET(hal_data->pos_scale), pos_cnt, 0, 0);

  // Position feedback for LinuxCNC.  The drive moves on its own while it is homing, and LinuxCNC can't
  // follow that, so report the commanded position until homing is done.  `srv-home` is cleared in this
  // same cycle (above) when the drive is done, so LinuxCNC gets the new position of the drive together
  // with the falling edge of `srv-home`.
  if (hal_data->pos_fb != NULL) {
    if (LCEC_PIN_BIT_GET(hal_data->home)) {
      LCEC_PIN_FLOAT_SET(hal_data->pos_fb, LCEC_PIN_FLOAT_GET(hal_data->pos_cmd != NULL ? hal_data->pos_cmd : hal_data->cmd_value));
    } else {
      // the position of the encoder class: same scaling, and extended to 64 bit, so it stays continuous
      // when 0x6064 rolls over
      LCEC_PIN_FLOAT_SET(hal_data->pos_fb, LCEC_PIN_FLOAT_GET(hal_data->enc.pos_enc));
    }
  }

  // update external encoder counter
  pos_cnt = EC_READ_U32(&pd[hal_data->extenc_pdo_os]);
  class_enc_update(&hal_data->extenc, 1, LCEC_PARAM_FLOAT_GET(hal_data->extenc_scale), pos_cnt, 0, 0);

  // read current
  LCEC_PIN_FLOAT_SET(hal_data->torque, (double)EC_READ_S16(&pd[hal_data->torque_pdo_os]) * 0.1);

  // read DI status word
  status_di = EC_READ_U32(&pd[hal_data->divalue_pdo_os]);

  LCEC_PIN_BIT_SET(hal_data->neg_lim_switch, (status_di >> 0) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->pos_lim_switch, (status_di >> 1) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->home_switch, (status_di >> 2) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->di_1, (status_di >> 16) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->di_2, (status_di >> 17) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->di_3, (status_di >> 18) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->di_4, (status_di >> 19) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->di_5, (status_di >> 20) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->di_6, (status_di >> 21) & 0x01);
  LCEC_PIN_BIT_SET(hal_data->di_7, (status_di >> 22) & 0x01);
}

/// @brief Write a non-mapped setting to the drive through an SDO request if its pin has changed.
///
/// The value only counts as written once the request has succeeded.  A write that fails leaves
/// `confirmed` as it is, so it is tried again.
///
/// @return true if the drive holds `value`.
static int lcec_deasda_sdo_update(lcec_deasda_sdo_setting_t *setting, int32_t value) {
  uint8_t *data;

  if (setting->pending) {
    switch (ecrt_sdo_request_state(setting->request)) {
      case EC_REQUEST_BUSY:
        return 0;
      case EC_REQUEST_SUCCESS:
        setting->confirmed = setting->in_flight;
        break;
      default:
        break;
    }
    setting->pending = 0;
  }

  if (value == setting->confirmed) return 1;

  setting->in_flight = value;
  setting->pending = 1;
  data = ecrt_sdo_request_data(setting->request);
  if (setting->bytes == 1) {
    EC_WRITE_S8(data, value);
  } else {
    EC_WRITE_S32(data, value);
  }
  ecrt_sdo_request_write(setting->request);
  return 0;
}

/// @brief Write everything that is common to CSV and CSP: digital outputs, controlword and (with homing
/// enabled) the mode of operation.
///
/// @return true if the drive is not (yet) following cyclic setpoints, because it is homing or still
/// switching modes.
static int lcec_deasda_write_control(lcec_slave_t *slave, lcec_deasda_data_t *hal_data, uint8_t *pd) {
  uint16_t control;
  int switch_on_edge;
  int hold = 0;

  // do digital outputs
  if (hal_data->dout) lcec_dout_write_all(slave, hal_data->dout);

  // check for enable edge
  switch_on_edge = LCEC_PIN_BIT_GET(hal_data->switch_on) && !hal_data->last_switch_on;
  hal_data->last_switch_on = LCEC_PIN_BIT_GET(hal_data->switch_on);

  // check for autoreset
  if (LCEC_PARAM_U32_GET(hal_data->fault_autoreset_retries) > 0 && LCEC_PARAM_U32_GET(hal_data->fault_autoreset_cycles) > 0 && switch_on_edge && hal_data->internal_fault) {
    hal_data->fault_reset_retry = LCEC_PARAM_U32_GET(hal_data->fault_autoreset_retries);
    hal_data->fault_reset_state = 1;
    hal_data->fault_reset_cycle = 0;
  }

  // check for change in scale value
  lcec_deasda_check_scales(hal_data);

  // write dev ctrl
  control = 0;

  if (LCEC_PIN_BIT_GET(hal_data->enable_volt)) control |= (1 << 1);
  if (!LCEC_PIN_BIT_GET(hal_data->quick_stop)) control |= (1 << 2);
  if (LCEC_PIN_BIT_GET(hal_data->fault_reset)) control |= (1 << 7);
  if (LCEC_PIN_BIT_GET(hal_data->halt)) control |= (1 << 8);

  if (hal_data->fault_reset_retry > 0) {
    if (hal_data->fault_reset_state) control |= (1 << 7);
  } else {
    if (LCEC_PIN_BIT_GET(hal_data->switch_on)) control |= (1 << 0);
    if (LCEC_PIN_BIT_GET(hal_data->enable) && LCEC_PIN_BIT_GET(hal_data->switched_on)) control |= (1 << 3);
  }

  if (hal_data->hm_enabled) {
    int home = LCEC_PIN_BIT_GET(hal_data->home);
    int8_t opmode = hal_data->opmode_cyclic;
    int32_t method = LCEC_PIN_S32_GET(hal_data->home_method);
    int settings_ok;

    // Bring the drive in line with the homing setting pins.  0x6098 is an INTEGER8: a method that doesn't
    // fit is not sent, as the drive would get a different method than the pin shows.  Homing only starts
    // once the drive holds all four values of the pins.
    settings_ok = method >= INT8_MIN && method <= INT8_MAX && lcec_deasda_sdo_update(&hal_data->home_method_set, method);
    settings_ok &= lcec_deasda_sdo_update(&hal_data->home_velocity_fast_set, (int32_t)LCEC_PIN_U32_GET(hal_data->home_velocity_fast));
    settings_ok &= lcec_deasda_sdo_update(&hal_data->home_velocity_slow_set, (int32_t)LCEC_PIN_U32_GET(hal_data->home_velocity_slow));
    settings_ok &= lcec_deasda_sdo_update(&hal_data->home_accel_set, (int32_t)LCEC_PIN_U32_GET(hal_data->home_accel));

    // A fault aborts the homing run.  srv-home is not touched: LinuxCNC takes a falling edge of
    // joint.N.index-enable as "homing is done", so clearing it here would make an axis homed at whatever
    // position the fault left it in.  Whoever set srv-home has to take it back (LinuxCNC does when it
    // aborts homing, i.e. on joint.N.amp-fault-in), and until then no new run is started.
    if (!home) {
      hal_data->home_aborted = 0;
    } else if (hal_data->internal_fault) {
      hal_data->home_aborted = 1;
    }

    // Request homing mode first, and only set the (rising-edge triggered) start bit once the drive reports
    // homing mode with the servo on; an edge that arrives together with the mode change could get lost.
    hal_data->home_start_sent = 0;
    if (home && !hal_data->home_aborted) {
      opmode = DEASDA_OPMODE_HM;
      if (settings_ok && hal_data->opmode_current == DEASDA_OPMODE_HM && LCEC_PIN_BIT_GET(hal_data->oper_enabled) &&
          (control & (1 << 3))) {
        control |= DEASDA_CONTROL_HOMING_START;
        hal_data->home_start_sent = 1;
      }
    } else {
      hal_data->home_seen_unhomed = 0;
    }
    EC_WRITE_S8(&pd[hal_data->opmode_pdo_os], opmode);

    hold = home || hal_data->opmode_current != hal_data->opmode_cyclic;
  }

  EC_WRITE_U16(&pd[hal_data->control_pdo_os], control);

  return hold;
}

static void lcec_deasda_write_csv(lcec_slave_t *slave, long period) {
  lcec_deasda_data_t *hal_data = (lcec_deasda_data_t *)slave->hal_data;
  uint8_t *pd = lcec_slave_pd(slave);
  double speed_raw;
  int hold;

  // write control word, mode of operation and everything else that is not specific to CSV
  hold = lcec_deasda_write_control(slave, hal_data, pd);

  // calculate rpm command
  LCEC_PIN_FLOAT_SET(hal_data->vel_rpm, LCEC_PIN_FLOAT_GET(hal_data->cmd_value) * hal_data->pos_scale_rcpt * DEASDA_RPM_MUL);

  // set RPM
  speed_raw = LCEC_PIN_FLOAT_GET(hal_data->vel_rpm) * DEASDA_RPM_RCPT;
  // the drive moves on its own while homing, so don't request any speed until CSV is active again
  if (hold) speed_raw = 0.0;
  if (speed_raw > (double)0x7fffffff) speed_raw = (double)0x7fffffff;
  if (speed_raw < (double)-0x7fffffff) speed_raw = (double)-0x7fffffff;

  EC_WRITE_S32(&pd[hal_data->cmdvalue_pdo_os], (int32_t)speed_raw);
}

static void lcec_deasda_write_csp(lcec_slave_t *slave, long period) {
  lcec_deasda_data_t *hal_data = (lcec_deasda_data_t *)slave->hal_data;
  uint8_t *pd = lcec_slave_pd(slave);
  int32_t pos_puu;
  int hold;

  // write control word, mode of operation and everything else that is not specific to CSP
  hold = lcec_deasda_write_control(slave, hal_data, pd);

  // ASDA Drives expect target Position in PUU (Pulse per User Unit)
  // See https://www.deltaww.com/en-US/FAQ/228
  // Calculation accordingly based on pprev and pos_scale (i.e. pitch of ball screw)
  pos_puu = (int32_t)(LCEC_PIN_FLOAT_GET(hal_data->cmd_value) * LCEC_PARAM_U32_GET(hal_data->pprev) / LCEC_PARAM_FLOAT_GET(hal_data->pos_scale));
  // Homing redefines the position of the drive, and the drive moves on its own while it is homing.  Keep
  // the target on the actual position until CSP is active again, so there is no jump when it takes over.
  if (hold) pos_puu = EC_READ_S32(&pd[hal_data->currpos_pdo_os]);
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
