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
/// @brief Library for CiA 402 servo/stepper controllers

#include "../lcec.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

/// @brief This is the option list for CiA 402 devices.
///
/// This provides the naming prefix for devices, and also controls
/// which optional features are enabled.  Fortunately or
/// unfortunately, the vast bulk of CiA 402 CoE objects are optional,
/// and the exact object implemented by each device may vary widely.
/// This is intended to *only* require the objects that the standard
/// lists as mandatory.  Then, each device driver can enable optional
/// features to fit what the hardware provides.
///
/// In general, these options are broken into 3 categories:
///
/// 1. Technically optional but practically required.  This includes
///    `opmode`, which the standard doesn't require but nearly all
///    devices would be expected to support.  These default to on, but
///    can be disabled.
/// 2. Mode-required objects.  For instance, `pp` mode requires that
///    the `-actual-position` and `-target-position` objects be
///    available.
///
/// 3. Individually optional objects.  Other objects, like
///    `-actual-torque` are optional; some devices will implement them
///    and others will not.  These will need to be flagged on on a
///    per-device, per-object basis.
///
/// At the moment, only pp/pv/csp/csv are even slightly implemented.
/// More will follow as hardware support grows.
typedef struct {
  char *name_prefix;  ///< Prefix for device naming, defaults to "srv".
  int digital_in_channels;
  int digital_out_channels;

  int enable_opmode;  ///< Enable opmode and opmode-display.  They're technically optional in the spec.
  int enable_pp;      ///< If true, enable required PP-mode pins: `-actual-position` and `-target-position`.
  int enable_pv;      ///< If true, enable required PV-mode pins: `-actual-velocity` and `-target-velocity`.
  int enable_csp;     ///< If true, enable required CSP-mode pins: `-actual-position` and `-target-position`, plus others.
  int enable_csv;     ///< If true, enable required PV-mode pins: `-actual-velocity` and `-target-velocity`, plus others.
  int enable_hm;      ///< If true, enable required homing-mode pins.  TBD
  int enable_ip;      ///< If true, enable required interpolation-mode pins.  TBD.
  int enable_vl;      ///< If true, enable required velocity-mode pins.  TBD.
  int enable_tq;      ///< If true, enable required torque-mode pins.  TBD.
  int enable_cst;     ///< If true, enable required Cyclic Synchronous Torque mode pins.  TBD

  int enable_actual_current;
  int enable_actual_following_error;
  int enable_actual_torque;  ///< If true, enable `-actual-torque`.
  int enable_actual_velocity_sensor;
  int enable_actual_vl;
  int enable_actual_voltage;
  int enable_demand_vl;
  int enable_digital_input;   ///< If true, enable digital input PDO.
  int enable_digital_output;  ///< If true, enable digital output PDO.
  int enable_error_code;
  int enable_following_error_timeout;
  int enable_following_error_window;
  int enable_home_accel;  ///< If true, enable the home accel pin
  int enable_interpolation_time_period;
  int enable_maximum_acceleration;
  int enable_maximum_current;
  int enable_maximum_deceleration;
  int enable_maximum_motor_rpm;
  int enable_maximum_torque;
  int enable_motion_profile;
  int enable_motor_rated_current;
  int enable_motor_rated_torque;
  int enable_polarity;
  int enable_profile_accel;         ///< If true, enable the profile accel pin
  int enable_profile_decel;         ///< If true, enable the profile decel pin
  int enable_profile_end_velocity;  ///< If true, enable the profile end velocity pin
  int enable_profile_max_velocity;  ///< If true, enable the profile max velocity pin
  int enable_profile_velocity;      ///< If true, enable the profile velocity pin
  int enable_target_torque;
  int enable_target_vl;
  int enable_torque_demand;
  int enable_torque_profile_type;
  int enable_torque_slope;
  int enable_velocity_demand;
  int enable_velocity_error_time;
  int enable_velocity_error_window;
  int enable_velocity_sensor_selector;
  int enable_velocity_threshold_time;
  int enable_velocity_threshold_window;
  int enable_vl_accel;
  int enable_vl_decel;
  int enable_vl_maximum;
  int enable_vl_minimum;
} lcec_class_cia402_channel_options_t;

/// This is the "global" version of options; it contains data about
/// the CiA 402 device itself (such as the number of channels), plus
/// an array of per-channel options.
typedef struct {
  int channels;                                     ///< Number of channels;
  lcec_class_cia402_channel_options_t *channel[8];  ///< Room for 8 channel options.
} lcec_class_cia402_options_t;

/// This is the internal version of `lcec_class_cia402_channel_options_t`.  It
/// lists each specific pin (or atomic set of pins, in the case of
/// `opmode`), and decisions about mapping/etc can be based on this.
/// This is constructed from an options structure by
/// `lcec_cia402_enabled()`.
typedef struct {
  int enable_hm;

  int enable_actual_current;
  int enable_actual_following_error;
  int enable_actual_position;
  int enable_actual_torque;
  int enable_actual_velocity;
  int enable_actual_velocity_sensor;
  int enable_actual_vl;
  int enable_actual_voltage;
  int enable_demand_vl;
  int enable_digital_input;
  int enable_digital_output;
  int enable_error_code;
  int enable_following_error_timeout;
  int enable_following_error_window;
  int enable_home_accel;
  int enable_home_method;
  int enable_home_velocity_fast;
  int enable_home_velocity_slow;
  int enable_interpolation_time_period;
  int enable_maximum_acceleration;
  int enable_maximum_current;
  int enable_maximum_deceleration;
  int enable_maximum_motor_rpm;
  int enable_maximum_torque;
  int enable_motion_profile;
  int enable_motor_rated_current;
  int enable_motor_rated_torque;
  int enable_opmode;
  int enable_opmode_display;
  int enable_polarity;
  int enable_profile_accel;
  int enable_profile_decel;
  int enable_profile_end_velocity;
  int enable_profile_max_velocity;
  int enable_profile_velocity;
  int enable_target_position;
  int enable_target_torque;
  int enable_target_velocity;
  int enable_target_vl;
  int enable_torque_demand;
  int enable_torque_profile_type;
  int enable_torque_slope;
  int enable_velocity_demand;
  int enable_velocity_error_time;
  int enable_velocity_error_window;
  int enable_velocity_sensor_selector;
  int enable_velocity_threshold_time;
  int enable_velocity_threshold_window;
  int enable_vl_accel;
  int enable_vl_decel;
  int enable_vl_maximum;
  int enable_vl_minimum;
} lcec_class_cia402_enabled_t;

typedef struct {
#define PDO_PIN(name, pin_type) \
  pin_type *name;               \
  unsigned int name##_os;       \
  unsigned int name##_bp;

#define SDO_PIN(name, pin_type) \
  pin_type *name;               \
  pin_type name##_old;          \
  ec_sdo_request_t *name##_sdorequest;

  // Out
  PDO_PIN(controlword, hal_u32_t);
  PDO_PIN(home_method, hal_s32_t);
  PDO_PIN(opmode, hal_s32_t);
  PDO_PIN(target_position, hal_s32_t);
  PDO_PIN(target_torque, hal_s32_t);
  PDO_PIN(target_velocity, hal_s32_t);
  PDO_PIN(target_vl, hal_s32_t);
  PDO_PIN(interpolation_time_period, hal_u32_t);
  PDO_PIN(profile_velocity, hal_u32_t);

  SDO_PIN(following_error_timeout, hal_u32_t);
  SDO_PIN(following_error_window, hal_u32_t);
  SDO_PIN(home_accel, hal_u32_t);
  SDO_PIN(home_velocity_fast, hal_u32_t);
  SDO_PIN(home_velocity_slow, hal_u32_t);
  SDO_PIN(maximum_acceleration, hal_u32_t);
  SDO_PIN(maximum_current, hal_u32_t);
  SDO_PIN(maximum_deceleration, hal_u32_t);
  SDO_PIN(maximum_motor_rpm, hal_u32_t);
  SDO_PIN(maximum_torque, hal_u32_t);
  SDO_PIN(motion_profile, hal_s32_t);
  SDO_PIN(motor_rated_current, hal_u32_t);
  SDO_PIN(motor_rated_torque, hal_u32_t);
  SDO_PIN(polarity, hal_u32_t);
  SDO_PIN(profile_accel, hal_u32_t);
  SDO_PIN(profile_decel, hal_u32_t);
  SDO_PIN(profile_end_velocity, hal_u32_t);
  SDO_PIN(profile_max_velocity, hal_u32_t);
  SDO_PIN(torque_profile_type, hal_s32_t);
  SDO_PIN(torque_slope, hal_u32_t);
  SDO_PIN(velocity_error_time, hal_u32_t);
  SDO_PIN(velocity_error_window, hal_u32_t);
  SDO_PIN(velocity_sensor_selector, hal_s32_t);
  SDO_PIN(velocity_threshold_time, hal_u32_t);
  SDO_PIN(velocity_threshold_window, hal_u32_t);
  SDO_PIN(vl_accel, hal_u32_t);
  SDO_PIN(vl_decel, hal_u32_t);
  SDO_PIN(vl_maximum, hal_s32_t);
  SDO_PIN(vl_minimum, hal_s32_t);

  // In.
  PDO_PIN(statusword, hal_u32_t);
  PDO_PIN(opmode_display, hal_s32_t);
  hal_s32_t *supported_modes;
  hal_bit_t *supports_mode_pp, *supports_mode_vl, *supports_mode_pv, *supports_mode_tq, *supports_mode_hm, *supports_mode_ip,
      *supports_mode_csp, *supports_mode_csv, *supports_mode_cst;

  PDO_PIN(actual_current, hal_s32_t);
  PDO_PIN(actual_position, hal_s32_t);
  PDO_PIN(actual_torque, hal_s32_t);
  PDO_PIN(actual_velocity, hal_s32_t);
  PDO_PIN(actual_velocity_sensor, hal_s32_t);
  PDO_PIN(actual_vl, hal_s32_t);
  PDO_PIN(demand_vl, hal_s32_t);
  PDO_PIN(error_code, hal_u32_t);
  PDO_PIN(torque_demand, hal_s32_t);
  PDO_PIN(velocity_demand, hal_s32_t);

  PDO_PIN(actual_following_error, hal_u32_t);
  PDO_PIN(actual_voltage, hal_u32_t);

  unsigned int base_idx;  ///< The PDO/SDO offset for this channel

  lcec_class_din_channels_t *din;
  lcec_class_dout_channels_t *dout;

  lcec_class_cia402_channel_options_t *options;  ///< The options used to create this device.
  lcec_class_cia402_enabled_t *enabled;
} lcec_class_cia402_channel_t;

typedef struct {
  int count;                               ///< The number of channels described by this structure.
  lcec_class_cia402_channel_t **channels;  ///< a dynamic array of `lcec_class_cia402_channel_t` channels.  There should be 1 per axis.
} lcec_class_cia402_channels_t;

lcec_class_cia402_channels_t *lcec_cia402_allocate_channels(int count);
lcec_class_cia402_channel_t *lcec_cia402_register_channel(
    struct lcec_slave *slave, uint16_t base_idx, lcec_class_cia402_channel_options_t *opt);
void lcec_cia402_read(struct lcec_slave *slave, lcec_class_cia402_channel_t *data);
void lcec_cia402_read_all(struct lcec_slave *slave, lcec_class_cia402_channels_t *channels);
void lcec_cia402_write(struct lcec_slave *slave, lcec_class_cia402_channel_t *data);
void lcec_cia402_write_all(struct lcec_slave *slave, lcec_class_cia402_channels_t *channels);
lcec_class_cia402_options_t *lcec_cia402_options(void);
lcec_class_cia402_channel_options_t *lcec_cia402_channel_options(void);
void lcec_cia402_rename_multiaxis_channels(lcec_class_cia402_options_t *opt);
int lcec_cia402_handle_modparam(struct lcec_slave *slave, const lcec_slave_modparam_t *p, lcec_class_cia402_options_t *opt);
lcec_modparam_desc_t *lcec_cia402_channelized_modparams(lcec_modparam_desc_t const *orig);
lcec_modparam_desc_t *lcec_cia402_modparams(lcec_modparam_desc_t const *device_mps);
lcec_syncs_t *lcec_cia402_init_sync(lcec_slave_t *slave, lcec_class_cia402_options_t *options);
int lcec_cia402_add_output_sync(lcec_syncs_t *syncs, lcec_class_cia402_options_t *options);
int lcec_cia402_add_input_sync(lcec_syncs_t *syncs, lcec_class_cia402_options_t *options);

#define ADD_TYPES_WITH_CIA402_MODPARAMS(types, mps)        \
  static void AddTypes(void) __attribute__((constructor)); \
  static void AddTypes(void) {                             \
    const lcec_modparam_desc_t *all_modparams;             \
    int i;                                                 \
    all_modparams = lcec_cia402_modparams(mps);            \
    for (i = 0; types[i].name != NULL; i++) {              \
      types[i].modparams = all_modparams;                  \
    }                                                      \
    lcec_addtypes(types, __FILE__);                        \
  }

// modParam IDs
//
// These need to:
//   (a) be >= CIA402_MP_BASE and
//   (b) be a multiple of 8, with 7 unused IDs between each.
//       That is, the hex version should end in 0 or 8.
//
// These are run through `lcec_cia402_channelized_modparams()` which
// creates additional versions of these for 8 different channels (or
// axes).

#define CIA402_MP_BASE                 0x1000
#define CIA402_MP_POSLIMIT_MIN         0x1000  // 0x607b:01 "Minimum position range limit" S32
#define CIA402_MP_POSLIMIT_MAX         0x1010  // 0x607b:02 "Maximum position range limit" S32
#define CIA402_MP_SWPOSLIMIT_MIN       0x1020  // 0x607d:01 "Minimum software position limit" S32
#define CIA402_MP_SWPOSLIMIT_MAX       0x1030  // 0x607d:02 "Maximum software position limit" S32
#define CIA402_MP_HOME_OFFSET          0x1040  // 0x607c:00 "home offset" S32
#define CIA402_MP_MAXMOTORSPEED        0x1060  // 0x6080:00 "max motor speed" U32
#define CIA402_MP_QUICKDECEL           0x10b0  // 0x6085:00 "quick stop deceleration" U32
#define CIA402_MP_OPTCODE_QUICKSTOP    0x10c0  // 0x605a:00 "quick stop option code" S16
#define CIA402_MP_OPTCODE_SHUTDOWN     0x10d0  // 0x605b:00 "shutdown option code" S16
#define CIA402_MP_OPTCODE_DISABLE      0x10e0  // 0x605c:00 "disable operation option code" S16
#define CIA402_MP_OPTCODE_HALT         0x10f0  // 0x605d:00 "halt option code" S16
#define CIA402_MP_OPTCODE_FAULT        0x1100  // 0x605e:00 "fault option code" S16
#define CIA402_MP_PROBE_FUNCTION       0x1150  // 0x60b8:00 "probe function" U16
#define CIA402_MP_PROBE1_POS           0x1160  // 0x60ba:00 "touch probe 1 positive value" S32
#define CIA402_MP_PROBE1_NEG           0x1170  // 0x60bb:00 "touch probe 1 negative value" S32
#define CIA402_MP_PROBE2_POS           0x1180  // 0x60bc:00 "touch probe 2 positive value" S32
#define CIA402_MP_PROBE2_NEG           0x1190  // 0x60bd:00 "touch probe 2 negative value" S32
#define CIA402_MP_DIGITAL_IN_CHANNELS  0x11a0
#define CIA402_MP_DIGITAL_OUT_CHANNELS 0x11b0
// next is 0x11c0

// "enable" modParams
#define CIA402_MP_ENABLE_ACTUAL_CURRENT            0x22d0
#define CIA402_MP_ENABLE_ACTUAL_FOLLOWING_ERROR    0x2100
#define CIA402_MP_ENABLE_ACTUAL_TORQUE             0x2110
#define CIA402_MP_ENABLE_ACTUAL_VELOCITY_SENSOR    0x2120
#define CIA402_MP_ENABLE_ACTUAL_VL                 0x2330
#define CIA402_MP_ENABLE_ACTUAL_VOLTAGE            0x22e0
#define CIA402_MP_ENABLE_CSP                       0x2020
#define CIA402_MP_ENABLE_CST                       0x2080
#define CIA402_MP_ENABLE_CSV                       0x2030
#define CIA402_MP_ENABLE_DEMAND_VL                 0x2320
#define CIA402_MP_ENABLE_DIGITAL_INPUT             0x2490
#define CIA402_MP_ENABLE_DIGITAL_OUTPUT            0x24a0
#define CIA402_MP_ENABLE_ERROR_CODE                0x2480
#define CIA402_MP_ENABLE_FOLLOWING_ERROR_TIMEOUT   0x2130
#define CIA402_MP_ENABLE_FOLLOWING_ERROR_WINDOW    0x2140
#define CIA402_MP_ENABLE_HM                        0x2040
#define CIA402_MP_ENABLE_HOME_ACCEL                0x2150
#define CIA402_MP_ENABLE_INTERPOLATION_TIME_PERIOD 0x2160
#define CIA402_MP_ENABLE_IP                        0x2050
#define CIA402_MP_ENABLE_MAXIMUM_ACCELERATION      0x2170
#define CIA402_MP_ENABLE_MAXIMUM_CURRENT           0x22a0
#define CIA402_MP_ENABLE_MAXIMUM_DECELERATION      0x2180
#define CIA402_MP_ENABLE_MAXIMUM_MOTOR_RPM         0x2190
#define CIA402_MP_ENABLE_MAXIMUM_TORQUE            0x21a0
#define CIA402_MP_ENABLE_MOTION_PROFILE            0x21b0
#define CIA402_MP_ENABLE_MOTOR_RATED_CURRENT       0x22c0
#define CIA402_MP_ENABLE_MOTOR_RATED_TORQUE        0x21c0
#define CIA402_MP_ENABLE_POLARITY                  0x21d0
#define CIA402_MP_ENABLE_PP                        0x2000
#define CIA402_MP_ENABLE_PROFILE_ACCEL             0x21e0
#define CIA402_MP_ENABLE_PROFILE_DECEL             0x21f0
#define CIA402_MP_ENABLE_PROFILE_END_VELOCITY      0x2200
#define CIA402_MP_ENABLE_PROFILE_MAX_VELOCITY      0x2210
#define CIA402_MP_ENABLE_PROFILE_VELOCITY          0x2220
#define CIA402_MP_ENABLE_PV                        0x2010
#define CIA402_MP_ENABLE_TARGET_TORQUE             0x2290
#define CIA402_MP_ENABLE_TARGET_VL                 0x2310
#define CIA402_MP_ENABLE_TORQUE_DEMAND             0x22b0
#define CIA402_MP_ENABLE_TORQUE_PROFILE_TYPE       0x2300
#define CIA402_MP_ENABLE_TORQUE_SLOPE              0x22f0
#define CIA402_MP_ENABLE_TQ                        0x2070
#define CIA402_MP_ENABLE_VELOCITY_DEMAND           0x2230
#define CIA402_MP_ENABLE_VELOCITY_ERROR_TIME       0x2240
#define CIA402_MP_ENABLE_VELOCITY_ERROR_WINDOW     0x2250
#define CIA402_MP_ENABLE_VELOCITY_SENSOR_SELECTOR  0x2260
#define CIA402_MP_ENABLE_VELOCITY_THRESHOLD_TIME   0x2270
#define CIA402_MP_ENABLE_VELOCITY_THRESHOLD_WINDOW 0x2280
#define CIA402_MP_ENABLE_VL                        0x2060
#define CIA402_MP_ENABLE_VL_ACCEL                  0x2360
#define CIA402_MP_ENABLE_VL_DECEL                  0x2370
#define CIA402_MP_ENABLE_VL_MAXIMUM                0x2350
#define CIA402_MP_ENABLE_VL_MINIMUM                0x2340
// next is 0x24b0
