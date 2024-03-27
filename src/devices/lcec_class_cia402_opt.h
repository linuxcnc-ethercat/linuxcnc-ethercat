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

// The CiA 402 spec includes a *lot* of optional features; see
// `documentation/cia402.md` for a partial list.  Practically
// speaking, this means that nearly every single thing that we want to
// access is optional, and attempting to access them on hardware that
// doesn't support them will lead to, at best, a poor user experience.
// So, `class_cia402` goes to great lengths to make every single
// access to a non-required object optional.  Outside of this file,
// options are controlled via `lcec_class_cia402_channel_options_t`.  The
// options struct includes a flag for each supported CiA 402 operating
// mode, which toggles on all required objects for that mode, plus
// individual flags for each other object that may be useful.  This is
// then them condensed down to `lcec_class_cia402_enabled_t`
// internally, with one entry per optional object.
//
// Few things make code as unwieldy as page after page of almost but
// not *quite* identical `if (enabled->enabled_actual_following_error)
// ...` checks, so I've tried to package each set of checks up into a
// preprocessor macro.  As much as possible, I've tried to keep these
// macro definitions right next to where they're used.  They make
// certain assumptions about the structure of things:
//
// 1. The exact same name is used everywhere for things related to
//    optional objects.  If we're going to call a feature
//    `target_position`, then we'll have `enabled_target_position`, a
//    pin called `target_position` and an offset called
//    `target_position_os` in `hal_data`, so so on.
//
// 2. Pin values aren't converted at all before sending to hardware.
//    Since most of the CiA 402 movement values are
//    implementation-defined, that means that there's no point in
//    doing unit conversions, unlike cases where the hardware wants
//    milliAmps but we'd rather let the user talk about floating point
//    Amps.  We could deal with this if we had to without too much
//    work, but so far it hasn't really come up.
// 3. Each `enabled` entry only controls a single CoE object.
//
// This table helps keep things simple further down.  First, we're
// going to define `PDO_IDX_OFFSET`s for each optional feature.  This
// is the offset from the base address for each axis/channel (0x6000
// for the first axis, 0x6800 for the second, and so on).  It *also*
// needs to match the naming pattern described above, because it's
// only accessed from inside of `#define`s below.

#define PDO_IDX_OFFSET_actual_current            0x78
#define PDO_IDX_OFFSET_actual_following_error    0xf4
#define PDO_IDX_OFFSET_actual_position           0x64
#define PDO_IDX_OFFSET_actual_torque             0x77
#define PDO_IDX_OFFSET_actual_velocity           0x6c
#define PDO_IDX_OFFSET_actual_velocity_sensor    0x69
#define PDO_IDX_OFFSET_actual_vl                 0x44
#define PDO_IDX_OFFSET_actual_voltage            0x79
#define PDO_IDX_OFFSET_control_effort            0xfa
#define PDO_IDX_OFFSET_demand_vl                 0x43
#define PDO_IDX_OFFSET_digital_input             0xfd
#define PDO_IDX_OFFSET_digital_output            0xfe
#define PDO_IDX_OFFSET_error_code                0x3f
#define PDO_IDX_OFFSET_following_error_timeout   0x66
#define PDO_IDX_OFFSET_following_error_window    0x65
#define PDO_IDX_OFFSET_home_accel                0x9a
#define PDO_IDX_OFFSET_home_method               0x98
#define PDO_IDX_OFFSET_home_velocity_fast        0x99
#define PDO_IDX_OFFSET_home_velocity_slow        0x99
#define PDO_IDX_OFFSET_interpolation_time_period 0xc2
#define PDO_IDX_OFFSET_maximum_acceleration      0xc5
#define PDO_IDX_OFFSET_maximum_current           0x73
#define PDO_IDX_OFFSET_maximum_deceleration      0xc6
#define PDO_IDX_OFFSET_maximum_motor_rpm         0x80
#define PDO_IDX_OFFSET_maximum_slippage          0xf8
#define PDO_IDX_OFFSET_maximum_torque            0x72
#define PDO_IDX_OFFSET_motion_profile            0x86
#define PDO_IDX_OFFSET_motor_rated_current       0x75
#define PDO_IDX_OFFSET_motor_rated_torque        0x76
#define PDO_IDX_OFFSET_opmode                    0x60
#define PDO_IDX_OFFSET_opmode_display            0x61
#define PDO_IDX_OFFSET_polarity                  0x7e
#define PDO_IDX_OFFSET_position_demand           0x62
#define PDO_IDX_OFFSET_positioning_time          0x68
#define PDO_IDX_OFFSET_positioning_window        0x67
#define PDO_IDX_OFFSET_probe_status              0xb9
#define PDO_IDX_OFFSET_profile_accel             0x83
#define PDO_IDX_OFFSET_profile_decel             0x84
#define PDO_IDX_OFFSET_profile_end_velocity      0x82
#define PDO_IDX_OFFSET_profile_max_velocity      0x7f
#define PDO_IDX_OFFSET_profile_velocity          0x81
#define PDO_IDX_OFFSET_target_position           0x7a
#define PDO_IDX_OFFSET_target_torque             0x71
#define PDO_IDX_OFFSET_target_velocity           0xff
#define PDO_IDX_OFFSET_target_vl                 0x42
#define PDO_IDX_OFFSET_torque_demand             0x74
#define PDO_IDX_OFFSET_torque_profile_type       0x88
#define PDO_IDX_OFFSET_torque_slope              0x87
#define PDO_IDX_OFFSET_velocity_demand           0x6b
#define PDO_IDX_OFFSET_velocity_error_time       0x6e
#define PDO_IDX_OFFSET_velocity_error_window     0x6d
#define PDO_IDX_OFFSET_velocity_sensor_selector  0x6a
#define PDO_IDX_OFFSET_velocity_threshold_time   0x70
#define PDO_IDX_OFFSET_velocity_threshold_window 0x6f
#define PDO_IDX_OFFSET_vl_maximum                0x46
#define PDO_IDX_OFFSET_vl_minimum                0x46

// Next, we'll do the same thing for sub-index addresses.  With CiA
// 402, these are mostly (but not entirely) 0.
#define PDO_SIDX_actual_current            0
#define PDO_SIDX_actual_following_error    0
#define PDO_SIDX_actual_position           0
#define PDO_SIDX_actual_torque             0
#define PDO_SIDX_actual_velocity           0
#define PDO_SIDX_actual_velocity_sensor    0
#define PDO_SIDX_actual_vl                 0
#define PDO_SIDX_actual_voltage            0
#define PDO_SIDX_control_effort            0
#define PDO_SIDX_demand_vl                 0
#define PDO_SIDX_digital_input             0
#define PDO_SIDX_digital_output            1
#define PDO_SIDX_error_code                0
#define PDO_SIDX_following_error_timeout   0
#define PDO_SIDX_following_error_window    0
#define PDO_SIDX_home_accel                0
#define PDO_SIDX_home_method               0
#define PDO_SIDX_home_velocity_fast        1
#define PDO_SIDX_home_velocity_slow        2
#define PDO_SIDX_interpolation_time_period 1
#define PDO_SIDX_maximum_acceleration      0
#define PDO_SIDX_maximum_current           0
#define PDO_SIDX_maximum_deceleration      0
#define PDO_SIDX_maximum_motor_rpm         0
#define PDO_SIDX_maximum_slippage          0
#define PDO_SIDX_maximum_torque            0
#define PDO_SIDX_motion_profile            0
#define PDO_SIDX_motor_rated_current       0
#define PDO_SIDX_motor_rated_torque        0
#define PDO_SIDX_opmode                    0
#define PDO_SIDX_opmode_display            0
#define PDO_SIDX_polarity                  0
#define PDO_SIDX_position_demand           0
#define PDO_SIDX_positioning_time          0
#define PDO_SIDX_positioning_window        0
#define PDO_SIDX_probe_status              0
#define PDO_SIDX_profile_accel             0
#define PDO_SIDX_profile_decel             0
#define PDO_SIDX_profile_end_velocity      0
#define PDO_SIDX_profile_max_velocity      0
#define PDO_SIDX_profile_velocity          0
#define PDO_SIDX_target_position           0
#define PDO_SIDX_target_torque             0
#define PDO_SIDX_target_velocity           0
#define PDO_SIDX_target_vl                 0
#define PDO_SIDX_torque_demand             0
#define PDO_SIDX_torque_profile_type       0
#define PDO_SIDX_torque_slope              0
#define PDO_SIDX_velocity_demand           0
#define PDO_SIDX_velocity_error_time       0
#define PDO_SIDX_velocity_error_window     0
#define PDO_SIDX_velocity_sensor_selector  0
#define PDO_SIDX_velocity_threshold_time   0
#define PDO_SIDX_velocity_threshold_window 0
#define PDO_SIDX_vl_maximum                2
#define PDO_SIDX_vl_minimum                1

// Next, the type of pin that each uses.  *Generally* either HAL_U32
// or HAL_S32.  We could probably generate this by concatenating
// PDO_SIGN_foo (below) and `32`, but it'd probably bite us
// eventually.
#define PDO_PIN_TYPE_actual_current            HAL_S32
#define PDO_PIN_TYPE_actual_following_error    HAL_U32
#define PDO_PIN_TYPE_actual_position           HAL_S32
#define PDO_PIN_TYPE_actual_torque             HAL_S32
#define PDO_PIN_TYPE_actual_velocity           HAL_S32
#define PDO_PIN_TYPE_actual_velocity_sensor    HAL_S32
#define PDO_PIN_TYPE_actual_vl                 HAL_S32
#define PDO_PIN_TYPE_actual_voltage            HAL_U32
#define PDO_PIN_TYPE_control_effort            HAL_S32
#define PDO_PIN_TYPE_demand_vl                 HAL_S32
#define PDO_PIN_TYPE_error_code                HAL_U32
#define PDO_PIN_TYPE_following_error_timeout   HAL_U32
#define PDO_PIN_TYPE_following_error_window    HAL_U32
#define PDO_PIN_TYPE_home_accel                HAL_U32
#define PDO_PIN_TYPE_home_method               HAL_S32
#define PDO_PIN_TYPE_home_velocity_fast        HAL_U32
#define PDO_PIN_TYPE_home_velocity_slow        HAL_U32
#define PDO_PIN_TYPE_interpolation_time_period HAL_U32
#define PDO_PIN_TYPE_maximum_acceleration      HAL_U32
#define PDO_PIN_TYPE_maximum_current           HAL_U32
#define PDO_PIN_TYPE_maximum_deceleration      HAL_U32
#define PDO_PIN_TYPE_maximum_motor_rpm         HAL_U32
#define PDO_PIN_TYPE_maximum_slippage          HAL_S32
#define PDO_PIN_TYPE_maximum_torque            HAL_U32
#define PDO_PIN_TYPE_motion_profile            HAL_S32
#define PDO_PIN_TYPE_motor_rated_current       HAL_U32
#define PDO_PIN_TYPE_motor_rated_torque        HAL_U32
#define PDO_PIN_TYPE_opmode                    HAL_S32
#define PDO_PIN_TYPE_opmode_display            HAL_S32
#define PDO_PIN_TYPE_polarity                  HAL_U32
#define PDO_PIN_TYPE_position_demand           HAL_S32
#define PDO_PIN_TYPE_positioning_time          HAL_U32
#define PDO_PIN_TYPE_positioning_window        HAL_U32
#define PDO_PIN_TYPE_probe_status              HAL_U32
#define PDO_PIN_TYPE_profile_accel             HAL_U32
#define PDO_PIN_TYPE_profile_decel             HAL_U32
#define PDO_PIN_TYPE_profile_end_velocity      HAL_U32
#define PDO_PIN_TYPE_profile_max_velocity      HAL_U32
#define PDO_PIN_TYPE_profile_velocity          HAL_U32
#define PDO_PIN_TYPE_target_position           HAL_S32
#define PDO_PIN_TYPE_target_torque             HAL_S32
#define PDO_PIN_TYPE_target_velocity           HAL_S32
#define PDO_PIN_TYPE_target_vl                 HAL_S32
#define PDO_PIN_TYPE_torque_demand             HAL_S32
#define PDO_PIN_TYPE_torque_profile_type       HAL_S32
#define PDO_PIN_TYPE_torque_slope              HAL_U32
#define PDO_PIN_TYPE_velocity_demand           HAL_S32
#define PDO_PIN_TYPE_velocity_error_time       HAL_U32
#define PDO_PIN_TYPE_velocity_error_window     HAL_U32
#define PDO_PIN_TYPE_velocity_sensor_selector  HAL_S32
#define PDO_PIN_TYPE_velocity_threshold_time   HAL_U32
#define PDO_PIN_TYPE_velocity_threshold_window HAL_U32
#define PDO_PIN_TYPE_vl_maximum                HAL_U32
#define PDO_PIN_TYPE_vl_minimum                HAL_U32

// Pin names
#define PDO_PIN_NAME_actual_current            "actual-current"
#define PDO_PIN_NAME_actual_following_error    "actual-following-error"
#define PDO_PIN_NAME_actual_position           "actual-position"
#define PDO_PIN_NAME_actual_torque             "actual-torque"
#define PDO_PIN_NAME_actual_velocity           "actual-velocity"
#define PDO_PIN_NAME_actual_velocity_sensor    "actual-velocity-sensor"
#define PDO_PIN_NAME_actual_vl                 "actual-vl"
#define PDO_PIN_NAME_actual_voltage            "actual-voltage"
#define PDO_PIN_NAME_control_effort            "control-effort"
#define PDO_PIN_NAME_demand_vl                 "demand-vl"
#define PDO_PIN_NAME_error_code                "error-code"
#define PDO_PIN_NAME_following_error_timeout   "following-error-timeout"
#define PDO_PIN_NAME_following_error_window    "following-error-window"
#define PDO_PIN_NAME_home_accel                "home-accel"
#define PDO_PIN_NAME_home_method               "home-method"
#define PDO_PIN_NAME_home_velocity_fast        "home-velocity-fast"
#define PDO_PIN_NAME_home_velocity_slow        "home-velocity-slow"
#define PDO_PIN_NAME_interpolation_time_period "interpolation-time-period"
#define PDO_PIN_NAME_maximum_acceleration      "maximum-acceleration"
#define PDO_PIN_NAME_maximum_current           "maximum-current"
#define PDO_PIN_NAME_maximum_deceleration      "maximum-deceleration"
#define PDO_PIN_NAME_maximum_motor_rpm         "maximum-motor-rpm"
#define PDO_PIN_NAME_maximum_slippage          "maximum-slippage"
#define PDO_PIN_NAME_maximum_torque            "torque-maximum"
#define PDO_PIN_NAME_motion_profile            "motion-profile"
#define PDO_PIN_NAME_motor_rated_current       "motor-rated-current"
#define PDO_PIN_NAME_motor_rated_torque        "motor-rated-torque"
#define PDO_PIN_NAME_opmode                    "opmode"
#define PDO_PIN_NAME_opmode_display            "opmode-display"
#define PDO_PIN_NAME_polarity                  "polarity"
#define PDO_PIN_NAME_position_demand           "position-demand"
#define PDO_PIN_NAME_positioning_time          "positioning-time"
#define PDO_PIN_NAME_positioning_window        "positioning-window"
#define PDO_PIN_NAME_probe_status              "probe-status"
#define PDO_PIN_NAME_profile_accel             "profile-accel"
#define PDO_PIN_NAME_profile_decel             "profile-decel"
#define PDO_PIN_NAME_profile_end_velocity      "profile-end-velocity"
#define PDO_PIN_NAME_profile_max_velocity      "profile-max-velocity"
#define PDO_PIN_NAME_profile_velocity          "profile-velocity"
#define PDO_PIN_NAME_target_position           "target-position"
#define PDO_PIN_NAME_target_torque             "target-torque"
#define PDO_PIN_NAME_target_velocity           "target-velocity"
#define PDO_PIN_NAME_target_vl                 "target-vl"
#define PDO_PIN_NAME_torque_demand             "torque-demand"
#define PDO_PIN_NAME_torque_profile_type       "torque-profile-type"
#define PDO_PIN_NAME_torque_slope              "torque-slope"
#define PDO_PIN_NAME_velocity_demand           "velocity-demand"
#define PDO_PIN_NAME_velocity_error_time       "velocity-error-time"
#define PDO_PIN_NAME_velocity_error_window     "velocity-error-window"
#define PDO_PIN_NAME_velocity_sensor_selector  "velocity-sensor-selector"
#define PDO_PIN_NAME_velocity_threshold_time   "velocity-threshold-time"
#define PDO_PIN_NAME_velocity_threshold_window "velocity-threshold-window"
#define PDO_PIN_NAME_vl_maximum                "vl-maximum"
#define PDO_PIN_NAME_vl_minimum                "vl-minimum"

// Next, the length of the object for each.   Usually 8, 16, or 32.
#define PDO_BITS_actual_current            16
#define PDO_BITS_actual_following_error    32
#define PDO_BITS_actual_position           32
#define PDO_BITS_actual_torque             32
#define PDO_BITS_actual_velocity           32
#define PDO_BITS_actual_velocity_sensor    32
#define PDO_BITS_actual_vl                 16
#define PDO_BITS_actual_voltage            32
#define PDO_BITS_control_effort            32
#define PDO_BITS_demand_vl                 16
#define PDO_BITS_digital_input             32
#define PDO_BITS_digital_output            32
#define PDO_BITS_error_code                16
#define PDO_BITS_following_error_timeout   16
#define PDO_BITS_following_error_window    32
#define PDO_BITS_home_accel                32
#define PDO_BITS_home_method               8
#define PDO_BITS_home_velocity_fast        32
#define PDO_BITS_home_velocity_slow        32
#define PDO_BITS_interpolation_time_period 8
#define PDO_BITS_maximum_acceleration      32
#define PDO_BITS_maximum_current           16
#define PDO_BITS_maximum_deceleration      32
#define PDO_BITS_maximum_motor_rpm         32
#define PDO_BITS_maximum_slippage          32
#define PDO_BITS_maximum_torque            16
#define PDO_BITS_motion_profile            16
#define PDO_BITS_motor_rated_current       32
#define PDO_BITS_motor_rated_torque        32
#define PDO_BITS_opmode                    8
#define PDO_BITS_opmode_display            8
#define PDO_BITS_polarity                  8
#define PDO_BITS_position_demand           32
#define PDO_BITS_positioning_time          16
#define PDO_BITS_positioning_window        32
#define PDO_BITS_probe_status              16
#define PDO_BITS_profile_accel             32
#define PDO_BITS_profile_decel             32
#define PDO_BITS_profile_end_velocity      32
#define PDO_BITS_profile_max_velocity      32
#define PDO_BITS_profile_velocity          32
#define PDO_BITS_target_position           32
#define PDO_BITS_target_torque             16
#define PDO_BITS_target_velocity           32
#define PDO_BITS_target_vl                 16
#define PDO_BITS_torque_demand             16
#define PDO_BITS_torque_profile_type       16
#define PDO_BITS_torque_slope              32
#define PDO_BITS_velocity_demand           32
#define PDO_BITS_velocity_error_time       16
#define PDO_BITS_velocity_error_window     16
#define PDO_BITS_velocity_sensor_selector  16
#define PDO_BITS_velocity_threshold_time   16
#define PDO_BITS_velocity_threshold_window 16
#define PDO_BITS_vl_maximum                32
#define PDO_BITS_vl_minimum                32

// The signeded-ness of each object.  This should match
// PDO_PIN_TYPE_foo, but we don't *necessarily* have PDO_PIN_TYPE_foo
// for all objects.
#define PDO_SIGN_actual_current            S
#define PDO_SIGN_actual_following_error    U
#define PDO_SIGN_actual_position           S
#define PDO_SIGN_actual_torque             S
#define PDO_SIGN_actual_velocity           S
#define PDO_SIGN_actual_velocity_sensor    S
#define PDO_SIGN_actual_vl                 S
#define PDO_SIGN_actual_voltage            U
#define PDO_SIGN_control_effort            S
#define PDO_SIGN_demand_vl                 S
#define PDO_SIGN_error_code                U
#define PDO_SIGN_following_error_timeout   U
#define PDO_SIGN_following_error_window    U
#define PDO_SIGN_home_accel                U
#define PDO_SIGN_home_method               S
#define PDO_SIGN_home_velocity_fast        U
#define PDO_SIGN_home_velocity_slow        U
#define PDO_SIGN_interpolation_time_period U
#define PDO_SIGN_maximum_acceleration      U
#define PDO_SIGN_maximum_current           U
#define PDO_SIGN_maximum_deceleration      U
#define PDO_SIGN_maximum_motor_rpm         U
#define PDO_SIGN_maximum_slippage          S
#define PDO_SIGN_maximum_torque            U
#define PDO_SIGN_motion_profile            S
#define PDO_SIGN_motor_rated_current       U
#define PDO_SIGN_motor_rated_torque        U
#define PDO_SIGN_opmode                    S
#define PDO_SIGN_opmode_display            S
#define PDO_SIGN_polarity                  U
#define PDO_SIGN_position_demand           S
#define PDO_SIGN_positioning_time          U
#define PDO_SIGN_positioning_window        U
#define PDO_SIGN_probe_status              U
#define PDO_SIGN_profile_accel             U
#define PDO_SIGN_profile_decel             U
#define PDO_SIGN_profile_end_velocity      U
#define PDO_SIGN_profile_max_velocity      U
#define PDO_SIGN_profile_velocity          U
#define PDO_SIGN_target_position           S
#define PDO_SIGN_target_torque             S
#define PDO_SIGN_target_velocity           S
#define PDO_SIGN_target_vl                 S
#define PDO_SIGN_torque_demand             S
#define PDO_SIGN_torque_profile_type       S
#define PDO_SIGN_torque_slope              U
#define PDO_SIGN_velocity_demand           S
#define PDO_SIGN_velocity_error_time       U
#define PDO_SIGN_velocity_error_window     U
#define PDO_SIGN_velocity_sensor_selector  S
#define PDO_SIGN_velocity_threshold_time   U
#define PDO_SIGN_velocity_threshold_window U
#define PDO_SIGN_vl_maximum                U
#define PDO_SIGN_vl_minimum                U

// define modParam names for each setting.
#define PDO_MP_NAME_actual_current            "enableActualCurrent"
#define PDO_MP_NAME_actual_following_error    "enableActualFollowingError"
#define PDO_MP_NAME_actual_torque             "enableActualTorque"
#define PDO_MP_NAME_actual_velocity_sensor    "enableActualVelocitySensor"
#define PDO_MP_NAME_actual_vl                 "enableActualVL"
#define PDO_MP_NAME_actual_voltage            "enableActualVoltage"
#define PDO_MP_NAME_control_effort            "enableControlEffort"
#define PDO_MP_NAME_csp                       "enableCSP"
#define PDO_MP_NAME_cst                       "enableCST"
#define PDO_MP_NAME_csv                       "enableCSV"
#define PDO_MP_NAME_demand_vl                 "enableDemandVL"
#define PDO_MP_NAME_digital_input             "enableDigitalInput"
#define PDO_MP_NAME_digital_output            "enableDigitalOutput"
#define PDO_MP_NAME_error_code                "enableErrorCode"
#define PDO_MP_NAME_following_error_timeout   "enableFollowingErrorTimeout"
#define PDO_MP_NAME_following_error_window    "enableFollowingErrorWindow"
#define PDO_MP_NAME_hm                        "enableHM"
#define PDO_MP_NAME_home_accel                "enableHomeAccel"
#define PDO_MP_NAME_interpolation_time_period "enableInterpolationTimePeriod"
#define PDO_MP_NAME_ip                        "enableIP"
#define PDO_MP_NAME_maximum_acceleration      "enableMaximumAcceleration"
#define PDO_MP_NAME_maximum_current           "enableMaximumCurrent"
#define PDO_MP_NAME_maximum_deceleration      "enableMaximumDeceleration"
#define PDO_MP_NAME_maximum_motor_rpm         "enableMaximumMotorRPM"
#define PDO_MP_NAME_maximum_slippage          "enableMaximumSlippage"
#define PDO_MP_NAME_maximum_torque            "enableMaximumTorque"
#define PDO_MP_NAME_motion_profile            "enableMotionProfile"
#define PDO_MP_NAME_motor_rated_current       "enableMotorRatedCurrent"
#define PDO_MP_NAME_motor_rated_torque        "enableMotorRatedTorque"
#define PDO_MP_NAME_opmode                    "enableOpmode"
#define PDO_MP_NAME_polarity                  "enablePolarity"
#define PDO_MP_NAME_position_demand           "enablePositionDemand"
#define PDO_MP_NAME_positioning_time          "enablePositioningTime"
#define PDO_MP_NAME_positioning_window        "enablePositioningWindow"
#define PDO_MP_NAME_pp                        "enablePP"
#define PDO_MP_NAME_probe_status              "enableProbeStatus"
#define PDO_MP_NAME_profile_accel             "enableProfileAccel"
#define PDO_MP_NAME_profile_decel             "enableProfileDecel"
#define PDO_MP_NAME_profile_end_velocity      "enableProfileEndVelocity"
#define PDO_MP_NAME_profile_max_velocity      "enableProfileMaxVelocity"
#define PDO_MP_NAME_profile_velocity          "enableProfileVelocity"
#define PDO_MP_NAME_pv                        "enablePV"
#define PDO_MP_NAME_target_torque             "enableTargetTorque"
#define PDO_MP_NAME_target_vl                 "enableTargetVL"
#define PDO_MP_NAME_torque_demand             "enableTorqueDemand"
#define PDO_MP_NAME_torque_profile_type       "enableTorqueProfileType"
#define PDO_MP_NAME_torque_slope              "enableTorqueSlope"
#define PDO_MP_NAME_tq                        "enableTQ"
#define PDO_MP_NAME_velocity_demand           "enableVelocityDemand"
#define PDO_MP_NAME_velocity_error_time       "enableVelocityErrorTime"
#define PDO_MP_NAME_velocity_error_window     "enableVelocityErrorWindow"
#define PDO_MP_NAME_velocity_sensor_selector  "enableVelocitySensorSelector"
#define PDO_MP_NAME_velocity_threshold_time   "enableVelocityThresholdTime"
#define PDO_MP_NAME_velocity_threshold_window "enableVelocityThresholdWindow"
#define PDO_MP_NAME_vl                        "enableVL"
#define PDO_MP_NAME_vl_maximum                "enableVLMaximum"
#define PDO_MP_NAME_vl_minimum                "enableVLMinimum"
#define PDO_MP_NAME_target_position           ""  // not individually enableable
#define PDO_MP_NAME_target_velocity           ""  // not individually enableable
#define PDO_MP_NAME_actual_position           ""  // not individually enableable
#define PDO_MP_NAME_actual_velocity           ""  // not individually enableable
#define PDO_MP_NAME_opmode_display            ""  // not individually enableable

// These work around a couple C pre-processor shortcomings.  In a few
// places, I want to concatenate FOO and BAR into FOOBAR, and then
// evaluate FOOBAR for macro substitution before then taking the
// result of that and concatenating it into something else.  For
// example, I'd like to take `(PDO_SIGN_##name)##(PDO_BITS##name)` and
// get `U32` for name=`home_accel`.  Unfortunately, `##` doesn't work
// like that, or use parenthesis for grouping.  The workaround is to
// put it inside of yet another macro.  Except *that* won't result in
// macro expansion happening, so you need to nest that inside *yet
// another* macro.  Go read the GCC preprocessor manual, it's in there
// and this is how they recommend handling it.
#define JOIN2(a, b)               a##b
#define JOIN3(a, b, c)            a##b##c
#define JOIN5(a, b, c, d, e)      a##b##c##d##e
#define SUBSTJOIN2(a, b)          JOIN2(a, b)
#define SUBSTJOIN3(a, b, c)       JOIN3(a, b, c)
#define SUBSTJOIN5(a, b, c, d, e) JOIN5(a, b, c, d, e)

// Now for the fun stuff.  These iterate over a set of things, calling
// a specified function (or macro!) for each.

// This is the list of read PDOs.  These all need to be initialized,
// mapped, and read.
#define FOR_ALL_READ_PDOS_DO(thing) \
  thing(actual_current);            \
  thing(actual_following_error);    \
  thing(actual_position);           \
  thing(actual_torque);             \
  thing(actual_velocity);           \
  thing(actual_velocity_sensor);    \
  thing(actual_vl);                 \
  thing(actual_voltage);            \
  thing(control_effort);            \
  thing(demand_vl);                 \
  thing(error_code);                \
  thing(opmode_display);            \
  thing(position_demand);           \
  thing(probe_status);              \
  thing(torque_demand);             \
  thing(velocity_demand);

// This is the list of write SDOs.  These need to be initialized, mapped, and written.
#define FOR_ALL_WRITE_PDOS_DO(thing) \
  thing(opmode);                     \
  thing(profile_velocity);           \
  thing(target_position);            \
  thing(target_torque);              \
  thing(target_velocity);            \
  thing(target_vl);

// This is the list of write *SDOs*.  These need to be initialized differently and written, but not mapped.
#define FOR_ALL_WRITE_SDOS_DO(thing) \
  thing(following_error_timeout);    \
  thing(following_error_window);     \
  thing(home_accel);                 \
  thing(home_method);                \
  thing(home_velocity_fast);         \
  thing(home_velocity_slow);         \
  thing(interpolation_time_period);  \
  thing(maximum_acceleration);       \
  thing(maximum_current);            \
  thing(maximum_deceleration);       \
  thing(maximum_motor_rpm);          \
  thing(maximum_slippage);           \
  thing(maximum_torque);             \
  thing(motion_profile);             \
  thing(motor_rated_current);        \
  thing(motor_rated_torque);         \
  thing(polarity);                   \
  thing(positioning_time);           \
  thing(positioning_window);         \
  thing(profile_accel);              \
  thing(profile_decel);              \
  thing(profile_end_velocity);       \
  thing(profile_max_velocity);       \
  thing(torque_profile_type);        \
  thing(torque_slope);               \
  thing(velocity_error_time);        \
  thing(velocity_error_window);      \
  thing(velocity_sensor_selector);   \
  thing(velocity_threshold_time);    \
  thing(velocity_threshold_window);  \
  thing(vl_maximum);                 \
  thing(vl_minimum);

// This is the list of CiA 402 modes:
#define FOR_ALL_CIA402_MODES_DO(thing) thing(pp) thing(pv) thing(csp) thing(csv) thing(cst) thing(hm) thing(ip) thing(tq) thing(vl)

// This is all items that have entries in `enabled`.  It is neither a
// proper superset not proper subset of the above entries.
//
// Also, `clang-format` *really* doesn't know what to do with this.
#define FOR_ALL_OPTS_DO(thing)                                                                                                             \
  thing(actual_current) thing(actual_following_error) thing(actual_torque) thing(actual_velocity_sensor) thing(actual_vl)                  \
      thing(actual_voltage) thing(csp) thing(cst) thing(csv) thing(demand_vl) thing(digital_input) thing(digital_output) thing(error_code) \
          thing(following_error_timeout) thing(following_error_window) thing(hm) thing(home_accel) thing(interpolation_time_period)        \
              thing(ip) thing(maximum_acceleration) thing(maximum_current) thing(maximum_deceleration) thing(maximum_motor_rpm)            \
                  thing(maximum_torque) thing(motion_profile) thing(motor_rated_current) thing(motor_rated_torque) thing(opmode)           \
                      thing(polarity) thing(pp) thing(profile_accel) thing(profile_decel) thing(profile_end_velocity)                      \
                          thing(profile_max_velocity) thing(profile_velocity) thing(pv) thing(target_torque) thing(target_vl)              \
                              thing(torque_demand) thing(torque_profile_type) thing(torque_slope) thing(tq) thing(velocity_demand)         \
                                  thing(velocity_error_time) thing(velocity_error_window) thing(velocity_sensor_selector)                  \
                                      thing(velocity_threshold_time) thing(velocity_threshold_window) thing(vl) thing(vl_maximum) thing(vl_minimum) thing(positioning_window)                    \
                                              thing(positioning_time) thing(maximum_slippage) thing(probe_status) thing(position_demand)   \
                                                  thing(control_effort)
