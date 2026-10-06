//
//    Copyright (C) 2019 Dominik Braun <dominik.braun@eventor.de>
//    Copyright (C) 2026 Luca Toniolo
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
/// @brief HAL CiA402 drive interface layer
///
/// C port of Dominik Braun's `cia402.comp` from
/// https://github.com/dbraun1981/hal-cia402 (unmaintained since 2021),
/// assimilated into linuxcnc-ethercat so the CiA 402 HAL layer ships with
/// the driver.  Component name, pin/param names and behavior are unchanged,
/// so existing `loadrt cia402 count=N` configurations keep working.
///
/// Two deliberate behavior changes relative to the original .comp:
///
/// 1. Idle controlword is 0x0000 (disable voltage), not 0x0004.  The
///    quick-stop bit is only raised in the enable path, where the state
///    machine walk 0x0006 -> 0x0007 -> 0x000F needs it.  0x0004 is not a
///    CiA 402 command word and wedges some drives (e.g. Wecon VD3E stays in
///    ready-to-switch-on with the power stage fed).  From upstream PR #9.
///
/// 2. CSV mode is reachable: the CSP opmode branch is now gated on
///    `pos_mode`, so `setp cia402.N.csp-mode 0` actually selects cyclic
///    velocity mode as documented.  Upstream checked the CSP condition
///    first without `pos_mode`, making the CSV branch dead code.
///
/// 3. Position survives the 32-bit rollover of 0x6064: `pos-fb` is built
///    from the accumulated modular delta in a 64-bit counter, and the
///    target is written modulo 2^32 so it wraps together with the drive.
///    With a 2^23 encoder 0x6064 wraps after 256 turns.

#include <rtapi.h>
#include <rtapi_app.h>
#include <rtapi_errno.h>
#include <rtapi_string.h>
#include <hal.h>
#include <string.h>

#include "lcec_hal_compat.h"

MODULE_LICENSE("GPL")
MODULE_AUTHOR("Dominik Braun; C port by linuxcnc-ethercat")
MODULE_DESCRIPTION("HAL CiA402 drive interface layer")

// constants
#define FAULT_AUTORESET_DELAY_NS 100000000LL
#define OPMODE_CYCLIC_POSITION 8
#define OPMODE_CYCLIC_VELOCITY 9
#define OPMODE_HOMING 6
#define OPMODE_NONE 0

/// @brief Per-instance state.  Pin fields use the legacy `hal_*_t *`
/// pointer types; on the new HAL API they hold opaque references (see
/// lcec_hal_compat.h).  Param fields are references on the new API and
/// value storage on the old one.
typedef struct {
  // CiA IOs - inputs from drive
  hal_u32_t *statusword;          // pin in, 0x6041
  hal_s32_t *opmode_display;      // pin in, 0x6061
  hal_s32_t *drv_actual_position; // pin in, 0x6064
  hal_s32_t *drv_actual_velocity; // pin in, 0x606C
  // CiA IOs - outputs to drive
  hal_u32_t *controlword;         // pin out, 0x6040
  hal_s32_t *opmode;              // pin out, 0x6060
  hal_s32_t *drv_target_position; // pin out, 0x607A
  hal_s32_t *drv_target_velocity; // pin out, 0x60FF
  // control IOs
  hal_bit_t *enable;              // pin in
  hal_float_t *pos_cmd;           // pin in
  hal_float_t *velocity_cmd;      // pin in
  hal_float_t *pos_fb;            // pin out
  hal_float_t *velocity_fb;       // pin out
  hal_bit_t *drv_fault;           // pin out
  // homing IOs
  hal_bit_t *home;                // pin io
  hal_bit_t *stat_homed;          // pin out
  hal_bit_t *stat_homing;         // pin out
  // auxilary status IOs
  hal_bit_t *stat_switchon_ready;
  hal_bit_t *stat_switched_on;
  hal_bit_t *stat_op_enabled;
  hal_bit_t *stat_voltage_enabled;
  hal_bit_t *stat_fault;
  hal_bit_t *stat_quick_stop;
  hal_bit_t *stat_switchon_disabled;
  hal_bit_t *stat_warning;
  hal_bit_t *stat_remote;
  hal_bit_t *stat_target_reached;
  hal_bit_t *opmode_no_mode;
  hal_bit_t *opmode_homing;
  hal_bit_t *opmode_cyclic_position;
  hal_bit_t *opmode_cyclic_velocity;
  hal_bit_t *fault_reset; // pin in
  // parameters (references on the new HAL API, values on the old one)
  lcec_param_float_t pos_scale;
  lcec_param_float_t velo_scale;
  lcec_param_bit_t auto_fault_reset;
  lcec_param_bit_t csp_mode;
  // internals
  double pos_scale_old;
  double velo_scale_old;
  bool enable_old;
  bool stat_homed_old;
  double pos_scale_rcpt;
  double velo_scale_rcpt;
  bool pos_mode;
  bool init_pos_mode;
  long auto_fault_reset_delay;
  int64_t pos_acc;  // unwrapped 0x6064
  int32_t pos_last; // 0x6064 of the previous cycle
  bool pos_seeded;
} cia402_inst_t;

static int comp_id;
static int default_count = 1, count = 0;
RTAPI_MP_INT(count, "number of cia402 instances");
static char *names = "";
RTAPI_MP_STRING(names, "comma-separated names of cia402 instances");

static void read_all(cia402_inst_t *inst, long period);
static void write_all(cia402_inst_t *inst, long period);

// Dual-API creation helpers, modeled on lcec_pin_newfv()/lcec_param_newfv().
// The typed creators exist on both the transitional and the post-break new
// API; the legacy creators are used on 2.9.x.
static int pin_new(hal_type_t type, hal_pin_dir_t dir, void **data_ptr_addr, const char *name) {
#ifdef LCEC_HAL_NEW_API
  switch (type) {
    case HAL_BIT:
      return hal_pin_new_bool(comp_id, dir, (hal_bool_t *)data_ptr_addr, 0, "%s", name);
    case HAL_FLOAT:
      return hal_pin_new_real(comp_id, dir, (hal_real_t *)data_ptr_addr, 0.0, "%s", name);
    case HAL_S32:
      return hal_pin_new_si32(comp_id, dir, (hal_sint_t *)data_ptr_addr, 0, "%s", name);
    case HAL_U32:
      return hal_pin_new_ui32(comp_id, dir, (hal_uint_t *)data_ptr_addr, 0, "%s", name);
    default:
      return -EINVAL;
  }
#else
  return hal_pin_new(name, type, dir, data_ptr_addr, comp_id);
#endif
}

static int param_new(hal_type_t type, void *data_addr, double def, const char *name) {
#ifdef LCEC_HAL_NEW_API
  // New API: storage is HAL-owned, the creator applies the default and
  // writes the opaque reference into the param field.
  switch (type) {
    case HAL_BIT:
      return hal_param_new_bool(comp_id, HAL_RW, (hal_bool_t *)data_addr, def != 0, "%s", name);
    case HAL_FLOAT:
      return hal_param_new_real(comp_id, HAL_RW, (hal_real_t *)data_addr, def, "%s", name);
    default:
      return -EINVAL;
  }
#else
  // Old API: caller-provided value storage.  Keep the writes narrow.
  int err = hal_param_new(name, type, HAL_RW, data_addr, comp_id);
  if (err) {
    return err;
  }
  switch (type) {
    case HAL_BIT:
      *((hal_bit_t *)data_addr) = (def != 0);
      break;
    case HAL_FLOAT:
      *((hal_float_t *)data_addr) = def;
      break;
    default:
      break;
  }
  return 0;
#endif
}

static int export(char *prefix, cia402_inst_t **inst_out) {
  char buf[HAL_NAME_LEN + 1];
  int r;
  cia402_inst_t *inst = hal_malloc(sizeof(cia402_inst_t));
  if (inst == NULL) {
    return -ENOMEM;
  }
  memset(inst, 0, sizeof(cia402_inst_t));
  if (inst_out != NULL) {
    *inst_out = inst;
  }

  rtapi_snprintf(buf, sizeof(buf), "%s.statusword", prefix);
  if ((r = pin_new(HAL_U32, HAL_IN, (void **)&inst->statusword, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.opmode-display", prefix);
  if ((r = pin_new(HAL_S32, HAL_IN, (void **)&inst->opmode_display, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.drv-actual-position", prefix);
  if ((r = pin_new(HAL_S32, HAL_IN, (void **)&inst->drv_actual_position, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.drv-actual-velocity", prefix);
  if ((r = pin_new(HAL_S32, HAL_IN, (void **)&inst->drv_actual_velocity, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.controlword", prefix);
  if ((r = pin_new(HAL_U32, HAL_OUT, (void **)&inst->controlword, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.opmode", prefix);
  if ((r = pin_new(HAL_S32, HAL_OUT, (void **)&inst->opmode, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.drv-target-position", prefix);
  if ((r = pin_new(HAL_S32, HAL_OUT, (void **)&inst->drv_target_position, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.drv-target-velocity", prefix);
  if ((r = pin_new(HAL_S32, HAL_OUT, (void **)&inst->drv_target_velocity, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.enable", prefix);
  if ((r = pin_new(HAL_BIT, HAL_IN, (void **)&inst->enable, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.pos-cmd", prefix);
  if ((r = pin_new(HAL_FLOAT, HAL_IN, (void **)&inst->pos_cmd, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.velocity-cmd", prefix);
  if ((r = pin_new(HAL_FLOAT, HAL_IN, (void **)&inst->velocity_cmd, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.pos-fb", prefix);
  if ((r = pin_new(HAL_FLOAT, HAL_OUT, (void **)&inst->pos_fb, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.velocity-fb", prefix);
  if ((r = pin_new(HAL_FLOAT, HAL_OUT, (void **)&inst->velocity_fb, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.drv-fault", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->drv_fault, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.home", prefix);
  if ((r = pin_new(HAL_BIT, HAL_IO, (void **)&inst->home, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-homed", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_homed, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-homing", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_homing, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-switchon-ready", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_switchon_ready, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-switched-on", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_switched_on, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-op-enabled", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_op_enabled, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-voltage-enabled", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_voltage_enabled, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-fault", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_fault, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-quick-stop", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_quick_stop, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-switchon-disabled", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_switchon_disabled, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-warning", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_warning, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-remote", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_remote, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.stat-target-reached", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->stat_target_reached, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.opmode-no-mode", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->opmode_no_mode, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.opmode-homing", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->opmode_homing, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.opmode-cyclic-position", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->opmode_cyclic_position, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.opmode-cyclic-velocity", prefix);
  if ((r = pin_new(HAL_BIT, HAL_OUT, (void **)&inst->opmode_cyclic_velocity, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.fault-reset", prefix);
  if ((r = pin_new(HAL_BIT, HAL_IN, (void **)&inst->fault_reset, buf)) != 0) return r;

  rtapi_snprintf(buf, sizeof(buf), "%s.pos-scale", prefix);
  if ((r = param_new(HAL_FLOAT, &inst->pos_scale, 1.0, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.velo-scale", prefix);
  if ((r = param_new(HAL_FLOAT, &inst->velo_scale, 1.0, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.auto-fault-reset", prefix);
  if ((r = param_new(HAL_BIT, &inst->auto_fault_reset, 1, buf)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.csp-mode", prefix);
  if ((r = param_new(HAL_BIT, &inst->csp_mode, 1, buf)) != 0) return r;

  // Force a scale update on the first read_all() call.
  inst->pos_scale_old = 2.0;
  inst->pos_scale_rcpt = 1.0;
  inst->velo_scale_old = 2.0;
  inst->velo_scale_rcpt = 1.0;

  rtapi_snprintf(buf, sizeof(buf), "%s.read-all", prefix);
  if ((r = LCEC_HAL_EXPORT_FUNCT(buf, (void (*)(void *, long))read_all, inst, 1, 0, comp_id)) != 0) return r;
  rtapi_snprintf(buf, sizeof(buf), "%s.write-all", prefix);
  if ((r = LCEC_HAL_EXPORT_FUNCT(buf, (void (*)(void *, long))write_all, inst, 1, 0, comp_id)) != 0) return r;

  return 0;
}

int rtapi_app_main(void) {
  int r = 0;
  int i;

  comp_id = hal_init("cia402");
  if (comp_id < 0) {
    return comp_id;
  }

  if (count && names[0]) {
    rtapi_print_msg(RTAPI_MSG_ERR, "cia402: count= and names= are mutually exclusive\n");
    hal_exit(comp_id);
    return -EINVAL;
  }
  if (!count && !names[0]) {
    count = default_count;
  }
  if (count) {
    for (i = 0; i < count; i++) {
      char buf[HAL_NAME_LEN + 1];
      rtapi_snprintf(buf, sizeof(buf), "cia402.%d", i);
      if ((r = export(buf, NULL)) != 0) {
        break;
      }
    }
  } else {
    size_t i, j;
    int idx;
    char buf[HAL_NAME_LEN + 1];
    const size_t length = strlen(names);
    for (i = j = idx = 0; i <= length; i++) {
      const char c = buf[j] = names[i];
      if ((c == ',') || (c == '\0')) {
        buf[j] = '\0';
        if ((r = export(buf, NULL)) != 0) {
          break;
        }
        idx++;
        j = 0;
      } else {
        if (++j == (sizeof(buf) / sizeof(buf[0]))) {
          buf[j - 1] = '\0';
          rtapi_print_msg(RTAPI_MSG_ERR, "cia402: names: \"%s\" too long\n", buf);
          r = -EINVAL;
          break;
        }
      }
    }
  }

  if (r) {
    hal_exit(comp_id);
  } else {
    hal_ready(comp_id);
  }
  return r;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }

// Returns the scale to use: 1.0 if it is too small to divide by.  The
// caller writes the result back to the param so a clamped value sticks.
static double check_scale(double scale, double *scale_old, double *scale_rcpt) {
  if (scale != *scale_old) {
    if ((scale < 1e-20) && (scale > -1e-20)) {
      // value too small, divide by zero is a bad thing
      scale = 1.0;
    }
    // save new scale to detect future changes
    *scale_old = scale;
    // we actually want the reciprocal
    *scale_rcpt = 1.0 / scale;
  }
  return scale;
}

static void read_all(cia402_inst_t *inst, long period) {
  hal_u32_t statusword = LCEC_PIN_U32_GET(inst->statusword);
  hal_s32_t opmode_display = LCEC_PIN_S32_GET(inst->opmode_display);
  bool opmode_cyclic_position, opmode_cyclic_velocity, opmode_homing;
  bool stat_fault, stat_homed;
  hal_s32_t drv_pos = LCEC_PIN_S32_GET(inst->drv_actual_position);

  // check for change in scale values
  LCEC_PARAM_FLOAT_SET(inst->pos_scale,
      check_scale(LCEC_PARAM_FLOAT_GET(inst->pos_scale), &inst->pos_scale_old, &inst->pos_scale_rcpt));
  LCEC_PARAM_FLOAT_SET(inst->velo_scale,
      check_scale(LCEC_PARAM_FLOAT_GET(inst->velo_scale), &inst->velo_scale_old, &inst->velo_scale_rcpt));

  // read position feedback, unwrapped across the int32 rollover of 0x6064
  if (!inst->pos_seeded) {
    inst->pos_acc = drv_pos;
    inst->pos_seeded = 1;
  } else {
    inst->pos_acc += (int32_t)((uint32_t)drv_pos - (uint32_t)inst->pos_last);
  }
  inst->pos_last = drv_pos;
  LCEC_PIN_FLOAT_SET(inst->pos_fb, ((double)inst->pos_acc) * inst->pos_scale_rcpt);

  // read velocity feedback
  LCEC_PIN_FLOAT_SET(inst->velocity_fb, ((double)LCEC_PIN_S32_GET(inst->drv_actual_velocity)) * inst->velo_scale_rcpt);

  // read Modes of Operation
  opmode_cyclic_position = (opmode_display == OPMODE_CYCLIC_POSITION);
  opmode_cyclic_velocity = (opmode_display == OPMODE_CYCLIC_VELOCITY);
  opmode_homing = (opmode_display == OPMODE_HOMING);
  LCEC_PIN_BIT_SET(inst->opmode_no_mode, (opmode_display == OPMODE_NONE));
  LCEC_PIN_BIT_SET(inst->opmode_homing, opmode_homing);
  LCEC_PIN_BIT_SET(inst->opmode_cyclic_velocity, opmode_cyclic_velocity);
  LCEC_PIN_BIT_SET(inst->opmode_cyclic_position, opmode_cyclic_position);

  // read status
  stat_fault = (statusword >> 3) & 1;
  LCEC_PIN_BIT_SET(inst->stat_switchon_ready, (statusword >> 0) & 1);
  LCEC_PIN_BIT_SET(inst->stat_switched_on, (statusword >> 1) & 1);
  LCEC_PIN_BIT_SET(inst->stat_op_enabled, (statusword >> 2) & 1);
  LCEC_PIN_BIT_SET(inst->stat_fault, stat_fault);
  LCEC_PIN_BIT_SET(inst->stat_voltage_enabled, (statusword >> 4) & 1);
  LCEC_PIN_BIT_SET(inst->stat_quick_stop, (statusword >> 5) & 1);
  LCEC_PIN_BIT_SET(inst->stat_switchon_disabled, (statusword >> 6) & 1);
  LCEC_PIN_BIT_SET(inst->stat_warning, (statusword >> 7) & 1);
  LCEC_PIN_BIT_SET(inst->stat_remote, (statusword >> 9) & 1);

  if (opmode_cyclic_position || opmode_cyclic_velocity) {
    LCEC_PIN_BIT_SET(inst->stat_target_reached, (statusword >> 10) & 1);
  } else {
    LCEC_PIN_BIT_SET(inst->stat_target_reached, 0);
  }

  // home states
  if (opmode_homing) {
    stat_homed = ((statusword >> 10) & 1) && ((statusword >> 12) & 1);
    LCEC_PIN_BIT_SET(inst->stat_homed, stat_homed);
    LCEC_PIN_BIT_SET(inst->stat_homing, !stat_homed && !((statusword >> 10) & 1));
  }

  // update fault output
  if (inst->auto_fault_reset_delay > 0) {
    inst->auto_fault_reset_delay -= period;
    LCEC_PIN_BIT_SET(inst->drv_fault, 0);
  } else {
    LCEC_PIN_BIT_SET(inst->drv_fault, stat_fault && LCEC_PIN_BIT_GET(inst->enable));
  }
}

static void write_all(cia402_inst_t *inst, long period) {
  (void)period;
  bool enable = LCEC_PIN_BIT_GET(inst->enable);
  bool home = LCEC_PIN_BIT_GET(inst->home);
  bool enable_edge;
  hal_u32_t controlword = 0; // idle: disable voltage, see file header

  // init opmode
  if (!inst->init_pos_mode) {
    inst->pos_mode = LCEC_PARAM_BIT_GET(inst->csp_mode);
    inst->init_pos_mode = 1;
  }

  // detect enable edge
  enable_edge = enable && !inst->enable_old;
  inst->enable_old = enable;

  // write control register
  if (LCEC_PIN_BIT_GET(inst->stat_fault)) {
    home = 0;
    LCEC_PIN_BIT_SET(inst->home, 0);
    if (LCEC_PIN_BIT_GET(inst->fault_reset)) {
      controlword |= (1 << 7); // fault reset
    }
    if (LCEC_PARAM_BIT_GET(inst->auto_fault_reset) && enable_edge) {
      inst->auto_fault_reset_delay = FAULT_AUTORESET_DELAY_NS;
      controlword |= (1 << 7); // fault reset
    }
  } else {
    if (enable) {
      controlword |= (1 << 2); // quick stop released
      controlword |= (1 << 1); // enable voltage
      if (LCEC_PIN_BIT_GET(inst->stat_switchon_ready)) {
        controlword |= (1 << 0); // switch on
        if (LCEC_PIN_BIT_GET(inst->stat_switched_on)) {
          controlword |= (1 << 3); // enable op
        }
      }
    }
  }
  LCEC_PIN_U32_SET(inst->controlword, controlword);

  // write position command modulo 2^32, wrapping together with 0x6064;
  // a direct (int32_t) cast of a double out of range is undefined
  LCEC_PIN_S32_SET(
      inst->drv_target_position, (int32_t)(uint32_t)(int64_t)(LCEC_PIN_FLOAT_GET(inst->pos_cmd) * LCEC_PARAM_FLOAT_GET(inst->pos_scale)));
  // write velocity command
  LCEC_PIN_S32_SET(
      inst->drv_target_velocity, (int32_t)(LCEC_PIN_FLOAT_GET(inst->velocity_cmd) * LCEC_PARAM_FLOAT_GET(inst->velo_scale)));

  // reset home command
  if (home && (LCEC_PIN_BIT_GET(inst->stat_homed) && !inst->stat_homed_old) && LCEC_PIN_BIT_GET(inst->opmode_homing)) {
    home = 0;
    LCEC_PIN_BIT_SET(inst->home, 0);
  }
  inst->stat_homed_old = LCEC_PIN_BIT_GET(inst->stat_homed);

  // OP Mode
  if (LCEC_PIN_BIT_GET(inst->stat_voltage_enabled) && !home) {
    if (inst->pos_mode) {
      LCEC_PIN_S32_SET(inst->opmode, OPMODE_CYCLIC_POSITION);
    } else {
      LCEC_PIN_S32_SET(inst->opmode, OPMODE_CYCLIC_VELOCITY);
    }
  }
  // mode Home and start homing
  if (home) {
    LCEC_PIN_S32_SET(inst->opmode, OPMODE_HOMING);
    LCEC_PIN_U32_SET(inst->controlword, controlword | (1 << 4));
  }
}
