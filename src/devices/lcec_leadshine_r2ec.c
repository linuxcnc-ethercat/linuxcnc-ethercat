//
//    Copyright (C) 2026 linuxcnc-ethercat contributors
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
/// @brief Driver for Leadshine R2EC EtherCAT bus coupler and digital
/// I/O modules.
///
/// First-draft implementation per issue #434.  The R2EC is a single
/// EtherCAT slave that hosts up to 32 modular sub-devices on a passive
/// backplane.  At PreOp the coupler scans connected modules and
/// reports their identifiers in 0xF050.  The master must mirror that
/// list back into 0xF030 ("module ident list") for the coupler to
/// accept the module-PDO mappings on the SafeOp transition.
///
/// Slot layout (per ESI):
///   SlotIndexIncrement = 0x10 (CoE objects)
///   SlotPdoIncrement   = 0x10 (PDO indices)
///
/// Per slot N (0-based):
///   Digital input  : TxPdo 0x1A00 + N*0x10, entries 0x6000 + N*0x10:1..ch (1 bit each)
///   Digital output : RxPdo 0x1600 + N*0x10, entries 0x7000 + N*0x10:1..ch (1 bit each)
///
/// Only the digital I/O families are wired up in this draft.  Analog,
/// encoder, relay-only and temperature modules need their own PDO
/// shape and per-channel HAL classes; placeholders are listed in the
/// module table for follow-up work.

#include <stdio.h>

#include "../lcec.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

#define R2EC_PID 0x61400005

#define R2EC_MAX_SLOTS  32
#define R2EC_SLOT_INCR  0x10
#define R2EC_TXPDO_BASE 0x1A00
#define R2EC_RXPDO_BASE 0x1600
#define R2EC_TXOBJ_BASE 0x6000
#define R2EC_RXOBJ_BASE 0x7000

#define R2EC_F050 0xF050  // module ident detected (read-only, scanned)
#define R2EC_F030 0xF030  // module ident list (write to match F050)

typedef enum {
  R2EC_MODKIND_UNKNOWN = 0,
  R2EC_MODKIND_DIN,
  R2EC_MODKIND_DOUT,
  R2EC_MODKIND_DINOUT,
  // TODO: ANALOG_IN, ANALOG_OUT, ENCODER, TEMPERATURE
} r2ec_modkind_t;

typedef struct {
  uint32_t ident;
  const char *name;
  r2ec_modkind_t kind;
  uint8_t din_channels;
  uint8_t dout_channels;
} r2ec_module_def_t;

static const r2ec_module_def_t r2ec_module_table[] = {
    // Digital input modules
    {0x61100025, "PM-1600", R2EC_MODKIND_DIN, 16, 0},
    {0x61100045, "PM-3200", R2EC_MODKIND_DIN, 32, 0},
    {0x61101045, "PM-3200-1", R2EC_MODKIND_DIN, 32, 0},
    {0x61102045, "PM-3200-2", R2EC_MODKIND_DIN, 32, 0},

    // Digital output modules
    {0x61100205, "PM-0016-N", R2EC_MODKIND_DOUT, 0, 16},
    {0x61110205, "PM-0016-P", R2EC_MODKIND_DOUT, 0, 16},
    {0x61100405, "PM-0032-N", R2EC_MODKIND_DOUT, 0, 32},
    {0x61101405, "PM-0032-N-1", R2EC_MODKIND_DOUT, 0, 32},
    {0x61102405, "PM-0032-N-2", R2EC_MODKIND_DOUT, 0, 32},
    {0x61900205, "PM-0016-R", R2EC_MODKIND_DOUT, 0, 16},

    // Mixed digital module
    {0x61100225, "PM-1616-N", R2EC_MODKIND_DINOUT, 16, 16},

    // TODO: analog / encoder / temperature
    // 0x61000025  PM-A0400-IV (analog in,  4ch)
    // 0x61000205  PM-A0004-IV (analog out, 4ch)
    // 0x61300025  PM-E0200-S  (encoder in, single-ended, 2ch)
    // 0x61300125  PM-E0200-D  (encoder in, balanced,     2ch)
    // 0x61700045  PM-T0400-TC (thermocouple, 4ch)
    // 0x61800045  PM-T0400-TR (thermistor,   4ch)

    {0, NULL, R2EC_MODKIND_UNKNOWN, 0, 0},
};

static const r2ec_module_def_t *r2ec_lookup(uint32_t ident) {
  for (const r2ec_module_def_t *m = r2ec_module_table; m->ident != 0; m++) {
    if (m->ident == ident) return m;
  }
  return NULL;
}

typedef struct {
  uint32_t ident;
  const r2ec_module_def_t *def;  // NULL when unknown
} r2ec_slot_t;

typedef struct {
  int slot_count;
  r2ec_slot_t slots[R2EC_MAX_SLOTS];
  lcec_class_din_channels_t *din;
  lcec_class_dout_channels_t *dout;
} lcec_r2ec_data_t;

static int lcec_r2ec_init(int comp_id, lcec_slave_t *slave);
static void lcec_r2ec_read(lcec_slave_t *slave, long period);
static void lcec_r2ec_write(lcec_slave_t *slave, long period);

static lcec_typelist_t types[] = {
    {"R2EC", LCEC_LEADSHINE_VID, R2EC_PID, 0, NULL, lcec_r2ec_init},
    {NULL},
};

ADD_TYPES(types);

/// Read F050 to discover modules.  Returns slot count, or <0 on error.
static int r2ec_scan_modules(lcec_slave_t *slave, lcec_r2ec_data_t *hal_data) {
  uint8_t count = 0;

  if (lcec_read_sdo8(slave, R2EC_F050, 0x00, &count) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "R2EC %s.%s: failed to read 0xF050:00 (module count)\n", slave->master->name, slave->name);
    return -1;
  }
  if (count > R2EC_MAX_SLOTS) {
    rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "R2EC %s.%s: reported %u modules, clamping to %d\n", slave->master->name, slave->name,
        count, R2EC_MAX_SLOTS);
    count = R2EC_MAX_SLOTS;
  }

  for (int i = 0; i < count; i++) {
    uint32_t ident = 0;
    if (lcec_read_sdo32(slave, R2EC_F050, i + 1, &ident) < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "R2EC %s.%s: failed to read 0xF050:%02x\n", slave->master->name, slave->name, i + 1);
      return -1;
    }
    hal_data->slots[i].ident = ident;
    hal_data->slots[i].def = r2ec_lookup(ident);

    if (hal_data->slots[i].def == NULL) {
      rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "R2EC %s.%s: slot %d ident 0x%08x is not in the known-module table; skipping\n",
          slave->master->name, slave->name, i, ident);
    } else {
      rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "R2EC %s.%s: slot %d = %s (0x%08x)\n", slave->master->name, slave->name, i,
          hal_data->slots[i].def->name, ident);
    }
  }

  hal_data->slot_count = count;
  return count;
}

/// Mirror F050 into F030 so the coupler accepts the module list when
/// it transitions to SafeOp.
static int r2ec_program_module_list(lcec_slave_t *slave, lcec_r2ec_data_t *hal_data) {
  // Per the Leadshine manual, F030:0 must be written before any
  // F030:N entries are accepted.  Write count first, then each ident.
  if (lcec_write_sdo8(slave, R2EC_F030, 0x00, 0) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "R2EC %s.%s: failed to clear 0xF030:00\n", slave->master->name, slave->name);
    return -1;
  }
  for (int i = 0; i < hal_data->slot_count; i++) {
    if (lcec_write_sdo32(slave, R2EC_F030, i + 1, hal_data->slots[i].ident) < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "R2EC %s.%s: failed to write 0xF030:%02x = 0x%08x\n", slave->master->name, slave->name,
          i + 1, hal_data->slots[i].ident);
      return -1;
    }
  }
  if (lcec_write_sdo8(slave, R2EC_F030, 0x00, hal_data->slot_count) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "R2EC %s.%s: failed to write 0xF030:00 = %d\n", slave->master->name, slave->name,
        hal_data->slot_count);
    return -1;
  }
  return 0;
}

/// Build sync-manager + PDO mapping based on the discovered slots.
static int r2ec_build_syncs(lcec_slave_t *slave, lcec_r2ec_data_t *hal_data) {
  lcec_syncs_t *syncs = LCEC_HAL_ALLOCATE(lcec_syncs_t);
  lcec_syncs_init(slave, syncs);

  // SM0/SM1 are the mailbox sync managers; the coupler reports them
  // empty and they don't carry PDOs.
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DISABLE);
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);

  // SM2: outputs (RxPDOs).  Watchdog enabled per coupler ESI.
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_ENABLE);
  for (int i = 0; i < hal_data->slot_count; i++) {
    const r2ec_module_def_t *def = hal_data->slots[i].def;
    if (def == NULL) continue;
    if (def->dout_channels == 0) continue;

    uint16_t pdo_idx = R2EC_RXPDO_BASE + i * R2EC_SLOT_INCR;
    uint16_t obj_idx = R2EC_RXOBJ_BASE + i * R2EC_SLOT_INCR;
    lcec_syncs_add_pdo_info(syncs, pdo_idx);
    for (int ch = 0; ch < def->dout_channels; ch++) {
      lcec_syncs_add_pdo_entry(syncs, obj_idx, ch + 1, 1);
    }
  }

  // SM3: inputs (TxPDOs).
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);
  for (int i = 0; i < hal_data->slot_count; i++) {
    const r2ec_module_def_t *def = hal_data->slots[i].def;
    if (def == NULL) continue;
    if (def->din_channels == 0) continue;

    uint16_t pdo_idx = R2EC_TXPDO_BASE + i * R2EC_SLOT_INCR;
    uint16_t obj_idx = R2EC_TXOBJ_BASE + i * R2EC_SLOT_INCR;
    lcec_syncs_add_pdo_info(syncs, pdo_idx);
    for (int ch = 0; ch < def->din_channels; ch++) {
      lcec_syncs_add_pdo_entry(syncs, obj_idx, ch + 1, 1);
    }
  }

  slave->sync_info = &syncs->syncs[0];
  return 0;
}

static int r2ec_register_pins(lcec_slave_t *slave, lcec_r2ec_data_t *hal_data) {
  int total_din = 0, total_dout = 0;
  for (int i = 0; i < hal_data->slot_count; i++) {
    const r2ec_module_def_t *def = hal_data->slots[i].def;
    if (def == NULL) continue;
    total_din += def->din_channels;
    total_dout += def->dout_channels;
  }

  hal_data->din = lcec_din_allocate_channels(total_din);
  hal_data->dout = lcec_dout_allocate_channels(total_dout);
  if (hal_data->din == NULL || hal_data->dout == NULL) return -EIO;

  int din_i = 0, dout_i = 0;
  for (int i = 0; i < hal_data->slot_count; i++) {
    const r2ec_module_def_t *def = hal_data->slots[i].def;
    if (def == NULL) continue;

    uint16_t tx_obj = R2EC_TXOBJ_BASE + i * R2EC_SLOT_INCR;
    uint16_t rx_obj = R2EC_RXOBJ_BASE + i * R2EC_SLOT_INCR;

    for (int ch = 0; ch < def->din_channels; ch++) {
      char *name = LCEC_HAL_ALLOCATE_STRING(32);
      snprintf(name, 32, "slot-%d-din-%d", i, ch + 1);
      hal_data->din->channels[din_i++] = lcec_din_register_channel_named(slave, tx_obj, ch + 1, name);
    }
    for (int ch = 0; ch < def->dout_channels; ch++) {
      char *name = LCEC_HAL_ALLOCATE_STRING(32);
      snprintf(name, 32, "slot-%d-dout-%d", i, ch + 1);
      hal_data->dout->channels[dout_i++] = lcec_dout_register_channel_named(slave, rx_obj, ch + 1, name);
    }
  }
  return 0;
}

static int lcec_r2ec_init(int comp_id, lcec_slave_t *slave) {
  lcec_r2ec_data_t *hal_data = LCEC_HAL_ALLOCATE(lcec_r2ec_data_t);
  slave->hal_data = hal_data;
  slave->proc_read = lcec_r2ec_read;
  slave->proc_write = lcec_r2ec_write;

  if (r2ec_scan_modules(slave, hal_data) < 0) return -EIO;

  // Issue #434: on the first power-up after wiring up the coupler the
  // module scan can race the SDO read here, so F050 may report the
  // wrong count.  When that happens the user has to restart LinuxCNC
  // for everything to settle (see comments from @stean111).  TODO:
  // retry / re-scan loop with a short delay.

  if (r2ec_program_module_list(slave, hal_data) < 0) return -EIO;
  if (r2ec_build_syncs(slave, hal_data) < 0) return -EIO;
  if (r2ec_register_pins(slave, hal_data) < 0) return -EIO;

  return 0;
}

static void lcec_r2ec_read(lcec_slave_t *slave, long period) {
  lcec_r2ec_data_t *hal_data = (lcec_r2ec_data_t *)slave->hal_data;
  if (!slave->state.operational) return;
  lcec_din_read_all(slave, hal_data->din);
}

static void lcec_r2ec_write(lcec_slave_t *slave, long period) {
  lcec_r2ec_data_t *hal_data = (lcec_r2ec_data_t *)slave->hal_data;
  if (!slave->state.operational) return;
  lcec_dout_write_all(slave, hal_data->dout);
}
