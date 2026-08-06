// SPDX-License-Identifier: GPL-2.0-or-later
//
// lcec_il2301.c -- LinuxCNC EtherCAT driver for Beckhoff IL2301-B110-0000
//
// Copyright (C) 2025 LinuxCNC EtherCAT
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//
// Initiated and committed by mintracer - miniwinis Bastelbude - https://github.com/mintracer
//
//
// Driver for Beckhoff IL2301-B110-0000 EtherCAT Fieldbus Coupler Box
//
// Device:   IL2301-B110-0000  EtherCAT Fieldbus Coupler Box (IP67)
//           Built-in: 4x DI (24V, 3ms) + 4x DO (24V, 0.5A, M8)
//           IP-Link:  up to 8 extension boxes via LWL fiber ring
//
// Identity: VendorID 0x00000002  ProductCode 0x08FD3094  RevNo 0x00121B82
//
// The driver reads the live PDO assignment via CoE SDO (0x1C12/0x1C13) at
// init time and builds sync-managers and HAL pins dynamically -- no
// recompilation needed when the IP-Link chain changes.
//
// HAL pin tree (example with one IE1011 extension box):
//
//   lcec.0.IL2301.coupler-state          u32  -- IP-Link status word
//   lcec.0.IL2301.0.din-0..3            bit  -- built-in DI  (box 0, 0-based)
//   lcec.0.IL2301.0.din-0-not..3-not    bit
//   lcec.0.IL2301.0.dout-0..3           bit  -- built-in DO  (box 0, 0-based)
//   lcec.0.IL2301.1.din-1..8            bit  -- ext box 1 DI (box 1, 1-based)
//   lcec.0.IL2301.1.din-1-not..8-not    bit
//   lcec.0.IL2301.2.din-1..N            bit  -- next ext box (if present)
//
// Box numbering follows the CoE object index: 0x6000/0x7000 = box 0 (built-in),
// 0x6010/0x7010 = box 1, 0x6020/0x7020 = box 2, ...
// Box 0 channels are 0-based; extension box channels are 1-based (matching
// the physical channel labels on the module housing).

#include "../lcec.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

// ============================================================================
// Discovery buffer limits
// ============================================================================

#define LCEC_IL2301_MAX_MAPPINGS    16   // matches LCEC_MAX_PDO_INFO_COUNT
#define LCEC_IL2301_MAX_ENTRIES    128   // matches LCEC_MAX_PDO_ENTRY_COUNT
#define LCEC_IL2301_MAX_PER_MAP     32   // generous per-mapping-object entry count

typedef struct {
  uint16_t idx;
  uint8_t  sidx;
  uint8_t  bitlen;
} lcec_il2301_entry_t;

// ============================================================================
// HAL data
// ============================================================================

typedef struct {
  hal_u32_t   *coupler_state;
  unsigned int state_pdo_os;
  unsigned int ctrl_pdo_os;

  lcec_class_din_channels_t  *din;   // all digital inputs (built-in + extension)
  lcec_class_dout_channels_t *dout;  // all digital outputs (built-in + extension)
} lcec_il2301_data_t;

// ============================================================================
// Forward declarations
// ============================================================================

static int  lcec_il2301_init (int comp_id, lcec_slave_t *slave);
static void lcec_il2301_read (lcec_slave_t *slave, long period);
static void lcec_il2301_write(lcec_slave_t *slave, long period);

static int lcec_il2301_read_assign(lcec_slave_t *slave, uint16_t assign_idx, uint16_t *out, int max_out);
static int lcec_il2301_read_mapping(lcec_slave_t *slave, uint16_t mapping_idx, lcec_il2301_entry_t *out, int max_out);

// ============================================================================
// Self-registration
// ============================================================================

static lcec_typelist_t types[] = {
  { "IL2301", LCEC_BECKHOFF_VID, 0x08FD3094, 0, NULL, lcec_il2301_init, NULL, 0 },
  { NULL },
};

ADD_TYPES(types)

// ============================================================================
// Discovery helpers
// ============================================================================

/// @brief Read a PDO assignment list (0x1C12 or 0x1C13): subindex 0 is the
/// count, subindices 1..count are 16-bit indices of PDO mapping objects.
/// @return number of entries read, or <0 on SDO failure.
static int lcec_il2301_read_assign(lcec_slave_t *slave, uint16_t assign_idx, uint16_t *out, int max_out) {
  uint8_t count;
  int i;

  if (lcec_read_sdo8(slave, assign_idx, 0x00, &count) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "IL2301 %s.%s: failed reading assign count 0x%04x:00\n", slave->master->name,
        slave->name, assign_idx);
    return -EIO;
  }

  if (count > max_out) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "IL2301 %s.%s: assign 0x%04x has %d entries, only room for %d -- truncating\n",
        slave->master->name, slave->name, assign_idx, count, max_out);
    count = max_out;
  }

  for (i = 0; i < count; i++) {
    if (lcec_read_sdo16(slave, assign_idx, i + 1, &out[i]) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "IL2301 %s.%s: failed reading 0x%04x:%02x\n", slave->master->name, slave->name,
          assign_idx, i + 1);
      return -EIO;
    }
  }

  return count;
}

/// @brief Read one PDO mapping object's entry list: subindex 0 is the entry
/// count, subindices 1..count are packed 32-bit values (index<<16 |
/// subindex<<8 | bit_length).
/// @return number of entries read, or <0 on SDO failure.
static int lcec_il2301_read_mapping(lcec_slave_t *slave, uint16_t mapping_idx, lcec_il2301_entry_t *out, int max_out) {
  uint8_t count;
  int i;

  if (lcec_read_sdo8(slave, mapping_idx, 0x00, &count) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "IL2301 %s.%s: failed reading mapping count 0x%04x:00\n", slave->master->name,
        slave->name, mapping_idx);
    return -EIO;
  }

  if (count > max_out) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "IL2301 %s.%s: mapping 0x%04x has %d entries, only room for %d -- truncating\n",
        slave->master->name, slave->name, mapping_idx, count, max_out);
    count = max_out;
  }

  for (i = 0; i < count; i++) {
    uint32_t raw;
    if (lcec_read_sdo32(slave, mapping_idx, i + 1, &raw) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "IL2301 %s.%s: failed reading 0x%04x:%02x\n", slave->master->name, slave->name,
          mapping_idx, i + 1);
      return -EIO;
    }
    out[i].idx    = (uint16_t)(raw >> 16);
    out[i].sidx   = (uint8_t)(raw >> 8);
    out[i].bitlen = (uint8_t)(raw & 0xff);
  }

  return count;
}

// ============================================================================
// Init -- discover live PDO configuration and build syncs + HAL pins
// ============================================================================

static int lcec_il2301_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t       *master = slave->master;
  lcec_il2301_data_t  *hal_data;
  lcec_syncs_t        *syncs;

  uint16_t out_mappings[LCEC_IL2301_MAX_MAPPINGS];
  uint16_t in_mappings[LCEC_IL2301_MAX_MAPPINGS];
  int n_out_mappings, n_in_mappings;

  lcec_il2301_entry_t all_out[LCEC_IL2301_MAX_ENTRIES];
  lcec_il2301_entry_t all_in[LCEC_IL2301_MAX_ENTRIES];
  int n_all_out = 0, n_all_in = 0;

  lcec_il2301_entry_t tmp[LCEC_IL2301_MAX_PER_MAP];

  int i, j, n, err;
  int n_din = 0, n_dout = 0, din_i = 0, dout_i = 0;

  // -- 1. Discover RxPDO (SM2/outputs) assignment ----------------------------
  n_out_mappings = lcec_il2301_read_assign(slave, 0x1C12, out_mappings, LCEC_IL2301_MAX_MAPPINGS);
  if (n_out_mappings < 0) return n_out_mappings;

  // -- 2. Discover TxPDO (SM3/inputs) assignment -----------------------------
  n_in_mappings = lcec_il2301_read_assign(slave, 0x1C13, in_mappings, LCEC_IL2301_MAX_MAPPINGS);
  if (n_in_mappings < 0) return n_in_mappings;

  rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "IL2301 %s.%s: discovered %d RxPDO(s), %d TxPDO(s) on live bus\n", master->name,
      slave->name, n_out_mappings, n_in_mappings);

  // -- 3. Build sync-manager / PDO configuration dynamically -----------------
  syncs = LCEC_HAL_ALLOCATE(lcec_syncs_t);
  lcec_syncs_init(slave, syncs);

  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DISABLE);  // SM0 MBoxOut
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);   // SM1 MBoxIn

  // SM2: outputs -- walk every discovered RxPDO mapping object
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_ENABLE);
  for (i = 0; i < n_out_mappings; i++) {
    lcec_syncs_add_pdo_info(syncs, out_mappings[i]);

    n = lcec_il2301_read_mapping(slave, out_mappings[i], tmp, LCEC_IL2301_MAX_PER_MAP);
    if (n < 0) return n;

    for (j = 0; j < n; j++) {
      lcec_syncs_add_pdo_entry(syncs, tmp[j].idx, tmp[j].sidx, tmp[j].bitlen);
      if (n_all_out < LCEC_IL2301_MAX_ENTRIES) {
        all_out[n_all_out++] = tmp[j];
      }
    }
  }

  // SM3: inputs -- walk every discovered TxPDO mapping object
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);
  for (i = 0; i < n_in_mappings; i++) {
    lcec_syncs_add_pdo_info(syncs, in_mappings[i]);

    n = lcec_il2301_read_mapping(slave, in_mappings[i], tmp, LCEC_IL2301_MAX_PER_MAP);
    if (n < 0) return n;

    for (j = 0; j < n; j++) {
      lcec_syncs_add_pdo_entry(syncs, tmp[j].idx, tmp[j].sidx, tmp[j].bitlen);
      if (n_all_in < LCEC_IL2301_MAX_ENTRIES) {
        all_in[n_all_in++] = tmp[j];
      }
    }
  }

  slave->sync_info = syncs->syncs;

  slave->proc_read  = lcec_il2301_read;
  slave->proc_write = lcec_il2301_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_il2301_data_t);
  slave->hal_data = hal_data;

  // -- 4. Classify discovered entries and count channels ---------------------
  for (i = 0; i < n_all_in; i++) {
    lcec_il2301_entry_t *e = &all_in[i];
    if (e->idx >= 0x6000 && e->idx < 0x7000 && e->bitlen == 1) n_din++;
  }
  for (i = 0; i < n_all_out; i++) {
    lcec_il2301_entry_t *e = &all_out[i];
    if (e->idx >= 0x7000 && e->idx < 0x8000 && e->bitlen == 1) n_dout++;
  }

  rtapi_print_msg(
      RTAPI_MSG_INFO, LCEC_MSG_PFX "IL2301 %s.%s: discovered %d digital input(s), %d digital output(s) total\n", master->name,
      slave->name, n_din, n_dout);

  hal_data->din  = lcec_din_allocate_channels(n_din);
  hal_data->dout = lcec_dout_allocate_channels(n_dout);
  if (n_din > 0 && hal_data->din == NULL) return -EIO;
  if (n_dout > 0 && hal_data->dout == NULL) return -EIO;

  // -- 5. Register inputs: status word, digital inputs, or skip --------------
  for (i = 0; i < n_all_in; i++) {
    lcec_il2301_entry_t *e = &all_in[i];

    if (e->idx == 0xf100 && e->sidx == 0x01) {
      // Coupler status word
      lcec_pdo_init(slave, e->idx, e->sidx, &hal_data->state_pdo_os, NULL);
      err = lcec_pin_newf(HAL_U32, HAL_OUT, (void **)&hal_data->coupler_state, "%s.%s.%s.coupler-state", LCEC_MODULE_NAME,
          master->name, slave->name);
      if (err) return err;

    } else if (e->idx >= 0x6000 && e->idx < 0x7000 && e->bitlen == 1) {
      // Digital input channel -- box 0 = built-in coupler, box >0 = IP-Link extension.
      // Channel numbering: box 0 is 0-based (0.din-0..3, matches original lcec
      // convention); extension boxes are 1-based (N.din-1..) matching the
      // module's physical channel labels printed on its housing.
      int box = (e->idx - 0x6000) / 0x10;
      int chan = (box == 0) ? (e->sidx - 1) : e->sidx;
      char *name = LCEC_HAL_ALLOCATE_STRING(40);

      rtapi_snprintf(name, 40, "%d.din-%d", box, chan);

      hal_data->din->channels[din_i++] = lcec_din_register_channel_named(slave, e->idx, e->sidx, name);
      if (hal_data->din->channels[din_i - 1] == NULL) return -EIO;

    } else if (e->idx == 0x0000) {
      // Alignment padding -- no pin needed

    } else {
      rtapi_print_msg(RTAPI_MSG_INFO,
          LCEC_MSG_PFX "IL2301 %s.%s: skipped unsupported input entry 0x%04x:%02x (%d bit) -- not yet handled by this driver\n",
          master->name, slave->name, e->idx, e->sidx, e->bitlen);
    }
  }

  // -- 6. Register outputs: control word, digital outputs, or skip -----------
  for (i = 0; i < n_all_out; i++) {
    lcec_il2301_entry_t *e = &all_out[i];

    if (e->idx == 0xf200 && e->sidx == 0x01) {
      // Coupler control word -- not exposed as a pin, written 0x0001 every cycle
      lcec_pdo_init(slave, e->idx, e->sidx, &hal_data->ctrl_pdo_os, NULL);

    } else if (e->idx >= 0x7000 && e->idx < 0x8000 && e->bitlen == 1) {
      int box = (e->idx - 0x7000) / 0x10;
      int chan = (box == 0) ? (e->sidx - 1) : e->sidx;
      char *name = LCEC_HAL_ALLOCATE_STRING(40);

      rtapi_snprintf(name, 40, "%d.dout-%d", box, chan);

      hal_data->dout->channels[dout_i++] = lcec_dout_register_channel_named(slave, e->idx, e->sidx, name);
      if (hal_data->dout->channels[dout_i - 1] == NULL) return -EIO;

    } else if (e->idx == 0x0000) {
      // Alignment padding -- no pin needed

    } else {
      rtapi_print_msg(RTAPI_MSG_INFO,
          LCEC_MSG_PFX "IL2301 %s.%s: skipped unsupported output entry 0x%04x:%02x (%d bit) -- not yet handled by this driver\n",
          master->name, slave->name, e->idx, e->sidx, e->bitlen);
    }
  }

  return 0;
}

// ============================================================================
// Read callback
// ============================================================================

static void lcec_il2301_read(lcec_slave_t *slave, long period) {
  lcec_master_t      *master   = slave->master;
  lcec_il2301_data_t *hal_data = (lcec_il2301_data_t *)slave->hal_data;
  uint8_t            *pd       = master->process_data;

  if (!slave->state.operational) return;

  *(hal_data->coupler_state) = EC_READ_U16(pd + hal_data->state_pdo_os);

  lcec_din_read_all(slave, hal_data->din);
}

// ============================================================================
// Write callback
// ============================================================================

static void lcec_il2301_write(lcec_slave_t *slave, long period) {
  lcec_master_t      *master   = slave->master;
  lcec_il2301_data_t *hal_data = (lcec_il2301_data_t *)slave->hal_data;
  uint8_t            *pd       = master->process_data;

  // Control word 0x0001: enable IP-Link bus. Must be written every cycle.
  EC_WRITE_U16(pd + hal_data->ctrl_pdo_os, 0x0001);

  lcec_dout_write_all(slave, hal_data->dout);
}
