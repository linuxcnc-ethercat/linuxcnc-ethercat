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
// Optimized and committed by mintracer - miniwinis Bastelbude - https://github.com/mintracer
//
/// @file
/// @brief Driver for Beckhoff IL2301-B110-0000 EtherCAT Fieldbus Coupler Box
///
/// Device:   IL2301-B110-0000 EtherCAT Fieldbus Coupler Box (IP67)
///           Built-in: 4x DI (24 V, 3 ms) + 4x DO (24 V, 0.5 A, M8)
///           IP-Link: up to 8 extension boxes via LWL fiber ring
///
/// Identity: VendorID 0x00000002 ProductCode 0x08FD3094 RevNo 0x00121B82
///
/// The driver reads the active PDO assignment via CoE SDO (0x1C12/0x1C13)
/// during initialization and creates sync managers and HAL pins dynamically.

#include "../lcec.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

#define LCEC_IL2301_MAX_MAPPINGS  16
#define LCEC_IL2301_MAX_ENTRIES   128
#define LCEC_IL2301_MAX_PER_MAP   32
#define LCEC_IL2301_PIN_NAME_LEN  40

typedef struct {
  uint16_t idx;
  uint8_t sidx;
  uint8_t bitlen;
} lcec_il2301_entry_t;

typedef struct {
  hal_u32_t *coupler_state;
  unsigned int state_pdo_os;
  unsigned int ctrl_pdo_os;
  int state_pdo_valid;
  int ctrl_pdo_valid;

  lcec_class_din_channels_t *din;
  lcec_class_dout_channels_t *dout;
} lcec_il2301_data_t;

static int lcec_il2301_init(int comp_id, lcec_slave_t *slave);
static void lcec_il2301_read(lcec_slave_t *slave, long period);
static void lcec_il2301_write(lcec_slave_t *slave, long period);

static int lcec_il2301_read_assign(lcec_slave_t *slave, uint16_t assign_idx,
    uint16_t *out, int max_out);
static int lcec_il2301_read_mapping(lcec_slave_t *slave, uint16_t mapping_idx,
    lcec_il2301_entry_t *out, int max_out);

static lcec_typelist_t types[] = {
  { "IL2301", LCEC_BECKHOFF_VID, 0x08FD3094, 0, NULL, lcec_il2301_init, NULL, 0 },
  { NULL },
};

ADD_TYPES(types)

static int lcec_il2301_read_assign(lcec_slave_t *slave, uint16_t assign_idx,
    uint16_t *out, int max_out) {
  uint8_t count;
  int i;

  if (lcec_read_sdo8(slave, assign_idx, 0x00, &count) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "IL2301 %s.%s: failed reading assignment count 0x%04x:00\n",
        slave->master->name, slave->name, assign_idx);
    return -EIO;
  }

  if (count > max_out) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "IL2301 %s.%s: assignment 0x%04x has %u entries, maximum is %d\n",
        slave->master->name, slave->name, assign_idx, (unsigned int)count, max_out);
    return -E2BIG;
  }

  for (i = 0; i < count; i++) {
    if (lcec_read_sdo16(slave, assign_idx, i + 1, &out[i]) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          LCEC_MSG_PFX "IL2301 %s.%s: failed reading assignment 0x%04x:%02x\n",
          slave->master->name, slave->name, assign_idx, i + 1);
      return -EIO;
    }
  }

  return count;
}

static int lcec_il2301_read_mapping(lcec_slave_t *slave, uint16_t mapping_idx,
    lcec_il2301_entry_t *out, int max_out) {
  uint8_t count;
  int i;

  if (lcec_read_sdo8(slave, mapping_idx, 0x00, &count) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "IL2301 %s.%s: failed reading mapping count 0x%04x:00\n",
        slave->master->name, slave->name, mapping_idx);
    return -EIO;
  }

  if (count > max_out) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "IL2301 %s.%s: mapping 0x%04x has %u entries, maximum is %d\n",
        slave->master->name, slave->name, mapping_idx, (unsigned int)count, max_out);
    return -E2BIG;
  }

  for (i = 0; i < count; i++) {
    uint32_t raw;

    if (lcec_read_sdo32(slave, mapping_idx, i + 1, &raw) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          LCEC_MSG_PFX "IL2301 %s.%s: failed reading mapping 0x%04x:%02x\n",
          slave->master->name, slave->name, mapping_idx, i + 1);
      return -EIO;
    }

    out[i].idx = (uint16_t)(raw >> 16);
    out[i].sidx = (uint8_t)(raw >> 8);
    out[i].bitlen = (uint8_t)(raw & 0xff);
  }

  return count;
}

static int lcec_il2301_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_il2301_data_t *hal_data;
  lcec_syncs_t *syncs;

  uint16_t out_mappings[LCEC_IL2301_MAX_MAPPINGS];
  uint16_t in_mappings[LCEC_IL2301_MAX_MAPPINGS];
  int out_map_first[LCEC_IL2301_MAX_MAPPINGS];
  int out_map_count[LCEC_IL2301_MAX_MAPPINGS];
  int in_map_first[LCEC_IL2301_MAX_MAPPINGS];
  int in_map_count[LCEC_IL2301_MAX_MAPPINGS];

  lcec_il2301_entry_t all_out[LCEC_IL2301_MAX_ENTRIES];
  lcec_il2301_entry_t all_in[LCEC_IL2301_MAX_ENTRIES];
  lcec_il2301_entry_t tmp[LCEC_IL2301_MAX_PER_MAP];

  int n_out_mappings;
  int n_in_mappings;
  int n_all_out = 0;
  int n_all_in = 0;
  int n_din = 0;
  int n_dout = 0;
  int din_i = 0;
  int dout_i = 0;
  int i;
  int j;
  int n;
  int err;

  (void)comp_id;

  n_out_mappings = lcec_il2301_read_assign(slave, 0x1C12,
      out_mappings, LCEC_IL2301_MAX_MAPPINGS);
  if (n_out_mappings < 0) return n_out_mappings;

  n_in_mappings = lcec_il2301_read_assign(slave, 0x1C13,
      in_mappings, LCEC_IL2301_MAX_MAPPINGS);
  if (n_in_mappings < 0) return n_in_mappings;

  if ((n_out_mappings + n_in_mappings) > LCEC_MAX_PDO_INFO_COUNT) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "IL2301 %s.%s: %d RxPDO + %d TxPDO mappings exceed LCEC_MAX_PDO_INFO_COUNT (%d)\n",
        master->name, slave->name, n_out_mappings, n_in_mappings,
        LCEC_MAX_PDO_INFO_COUNT);
    return -E2BIG;
  }

  rtapi_print_msg(RTAPI_MSG_INFO,
      LCEC_MSG_PFX "IL2301 %s.%s: discovered %d RxPDO(s), %d TxPDO(s)\n",
      master->name, slave->name, n_out_mappings, n_in_mappings);

  /* Read every mapping exactly once and remember its range in the cache. */
  for (i = 0; i < n_out_mappings; i++) {
    n = lcec_il2301_read_mapping(slave, out_mappings[i], tmp,
        LCEC_IL2301_MAX_PER_MAP);
    if (n < 0) return n;

    if ((n_all_out + n) > LCEC_IL2301_MAX_ENTRIES) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          LCEC_MSG_PFX "IL2301 %s.%s: output PDO entries exceed local limit (%d)\n",
          master->name, slave->name, LCEC_IL2301_MAX_ENTRIES);
      return -E2BIG;
    }

    out_map_first[i] = n_all_out;
    out_map_count[i] = n;
    for (j = 0; j < n; j++) all_out[n_all_out++] = tmp[j];
  }

  for (i = 0; i < n_in_mappings; i++) {
    n = lcec_il2301_read_mapping(slave, in_mappings[i], tmp,
        LCEC_IL2301_MAX_PER_MAP);
    if (n < 0) return n;

    if ((n_all_in + n) > LCEC_IL2301_MAX_ENTRIES) {
      rtapi_print_msg(RTAPI_MSG_ERR,
          LCEC_MSG_PFX "IL2301 %s.%s: input PDO entries exceed local limit (%d)\n",
          master->name, slave->name, LCEC_IL2301_MAX_ENTRIES);
      return -E2BIG;
    }

    in_map_first[i] = n_all_in;
    in_map_count[i] = n;
    for (j = 0; j < n; j++) all_in[n_all_in++] = tmp[j];
  }

  if ((n_all_out + n_all_in) > LCEC_MAX_PDO_ENTRY_COUNT) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "IL2301 %s.%s: %d output + %d input entries exceed LCEC_MAX_PDO_ENTRY_COUNT (%d)\n",
        master->name, slave->name, n_all_out, n_all_in,
        LCEC_MAX_PDO_ENTRY_COUNT);
    return -E2BIG;
  }

  syncs = LCEC_HAL_ALLOCATE(lcec_syncs_t);
  if (syncs == NULL) return -ENOMEM;

  lcec_syncs_init(slave, syncs);
  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DISABLE); /* SM0 MBoxOut */
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);  /* SM1 MBoxIn  */

  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_ENABLE);  /* SM2 outputs */
  for (i = 0; i < n_out_mappings; i++) {
    lcec_syncs_add_pdo_info(syncs, out_mappings[i]);
    for (j = 0; j < out_map_count[i]; j++) {
      const lcec_il2301_entry_t *e = &all_out[out_map_first[i] + j];
      lcec_syncs_add_pdo_entry(syncs, e->idx, e->sidx, e->bitlen);
    }
  }

  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DISABLE);  /* SM3 inputs */
  for (i = 0; i < n_in_mappings; i++) {
    lcec_syncs_add_pdo_info(syncs, in_mappings[i]);
    for (j = 0; j < in_map_count[i]; j++) {
      const lcec_il2301_entry_t *e = &all_in[in_map_first[i] + j];
      lcec_syncs_add_pdo_entry(syncs, e->idx, e->sidx, e->bitlen);
    }
  }

  slave->sync_info = syncs->syncs;
  slave->proc_read = lcec_il2301_read;
  slave->proc_write = lcec_il2301_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_il2301_data_t);
  if (hal_data == NULL) return -ENOMEM;

  hal_data->coupler_state = NULL;
  hal_data->state_pdo_os = 0;
  hal_data->ctrl_pdo_os = 0;
  hal_data->state_pdo_valid = 0;
  hal_data->ctrl_pdo_valid = 0;
  hal_data->din = NULL;
  hal_data->dout = NULL;
  slave->hal_data = hal_data;

  for (i = 0; i < n_all_in; i++) {
    const lcec_il2301_entry_t *e = &all_in[i];
    if (e->idx >= 0x6000 && e->idx < 0x7000 &&
        e->sidx > 0 && e->bitlen == 1) {
      n_din++;
    }
  }

  for (i = 0; i < n_all_out; i++) {
    const lcec_il2301_entry_t *e = &all_out[i];
    if (e->idx >= 0x7000 && e->idx < 0x8000 &&
        e->sidx > 0 && e->bitlen == 1) {
      n_dout++;
    }
  }

  rtapi_print_msg(RTAPI_MSG_INFO,
      LCEC_MSG_PFX "IL2301 %s.%s: discovered %d digital input(s), %d digital output(s)\n",
      master->name, slave->name, n_din, n_dout);

  hal_data->din = lcec_din_allocate_channels(n_din);
  hal_data->dout = lcec_dout_allocate_channels(n_dout);
  if (n_din > 0 && hal_data->din == NULL) return -ENOMEM;
  if (n_dout > 0 && hal_data->dout == NULL) return -ENOMEM;

  for (i = 0; i < n_all_in; i++) {
    const lcec_il2301_entry_t *e = &all_in[i];

    if (e->idx == 0xf100 && e->sidx == 0x01 && e->bitlen == 16) {
      lcec_pdo_init(slave, e->idx, e->sidx,
          &hal_data->state_pdo_os, NULL);
      hal_data->state_pdo_valid = 1;

      err = lcec_pin_newf(HAL_U32, HAL_OUT,
          (void **)&hal_data->coupler_state,
          "%s.%s.%s.coupler-state", LCEC_MODULE_NAME,
          master->name, slave->name);
      if (err) return err;

    } else if (e->idx >= 0x6000 && e->idx < 0x7000 &&
        e->sidx > 0 && e->bitlen == 1) {
      int box = (e->idx - 0x6000) / 0x10;
      int chan = (box == 0) ? (e->sidx - 1) : e->sidx;
      char *name = LCEC_HAL_ALLOCATE_STRING(LCEC_IL2301_PIN_NAME_LEN);

      if (name == NULL) return -ENOMEM;
      rtapi_snprintf(name, LCEC_IL2301_PIN_NAME_LEN,
          "%d.din-%d", box, chan);

      hal_data->din->channels[din_i] =
          lcec_din_register_channel_named(slave, e->idx, e->sidx, name);
      if (hal_data->din->channels[din_i] == NULL) return -EIO;
      din_i++;

    } else if (e->idx != 0x0000) {
      rtapi_print_msg(RTAPI_MSG_DBG,
          LCEC_MSG_PFX "IL2301 %s.%s: unsupported input entry 0x%04x:%02x (%u bit) skipped\n",
          master->name, slave->name, e->idx, e->sidx,
          (unsigned int)e->bitlen);
    }
  }

  for (i = 0; i < n_all_out; i++) {
    const lcec_il2301_entry_t *e = &all_out[i];

    if (e->idx == 0xf200 && e->sidx == 0x01 && e->bitlen == 16) {
      lcec_pdo_init(slave, e->idx, e->sidx,
          &hal_data->ctrl_pdo_os, NULL);
      hal_data->ctrl_pdo_valid = 1;

    } else if (e->idx >= 0x7000 && e->idx < 0x8000 &&
        e->sidx > 0 && e->bitlen == 1) {
      int box = (e->idx - 0x7000) / 0x10;
      int chan = (box == 0) ? (e->sidx - 1) : e->sidx;
      char *name = LCEC_HAL_ALLOCATE_STRING(LCEC_IL2301_PIN_NAME_LEN);

      if (name == NULL) return -ENOMEM;
      rtapi_snprintf(name, LCEC_IL2301_PIN_NAME_LEN,
          "%d.dout-%d", box, chan);

      hal_data->dout->channels[dout_i] =
          lcec_dout_register_channel_named(slave, e->idx, e->sidx, name);
      if (hal_data->dout->channels[dout_i] == NULL) return -EIO;
      dout_i++;

    } else if (e->idx != 0x0000) {
      rtapi_print_msg(RTAPI_MSG_DBG,
          LCEC_MSG_PFX "IL2301 %s.%s: unsupported output entry 0x%04x:%02x (%u bit) skipped\n",
          master->name, slave->name, e->idx, e->sidx,
          (unsigned int)e->bitlen);
    }
  }

  if (!hal_data->ctrl_pdo_valid) {
    rtapi_print_msg(RTAPI_MSG_ERR,
        LCEC_MSG_PFX "IL2301 %s.%s: mandatory control word 0xF200:01 (16 bit) not found in RxPDO mapping\n",
        master->name, slave->name);
    return -ENODEV;
  }

  if (!hal_data->state_pdo_valid) {
    rtapi_print_msg(RTAPI_MSG_WARN,
        LCEC_MSG_PFX "IL2301 %s.%s: status word 0xF100:01 (16 bit) not found; coupler-state pin is unavailable\n",
        master->name, slave->name);
  }

  return 0;
}

static void lcec_il2301_read(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_il2301_data_t *hal_data = (lcec_il2301_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;

  (void)period;

  if (!slave->state.operational) return;

  if (hal_data->state_pdo_valid && hal_data->coupler_state != NULL) {
    LCEC_PIN_U32_SET(hal_data->coupler_state, EC_READ_U16(pd + hal_data->state_pdo_os));
  }

  if (hal_data->din != NULL) lcec_din_read_all(slave, hal_data->din);
}

static void lcec_il2301_write(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_il2301_data_t *hal_data = (lcec_il2301_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;

  (void)period;

  if (!slave->state.operational) return;

  if (hal_data->ctrl_pdo_valid) {
    EC_WRITE_U16(pd + hal_data->ctrl_pdo_os, 0x0001);
  }

  if (hal_data->dout != NULL) lcec_dout_write_all(slave, hal_data->dout);
}
