//
//    Copyright (C) 2026 SyncTwin <info@synctwin.ru>
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
/// @brief Driver for Wecon CiA 402 servo drives (VD3E).
///
/// Single-axis EtherCAT servo drive, CoE only. Vendor 0x00000eff,
/// product 0x0d3e0001, revision 0x00000073, matching the vendor ESI
/// "Wecon VD3E EtherCAT Servo V1.15.0.xml".
///
/// PDO layout: the drive assigns exactly one RxPDO and one TxPDO, and
/// every mapping object holds at most 10 entries. A longer mapping is
/// rejected and the slave never reaches OP, hence rxpdolimit/txpdolimit
/// = 10 below.
///
/// DC: assignActivate 0x300, SYNC0 at the master period, shifted by half
/// the cycle. The vendor manual asks for the PDI advanced by 50% of the
/// SYNC0 period, and fault E.101 ("ECAT sync error") names a wrong
/// master SYNC Shift Time as its first cause. Override with <dcConf>.
///
/// Vendor caveats (0x608f/0x6092 absent, encoder resolution only via
/// 0x201e:0x34, 0x603f is a decimal vendor fault code, no SDO complete
/// access, VD5/VD5L need revision matching) are documented in
/// documentation/wecon.md.

#include "../lcec.h"
#include "lcec_class_cia402.h"

static int lcec_wecon_init(int comp_id, lcec_slave_t *slave);

static const lcec_modparam_desc_t modparams_perchannel[] = {
    {NULL},
};

static const lcec_modparam_desc_t modparams_base[] = {
    {NULL},
};

static const lcec_modparam_doc_t chan_docs[] = {
    {NULL},
};

static const lcec_modparam_doc_t base_docs[] = {
    {NULL},
};

static lcec_typelist_t types[] = {
    {"VD3E", LCEC_WECON_VID, 0x0d3e0001, 0, NULL, lcec_wecon_init, NULL, 0},
    {NULL},
};
ADD_TYPES_WITH_CIA402_MODPARAMS(types, 1, modparams_perchannel, modparams_base, chan_docs, base_docs)

static void lcec_wecon_read(lcec_slave_t *slave, long period);
static void lcec_wecon_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
} lcec_wecon_data_t;

static int lcec_wecon_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_wecon_data_t *hal_data;

  slave->proc_read = lcec_wecon_read;
  slave->proc_write = lcec_wecon_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_wecon_data_t);
  slave->hal_data = hal_data;

  // Apply default Distributed Clock settings if not already set. Values
  // come from the ESI DC OpMode "DC"; the SYNC0 shift follows the
  // manual's recommendation of advancing the PDI by 50% of the SYNC0
  // period (see the file comment on E.101).
  if (slave->dc_conf == NULL) {
    lcec_slave_dc_t *dc = LCEC_HAL_ALLOCATE(lcec_slave_dc_t);
    dc->assignActivate = 0x300;
    dc->sync0Cycle = master->app_time_period;
    dc->sync0Shift = master->app_time_period / 2;
    slave->dc_conf = dc;
  }

  lcec_class_cia402_options_t *options = lcec_cia402_options();

  // One RxPDO and one TxPDO, at most 10 entries each — see the file comment.
  options->rxpdolimit = 10;
  options->txpdolimit = 10;
  options->channels = 1;

  options->channel[0]->enable_opmode = 1;
  options->channel[0]->enable_csp = 1;
  options->channel[0]->enable_csv = 1;
  options->channel[0]->enable_cst = 1;
  options->channel[0]->enable_target_torque = 1;
  options->channel[0]->enable_actual_torque = 1;
  options->channel[0]->enable_actual_following_error = 1;
  // 0x603f ships in the factory TxPDO 0x1b01, so exporting it costs nothing.
  options->channel[0]->enable_error_code = 1;

  lcec_syncs_t *syncs = lcec_cia402_init_sync(slave, options);
  lcec_cia402_add_output_sync(slave, syncs, options);
  lcec_cia402_add_input_sync(slave, syncs, options);
  slave->sync_info = &syncs->syncs[0];

  hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);
  hal_data->cia402->channels[0] = lcec_cia402_register_channel(slave, 0x6000, options->channel[0]);

  return 0;
}

static void lcec_wecon_read(lcec_slave_t *slave, long period) {
  lcec_wecon_data_t *hal_data = (lcec_wecon_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_read_all(slave, hal_data->cia402);
}

static void lcec_wecon_write(lcec_slave_t *slave, long period) {
  lcec_wecon_data_t *hal_data = (lcec_wecon_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_write_all(slave, hal_data->cia402);
}
