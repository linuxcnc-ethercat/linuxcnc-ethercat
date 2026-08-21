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
/// @brief Driver for Inovance IS620N and SV660 servo drives

#include "../lcec.h"
#include "lcec_class_cia402.h"

static int lcec_inovance_init(int comp_id, lcec_slave_t *slave);

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
    {"IS620N", LCEC_INOVANCE_VID, 0x000c0108, 0, NULL, lcec_inovance_init, NULL, 0},
    {"SV660", LCEC_INOVANCE_VID, 0x000c010d, 0, NULL, lcec_inovance_init, NULL, 0},
    {NULL},
};
ADD_TYPES_WITH_CIA402_MODPARAMS(types, 1, modparams_perchannel, modparams_base, chan_docs, base_docs)

static void lcec_inovance_read(lcec_slave_t *slave, long period);
static void lcec_inovance_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
} lcec_inovance_data_t;

static int lcec_inovance_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_inovance_data_t *hal_data;

  slave->proc_read = lcec_inovance_read;
  slave->proc_write = lcec_inovance_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_inovance_data_t);
  slave->hal_data = hal_data;

  // Default DC settings, overridable with <dcConf>.
  if (slave->dc_conf == NULL) {
    lcec_slave_dc_t *dc = LCEC_HAL_ALLOCATE(lcec_slave_dc_t);
    dc->assignActivate = 0x300;
    dc->sync0Cycle = master->app_time_period;
    slave->dc_conf = dc;
  }

  lcec_class_cia402_options_t *options = lcec_cia402_options();

  // These drives assign one PDO per direction, 0x1600 and 0x1A00, and both
  // hold 10 entries at most.  There is no 0x1A01; writing it is an SDO
  // error and the slave stays out of OP.
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

  lcec_syncs_t *syncs = lcec_cia402_init_sync(slave, options);
  lcec_cia402_add_output_sync(slave, syncs, options);
  lcec_cia402_add_input_sync(slave, syncs, options);
  slave->sync_info = &syncs->syncs[0];

  hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);
  hal_data->cia402->channels[0] = lcec_cia402_register_channel(slave, 0x6000, options->channel[0]);

  return 0;
}

static void lcec_inovance_read(lcec_slave_t *slave, long period) {
  lcec_inovance_data_t *hal_data = (lcec_inovance_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_read_all(slave, hal_data->cia402);
}

static void lcec_inovance_write(lcec_slave_t *slave, long period) {
  lcec_inovance_data_t *hal_data = (lcec_inovance_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_write_all(slave, hal_data->cia402);
}
