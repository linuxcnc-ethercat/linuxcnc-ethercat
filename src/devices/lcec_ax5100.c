//    Copyright (C) 2018 Sascha Ittner <sascha.ittner@modusoft.de>
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
/// @brief Driver for Beckhoff AX5100 Servo controllers

#include "lcec_ax5100.h"

#include "../lcec.h"

static int lcec_ax5100_init(int comp_id, lcec_slave_t *slave);

static lcec_modparam_desc_t lcec_ax5100_modparams[] = {
  {"enableFB2", LCEC_AX5_PARAM_ENABLE_FB2, MODPARAM_TYPE_BIT}, {"enableDiag", LCEC_AX5_PARAM_ENABLE_DIAG, MODPARAM_TYPE_BIT}, {NULL},};

static lcec_typelist_t types[] = {
    // AX5000 servo drives
    {"AX5101", LCEC_BECKHOFF_VID, 0x13ed6012, 0, lcec_ax5100_preinit, lcec_ax5100_init, lcec_ax5100_modparams},
    {"AX5103", LCEC_BECKHOFF_VID, 0x13ef6012, 0, lcec_ax5100_preinit, lcec_ax5100_init, lcec_ax5100_modparams},
    {"AX5106", LCEC_BECKHOFF_VID, 0x13f26012, 0, lcec_ax5100_preinit, lcec_ax5100_init, lcec_ax5100_modparams},
    {"AX5112", LCEC_BECKHOFF_VID, 0x13f86012, 0, lcec_ax5100_preinit, lcec_ax5100_init, lcec_ax5100_modparams},
    {"AX5118", LCEC_BECKHOFF_VID, 0x13fe6012, 0, lcec_ax5100_preinit, lcec_ax5100_init, lcec_ax5100_modparams},
    {NULL},
};
ADD_TYPES(types);

typedef struct {
  lcec_syncs_t syncs;
  lcec_class_ax5_chan_t chan;
} lcec_ax5100_data_t;

static const LCEC_CONF_FSOE_T fsoe_conf = {
    .slave_data_len = 2,
    .master_data_len = 2,
    .data_channels = 1,
};

static void lcec_ax5100_read(lcec_slave_t *slave, long period);
static void lcec_ax5100_write(lcec_slave_t *slave, long period);

/*static*/ int lcec_ax5100_preinit(lcec_slave_t *slave) {
  // check if already initialized
  if (slave->fsoeConf != NULL) {
    return 0;
  }

  // set FSOE conf (this will be used by the corresponding AX5805
  slave->fsoeConf = &fsoe_conf;

  return 0;
}

static int lcec_ax5100_init(int comp_id, lcec_slave_t *slave) {
  lcec_ax5100_data_t *hal_data;
  int err;

  // initialize callbacks
  slave->proc_read = lcec_ax5100_read;
  slave->proc_write = lcec_ax5100_write;

  // alloc hal memory
  hal_data = LCEC_HAL_ALLOCATE(lcec_ax5100_data_t);
  slave->hal_data = hal_data;

  // init subclasses
  if ((err = lcec_class_ax5_init(slave, &hal_data->chan, 0, "")) != 0) {
    return err;
  }

  // initialize sync info
  lcec_syncs_init(slave, &hal_data->syncs);
  lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
  lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_INPUT, EC_WD_DEFAULT);

  lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);
  lcec_syncs_add_pdo_info(&hal_data->syncs, 0x0018);
  lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x0086, 0x01, 16);  // control-word
  lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x0018, 0x01, 32);  // velo-command

  lcec_syncs_add_sync(&hal_data->syncs, EC_DIR_INPUT, EC_WD_DEFAULT);
  lcec_syncs_add_pdo_info(&hal_data->syncs, 0x0010);
  lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x0087, 0x01, 16);  // status word
  lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x0033, 0x01, 32);  // position feedback
  lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x0054, 0x01, 16);  // torque feedback

  if (hal_data->chan.fb2_enabled) {
    lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x0035, 0x01, 32);  // position feedback 2
  }
  if (hal_data->chan.diag_enabled) {
    lcec_syncs_add_pdo_entry(&hal_data->syncs, 0x0186, 0x01, 32);  // diagnostic number
  }
  slave->sync_info = &hal_data->syncs.syncs[0];

  return 0;
}

static void lcec_ax5100_read(lcec_slave_t *slave, long period) {
  lcec_ax5100_data_t *hal_data = (lcec_ax5100_data_t *)slave->hal_data;

  // check inputs
  lcec_class_ax5_read(slave, &hal_data->chan);
}

static void lcec_ax5100_write(lcec_slave_t *slave, long period) {
  lcec_ax5100_data_t *hal_data = (lcec_ax5100_data_t *)slave->hal_data;

  // write outputs
  lcec_class_ax5_write(slave, &hal_data->chan);
}
