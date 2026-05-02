//
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
/// @brief Driver for Lichuan multi-axis CiA 402 stepper drives.
///
/// Initial support: OL57E-4A (open loop, 4 axes). The closed-loop
/// CL57E-4A variant uses the same protocol with a different control
/// mode SDO; once the PID is confirmed it can be added to the
/// typelist with the same init function.

#include "../lcec.h"
#include "lcec_class_cia402.h"

#define AXES(flags)         ((flags >> 60) & 0xf)
#define PDOINCREMENT(flags) ((flags >> 52) & 0xff)
#define F_AXES(axes)        ((uint64_t)axes << 60)
#define F_PDOINCREMENT(inc) ((uint64_t)inc << 52)

static int lcec_lichuan_init(int comp_id, lcec_slave_t *slave);

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
    {"OL57E-4A", LCEC_LICHUAN_VID, 0x00006000, 0, NULL, lcec_lichuan_init, NULL, F_AXES(4) | F_PDOINCREMENT(0x10)},
    {NULL},
};
ADD_TYPES_WITH_CIA402_MODPARAMS(types, 4, modparams_perchannel, modparams_base, chan_docs, base_docs)

static void lcec_lichuan_read(lcec_slave_t *slave, long period);
static void lcec_lichuan_write(lcec_slave_t *slave, long period);

typedef struct {
  lcec_class_cia402_channels_t *cia402;
} lcec_lichuan_data_t;

static int lcec_lichuan_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_lichuan_data_t *hal_data;
  int channel;

  slave->proc_read = lcec_lichuan_read;
  slave->proc_write = lcec_lichuan_write;

  hal_data = LCEC_HAL_ALLOCATE(lcec_lichuan_data_t);
  slave->hal_data = hal_data;

  // Apply default Distributed Clock settings if not already set.
  // The OL57E-4A advertises DC support; assignActivate=0x300 matches
  // the Rtelligent / generic CiA 402 stepper convention. Override in
  // XML with <dcConf> if your bus needs different values.
  if (slave->dc_conf == NULL) {
    lcec_slave_dc_t *dc = LCEC_HAL_ALLOCATE(lcec_slave_dc_t);
    dc->assignActivate = 0x300;
    dc->sync0Cycle = master->app_time_period;
    slave->dc_conf = dc;
  }

  lcec_class_cia402_options_t *options = lcec_cia402_options();
  // OL57E-4A / CL57E-4A allow up to 12 entries per PDO (0x1600:01..12,
  // 0x1A00:01..12 per the manual).
  options->rxpdolimit = 12;
  options->txpdolimit = 12;

  if (AXES(slave->flags) != 0) {
    options->channels = AXES(slave->flags);
  } else {
    options->channels = 1;
  }

  if (PDOINCREMENT(slave->flags) != 0) {
    options->pdo_increment = PDOINCREMENT(slave->flags);
  } else {
    options->pdo_increment = 1;
  }

  // 0x6077 (actual torque) is NOT available on these open-loop steppers; do
  // not enable enable_actual_torque. Each axis has 3 DI and 2 DO per the
  // CL57E-4A manual.
  for (channel = 0; channel < options->channels; channel++) {
    options->channel[channel]->enable_csp = 1;
    options->channel[channel]->enable_csv = 1;
    options->channel[channel]->enable_pp = 1;
    options->channel[channel]->enable_pv = 1;
    options->channel[channel]->enable_hm = 1;
    options->channel[channel]->enable_actual_following_error = 1;
    options->channel[channel]->enable_error_code = 1;
    options->channel[channel]->enable_digital_input = 1;
    options->channel[channel]->digital_in_channels = 3;
    options->channel[channel]->enable_digital_output = 1;
    options->channel[channel]->digital_out_channels = 2;
  }

  if (options->channels > 1) {
    lcec_cia402_rename_multiaxis_channels(options);
  }

  lcec_syncs_t *syncs = lcec_cia402_init_sync(slave, options);
  lcec_cia402_add_output_sync(slave, syncs, options);
  lcec_cia402_add_input_sync(slave, syncs, options);
  slave->sync_info = &syncs->syncs[0];

  hal_data->cia402 = lcec_cia402_allocate_channels(options->channels);

  for (channel = 0; channel < options->channels; channel++) {
    hal_data->cia402->channels[channel] = lcec_cia402_register_channel(slave, 0x6000 + 0x800 * channel, options->channel[channel]);
  }

  return 0;
}

static void lcec_lichuan_read(lcec_slave_t *slave, long period) {
  lcec_lichuan_data_t *hal_data = (lcec_lichuan_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_read_all(slave, hal_data->cia402);
}

static void lcec_lichuan_write(lcec_slave_t *slave, long period) {
  lcec_lichuan_data_t *hal_data = (lcec_lichuan_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }

  lcec_cia402_write_all(slave, hal_data->cia402);
}
