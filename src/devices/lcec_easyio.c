//
//    Copyright (C) 2011 Sascha Ittner <sascha.ittner@modusoft.de>
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
/// @brief Driver for AB&T EasyIO module.
///
/// 16 din, 16 dout, 4 ain, 2 aout.
/// https://www.bausano.net/en/hardware/easyio.html

#include "../lcec.h"
#include "lcec_class_ain.h"
#include "lcec_class_aout.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

typedef struct {
  lcec_class_din_channels_t *digital_in;
  lcec_class_dout_channels_t *digital_out;
  lcec_class_ain_channels_t *analog_in;
  lcec_class_aout_channels_t *analog_out;
} lcec_easyio_data_t;

static int lcec_easyio_init(int comp_id, lcec_slave_t *slave);

static lcec_typelist_t types[] = {
    {"EasyIO", LCEC_ABET_VID, 0x0debacca, 0, NULL, lcec_easyio_init},
    {NULL},
};
ADD_TYPES(types)

static void lcec_easyio_write(lcec_slave_t *slave, long period);
static void lcec_easyio_read(lcec_slave_t *slave, long period);

static int lcec_easyio_init(int comp_id, lcec_slave_t *slave) {
  lcec_easyio_data_t *hal_data;
  int i;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_easyio_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", slave->master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_easyio_data_t));
  slave->hal_data = hal_data;

  // initialize callbacks
  slave->proc_read = lcec_easyio_read;
  slave->proc_write = lcec_easyio_write;

  hal_data->digital_in = lcec_din_allocate_channels(16);
  hal_data->digital_out = lcec_dout_allocate_channels(16);
  hal_data->analog_in = lcec_ain_allocate_channels(4);
  hal_data->analog_out = lcec_aout_allocate_channels(2);

  // initialize digital channels 0-7
  for (i = 0; i < 8; i++) {
    hal_data->digital_in->channels[i] = lcec_din_register_channel(slave, i, 0x6001, i + 1);
    hal_data->digital_out->channels[i] = lcec_dout_register_channel(slave, i, 0x7001, i + 1);
  }

  // initialize digital channels 8-15.  They're on a different PDO.
  for (i = 8; i < 16; i++) {
    hal_data->digital_in->channels[i] = lcec_din_register_channel(slave, i, 0x6002, i - 7);
    hal_data->digital_out->channels[i] = lcec_dout_register_channel(slave, i, 0x7002, i - 7);
  }

  // Initialize analog in 0-3.
  for (i = 0; i < 4; i++) {
    lcec_class_ain_options_t *ain_opt = lcec_ain_options();

    ain_opt->valueonly = 1;
    ain_opt->value_sidx = i + 1;

    hal_data->analog_in->channels[i] = lcec_ain_register_channel(slave, i, 0x6000, ain_opt);
  }

  // Initialize analog out 0-1.
  for (i = 0; i < 2; i++) {
    lcec_class_aout_options_t *aout_opt = lcec_aout_options();

    aout_opt->value_sidx = i + 1;
    hal_data->analog_out->channels[i] = lcec_aout_register_channel(slave, i, 0x7000, aout_opt);
  }

  return 0;
}

static void lcec_easyio_write(lcec_slave_t *slave, long period) {
  lcec_easyio_data_t *hal_data = (lcec_easyio_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }
  lcec_dout_write_all(slave, hal_data->digital_out);
  lcec_aout_write_all(slave, hal_data->analog_out);
}

static void lcec_easyio_read(lcec_slave_t *slave, long period) {
  lcec_easyio_data_t *hal_data = (lcec_easyio_data_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }
  lcec_din_read_all(slave, hal_data->digital_in);
  lcec_ain_read_all(slave, hal_data->analog_in);
}
