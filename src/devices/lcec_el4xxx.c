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
/// @brief Driver for Beckhoff El4xxx Analog output modules

#include "../lcec.h"
#include "lcec_class_aout.h"

static int lcec_el4xxx_init(int comp_id, lcec_slave_t *slave);

/// Flags for describing devices
#define F_S11         1 << 8  ///< Uses subindex 11 instead of sub-index 1 for ports.
#define F_CHANNELS(x) (x)     ///< Number of output channels

#define OUTPORTS(flag) ((flag)&0xf)  // Number of output channels

/// Macro to avoid repeating all of the unchanging fields in
/// `lcec_typelist_t`.
#define BECKHOFF_AOUT_DEVICE(name, pid, flags) \
  { name, LCEC_BECKHOFF_VID, pid, 0, NULL, lcec_el4xxx_init, NULL, flags }

static lcec_typelist_t types[] = {
    // analog out, 1ch, 12 bits
    BECKHOFF_AOUT_DEVICE("EL4001", 0x0fa13052, F_CHANNELS(1)),
    BECKHOFF_AOUT_DEVICE("EL4011", 0x0fab3052, F_CHANNELS(1)),
    BECKHOFF_AOUT_DEVICE("EL4021", 0x0fb53052, F_CHANNELS(1)),
    BECKHOFF_AOUT_DEVICE("EL4031", 0x0fbf3052, F_CHANNELS(1)),
    // analog out, 2ch, 12 bits
    BECKHOFF_AOUT_DEVICE("EL4002", 0x04a23052, F_CHANNELS(2)),
    BECKHOFF_AOUT_DEVICE("EL4012", 0x0fac3052, F_CHANNELS(2)),
    BECKHOFF_AOUT_DEVICE("EL4022", 0x0fb63052, F_CHANNELS(2)),
    BECKHOFF_AOUT_DEVICE("EL4032", 0x0fc03052, F_CHANNELS(2)),
    BECKHOFF_AOUT_DEVICE("EJ4002", 0x0fa22852, F_CHANNELS(2)),
    // analog out, 4ch, 12 bits
    BECKHOFF_AOUT_DEVICE("EL4004", 0x0fa43052, F_CHANNELS(4)),
    BECKHOFF_AOUT_DEVICE("EL4014", 0x0fae3052, F_CHANNELS(4)),
    BECKHOFF_AOUT_DEVICE("EL4024", 0x0fb83052, F_CHANNELS(4)),
    BECKHOFF_AOUT_DEVICE("EL4034", 0x0fc23052, F_CHANNELS(4)),
    BECKHOFF_AOUT_DEVICE("EJ4004", 0x0fa42852, F_CHANNELS(4)),
    BECKHOFF_AOUT_DEVICE("EJ4024", 0x0fb82852, F_CHANNELS(4)),
    // analog out, 8ch, 12 bits
    BECKHOFF_AOUT_DEVICE("EL4008", 0x0fa83052, F_CHANNELS(8)),
    BECKHOFF_AOUT_DEVICE("EL4018", 0x0fb23052, F_CHANNELS(8)),
    BECKHOFF_AOUT_DEVICE("EL4028", 0x0fbc3052, F_CHANNELS(8)),
    BECKHOFF_AOUT_DEVICE("EL4038", 0x0fc63052, F_CHANNELS(8)),
    BECKHOFF_AOUT_DEVICE("EJ4008", 0x0fa82852, F_CHANNELS(8)),
    BECKHOFF_AOUT_DEVICE("EJ4018", 0x0fb22852, F_CHANNELS(8)),
    // analog out, 2ch, 16 bits
    BECKHOFF_AOUT_DEVICE("EL4102", 0x10063052, F_CHANNELS(2)),
    BECKHOFF_AOUT_DEVICE("EL4112", 0x10103052, F_CHANNELS(2)),
    BECKHOFF_AOUT_DEVICE("EL4122", 0x101A3052, F_CHANNELS(2)),
    BECKHOFF_AOUT_DEVICE("EL4132", 0x10243052, F_CHANNELS(2)),
    BECKHOFF_AOUT_DEVICE("EJ4132", 0x10242852, F_CHANNELS(2)),
    // analog out, 4ch, 16 bits
    BECKHOFF_AOUT_DEVICE("EL4104", 0x10083052, F_CHANNELS(4)),
    BECKHOFF_AOUT_DEVICE("EL4114", 0x10123052, F_CHANNELS(4)),
    BECKHOFF_AOUT_DEVICE("EL4124", 0x101c3052, F_CHANNELS(4)),
    BECKHOFF_AOUT_DEVICE("EL4134", 0x10263052, F_CHANNELS(4)),
    BECKHOFF_AOUT_DEVICE("EJ4134", 0x10262852, F_CHANNELS(4)),
    BECKHOFF_AOUT_DEVICE("EP4174", 0x104e4052, F_CHANNELS(4) | F_S11),
    {NULL},
};
ADD_TYPES(types);

static void lcec_el4xxx_write(lcec_slave_t *slave, long period);

static int lcec_el4xxx_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_class_aout_channels_t *hal_data;
  int i;

  // initialize callbacks
  slave->proc_write = lcec_el4xxx_write;

  hal_data = lcec_aout_allocate_channels(OUTPORTS(slave->flags));
  if (hal_data == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  slave->hal_data = hal_data;

  // initialize pins
  for (i = 0; i < OUTPORTS(slave->flags); i++) {
    lcec_class_aout_options_t *options = lcec_aout_options();
    options->value_sidx = 0x01;
    if (slave->flags & F_S11) options->value_sidx = 0x11;

    hal_data->channels[i] = lcec_aout_register_channel(slave, i, 0x7000 + (i << 4), options);
  }

  return 0;
}

static void lcec_el4xxx_write(lcec_slave_t *slave, long period) {
  lcec_class_aout_channels_t *hal_data = (lcec_class_aout_channels_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  lcec_aout_write_all(slave, hal_data);
}
