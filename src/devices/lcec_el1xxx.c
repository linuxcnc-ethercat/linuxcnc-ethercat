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
/// @brief Driver for Beckhoff EL1xxx digital input modules

#include "../lcec.h"
#include "lcec_class_din.h"

static int lcec_el1xxx_init(int comp_id, lcec_slave_t *slave);

static lcec_typelist_t types[] = {
    {"EL1002", LCEC_BECKHOFF_VID, 0x03EA3052, 0, NULL, lcec_el1xxx_init, NULL, 2},
    {"EL1004", LCEC_BECKHOFF_VID, 0x03EC3052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1008", LCEC_BECKHOFF_VID, 0x03F03052, 0, NULL, lcec_el1xxx_init, NULL, 8},
    {"EL1012", LCEC_BECKHOFF_VID, 0x03F43052, 0, NULL, lcec_el1xxx_init, NULL, 2},
    {"EL1014", LCEC_BECKHOFF_VID, 0x03F63052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1018", LCEC_BECKHOFF_VID, 0x03FA3052, 0, NULL, lcec_el1xxx_init, NULL, 8},
    {"EL1024", LCEC_BECKHOFF_VID, 0x04003052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1034", LCEC_BECKHOFF_VID, 0x040A3052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1084", LCEC_BECKHOFF_VID, 0x043C3052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1088", LCEC_BECKHOFF_VID, 0x04403052, 0, NULL, lcec_el1xxx_init, NULL, 8},
    {"EL1094", LCEC_BECKHOFF_VID, 0x04463052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1098", LCEC_BECKHOFF_VID, 0x044A3052, 0, NULL, lcec_el1xxx_init, NULL, 8},
    {"EL1104", LCEC_BECKHOFF_VID, 0x04503052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1114", LCEC_BECKHOFF_VID, 0x045A3052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1124", LCEC_BECKHOFF_VID, 0x04643052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1134", LCEC_BECKHOFF_VID, 0x046E3052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1144", LCEC_BECKHOFF_VID, 0x04783052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1804", LCEC_BECKHOFF_VID, 0x070C3052, 0, NULL, lcec_el1xxx_init, NULL, 4},
    {"EL1808", LCEC_BECKHOFF_VID, 0x07103052, 0, NULL, lcec_el1xxx_init, NULL, 8},
    {"EL1809", LCEC_BECKHOFF_VID, 0x07113052, 0, NULL, lcec_el1xxx_init, NULL, 16},
    {"EL1819", LCEC_BECKHOFF_VID, 0x071B3052, 0, NULL, lcec_el1xxx_init, NULL, 16},
    {"EP1008", LCEC_BECKHOFF_VID, 0x03f04052, 0, NULL, lcec_el1xxx_init, NULL, 8},
    {"EP1018", LCEC_BECKHOFF_VID, 0x03fa4052, 0, NULL, lcec_el1xxx_init, NULL, 8},
    {"EP1819", LCEC_BECKHOFF_VID, 0x071b4052, 0, NULL, lcec_el1xxx_init, NULL, 16},
    {NULL},
};

ADD_TYPES(types)

static void lcec_el1xxx_read(lcec_slave_t *slave, long period);

static int lcec_el1xxx_init(int comp_id, lcec_slave_t *slave) {
  lcec_class_din_channels_t *hal_data;
  unsigned int i;

  // initialize callbacks
  slave->proc_read = lcec_el1xxx_read;

  hal_data = lcec_din_allocate_channels(slave->flags);
  if (hal_data == NULL) {
    return -EIO;
  }
  slave->hal_data = hal_data;

  // initialize channels
  for (i = 0; i < slave->flags; i++) {
    hal_data->channels[i] = lcec_din_register_channel(slave, i, 0x6000 + (i << 4), 0x01);

    if (hal_data->channels[i] == NULL) {
      return -EIO;
    }
  }

  return 0;
}

static void lcec_el1xxx_read(lcec_slave_t *slave, long period) {
  lcec_class_din_channels_t *hal_data = (lcec_class_din_channels_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  lcec_din_read_all(slave, hal_data);
}
