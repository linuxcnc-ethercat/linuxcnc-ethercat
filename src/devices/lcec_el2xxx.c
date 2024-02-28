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
/// @brief Driver for Beckhoff EL2xxx Digital output modules

#include "../lcec.h"
#include "lcec_class_dout.h"

static int lcec_el2xxx_init(int comp_id, lcec_slave_t *slave);

static lcec_typelist_t types[] = {
    {"EL2002", LCEC_BECKHOFF_VID, 0x07D23052, 0, NULL, lcec_el2xxx_init, NULL, 2},
    {"EL2004", LCEC_BECKHOFF_VID, 0x07D43052, 0, NULL, lcec_el2xxx_init, NULL, 4},
    {"EL2008", LCEC_BECKHOFF_VID, 0x07D83052, 0, NULL, lcec_el2xxx_init, NULL, 8},
    {"EL2022", LCEC_BECKHOFF_VID, 0x07E63052, 0, NULL, lcec_el2xxx_init, NULL, 2},
    {"EL2024", LCEC_BECKHOFF_VID, 0x07E83052, 0, NULL, lcec_el2xxx_init, NULL, 4},
    {"EL2032", LCEC_BECKHOFF_VID, 0x07F03052, 0, NULL, lcec_el2xxx_init, NULL, 2},
    {"EL2034", LCEC_BECKHOFF_VID, 0x07F23052, 0, NULL, lcec_el2xxx_init, NULL, 4},
    {"EL2042", LCEC_BECKHOFF_VID, 0x07FA3052, 0, NULL, lcec_el2xxx_init, NULL, 2},
    {"EL2084", LCEC_BECKHOFF_VID, 0x08243052, 0, NULL, lcec_el2xxx_init, NULL, 4},
    {"EL2088", LCEC_BECKHOFF_VID, 0x08283052, 0, NULL, lcec_el2xxx_init, NULL, 8},
    {"EL2124", LCEC_BECKHOFF_VID, 0x084C3052, 0, NULL, lcec_el2xxx_init, NULL, 4},
    {"EL2612", LCEC_BECKHOFF_VID, 0x0A343052, 0, NULL, lcec_el2xxx_init, NULL, 2},
    {"EL2622", LCEC_BECKHOFF_VID, 0x0A3E3052, 0, NULL, lcec_el2xxx_init, NULL, 2},
    {"EL2634", LCEC_BECKHOFF_VID, 0x0A4A3052, 0, NULL, lcec_el2xxx_init, NULL, 4},
    {"EL2652", LCEC_BECKHOFF_VID, 0x0A5C3052, 0, NULL, lcec_el2xxx_init, NULL, 2},
    {"EL2808", LCEC_BECKHOFF_VID, 0x0AF83052, 0, NULL, lcec_el2xxx_init, NULL, 8},
    {"EL2798", LCEC_BECKHOFF_VID, 0x0AEE3052, 0, NULL, lcec_el2xxx_init, NULL, 8},
    {"EL2809", LCEC_BECKHOFF_VID, 0x0AF93052, 0, NULL, lcec_el2xxx_init, NULL, 16},
    {"EP2008", LCEC_BECKHOFF_VID, 0x07D84052, 0, NULL, lcec_el2xxx_init, NULL, 8},
    {"EP2028", LCEC_BECKHOFF_VID, 0x07EC4052, 0, NULL, lcec_el2xxx_init, NULL, 8},
    {"EP2809", LCEC_BECKHOFF_VID, 0x0AF94052, 0, NULL, lcec_el2xxx_init, NULL, 16},
    {NULL},
};
ADD_TYPES(types);

static void lcec_el2xxx_write(lcec_slave_t *slave, long period);

static int lcec_el2xxx_init(int comp_id, lcec_slave_t *slave) {
  lcec_class_dout_channels_t *hal_data;
  int i;

  // initialize callbacks
  slave->proc_write = lcec_el2xxx_write;

  hal_data = lcec_dout_allocate_channels(slave->flags);
  if (hal_data == NULL) {
    return -EIO;
  }
  slave->hal_data = hal_data;

  // initialize channels
  for (i = 0; i < slave->flags; i++) {
    hal_data->channels[i] = lcec_dout_register_channel(slave, i, 0x7000 + (i << 4), 0x01);
  }

  return 0;
}

static void lcec_el2xxx_write(lcec_slave_t *slave, long period) {
  lcec_class_dout_channels_t *hal_data = (lcec_class_dout_channels_t *)slave->hal_data;

  if (!slave->state.operational) {
    return;
  }
  lcec_dout_write_all(slave, hal_data);
}
