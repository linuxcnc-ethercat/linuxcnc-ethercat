//
//    Copyright (C) 2024 Scott Laird <scott@sigkill.org>
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
/// @brief Driver for Beckhoff EP9410

#include "../lcec.h"

static int lcec_el9410_init(int comp_id, lcec_slave_t *slave);

/// @brief Devices supported by this driver.
static lcec_typelist_t types[] = {
    {"EL9410", LCEC_BECKHOFF_VID, 0x24c23052, .proc_init = lcec_el9410_init},
    {NULL},
};
ADD_TYPES(types)

typedef struct {
  hal_bit_t *us_undervoltage;
  hal_bit_t *up_undervoltage;
  unsigned int us_undervoltage_os, us_undervoltage_bp;
  unsigned int up_undervoltage_os, up_undervoltage_bp;
} lcec_el9410_data_t;

static const lcec_pindesc_t output_pins[] = {
    {HAL_BIT, HAL_OUT, offsetof(lcec_el9410_data_t, us_undervoltage), "%s.%s.%s.us-undervoltage"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el9410_data_t, up_undervoltage), "%s.%s.%s.up-undervoltage"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static void lcec_el9410_read(lcec_slave_t *slave, long period);

/// @brief Initialize an EL3xxx device.
static int lcec_el9410_init(int comp_id, lcec_slave_t *slave) {
  int err;
  lcec_el9410_data_t *hal_data = LCEC_HAL_ALLOCATE(lcec_el9410_data_t);
  slave->hal_data = hal_data;

  // initialize POD entries
  lcec_pdo_init(slave, 0x6000, 0x01, &hal_data->us_undervoltage_os, &hal_data->us_undervoltage_bp);
  lcec_pdo_init(slave, 0x6010, 0x01, &hal_data->up_undervoltage_os, &hal_data->up_undervoltage_bp);

  if ((err = lcec_pin_newf_list(hal_data, output_pins, LCEC_MODULE_NAME, slave->master->name, slave->name)) != 0) {
    return err;
  }

  LCEC_PIN_BIT_SET(hal_data->us_undervoltage, 0);
  LCEC_PIN_BIT_SET(hal_data->up_undervoltage, 0);

  slave->proc_read = lcec_el9410_read;
  return 0;
}

/// @brief Read values from the device.
static void lcec_el9410_read(lcec_slave_t *slave, long period) {
  uint8_t *pd = slave->master->process_data;
  lcec_el9410_data_t *hal_data = (lcec_el9410_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  LCEC_PIN_BIT_SET(hal_data->us_undervoltage, EC_READ_BIT(&pd[hal_data->us_undervoltage_os], hal_data->us_undervoltage_bp));
  LCEC_PIN_BIT_SET(hal_data->up_undervoltage, EC_READ_BIT(&pd[hal_data->up_undervoltage_os], hal_data->up_undervoltage_bp));
}
