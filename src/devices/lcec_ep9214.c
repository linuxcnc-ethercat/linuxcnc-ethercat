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

static int lcec_ep9214_init(int comp_id, lcec_slave_t *slave);

/// @brief Devices supported by this driver.
static lcec_typelist_t types[] = {
    {"EP9214", LCEC_BECKHOFF_VID, 0x23fe4052, .proc_init = lcec_ep9214_init},
    {NULL},
};
ADD_TYPES(types)

#define CHANNELS 4
  
typedef struct {
  hal_bit_t *error_us, *error_up;
  hal_bit_t *warning_us, *warning_up;
  hal_bit_t *poweron_us, *poweron_up;

  hal_float_t *current_limit_us, *current_limit_up; // Milliamps
  hal_float_t current_limit_us_old, current_limit_up_old;
  hal_s32_t *current_limit_type;  // 0x80x0:11: Characteristic.  0->"very fast acting", 1->"fast acting", 2->"slow acting", 3->"time delay"
  hal_s32_t current_limit_type_old;
  
  hal_bit_t *enable_us, *enable_up;
  hal_bit_t *reset_us, *reset_up;

  unsigned int error_us_os, error_us_bp;
  unsigned  int error_up_os, error_up_bp;
  unsigned  int warning_us_os, warning_us_bp;
  unsigned  int warning_up_os, warning_up_bp;
  unsigned  int poweron_us_os, poweron_us_bp;
  unsigned  int poweron_up_os, poweron_up_bp;
  unsigned  int enable_us_os, enable_us_bp;
  unsigned  int enable_up_os, enable_up_bp;
  unsigned  int reset_us_os, reset_us_bp;
  unsigned  int reset_up_os, reset_up_bp;
} lcec_ep9214_channel_data_t;

typedef struct {
  lcec_ep9214_channel_data_t chan[CHANNELS];

  hal_bit_t *global_reset;
  hal_bit_t *error_us, *error_up, *error_temp;
  hal_bit_t *warning_us, *warning_up, *warning_temp;

  unsigned  int global_reset_os, global_reset_bp;
  unsigned  int error_us_os, error_us_bp;
  unsigned  int error_up_os, error_up_bp;
  unsigned  int error_temp_os, error_temp_bp;
  unsigned  int warning_us_os, warning_us_bp;
  unsigned  int warning_up_os, warning_up_bp;
  unsigned  int warning_temp_os, warning_temp_bp;
} lcec_ep9214_data_t;

static const lcec_pindesc_t channel_pins[] = {
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_channel_data_t, error_us), "%s.%s.%s.chan-%d-error-us"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_channel_data_t, error_up), "%s.%s.%s.chan-%d-error-up"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_channel_data_t, warning_us), "%s.%s.%s.chan-%d-warning-us"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_channel_data_t, warning_up), "%s.%s.%s.chan-%d-warning-up"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_channel_data_t, poweron_us), "%s.%s.%s.chan-%d-poweron-us"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_channel_data_t, poweron_up), "%s.%s.%s.chan-%d-poweron-up"},
    {HAL_FLOAT, HAL_IN, offsetof(lcec_ep9214_channel_data_t, current_limit_us), "%s.%s.%s.chan-%d-current-limit-us"},
    {HAL_FLOAT, HAL_IN, offsetof(lcec_ep9214_channel_data_t, current_limit_up), "%s.%s.%s.chan-%d-current-limit-up"},
    {HAL_S32, HAL_IN, offsetof(lcec_ep9214_channel_data_t, current_limit_type), "%s.%s.%s.chan-%d-current-limit-type"},
    {HAL_BIT, HAL_IN, offsetof(lcec_ep9214_channel_data_t, reset_us), "%s.%s.%s.chan-%d-reset-us"},
    {HAL_BIT, HAL_IN, offsetof(lcec_ep9214_channel_data_t, reset_up), "%s.%s.%s.chan-%d-reset-up"},
    {HAL_BIT, HAL_IN, offsetof(lcec_ep9214_channel_data_t, enable_us), "%s.%s.%s.chan-%d-enable-us"},
    {HAL_BIT, HAL_IN, offsetof(lcec_ep9214_channel_data_t, enable_up), "%s.%s.%s.chan-%d-enable-up"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static const lcec_pindesc_t device_pins[] = {
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_data_t, error_us), "%s.%s.%s.error-us"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_data_t, error_up), "%s.%s.%s.error-up"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_data_t, error_temp), "%s.%s.%s.error-temp"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_data_t, warning_us), "%s.%s.%s.warning-us"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_data_t, warning_up), "%s.%s.%s.warning-up"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_ep9214_data_t, warning_temp), "%s.%s.%s.warning-temp"},
    {HAL_BIT, HAL_IN, offsetof(lcec_ep9214_data_t, global_reset), "%s.%s.%s.global-reset"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static void lcec_ep9214_read(lcec_slave_t *slave, long period);
static void lcec_ep9214_write(lcec_slave_t *slave, long period);

/// @brief Initialize an EP9214
static int lcec_ep9214_init(int comp_id, lcec_slave_t *slave) {
  int err;
  lcec_ep9214_data_t *hal_data = LCEC_HAL_ALLOCATE(lcec_ep9214_data_t);
  int chan;
  
  slave->hal_data = hal_data;

  for (chan=0; chan<4; chan++) {
    lcec_ep9214_channel_data_t *c = &(hal_data->chan[chan]);
    
    lcec_pdo_init(slave, 0x6000+16*chan, 1, &(c->error_us_os), &(c->error_us_bp));
    lcec_pdo_init(slave, 0x6000+16*chan, 2, &(c->error_up_os), &(c->error_up_bp));
    lcec_pdo_init(slave, 0x6000+16*chan, 3, &(c->warning_us_os), &(c->warning_us_bp));
    lcec_pdo_init(slave, 0x6000+16*chan, 4, &(c->warning_up_os), &(c->warning_up_bp));
    lcec_pdo_init(slave, 0x6000+16*chan, 5, &(c->poweron_us_os), &(c->poweron_us_bp));
    lcec_pdo_init(slave, 0x6000+16*chan, 6, &(c->poweron_up_os), &(c->poweron_up_bp));

    lcec_pdo_init(slave, 0x7000+16*chan, 1, &(c->enable_us_os), &(c->enable_us_bp));
    lcec_pdo_init(slave, 0x7000+16*chan, 2, &(c->enable_up_os), &(c->enable_up_bp));
    lcec_pdo_init(slave, 0x7000+16*chan, 5, &(c->reset_us_os), &(c->reset_us_bp));
    lcec_pdo_init(slave, 0x7000+16*chan, 6, &(c->reset_up_os), &(c->reset_up_bp));


    if ((err = lcec_pin_newf_list(c, channel_pins, LCEC_MODULE_NAME, slave->master->name, slave->name, chan+1)) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_pin_newf_list for slave %s.%s failed\n", slave->master->name, slave->name); 
      return err;
    }
    
    uint16_t current_us, current_up, current_type;
    uint8_t enable_us, enable_up;
    lcec_read_sdo16(slave, 0x8000+16*chan, 0x11, &current_type);
    lcec_read_sdo16(slave, 0x8000+16*chan, 0x12, &current_us);
    lcec_read_sdo16(slave, 0x8000+16*chan, 0x13, &current_up);
    lcec_read_sdo8(slave, 0x7000+16*chan, 0x1, &enable_us);
    lcec_read_sdo8(slave, 0x7000+16*chan, 0x2, &enable_up);
    *(c->current_limit_type) = current_type;
    *(c->current_limit_us) = current_us / 1000.0;
    *(c->current_limit_up) = current_up / 1000.0;
    *(c->enable_us) = !!enable_us;
    *(c->enable_up) = !!enable_up;
    c->current_limit_type_old = *(c->current_limit_type);
    c->current_limit_us_old = *(c->current_limit_us);
    c->current_limit_up_old = *(c->current_limit_up);
  }

  lcec_pdo_init(slave, 0xf607, 1, &(hal_data->warning_temp_os), &(hal_data->warning_temp_bp));
  lcec_pdo_init(slave, 0xf607, 1, &(hal_data->warning_temp_os), &(hal_data->warning_temp_bp));
  lcec_pdo_init(slave, 0xf607, 2, &(hal_data->error_temp_os), &(hal_data->error_temp_bp));
  lcec_pdo_init(slave, 0xf607, 3, &(hal_data->warning_us_os), &(hal_data->warning_us_bp));
  lcec_pdo_init(slave, 0xf607, 4, &(hal_data->error_us_os), &(hal_data->error_us_bp));
  lcec_pdo_init(slave, 0xf607, 5, &(hal_data->warning_up_os), &(hal_data->warning_up_bp));
  lcec_pdo_init(slave, 0xf607, 6, &(hal_data->error_up_os), &(hal_data->error_up_bp));

  lcec_pdo_init(slave, 0xf707, 4, &(hal_data->global_reset_os), &(hal_data->global_reset_bp));

  if ((err = lcec_pin_newf_list(hal_data, device_pins, LCEC_MODULE_NAME, slave->master->name, slave->name)) != 0) {
    return err;
  }

  slave->proc_read = lcec_ep9214_read;
  slave->proc_write = lcec_ep9214_write;
  return 0;
}

/// @brief Read values from the device.
static void lcec_ep9214_read(lcec_slave_t *slave, long period) {
  uint8_t *pd = slave->master->process_data;
  lcec_ep9214_data_t *hal_data = (lcec_ep9214_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  for (int chan=0;chan<CHANNELS;chan++) {
    lcec_ep9214_channel_data_t *c = &(hal_data->chan[chan]);
    
    *(c->error_us) = EC_READ_BIT(&pd[c->error_us_os], c->error_us_bp);
    *(c->error_up) = EC_READ_BIT(&pd[c->error_up_os], c->error_up_bp);
    *(c->warning_us) = EC_READ_BIT(&pd[c->warning_us_os], c->warning_us_bp);
    *(c->warning_up) = EC_READ_BIT(&pd[c->warning_up_os], c->warning_up_bp);
    *(c->poweron_us) = EC_READ_BIT(&pd[c->poweron_us_os], c->poweron_us_bp);
    *(c->poweron_up) = EC_READ_BIT(&pd[c->poweron_up_os], c->poweron_up_bp);
    *(c->enable_us) = EC_READ_BIT(&pd[c->enable_us_os], c->enable_us_bp);
    *(c->enable_up) = EC_READ_BIT(&pd[c->enable_up_os], c->enable_up_bp);
  }

  *(hal_data->warning_temp) = EC_READ_BIT(&pd[hal_data->warning_temp_os], hal_data->warning_temp_bp);
  *(hal_data->error_temp) = EC_READ_BIT(&pd[hal_data->error_temp_os], hal_data->error_temp_bp);
  *(hal_data->warning_us) = EC_READ_BIT(&pd[hal_data->warning_us_os], hal_data->warning_us_bp);
  *(hal_data->error_us) = EC_READ_BIT(&pd[hal_data->error_us_os], hal_data->error_us_bp);
  *(hal_data->warning_us) = EC_READ_BIT(&pd[hal_data->warning_up_os], hal_data->warning_up_bp);
  *(hal_data->error_us) = EC_READ_BIT(&pd[hal_data->error_up_os], hal_data->error_up_bp);
}

/// @brief Write values to the device.
static void lcec_ep9214_write(lcec_slave_t *slave, long period) {
  uint8_t *pd = slave->master->process_data;
  lcec_ep9214_data_t *hal_data = (lcec_ep9214_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  for (int chan=0;chan<CHANNELS;chan++) {
    
  }

}
