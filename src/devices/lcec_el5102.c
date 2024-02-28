//
//    Copyright (C) 2012 Sascha Ittner <sascha.ittner@modusoft.de>
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
/// @brief Driver for Beckhoff EL5102 Encoder modules

#include <stdio.h>

#include "../lcec.h"

static int lcec_el5102_init(int comp_id, lcec_slave_t *slave);

static lcec_typelist_t types[] = {
    {"EL5102", LCEC_BECKHOFF_VID, 0x13ee3052, 0, NULL, lcec_el5102_init},
    {NULL},
};
ADD_TYPES(types);

#define LCEC_EL5102_PERIOD_SCALE    500e-9
#define LCEC_EL5102_FREQUENCY_SCALE 5e-2

typedef struct {
  hal_bit_t *ena_latch_c;
  hal_bit_t *ena_latch_ext_pos;
  hal_bit_t *ena_latch_ext_neg;
  hal_bit_t *reset;
  hal_bit_t *inext;
  hal_bit_t *overflow;
  hal_bit_t *underflow;
  hal_bit_t *latch_c_valid;
  hal_bit_t *latch_ext_valid;
  hal_bit_t *set_raw_count;
  hal_s32_t *set_raw_count_val;
  hal_s32_t *raw_count;
  hal_s32_t *raw_latch;
  hal_s32_t *raw_latch2;
  // hal_u32_t *raw_frequency;
  // hal_u32_t *raw_period;
  hal_s32_t *count;
  hal_float_t *pos_scale;
  hal_float_t *pos;
  // hal_float_t *period;
  // hal_float_t *frequency;

  unsigned int count_pdo_os;
  unsigned int latch_pdo_os;
  unsigned int latch2_pdo_os;
  // unsigned int frequency_pdo_os;
  // unsigned int period_pdo_os;

#define PDO_ADDRESSES_BOOL(name) unsigned int name##_os, name##_bp
  PDO_ADDRESSES_BOOL(status_latch_c);
  PDO_ADDRESSES_BOOL(status_latch_extern);
  PDO_ADDRESSES_BOOL(status_set_counter_done);
  PDO_ADDRESSES_BOOL(status_counter_underflow);
  PDO_ADDRESSES_BOOL(status_counter_overflow);
  PDO_ADDRESSES_BOOL(status_input_status);
  PDO_ADDRESSES_BOOL(status_open_circuit);
  PDO_ADDRESSES_BOOL(status_extrapolation_stall);
  PDO_ADDRESSES_BOOL(status_a);
  PDO_ADDRESSES_BOOL(status_b);
  PDO_ADDRESSES_BOOL(status_c);
  PDO_ADDRESSES_BOOL(status_input_gate);
  // PDO_ADDRESSES_BOOL(status_extern_latch);

  PDO_ADDRESSES_BOOL(control_enable_latch_c);
  PDO_ADDRESSES_BOOL(control_enable_latch_extern_pos);
  PDO_ADDRESSES_BOOL(control_enable_latch_extern_neg);
  PDO_ADDRESSES_BOOL(control_set_counter);
  PDO_ADDRESSES_BOOL(control_set_counter_latch_c);
  PDO_ADDRESSES_BOOL(control_set_software_gate);
  PDO_ADDRESSES_BOOL(control_set_counter_latch_extern_pos);
  PDO_ADDRESSES_BOOL(control_set_counter_latch_extern_neg);
  PDO_ADDRESSES_BOOL(control_enable_latch_extern2_pos);
  PDO_ADDRESSES_BOOL(control_enable_latch_extern2_neg);

  unsigned int setval_pdo_os;

  int do_init;
  int16_t last_count;
  double old_scale;
  double scale;

  int last_operational;
} lcec_el5102_channel_data_t;

typedef struct {
  lcec_el5102_channel_data_t channel[2];
} lcec_el5102_data_t;

static const lcec_pindesc_t slave_pins[] = {
    {HAL_BIT, HAL_IO, offsetof(lcec_el5102_channel_data_t, ena_latch_c), "%s.%s.%s.%s-index-c-enable"},
    {HAL_BIT, HAL_IO, offsetof(lcec_el5102_channel_data_t, ena_latch_ext_pos), "%s.%s.%s.%s-index-ext-pos-enable"},
    {HAL_BIT, HAL_IO, offsetof(lcec_el5102_channel_data_t, ena_latch_ext_neg), "%s.%s.%s.%s-index-ext-neg-enable"},
    {HAL_BIT, HAL_IN, offsetof(lcec_el5102_channel_data_t, reset), "%s.%s.%s.%s-reset"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5102_channel_data_t, inext), "%s.%s.%s.%s-inext"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5102_channel_data_t, overflow), "%s.%s.%s.%s-overflow"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5102_channel_data_t, underflow), "%s.%s.%s.%s-underflow"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5102_channel_data_t, latch_c_valid), "%s.%s.%s.%s-latch-c-valid"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el5102_channel_data_t, latch_ext_valid), "%s.%s.%s.%s-latch-ext-valid"},
    {HAL_BIT, HAL_IO, offsetof(lcec_el5102_channel_data_t, set_raw_count), "%s.%s.%s.%s-set-raw-count"},
    {HAL_S32, HAL_IN, offsetof(lcec_el5102_channel_data_t, set_raw_count_val), "%s.%s.%s.%s-set-raw-count-val"},
    {HAL_S32, HAL_OUT, offsetof(lcec_el5102_channel_data_t, raw_count), "%s.%s.%s.%s-raw-count"},
    {HAL_S32, HAL_OUT, offsetof(lcec_el5102_channel_data_t, count), "%s.%s.%s.%s-count"},
    {HAL_S32, HAL_OUT, offsetof(lcec_el5102_channel_data_t, raw_latch), "%s.%s.%s.%s-raw-latch"},
    //{HAL_U32, HAL_OUT, offsetof(lcec_el5102_channel_data_t, raw_frequency), "%s.%s.%s.%s-raw-freq"},
    //{HAL_U32, HAL_OUT, offsetof(lcec_el5102_channel_data_t, raw_period), "%s.%s.%s.%s-raw-period"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_el5102_channel_data_t, pos), "%s.%s.%s.%s-pos"},
    //{HAL_FLOAT, HAL_OUT, offsetof(lcec_el5102_channel_data_t, period), "%s.%s.%s.%s-period"},
    //{HAL_FLOAT, HAL_OUT, offsetof(lcec_el5102_channel_data_t, frequency), "%s.%s.%s.%s-frequency"},
    {HAL_FLOAT, HAL_IO, offsetof(lcec_el5102_channel_data_t, pos_scale), "%s.%s.%s.%s-pos-scale"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static void lcec_el5102_read(lcec_slave_t *slave, long period);
static void lcec_el5102_read_channel(lcec_slave_t *slave, long period, int channel);
static void lcec_el5102_write(lcec_slave_t *slave, long period);
static void lcec_el5102_write_channel(lcec_slave_t *slave, long period, int channel);

static int lcec_el5102_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_el5102_data_t *hal_data;
  int err;

  // initialize callbacks
  slave->proc_read = lcec_el5102_read;
  slave->proc_write = lcec_el5102_write;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_el5102_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_el5102_data_t));
  slave->hal_data = hal_data;

  for (int channel = 0; channel < 2; channel++) {
    // initialize POD entries
    lcec_el5102_channel_data_t *data = &hal_data->channel[channel];

#define PDO_INIT_BOOL(slave, idx, sidx, name) lcec_pdo_init(slave, idx, sidx, &data->name##_os, &data->name##_bp);

    // Input
    lcec_pdo_init(slave, 0x6000 + (channel * 1 << 4), 0x11, &data->count_pdo_os, NULL);
    lcec_pdo_init(slave, 0x6000 + (channel * 1 << 4), 0x12, &data->latch_pdo_os, NULL);
    lcec_pdo_init(slave, 0x6000 + (channel * 1 << 4), 0x22, &data->latch2_pdo_os, NULL);
    // lcec_pdo_init(slave, 0x6000 + (channel * 1 << 4), 0x13, &data->frequency_pdo_os, NULL);
    // lcec_pdo_init(slave, 0x6000 + (channel * 1 << 4), 0x14, &data->period_pdo_os, NULL);

    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0x1, status_latch_c);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0x2, status_latch_extern);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0x3, status_set_counter_done);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0x4, status_counter_underflow);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0x5, status_counter_overflow);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0x6, status_input_status);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0x7, status_open_circuit);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0x8, status_extrapolation_stall);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0x9, status_a);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0xa, status_b);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0xb, status_c);
    PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0xc, status_input_gate);
    // PDO_INIT_BOOL(slave, 0x6000 + (channel * 1 << 4), 0xd, status_extern_latch);

    // Output
    PDO_INIT_BOOL(slave, 0x7000 + (channel * 1 << 4), 0x01, control_enable_latch_c);
    PDO_INIT_BOOL(slave, 0x7000 + (channel * 1 << 4), 0x02, control_enable_latch_extern_pos);
    PDO_INIT_BOOL(slave, 0x7000 + (channel * 1 << 4), 0x03, control_set_counter);
    PDO_INIT_BOOL(slave, 0x7000 + (channel * 1 << 4), 0x04, control_enable_latch_extern_neg);
    PDO_INIT_BOOL(slave, 0x7000 + (channel * 1 << 4), 0x08, control_set_counter_latch_c);
    PDO_INIT_BOOL(slave, 0x7000 + (channel * 1 << 4), 0x09, control_set_software_gate);
    PDO_INIT_BOOL(slave, 0x7000 + (channel * 1 << 4), 0x0a, control_set_counter_latch_extern_pos);
    PDO_INIT_BOOL(slave, 0x7000 + (channel * 1 << 4), 0x0b, control_set_counter_latch_extern_neg);
    PDO_INIT_BOOL(slave, 0x7000 + (channel * 1 << 4), 0x0c, control_enable_latch_extern2_pos);
    PDO_INIT_BOOL(slave, 0x7000 + (channel * 1 << 4), 0x0d, control_enable_latch_extern2_neg);

    lcec_pdo_init(slave, 0x7000 + (channel * 1 << 4), 0x11, &data->setval_pdo_os, NULL);

    // export pins
    char channellabel[16];
    snprintf(channellabel, 16, "enc-%d", channel);
    if ((err = lcec_pin_newf_list(data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name, channellabel)) != 0) {
      return err;
    }

    // initialize pins
    *(data->pos_scale) = 1.0;

    // initialize variables
    data->do_init = 1;
    data->last_count = 0;
    data->old_scale = *(data->pos_scale) + 1.0;
    data->scale = 1.0;
  }

  return 0;
}

static void lcec_el5102_read(lcec_slave_t *slave, long period) {
  lcec_el5102_read_channel(slave, period, 0);
  lcec_el5102_read_channel(slave, period, 1);
}

static void lcec_el5102_read_channel(lcec_slave_t *slave, long period, int channel) {
  lcec_master_t *master = slave->master;
  lcec_el5102_channel_data_t *data = &((lcec_el5102_data_t *)slave->hal_data)->channel[channel];
  uint8_t *pd = master->process_data;
  int16_t raw_count, raw_latch, raw_delta;
  // uint16_t raw_period;
  // uint32_t raw_frequency;

  // wait for slave to be operational
  if (!slave->state.operational) {
    data->last_operational = 0;
    return;
  }

  // check for change in scale value
  if (*(data->pos_scale) != data->old_scale) {
    // scale value has changed, test and update it
    if ((*(data->pos_scale) < 1e-20) && (*(data->pos_scale) > -1e-20)) {
      // value too small, divide by zero is a bad thing
      *(data->pos_scale) = 1.0;
    }
    // save new scale to detect future changes
    data->old_scale = *(data->pos_scale);
    // we actually want the reciprocal
    data->scale = 1.0 / *(data->pos_scale);
  }

  // read raw values
  raw_count = EC_READ_S16(&pd[data->count_pdo_os]);
  raw_latch = EC_READ_S16(&pd[data->latch_pdo_os]);
  // raw_frequency = EC_READ_U32(&pd[data->frequency_pdo_os]);
  // raw_period = EC_READ_U16(&pd[data->period_pdo_os]);

  // Note that there are 13 status bits read above, and we're only
  // using 6 of them here (including "counter set done", below).
  // Someone should review which of the remaining status bits are
  // useful and add pins for them, and then delete the rest.
  *(data->inext) = EC_READ_BIT(&pd[data->status_input_status_os], data->status_input_status_bp);
  *(data->overflow) = EC_READ_BIT(&pd[data->status_counter_overflow_os], data->status_counter_overflow_bp);
  *(data->underflow) = EC_READ_BIT(&pd[data->status_counter_underflow_os], data->status_counter_underflow_bp);
  *(data->latch_ext_valid) = EC_READ_BIT(&pd[data->status_latch_extern_os], data->status_latch_extern_bp);
  *(data->latch_c_valid) = EC_READ_BIT(&pd[data->status_latch_c_os], data->status_latch_c_bp);

  // check for counter set done
  if (EC_READ_BIT(&pd[data->status_set_counter_done_os], data->status_set_counter_done_bp)) {
    data->last_count = raw_count;
    *(data->set_raw_count) = 0;
  }
  // check for operational change of slave
  if (!data->last_operational) {
    data->last_count = raw_count;
  }

  // update raw values
  if (!*(data->set_raw_count)) {
    *(data->raw_count) = raw_count;
    //*(data->raw_frequency) = raw_frequency;
    //*(data->raw_period) = raw_period;
  }

  // handle initialization
  if (data->do_init || *(data->reset)) {
    data->do_init = 0;
    data->last_count = raw_count;
    *(data->count) = 0;
  }

  // handle index
  if (*(data->latch_ext_valid)) {
    *(data->raw_latch) = raw_latch;
    data->last_count = raw_latch;
    *(data->count) = 0;
    *(data->ena_latch_ext_pos) = 0;
    *(data->ena_latch_ext_neg) = 0;
  }
  if (*(data->latch_c_valid)) {
    *(data->raw_latch) = raw_latch;
    data->last_count = raw_latch;
    *(data->count) = 0;
    *(data->ena_latch_c) = 0;
  }

  // compute net counts
  raw_delta = raw_count - data->last_count;
  data->last_count = raw_count;
  *(data->count) += raw_delta;

  // scale count to make floating point position
  *(data->pos) = *(data->count) * data->scale;

  // scale period
  //*(data->frequency) = ((double)(*(data->raw_frequency))) * LCEC_EL5102_FREQUENCY_SCALE;
  //*(data->period) = ((double)(*(data->raw_period))) * LCEC_EL5102_PERIOD_SCALE;

  data->last_operational = 1;
}

static void lcec_el5102_write(lcec_slave_t *slave, long period) {
  lcec_el5102_write_channel(slave, period, 0);
  lcec_el5102_write_channel(slave, period, 1);
}

static void lcec_el5102_write_channel(lcec_slave_t *slave, long period, int channel) {
  lcec_master_t *master = slave->master;
  lcec_el5102_channel_data_t *data = &((lcec_el5102_data_t *)slave->hal_data)->channel[channel];
  uint8_t *pd = master->process_data;

  // Set control bits.  Note that there are 10 of these defined above,
  // but we're only actually using 4 of them.  We should add the
  // remaining ones that are useful here (presumably also adding pins
  // for them), and then delete whatever is left.
  EC_WRITE_BIT(&pd[data->control_set_counter_os], data->control_set_counter_bp, *(data->set_raw_count));
  EC_WRITE_BIT(&pd[data->control_enable_latch_c_os], data->control_enable_latch_c_bp, *(data->ena_latch_c));
  EC_WRITE_BIT(&pd[data->control_enable_latch_extern_pos_os], data->control_enable_latch_extern_pos_bp, *(data->ena_latch_ext_pos));
  EC_WRITE_BIT(&pd[data->control_enable_latch_extern_neg_os], data->control_enable_latch_extern_neg_bp, *(data->ena_latch_ext_neg));

  // set output data
  EC_WRITE_S16(&pd[data->setval_pdo_os], *(data->set_raw_count_val));
}
