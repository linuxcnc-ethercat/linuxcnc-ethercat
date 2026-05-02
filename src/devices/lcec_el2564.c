//
//    Copyright (C) 2025 LinuxCNC EtherCAT
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
//    First initiation by medikusDKFZ - https://github.com/medicusdkfz
//    Optimized and commited by mintracer - miniwinis Bastelbude - https://github.com/mintracer

/// @file
/// @brief Driver for Beckhoff EL2564 4-channel PWM LED output

#include "../lcec.h"

#define LCEC_EL2564_VID LCEC_BECKHOFF_VID
#define LCEC_EL2564_PID 0x0a043052
#define LCEC_EL2564_CHANS 4

#define LCEC_EL2564_PWM_MAX_DC 0xffff

typedef struct {
  hal_float_t *pwm;
  hal_bit_t *enable;
  hal_bit_t *warning;
  hal_bit_t *error;

  hal_float_t scale;
  hal_float_t offset;
  hal_float_t gamma;
  hal_float_t ramp_time;

  unsigned int pwm_pdo_os;
  unsigned int warning_pdo_os;
  unsigned int warning_pdo_bp;
  unsigned int error_pdo_os;
  unsigned int error_pdo_bp;

} lcec_el2564_chan_t;

typedef struct {
  lcec_el2564_chan_t chans[LCEC_EL2564_CHANS];

  hal_u32_t frequency;
  hal_s32_t master_gain;
  int32_t   master_gain_prev;
} lcec_el2564_data_t;

static const lcec_pindesc_t slave_pins[] = {
  { HAL_FLOAT, HAL_IN, offsetof(lcec_el2564_chan_t, pwm), "%s.%s.%s.pwm-%d" },
  { HAL_BIT, HAL_IN, offsetof(lcec_el2564_chan_t, enable), "%s.%s.%s.enable-%d" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_el2564_chan_t, warning), "%s.%s.%s.warning-%d" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_el2564_chan_t, error), "%s.%s.%s.error-%d" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static const lcec_paramdesc_t slave_params[] = {
  { HAL_FLOAT, HAL_RW, offsetof(lcec_el2564_chan_t, scale), "%s.%s.%s.scale-%d" },
  { HAL_FLOAT, HAL_RW, offsetof(lcec_el2564_chan_t, offset), "%s.%s.%s.offset-%d" },
  { HAL_FLOAT, HAL_RO, offsetof(lcec_el2564_chan_t, gamma), "%s.%s.%s.gamma-%d" },
  { HAL_FLOAT, HAL_RO, offsetof(lcec_el2564_chan_t, ramp_time), "%s.%s.%s.ramp-time-%d" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static const lcec_paramdesc_t slave_params_global[] = {
  { HAL_U32, HAL_RO, offsetof(lcec_el2564_data_t, frequency), "%s.%s.%s.frequency" },
  { HAL_S32, HAL_RW, offsetof(lcec_el2564_data_t, master_gain), "%s.%s.%s.master-gain" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static void lcec_el2564_read(lcec_slave_t *slave, long period);
static void lcec_el2564_write(lcec_slave_t *slave, long period);

static int lcec_el2564_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  lcec_el2564_data_t *hal_data;
  lcec_el2564_chan_t *chan;
  int i;
  int err;

  // initialize callbacks
  slave->proc_read = lcec_el2564_read;
  slave->proc_write = lcec_el2564_write;

  // alloc hal memory
  hal_data = LCEC_HAL_ALLOCATE(lcec_el2564_data_t);
  slave->hal_data = hal_data;

  // EL2564 does not support PDO reconfiguration (CoE: Enable PDO Configuration: no)
  // We must use the default mapping that the slave provides

  // initialize all 4 channels
  for (i = 0; i < LCEC_EL2564_CHANS; i++) {
    chan = &hal_data->chans[i];

    // initialize PDO entries
    lcec_pdo_init(slave, 0x7000 + (i << 4), 0x11, &chan->pwm_pdo_os, NULL);
    lcec_pdo_init(slave, 0x6000 + (i << 4), 0x06, &chan->warning_pdo_os, &chan->warning_pdo_bp);
    lcec_pdo_init(slave, 0x6000 + (i << 4), 0x07, &chan->error_pdo_os, &chan->error_pdo_bp);

    // export pins
    if ((err = lcec_pin_newf_list(chan, slave_pins, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0) {
      return err;
    }

    // export parameters
    if ((err = lcec_param_newf_list(chan, slave_params, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0) {
      return err;
    }

    // initialize parameters
    chan->scale = 0.5;
    chan->offset = 0.0;
    chan->gamma = 1.0;     
    chan->ramp_time = 0.0; 
  }

  // export global parameters
  if ((err = lcec_param_newf_list(hal_data, slave_params_global, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  // Read current values from SDOs
  uint8_t sdo_buf[4];
  
  

  // Read frequency (0xf819:11, uint32)
  if (lcec_read_sdo(slave, 0xf819, 0x11, sdo_buf, 4) == 0) {
    hal_data->frequency = EC_READ_U32(sdo_buf);
  } else {
    hal_data->frequency = 5000;    // Default: 5kHz
  }

  // Read master gain (0xf819:12, int16)
  if (lcec_read_sdo(slave, 0xf819, 0x12, sdo_buf, 2) == 0) {
    hal_data->master_gain = EC_READ_S16(sdo_buf);
  } else {
    hal_data->master_gain = 32767; // Default: 100%
  }
  hal_data->master_gain_prev = hal_data->master_gain;
  // Read per-channel parameters
  for (i = 0; i < LCEC_EL2564_CHANS; i++) {
    chan = &hal_data->chans[i];
    uint16_t sdo_index = 0x8000 + (i << 4);

    // Read gamma (0x800x:24, float). SDO is IEEE 32-bit; HAL pin is 64-bit
    // double, so go through a float temporary instead of memcpy'ing 4 bytes
    // into an 8-byte location.
    if (lcec_read_sdo(slave, sdo_index, 0x24, sdo_buf, 4) == 0) {
      float f;
      memcpy(&f, sdo_buf, 4);
      chan->gamma = f;
    }
    // Read ramp time (0x800x:25, float)
    if (lcec_read_sdo(slave, sdo_index, 0x25, sdo_buf, 4) == 0) {
      float f;
      memcpy(&f, sdo_buf, 4);
      chan->ramp_time = f;
    }
  }

  return 0;
}

static void lcec_el2564_read(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_el2564_data_t *hal_data = (lcec_el2564_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;
  lcec_el2564_chan_t *chan;
  int i;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  // read all channels
  for (i = 0; i < LCEC_EL2564_CHANS; i++) {
    chan = &hal_data->chans[i];

    // read status bits
    *(chan->warning) = EC_READ_BIT(&pd[chan->warning_pdo_os], chan->warning_pdo_bp);
    *(chan->error) = EC_READ_BIT(&pd[chan->error_pdo_os], chan->error_pdo_bp);
  }
}

static void lcec_el2564_write(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_el2564_data_t *hal_data = (lcec_el2564_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;
  lcec_el2564_chan_t *chan;
  int i;
  int32_t value;
  double duty;
  uint8_t sdo_buf[4];

  // Check if master_gain changed and write via SDO
if (hal_data->master_gain != hal_data->master_gain_prev) {
    int16_t gain = hal_data->master_gain;
    if (gain < 0) gain = 0;
    if (gain > 32767) gain = 32767;

    EC_WRITE_S16(sdo_buf, gain);
    if (lcec_write_sdo(slave, 0xf819, 0x12, sdo_buf, 2) == 0) {
        hal_data->master_gain_prev = gain;  // only remember on success
    }
    hal_data->master_gain = gain;  // make the clamp visible on the HAL pin
}

  // write all channels
  for (i = 0; i < LCEC_EL2564_CHANS; i++) {
    chan = &hal_data->chans[i];

    // calculate PWM value
    if (*(chan->enable)) {
      duty = *(chan->pwm) * chan->scale + chan->offset;

      // clamp to valid range
      if (duty < 0.0) duty = 0.0;
      if (duty > 100.0) duty = 100.0;

      // convert to 16-bit value
      value = (int32_t)((duty / 100.0) * (double)LCEC_EL2564_PWM_MAX_DC);
    } else {
      value = 0;
    }

    // write PWM value
    EC_WRITE_U16(&pd[chan->pwm_pdo_os], value);
  }
}

static lcec_typelist_t types[] = {
    {"EL2564", LCEC_BECKHOFF_VID, LCEC_EL2564_PID, 0, NULL, lcec_el2564_init},
    {NULL},
};
ADD_TYPES(types);
