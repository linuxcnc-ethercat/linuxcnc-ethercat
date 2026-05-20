//
//    Copyright (C) 2026 Nathan Hofmeyer
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

/// @file  lcec_el2522.c
/// @brief Driver for Beckhoff EL2522 2-channel pulse train output module
///        Supports continuous positioning mode (similar to CiA402 CSP mode)

#include <ecrt.h>

#include "../lcec.h"
#include "hal.h"
#include "math.h"

static int lcec_el2522_init(int comp_id, lcec_slave_t *slave);
static void lcec_el2522_read(lcec_slave_t *slave, long period);
static void lcec_el2522_write(lcec_slave_t *slave, long period);

#define LCEC_EL2522_CHANNEL_COUNT      2
#define LCEC_EL2522_POS_SCALE_DEFAULT  1.0
#define LCEC_EL2522_MODPARAM_OUTMODE   0
#define LCEC_EL2522_MODPARAM_FREQLIMIT 2

static const lcec_modparam_desc_t modparams_base[] = {
  {"ch1OutputMode", LCEC_EL2522_MODPARAM_OUTMODE, MODPARAM_TYPE_STRING, "FrequencyModulation", "Output mode: FrequencyModulation|PulseDirection|IncrementalEncoder"},
  {"ch1FrequencyLimit", LCEC_EL2522_MODPARAM_FREQLIMIT, MODPARAM_TYPE_U32, "50000", "Maximum output frequency in Hz"},
  {"ch2OutputMode", LCEC_EL2522_MODPARAM_OUTMODE + 1, MODPARAM_TYPE_STRING, "FrequencyModulation", "Output mode: FrequencyModulation|PulseDirection|IncrementalEncoder"},
  {"ch2FrequencyLimit", LCEC_EL2522_MODPARAM_FREQLIMIT + 1, MODPARAM_TYPE_U32, "50000", "Maximum output frequency in Hz"},
    {NULL},
};

static const lcec_lookuptable_int_t lcec_el2522_outmode_table[] = {
    {"FrequencyModulation", 0x00},
    {"PulseDirection", 0x01},
    {"IncrementalEncoder", 0x02},
    {NULL, 0},
};

static lcec_typelist_t lcec_el2522_types[] = {
    {.name = "EL2522",
        .vid = LCEC_BECKHOFF_VID,
        .pid = 0x09da3052,
        .is_fsoe_logic = 0,
        .proc_preinit = NULL,
        .proc_init = lcec_el2522_init,
        .modparams = modparams_base},
    {NULL},
};
ADD_TYPES(lcec_el2522_types);

typedef struct {
  // HAL pins
  hal_float_t *pos_fb;   // pin: position feedback (position units)
  hal_float_t *pos_cmd;  // pin: position command (position units)
  hal_bit_t *enable;     // pin: enable command
  hal_s32_t *count;      // pin: raw count output (steps), useful for commissioning

  // HAL parameters
  hal_float_t pos_scale;  // param: pos scaling factor (steps/unit)

  // PDO offsets and bit positions
  unsigned int target_pdo_os;
  unsigned int count_pdo_os;
  unsigned int enable_pdo_os;
  unsigned int enable_pdo_bp;

  // Internal variables
  uint32_t last_count;
  double last_pos_scale;
  double pos_scale_recip;
  double step_offset;
} lcec_el2522_channel_t;

typedef struct {
  lcec_el2522_channel_t channels[LCEC_EL2522_CHANNEL_COUNT];
} lcec_el2522_data_t;

static const lcec_pindesc_t slave_pins[] = {
    {HAL_FLOAT, HAL_IN, offsetof(lcec_el2522_channel_t, pos_cmd), "%s.%s.%s.ch%d.pos-cmd"},
    {HAL_FLOAT, HAL_OUT, offsetof(lcec_el2522_channel_t, pos_fb), "%s.%s.%s.ch%d.pos-fb"},
    {HAL_S32, HAL_OUT, offsetof(lcec_el2522_channel_t, count), "%s.%s.%s.ch%d.count"},
    {HAL_BIT, HAL_IN, offsetof(lcec_el2522_channel_t, enable), "%s.%s.%s.ch%d.enable"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static const lcec_paramdesc_t slave_params[] = {
    {HAL_FLOAT, HAL_RW, offsetof(lcec_el2522_channel_t, pos_scale), "%s.%s.%s.ch%d.pos-scale"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static ec_pdo_entry_info_t el2522_1601_pdo_entries[] = {
    {0x0000, 0x00, 3},
    {0x7000, 0x04, 1},  // Automatic direction, used as 'enable'
    {0x7000, 0x05, 1},  // Forward
    {0x7000, 0x06, 1},  // Reverse
    {0x0000, 0x00, 10},
    {0x7000, 0x12, 32}, // Target counter value
};

static ec_pdo_entry_info_t el2522_1606_pdo_entries[] = {
    {0x0000, 0x00, 3},
    {0x7010, 0x04, 1},  // Automatic direction, used as 'enable'
    {0x7010, 0x05, 1},  // Forward
    {0x7010, 0x06, 1},  // Reverse
    {0x0000, 0x00, 10},
    {0x7010, 0x12, 32}, // Target counter value
};
static ec_pdo_entry_info_t el2522_160B_pdo_entries[] = {
    {0x0000, 0x00, 2},
    {0x7020, 0x03, 1},  // Set Counter
    {0x0000, 0x00, 12},
    {0x7020, 0x10, 1},  // Reserved
    {0x7020, 0x11, 32}, // Set counter Value
};

static ec_pdo_entry_info_t el2522_160D_pdo_entries[] = {
    {0x0000, 0x00, 2},
    {0x7030, 0x03, 1},  // Set Counter
    {0x0000, 0x00, 12},
    {0x7030, 0x10, 1},  // Reserved
    {0x7030, 0x11, 32}, // Set Counter Value
};

static ec_pdo_entry_info_t el2522_1A03_pdo_entries[] = {
    {0x0000, 0x00, 2},
    {0x6020, 0x03, 1},  // Set counter done
    {0x6020, 0x04, 1},  // Counter underflow
    {0x6020, 0x05, 1},  // Counter overflow
    {0x0000, 0x00, 8},
    {0x6020, 0x0E, 1},  // Sync error
    {0x6020, 0x0F, 1},  // TxPDO State
    {0x6020, 0x10, 1},  // TxPDO Toggle
    {0x6020, 0x11, 32}, // Counter value
};

static ec_pdo_entry_info_t el2522_1A05_pdo_entries[] = {
    {0x0000, 0x00, 2},
    {0x6030, 0x03, 1},  // Set counter done
    {0x6030, 0x04, 1},  // Counter underflow
    {0x6030, 0x05, 1},  // Counter overflow
    {0x0000, 0x00, 8},
    {0x6030, 0x0E, 1},  // Sync error
    {0x6030, 0x0F, 1},  // TxPDO State
    {0x6030, 0x10, 1},  // TxPDO Toggle
    {0x6030, 0x11, 32}, // Counter value
};

static ec_pdo_info_t lcec_el2522_pdos_out[] = {
    {0x1601, 6, el2522_1601_pdo_entries}, // Control Position Ch.1
    {0x1606, 6, el2522_1606_pdo_entries}, // Control Position Ch.2
    {0x160B, 5, el2522_160B_pdo_entries}, // Control Ch.1 (Set Counter)
    {0x160D, 5, el2522_160D_pdo_entries}, // Control Ch.2 (Set Counter)
};

static ec_pdo_info_t lcec_el2522_pdos_in[] = {
    {0x1A03, 9, el2522_1A03_pdo_entries}, // Status Ch.1 (Counter value + flags)
    {0x1A05, 9, el2522_1A05_pdo_entries}, // Status Ch.2 (Counter value + flags)
};

static ec_sync_info_t lcec_el2522_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT, 0, NULL},
    {2, EC_DIR_OUTPUT, 4, lcec_el2522_pdos_out},
    {3, EC_DIR_INPUT, 2, lcec_el2522_pdos_in},
    {0xff},
};

static int lcec_el2522_init(int comp_id, lcec_slave_t *slave) {
  int err = 0;

  slave->proc_read = lcec_el2522_read;
  slave->proc_write = lcec_el2522_write;
  slave->sync_info = lcec_el2522_syncs;

  lcec_master_t *master = slave->master;
  lcec_el2522_data_t *hal_data = LCEC_HAL_ALLOCATE(lcec_el2522_data_t);
  slave->hal_data = hal_data;

  for (int i = 0; i < LCEC_EL2522_CHANNEL_COUNT; i++) {
    LCEC_CONF_MODPARAM_VAL_T *pval = lcec_modparam_get(slave, LCEC_EL2522_MODPARAM_OUTMODE + i);
    if (pval != NULL) {
      int operating_mode = lcec_lookupint_i(lcec_el2522_outmode_table, pval->str, 0x00);
      if ((err = lcec_write_sdo8(slave, 0x8000 + (0x10 * i), 0x0E, operating_mode)) != 0) {
        return err;
      }
    }

    pval = lcec_modparam_get(slave, LCEC_EL2522_MODPARAM_FREQLIMIT + i);
    if (pval != NULL) {
      if ((err = lcec_write_sdo32(slave, 0x8000 + (i * 0x10), 0x12, pval->u32)) != 0) {
        return err;
      }
    }

    lcec_el2522_channel_t *channel = &hal_data->channels[i];
    if ((err = lcec_pdo_init(slave, 0x6020 + (i * 0x10), 0x11, &channel->count_pdo_os, NULL)) != 0) {
      return err;
    }
    if ((err = lcec_pdo_init(slave, 0x7000 + (i * 0x10), 0x04, &channel->enable_pdo_os, &channel->enable_pdo_bp)) != 0) {
      return err;
    }
    if ((err = lcec_pdo_init(slave, 0x7000 + (i * 0x10), 0x12, &channel->target_pdo_os, NULL)) != 0) {
      return err;
    }

    // register pins and params
    if ((err = lcec_pin_newf_list(channel, slave_pins, LCEC_MODULE_NAME, master->name, slave->name, i+1)) != 0) {
      return err;
    }
    if ((err = lcec_param_newf_list(channel, slave_params, LCEC_MODULE_NAME, master->name, slave->name, i+1)) != 0) {
      return err;
    }

    channel->pos_scale = LCEC_EL2522_POS_SCALE_DEFAULT;
    channel->pos_scale_recip = 1.0 / LCEC_EL2522_POS_SCALE_DEFAULT;
    channel->last_pos_scale = LCEC_EL2522_POS_SCALE_DEFAULT;
    channel->last_count = 0;
    channel->step_offset = 0.0;
  }

  return err;
}

static void lcec_el2522_read(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_el2522_data_t *hal_data = (lcec_el2522_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  for (int i = 0; i < LCEC_EL2522_CHANNEL_COUNT; i++) {
    lcec_el2522_channel_t *channel = &hal_data->channels[i];

    const uint32_t raw_count = EC_READ_U32(&pd[channel->count_pdo_os]);
    const int32_t diff = (int32_t)(raw_count - channel->last_count);
    channel->last_count = raw_count;

    *(channel->count) += diff;
    *(channel->pos_fb) += ((double)diff) * channel->pos_scale_recip;
  }
}

static void lcec_el2522_write(lcec_slave_t *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_el2522_data_t *hal_data = (lcec_el2522_data_t *)slave->hal_data;
  uint8_t *pd = master->process_data;

  for (int i = 0; i < LCEC_EL2522_CHANNEL_COUNT; i++) {
    lcec_el2522_channel_t *channel = &hal_data->channels[i];

    // check for change in scale value, and relcalculate step offset
    // to avoid position discontinuities.
    if (channel->pos_scale != channel->last_pos_scale) {
      if ((channel->pos_scale < 1e-20) && (channel->pos_scale > -1e-20)) {
        rtapi_print_msg(
            RTAPI_MSG_ERR, "Requested pos-scale for %s.%s.ch%i is too small, dropping\n", slave->master->name, slave->name, i+1);
      } else {
        channel->last_pos_scale = channel->pos_scale;
        channel->step_offset = *(channel->count) - *(channel->pos_fb) * channel->pos_scale;
        channel->pos_scale_recip = 1.0 / channel->pos_scale;
      }
    }

    // explicit conversion casts used here to ensure correct handling for negative double values
    const uint32_t target = (uint32_t)((int32_t)round(*(channel->pos_cmd) * channel->pos_scale + channel->step_offset));
    EC_WRITE_U32(&pd[channel->target_pdo_os], target);
    EC_WRITE_BIT(&pd[channel->enable_pdo_os], channel->enable_pdo_bp, *(channel->enable));
  }
}
