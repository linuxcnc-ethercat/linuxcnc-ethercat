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
/// @brief Driver for Beckhoff digital input/output modules
///
/// This driver should support any Beckhoff digital in/out board with
/// digital input and output ports.  As it stands, it's limited to 31
/// ports of each type, but that'd be relatively simple to overcome,
/// if a 32i/32o device existed.
///
/// It's technically capable of supporting input-only and output-only
/// devices, so we *could* roll lcec_el1xxx.c and lcec_el2xxx.c into
/// this driver, but they're each so simple that I don't want to mess
/// with them.
///
/// This driver uses flags in `types[]` describe the number of ports
/// for each device, as well as a couple "oddball" configurations that
/// some devices use.
///
/// When adding new devices, please use the `BECKHOFF_IO_DEVICE` macro
/// if possible.  It takes 3 parameters: the text name of the device
/// (which needs to match `type` in the XML configuration), the device
/// PID, and then a set of flags.  All other paramters in
/// `lcec_typelist_t` are auto-derived from these.
///
/// There are 4 basic flag settings to be aware of:
///
/// - `F_IN(x)`: this device has `x` digital input ports.
/// - `F_OUT(x)`: this device has `x` digital output ports.
/// - `F_OUTOFFSET`: the output PODs start at 0x7080, not 0x7000.  This
///   is needed for EL185x devices.
/// - `F_DENSEPDOS`: device's PDOs are packed into 0x6000:0n, instead of
///   being spread out over 0x60x0:01.  The EP2316 and several other
///   devices use this addressing model.
///
/// For any specific new device, just binary-OR (`|`) the various
/// flags for the device together.  For 8 input ports, 4 output ports,
/// and dense addressing, just say `F_IN(8)|F_OUT(4)|F_DENSEPDOS`.
///
/// You can tell which PDOs a device uses through a few means:
///
/// - run `ethercat pdos` and look at the output.
/// - look at the manufacturer's documentation.
/// - look at http://linuxcnc-ethercat.github.io/esi-data/devices

#include "../lcec.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

static int lcec_digitalcombo_init(int comp_id, lcec_slave_t *slave);

#define F_IN(x)     (x)       // Input channels
#define F_OUT(x)    (x * 32)  // Output channels
#define F_DENSEPDOS 1 << 10   // PDOs are 0x6000:01, 0x6000:02, ... instead of 0x6000:01, 0x6010:01
#define F_OUTOFFSET 1 << 11   // Out PDOs start at 0x70n0 instead of 0x7000, where n is the number of input ports.

#define INPORTS(flag)  ((flag)&31)           // Number of input channels
#define OUTPORTS(flag) (((flag) >> 5) & 31)  // Number of output channels

/// Macro to avoid repeating all of the unchanging fields in
/// `lcec_typelist_t`.  Calculates the `pdo_count` based on total port
/// count.  Digital I/O devices need one PDO per pin.
#define BECKHOFF_IO_DEVICE(name, pid, flags) \
  { name, LCEC_BECKHOFF_VID, pid, 0, NULL, lcec_digitalcombo_init, NULL, flags }

static lcec_typelist_t types[] = {
    BECKHOFF_IO_DEVICE("EL1252", 0x04E43052, F_IN(2) | F_DENSEPDOS),
    BECKHOFF_IO_DEVICE("EL1852", 0x73c3052, F_IN(8) | F_OUT(8) | F_OUTOFFSET),
    BECKHOFF_IO_DEVICE("EL1859", 0x07433052, F_IN(8) | F_OUT(8) | F_OUTOFFSET),
    BECKHOFF_IO_DEVICE("EJ1859", 0x07432852, F_IN(8) | F_OUT(8) | F_OUTOFFSET),
    BECKHOFF_IO_DEVICE("EK1814", 0x07162c52, F_IN(4) | F_OUT(4)),
    BECKHOFF_IO_DEVICE("EK1818", 0x071a2c52, F_IN(8) | F_OUT(4)),
    BECKHOFF_IO_DEVICE("EK1828", 0x07242c52, F_IN(4) | F_OUT(8)),
    BECKHOFF_IO_DEVICE("EK1828-0010", 0x07242c52, F_OUT(8)),  // No in
    BECKHOFF_IO_DEVICE("EP2308", 0x09044052, F_IN(4) | F_OUT(4) | F_OUTOFFSET),
    BECKHOFF_IO_DEVICE("EP2316", 0x090C4052, F_IN(8) | F_OUT(8) | F_DENSEPDOS),
    BECKHOFF_IO_DEVICE("EP2318", 0x090E4052, F_IN(4) | F_OUT(4) | F_OUTOFFSET),
    BECKHOFF_IO_DEVICE("EP2328", 0x09184052, F_IN(4) | F_OUT(4) | F_OUTOFFSET),
    BECKHOFF_IO_DEVICE("EP2338", 0x09224052, F_IN(8) | F_OUT(8)),  // Not offset
    BECKHOFF_IO_DEVICE("EP2339", 0x09234052, F_IN(16) | F_OUT(16)),
    BECKHOFF_IO_DEVICE("EP2349", 0x092d4052, F_IN(16) | F_OUT(16)),
    BECKHOFF_IO_DEVICE("EQ2339", 0x092d4052, F_IN(16) | F_OUT(16)),
    BECKHOFF_IO_DEVICE("EPP2308", 0x64765649, F_IN(4) | F_OUT(4)),
    BECKHOFF_IO_DEVICE("EPP2316", 0x090c4052, F_IN(8) | F_OUT(8) | F_DENSEPDOS),
    BECKHOFF_IO_DEVICE("EPP2318", 0x647656e9, F_IN(4) | F_OUT(4)),
    BECKHOFF_IO_DEVICE("EPP2328", 0x64765789, F_IN(4) | F_OUT(4)),
    BECKHOFF_IO_DEVICE("EPP2334", 0x647657e9, F_IN(4) | F_OUT(4)),
    BECKHOFF_IO_DEVICE("EPP2338", 0x64765829, F_IN(8) | F_OUT(8)),
    BECKHOFF_IO_DEVICE("EPP2339", 0x64765839, F_IN(8) | F_OUT(8)),
    BECKHOFF_IO_DEVICE("EPP2349", 0x647658d9, F_IN(8) | F_OUT(8)),
    {NULL},
};
ADD_TYPES(types)

typedef struct {
  lcec_class_din_channels_t *channels_in;
  lcec_class_dout_channels_t *channels_out;
} lcec_digitalcombo_data_t;

static void lcec_digitalcombo_read(lcec_slave_t *slave, long period);
static void lcec_digitalcombo_write(lcec_slave_t *slave, long period);

static int lcec_digitalcombo_init(int comp_id, lcec_slave_t *slave) {
  lcec_digitalcombo_data_t *hal_data;
  int i;
  int in_channels = INPORTS(slave->flags);
  int out_channels = OUTPORTS(slave->flags);
  int idx, sidx;

  // initialize callbacks
  if (in_channels > 0) slave->proc_read = lcec_digitalcombo_read;
  if (out_channels > 0) slave->proc_write = lcec_digitalcombo_write;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_digitalcombo_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", slave->master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_digitalcombo_data_t));
  slave->hal_data = hal_data;

  // Allocate memory for I/O pin definitions
  if (in_channels > 0) hal_data->channels_in = lcec_din_allocate_channels(in_channels);
  if (out_channels > 0) hal_data->channels_out = lcec_dout_allocate_channels(out_channels);

  // initialize input pins
  for (i = 0; i < in_channels; i++) {
    // Figure out which addresses to use based on slave->flags
    if (slave->flags & F_DENSEPDOS) {
      idx = 0x6000;
      sidx = 1 + i;
    } else {
      idx = 0x6000 + (i << 4);
      sidx = 1;
    }

    // Create pins
    hal_data->channels_in->channels[i] = lcec_din_register_channel(slave, i, idx, sidx);
  }

  // initialize output pins
  for (i = 0; i < out_channels; i++) {
    // Figure out which addresses to use based on slave->flags
    if (slave->flags & F_DENSEPDOS) {
      idx = 0x7000;
      sidx = 1 + i;
    } else {
      idx = 0x7000 + (i << 4);
      sidx = 1;
      if (slave->flags & F_OUTOFFSET) {
        idx += in_channels << 4;
      }
    }

    // Create pins
    hal_data->channels_out->channels[i] = lcec_dout_register_channel(slave, i, idx, sidx);
  }
  return 0;
}

static void lcec_digitalcombo_read(lcec_slave_t *slave, long period) {
  lcec_digitalcombo_data_t *hal_data = (lcec_digitalcombo_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  if (hal_data->channels_in != NULL) lcec_din_read_all(slave, hal_data->channels_in);
}

static void lcec_digitalcombo_write(lcec_slave_t *slave, long period) {
  lcec_digitalcombo_data_t *hal_data = (lcec_digitalcombo_data_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  if (hal_data->channels_out != NULL) lcec_dout_write_all(slave, hal_data->channels_out);
}
