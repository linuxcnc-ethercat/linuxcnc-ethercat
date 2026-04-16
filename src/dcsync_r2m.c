//
//    Copyright (C) 2026 Sascha Ittner <sascha.ittner@modusoft.de>
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

// DC synchronisation: Reference-to-Master (R2M) mode.
// EtherCAT reference clock tracks LinuxCNC master time.
// refClockSyncCycles >= 0 selects this mode.

#include "lcec.h"

extern int64_t dc_time_offset;

static void cycle_start(struct lcec_master *master) {
  master->app_time_ns = dc_time_offset + rtapi_task_pll_get_reference();
  ecrt_master_application_time(master->master, master->app_time_ns);
}

static void pre_send(struct lcec_master *master) {
  if (master->ref_clock_sync_cycles <= 0) {
    return;
  }

  master->ref_clock_sync_counter--;
  if (master->ref_clock_sync_counter <= 0) {
    master->ref_clock_sync_counter = master->ref_clock_sync_cycles;
    // use current time (not PLL ref) to compensate for cycle runtime delay
    ecrt_master_sync_reference_clock_to(master->master, dc_time_offset + rtapi_get_time());
  }

  ecrt_master_sync_slave_clocks(master->master);
}

static void post_send(struct lcec_master *master) {
  // NOP in R2M mode
}

void lcec_dc_init_r2m(struct lcec_master *master) {
  master->dcsync_callbacks.cycle_start = cycle_start;
  master->dcsync_callbacks.pre_send = pre_send;
  master->dcsync_callbacks.post_send = post_send;

  master->app_time_ns = 0;
  master->ref_clock_sync_counter = 0;
}
