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
/// @brief Initialization code for LinuxCNC-Ethercat

#ifndef __KERNEL__
#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#endif

#include "devices/lcec_generic.h"
#include "lcec.h"
#include "rtapi_app.h"
//#include <linuxcnc/rtapi_mutex.h>

MODULE_LICENSE("GPL")
MODULE_AUTHOR("Sascha Ittner <sascha.ittner@modusoft.de>")
MODULE_DESCRIPTION("Driver for EtherCAT devices")

/// @brief Global HAL Pins
static const lcec_pindesc_t master_global_pins[] = {
    {HAL_U32, HAL_OUT, offsetof(lcec_master_data_t, slaves_responding), "%s.slaves-responding"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_master_data_t, state_init), "%s.state-init"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_master_data_t, state_preop), "%s.state-preop"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_master_data_t, state_safeop), "%s.state-safeop"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_master_data_t, state_op), "%s.state-op"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_master_data_t, link_up), "%s.link-up"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_master_data_t, all_op), "%s.all-op"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/// @brief Master HAL pins
static const lcec_pindesc_t master_pins[] = {
#ifdef RTAPI_TASK_PLL_SUPPORT
    {HAL_S32, HAL_OUT, offsetof(lcec_master_data_t, pll_err), "%s.pll-err"},
    {HAL_S32, HAL_OUT, offsetof(lcec_master_data_t, pll_out), "%s.pll-out"},
    {HAL_U32, HAL_OUT, offsetof(lcec_master_data_t, pll_reset_cnt), "%s.pll-reset-count"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_master_data_t, dc_phased), "%s.dc-phased"},
    {HAL_S32, HAL_OUT, offsetof(lcec_master_data_t, app_phase), "%s.app-phase"},
    {HAL_S32, HAL_OUT, offsetof(lcec_master_data_t, phase_jitter_out), "%s.phase-jitter"},
    {HAL_S32, HAL_IN, offsetof(lcec_master_data_t, drift_mode), "%s.drift-mode"},
    {HAL_S32, HAL_IN, offsetof(lcec_master_data_t, pll_drift), "%s.pll-drift"},
    {HAL_S32, HAL_OUT, offsetof(lcec_master_data_t, pll_final), "%s.pll-final"},
#endif
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

/// @brief Master params
static const lcec_paramdesc_t master_params[] = {
#ifdef RTAPI_TASK_PLL_SUPPORT
    {HAL_U32, HAL_RW, offsetof(lcec_master_data_t, pll_step), "%s.pll-step"},
    {HAL_U32, HAL_RW, offsetof(lcec_master_data_t, pll_max_err), "%s.pll-max-err"},
#endif
    {HAL_TYPE_UNSPECIFIED},
};

/// @brief Basic Slave pins
static const lcec_pindesc_t slave_pins[] = {
    {HAL_BIT, HAL_OUT, offsetof(lcec_slave_state_t, online), "%s.%s.%s.slave-online"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_slave_state_t, operational), "%s.%s.%s.slave-oper"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_slave_state_t, state_init), "%s.%s.%s.slave-state-init"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_slave_state_t, state_preop), "%s.%s.%s.slave-state-preop"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_slave_state_t, state_safeop), "%s.%s.%s.slave-state-safeop"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_slave_state_t, state_op), "%s.%s.%s.slave-state-op"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

static lcec_master_t *first_master = NULL;
static lcec_master_t *last_master = NULL;
extern int lcec_comp_id;

static lcec_master_data_t *global_hal_data;
static ec_master_state_t global_ms;

int lcec_parse_config(void);
void lcec_clear_config(void);

#ifdef __KERNEL__
static void lcec_request_lock(void *data);
static void lcec_release_lock(void *data);
#endif

lcec_master_data_t *lcec_init_master_hal(const char *pfx, int global);
lcec_slave_state_t *lcec_init_slave_state_hal(char *master_name, char *slave_name);
void lcec_update_master_hal(lcec_master_data_t *hal_data, ec_master_state_t *ms);
void lcec_update_slave_state_hal(lcec_slave_state_t *hal_data, ec_slave_config_state_t *ss);

void lcec_read_all(void *arg, long period);
void lcec_write_all(void *arg, long period);
void lcec_read_master(void *arg, long period);
void lcec_write_master(void *arg, long period);
static int lcec_activate_master(lcec_master_t *master);

static void sigsegv_handler(int sig);

/// @brief Main entrypoint from LinuxCNC
int rtapi_app_main(void) {
  int slave_count;
  lcec_master_t *master;
  lcec_slave_t *slave;
  char name[HAL_NAME_LEN + 1];
  lcec_slave_sdoconf_t *sdo_config;
  lcec_slave_idnconf_t *idn_config;
  int pdo_entry_count = 0;

#ifndef __KERNEL
  struct sigaction handler;
  sigemptyset(&handler.sa_mask);
  handler.sa_flags = SA_NODEFER | SA_RESETHAND;
  handler.sa_handler = sigsegv_handler;

  sigaction(SIGSEGV, &handler, NULL);
  sigaction(SIGBUS, &handler, NULL);
  sigaction(SIGFPE, &handler, NULL);
  sigaction(SIGKILL, &handler, NULL);
#endif

  // connect to the HAL
  if ((lcec_comp_id = hal_init(LCEC_MODULE_NAME)) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_init() failed\n");
    goto fail0;
  }

  // parse configuration
  if ((slave_count = lcec_parse_config()) < 0) {
    goto fail1;
  }

  // init global hal data
  if ((global_hal_data = lcec_init_master_hal(LCEC_MODULE_NAME, 1)) == NULL) {
    goto fail2;
  }

  // initialize masters
  for (master = first_master; master != NULL; master = master->next) {
    // request ethercat master
    if (!(master->master = ecrt_request_master(master->index))) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "requesting master %s (index %d) failed\n", master->name, master->index);
      goto fail2;
    }

#ifdef __KERNEL__
    // register callbacks
    ecrt_master_callbacks(master->master, lcec_request_lock, lcec_release_lock, master);
#endif

    // create domain
    if (!(master->domain = ecrt_master_create_domain(master->master))) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "master %s domain creation failed\n", master->name);
      goto fail2;
    }

    // initialize slaves
    for (slave = master->first_slave; slave != NULL; slave = slave->next) {
      // read slave config

      rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "calling ecrt_master_slave_config for slave %s.%s\n", master->name, slave->name);
      if (!(slave->config = ecrt_master_slave_config(master->master, 0, slave->index, slave->vid, slave->pid))) {
        rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to read slave %s.%s configuration\n", master->name, slave->name);
        goto fail2;
      }

      // initialize sdos
      if (slave->sdo_config != NULL) {
        for (sdo_config = slave->sdo_config; sdo_config->index != 0xffff;
             sdo_config = (lcec_slave_sdoconf_t *)&sdo_config->data[sdo_config->length]) {
          if (sdo_config->subindex == LCEC_CONF_SDO_COMPLETE_SUBIDX) {
            if (ecrt_slave_config_complete_sdo(slave->config, sdo_config->index, &sdo_config->data[0], sdo_config->length) != 0) {
              rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to configure slave %s.%s sdo %04x (complete)\n", master->name,
                  slave->name, sdo_config->index);
            }
          } else {
            if (lcec_write_sdo(slave, sdo_config->index, sdo_config->subindex, &sdo_config->data[0], sdo_config->length) != 0) {
              rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to configure slave %s.%s sdo %04x:%02x\n", master->name, slave->name,
                  sdo_config->index, sdo_config->subindex);
            }
          }
        }
      }

      // initialize idns
      if (slave->idn_config != NULL) {
        for (idn_config = slave->idn_config; idn_config->state != 0;
             idn_config = (lcec_slave_idnconf_t *)&idn_config->data[idn_config->length]) {
          if (ecrt_slave_config_idn(
                  slave->config, idn_config->drive, idn_config->idn, idn_config->state, &idn_config->data[0], idn_config->length) != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s drive %d idn %c-%d-%d (state %d, length %u)\n",
                master->name, slave->name, idn_config->drive, (idn_config->idn & 0x8000) ? 'P' : 'S', (idn_config->idn >> 12) & 0x0007,
                idn_config->idn & 0x0fff, idn_config->state, (unsigned int)idn_config->length);
          }
        }
      }

      slave->regs = lcec_allocate_pdo_entry_reg(LCEC_MAX_PDO_REG_COUNT);
      if (slave->regs == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failure allocating PDO entries for slave %s.%s\n", master->name, slave->name);
        goto fail2;
      }

      // setup pdos
      if (slave->proc_init != NULL) {
        rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "proc_init for slave %s.%s\n", master->name, slave->name);
        if ((slave->proc_init(lcec_comp_id, slave)) != 0) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failure in proc_init for slave %s.%s\n", master->name, slave->name);
          goto fail2;
        }
      }

      // configure dc for this slave
      if (slave->dc_conf != NULL) {
        ecrt_slave_config_dc(slave->config, slave->dc_conf->assignActivate, slave->dc_conf->sync0Cycle, slave->dc_conf->sync0Shift,
            slave->dc_conf->sync1Cycle, slave->dc_conf->sync1Shift);
        rtapi_print_msg(RTAPI_MSG_DBG,
            LCEC_MSG_PFX "configuring DC for slave %s.%s: assignActivate=x%x sync0Cycle=%d sync0Shift=%d sync1Cycle=%d sync1Shift=%d\n",
            master->name, slave->name, slave->dc_conf->assignActivate, slave->dc_conf->sync0Cycle, slave->dc_conf->sync0Shift,
            slave->dc_conf->sync1Cycle, slave->dc_conf->sync1Shift);
      }

      // Configure the slave's watchdog times.
      if (slave->wd_conf != NULL) {
        ecrt_slave_config_watchdog(slave->config, slave->wd_conf->divider, slave->wd_conf->intervals);
      }

      // configure slave
      if (slave->sync_info != NULL) {
        rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "sync_info setup for slave %s.%s\n", master->name, slave->name);
        if (ecrt_slave_config_pdos(slave->config, EC_END, slave->sync_info)) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s\n", master->name, slave->name);
          goto fail2;
        }
      }

      // export state pins
      rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "init slave state hal for slave %s.%s\n", master->name, slave->name);
      if ((slave->hal_state_data = lcec_init_slave_state_hal(master->name, slave->name)) == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failure to export slave pins for slave %s.%s\n", master->name, slave->name);
        goto fail2;
      }

      pdo_entry_count += lcec_pdo_entry_reg_len(slave->regs);
    }

    lcec_pdo_entry_reg_t *master_regs = lcec_allocate_pdo_entry_reg(pdo_entry_count + 1);
    for (slave = master->first_slave; slave != NULL; slave = slave->next) {
      if (lcec_append_pdo_entry_reg(master_regs, slave->regs) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failure to append PDO entries for slave %s.%s\n", master->name, slave->name);
        goto fail2;
      }
    }

    // register PDO entries
    rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "register PDO entries\n");
    if (ecrt_domain_reg_pdo_entry_list(master->domain, master_regs->pdo_entry_regs)) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "master %s PDO entry registration failed\n", master->name);
      goto fail2;
    }

    // init hal data
    rtapi_snprintf(name, HAL_NAME_LEN, "%s.%s", LCEC_MODULE_NAME, master->name);
    if ((master->hal_data = lcec_init_master_hal(name, 0)) == NULL) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failure to init hal pins for slave %s.%s\n", master->name, slave->name);
      goto fail2;
    }

#ifdef RTAPI_TASK_PLL_SUPPORT
    // set default PLL_STEP: use +/-0.1% of period
    master->hal_data->pll_step = master->app_time_period / 1000;
    // set default PLL_MAX_ERR: one period
    master->hal_data->pll_max_err = master->app_time_period;
    // Initialize auto-drift delay counter (wait 100 cycles before applying)
    master->hal_data->auto_drift_delay = 0;
    // Initialize phase lock counter (require 100 consecutive cycles to confirm phased)
    master->hal_data->phase_lock_cnt = 0;
#endif

    // Activate master
    if (lcec_activate_master(master) != 0) {
      goto fail2;
    }

    // export read function
    rtapi_snprintf(name, HAL_NAME_LEN, "%s.%s.read", LCEC_MODULE_NAME, master->name);
    if (hal_export_funct(name, lcec_read_master, master, 0, 0, lcec_comp_id) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "master %s read funct export failed\n", master->name);
      goto fail2;
    }
    // export write function
    rtapi_snprintf(name, HAL_NAME_LEN, "%s.%s.write", LCEC_MODULE_NAME, master->name);
    if (hal_export_funct(name, lcec_write_master, master, 0, 0, lcec_comp_id) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "master %s write funct export failed\n", master->name);
      goto fail2;
    }
  }

  // export read-all function
  rtapi_snprintf(name, HAL_NAME_LEN, "%s.read-all", LCEC_MODULE_NAME);
  if (hal_export_funct(name, lcec_read_all, NULL, 0, 0, lcec_comp_id) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "read-all funct export failed\n");
    goto fail2;
  }
  // export write-all function
  rtapi_snprintf(name, HAL_NAME_LEN, "%s.write-all", LCEC_MODULE_NAME);
  if (hal_export_funct(name, lcec_write_all, NULL, 0, 0, lcec_comp_id) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "write-all funct export failed\n");
    goto fail2;
  }

  rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "installed driver for %d slaves\n", slave_count);
  hal_ready(lcec_comp_id);
  return 0;

fail2:
  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failure, clearing config\n");
  lcec_clear_config();
fail1:
  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "exiting\n");
  hal_exit(lcec_comp_id);
fail0:
  rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "returning -EINVAL\n");
  return -EINVAL;
}

/// @brief Shut down LinuxCNC-Ethercat
void rtapi_app_exit(void) {
  lcec_master_t *master;

  // deactivate all masters
  for (master = first_master; master != NULL; master = master->next) {
    ecrt_master_deactivate(master->master);
  }

  lcec_clear_config();
  hal_exit(lcec_comp_id);
}

/// @brief Parse configuration from `lcec_conf`.
int lcec_parse_config(void) {
  int shmem_id;
  void *shmem_ptr;
  LCEC_CONF_HEADER_T *header;
  size_t length;
  char *conf;
  int slave_count;
  const lcec_typelist_t *type;
  lcec_master_t *master;
  lcec_slave_t *slave;
  lcec_slave_dc_t *dc;
  lcec_slave_watchdog_t *wd;
  LCEC_CONF_TYPE_T conf_type;
  LCEC_CONF_MASTER_T *master_conf;
  LCEC_CONF_SLAVE_T *slave_conf;
  LCEC_CONF_DC_T *dc_conf;
  LCEC_CONF_WATCHDOG_T *wd_conf;
  LCEC_CONF_SYNCMANAGER_T *sm_conf;
  LCEC_CONF_PDO_T *pdo_conf;
  LCEC_CONF_PDOENTRY_T *pe_conf;
  LCEC_CONF_COMPLEXENTRY_T *ce_conf;
  LCEC_CONF_SDOCONF_T *sdo_conf;
  LCEC_CONF_IDNCONF_T *idn_conf;
  LCEC_CONF_MODPARAM_T *modparam_conf;
  ec_pdo_entry_info_t *generic_pdo_entries;
  ec_pdo_info_t *generic_pdos;
  ec_sync_info_t *generic_sync_managers;
  lcec_generic_pin_t *generic_hal_data;
  hal_pin_dir_t generic_hal_dir;
  lcec_slave_sdoconf_t *sdo_config;
  lcec_slave_idnconf_t *idn_config;
  lcec_slave_modparam_t *modparams;

  // initialize list
  first_master = NULL;
  last_master = NULL;

  // try to get config header
  shmem_id = rtapi_shmem_new(LCEC_CONF_SHMEM_KEY, lcec_comp_id, sizeof(LCEC_CONF_HEADER_T));
  if (shmem_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "couldn't allocate user/RT shared memory\n");
    goto fail0;
  }
  if (lcec_rtapi_shmem_getptr(shmem_id, &shmem_ptr) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "couldn't map user/RT shared memory\n");
    goto fail1;
  }

  // check magic, get length and close shmem
  header = (LCEC_CONF_HEADER_T *)shmem_ptr;
  if (header->magic != LCEC_CONF_SHMEM_MAGIC) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "lcec_conf is not loaded\n");
    goto fail1;
  }
  length = header->length;
  rtapi_shmem_delete(shmem_id, lcec_comp_id);

  // reopen shmem with proper size
  shmem_id = rtapi_shmem_new(LCEC_CONF_SHMEM_KEY, lcec_comp_id, sizeof(LCEC_CONF_HEADER_T) + length);
  if (shmem_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "couldn't allocate user/RT shared memory\n");
    goto fail0;
  }
  if (lcec_rtapi_shmem_getptr(shmem_id, &shmem_ptr) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "couldn't map user/RT shared memory\n");
    goto fail1;
  }

  // get pointer to config
  conf = ((char *)shmem_ptr + sizeof(LCEC_CONF_HEADER_T));

  // process config items
  slave_count = 0;
  master = NULL;
  slave = NULL;
  generic_pdo_entries = NULL;
  generic_pdos = NULL;
  generic_sync_managers = NULL;
  generic_hal_data = NULL;
  generic_hal_dir = HAL_DIR_UNSPECIFIED;
  sdo_config = NULL;
  idn_config = NULL;
  pe_conf = NULL;
  modparams = NULL;
  while ((conf_type = ((LCEC_CONF_NULL_T *)conf)->confType) != lcecConfTypeNone) {
    // get type
    switch (conf_type) {
      case lcecConfTypeMaster:
        // get config token
        master_conf = (LCEC_CONF_MASTER_T *)conf;
        conf += sizeof(LCEC_CONF_MASTER_T);

        // alloc master memory
        master = LCEC_ALLOCATE(lcec_master_t);

        // initialize master
        master->index = master_conf->index;
        strncpy(master->name, master_conf->name, LCEC_CONF_STR_MAXLEN);
        master->name[LCEC_CONF_STR_MAXLEN - 1] = 0;
        master->app_time_period = master_conf->appTimePeriod;
        master->sync_ref_cycles = master_conf->refClockSyncCycles;
        // sync_to_ref_clock controls clock sync direction:
        //   false: master is clock source, DC syncs to master (default)
        //   true: DC is clock source, servo thread syncs to DC via PLL
        // Note: legacy mode - negative refClockSyncCycles enables sync_to_ref_clock
        master->sync_to_ref_clock = master_conf->syncToRefClock;

        // add master to list
        LCEC_LIST_APPEND(first_master, last_master, master);
        break;

      case lcecConfTypeSlave:
        // get config token
        slave_conf = (LCEC_CONF_SLAVE_T *)conf;
        conf += sizeof(LCEC_CONF_SLAVE_T);

        // check for master
        if (master == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Master node for slave missing\n");
          goto fail2;
        }

        // check for valid slave type
        if (!strcmp(slave_conf->type_name, "generic")) {
          type = NULL;
        } else {
          type = lcec_findslavetype(slave_conf->type_name);

          if (type == NULL) {
            rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "Invalid slave name \"%s\"\n", slave_conf->type_name);
            continue;
          }
        }

        // create new slave
        slave = LCEC_ALLOCATE(lcec_slave_t);

        // initialize slave
        generic_pdo_entries = NULL;
        generic_pdos = NULL;
        generic_sync_managers = NULL;
        generic_hal_data = NULL;
        generic_hal_dir = HAL_DIR_UNSPECIFIED;
        sdo_config = NULL;
        idn_config = NULL;
        modparams = NULL;

        slave->index = slave_conf->index;
        strncpy(slave->name, slave_conf->name, LCEC_CONF_STR_MAXLEN);
        slave->name[LCEC_CONF_STR_MAXLEN - 1] = 0;
        slave->master = master;

        // add slave to list
        LCEC_LIST_APPEND(master->first_slave, master->last_slave, slave);

        if (type != NULL) {
          // normal slave
          if (slave_conf->vid)
            slave->vid = slave_conf->vid;
          else
            slave->vid = type->vid;

          if (slave_conf->pid)
            slave->pid = slave_conf->pid;
          else
            slave->pid = type->pid;

          slave->is_fsoe_logic = type->is_fsoe_logic;
          slave->proc_preinit = type->proc_preinit;
          slave->proc_init = type->proc_init;
          slave->flags = type->flags;
        } else {
          // generic slave
          slave->vid = slave_conf->vid;
          slave->pid = slave_conf->pid;
          slave->generic_pdo_entry_count = slave_conf->pdoMappingCount;
          slave->proc_init = lcec_generic_init;

          // alloc hal memory
          if ((generic_hal_data = LCEC_HAL_ALLOCATE_ARRAY(lcec_generic_pin_t, slave_conf->pdoMappingCount)) == NULL) {
            rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave_conf->name);
            goto fail2;
          }
          memset(generic_hal_data, 0, sizeof(lcec_generic_pin_t) * slave_conf->pdoMappingCount);

          // alloc pdo entry memory
          generic_pdo_entries = LCEC_ALLOCATE_ARRAY(ec_pdo_entry_info_t, slave_conf->pdoEntryCount);

          // alloc pdo memory
          generic_pdos = LCEC_ALLOCATE_ARRAY(ec_pdo_info_t, slave_conf->pdoCount);

          // alloc sync manager memory
          generic_sync_managers = LCEC_ALLOCATE_ARRAY(ec_sync_info_t, (slave_conf->syncManagerCount + 1));

          generic_sync_managers->index = 0xff;
        }

        // alloc sdo config memory
        if (slave_conf->sdoConfigLength > 0) {
          sdo_config = (lcec_slave_sdoconf_t *)lcec_zalloc(slave_conf->sdoConfigLength + sizeof(lcec_slave_sdoconf_t));
          if (sdo_config == NULL) {
            rtapi_print_msg(
                RTAPI_MSG_ERR, LCEC_MSG_PFX "Unable to allocate slave %s.%s sdo entry memory\n", master->name, slave_conf->name);
            goto fail2;
          }
        }

        // alloc idn config memory
        if (slave_conf->idnConfigLength > 0) {
          idn_config = (lcec_slave_idnconf_t *)lcec_zalloc(slave_conf->idnConfigLength + sizeof(lcec_slave_idnconf_t));
          if (idn_config == NULL) {
            rtapi_print_msg(
                RTAPI_MSG_ERR, LCEC_MSG_PFX "Unable to allocate slave %s.%s idn entry memory\n", master->name, slave_conf->name);
            goto fail2;
          }
        }

        // alloc modparam memory
        if (slave_conf->modParamCount > 0) {
          modparams = LCEC_ALLOCATE_ARRAY(lcec_slave_modparam_t, (slave_conf->modParamCount + 1));
          modparams[slave_conf->modParamCount].id = -1;
        }

        slave->hal_data = generic_hal_data;
        slave->generic_pdo_entries = generic_pdo_entries;
        slave->generic_pdos = generic_pdos;
        slave->generic_sync_managers = generic_sync_managers;
        if (slave_conf->configPdos) {
          slave->sync_info = generic_sync_managers;
        }
        slave->sdo_config = sdo_config;
        slave->idn_config = idn_config;
        slave->modparams = modparams;
        slave->dc_conf = NULL;
        slave->wd_conf = NULL;

        // update slave count
        slave_count++;
        break;

      case lcecConfTypeDcConf:
        // get config token
        dc_conf = (LCEC_CONF_DC_T *)conf;
        conf += sizeof(LCEC_CONF_DC_T);

        // check for slave
        if (slave == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Slave node for dc config missing\n");
          goto fail2;
        }

        // check for double dc config
        if (slave->dc_conf != NULL) {
          rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "Double dc config for slave %s.%s\n", master->name, slave->name);
          continue;
        }

        // create new dc config
        dc = LCEC_ALLOCATE(lcec_slave_dc_t);

        // initialize dc conf
        dc->assignActivate = dc_conf->assignActivate;
        dc->sync0Cycle = dc_conf->sync0Cycle;
        dc->sync0Shift = dc_conf->sync0Shift;
        dc->sync1Cycle = dc_conf->sync1Cycle;
        dc->sync1Shift = dc_conf->sync1Shift;

        // add to slave
        slave->dc_conf = dc;
        break;

      case lcecConfTypeWatchdog:
        // get config token
        wd_conf = (LCEC_CONF_WATCHDOG_T *)conf;
        conf += sizeof(LCEC_CONF_WATCHDOG_T);

        // check for slave
        if (slave == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Slave node for watchdog config missing\n");
          goto fail2;
        }

        // check for double wd config
        if (slave->wd_conf != NULL) {
          rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "Double watchdog config for slave %s.%s\n", master->name, slave->name);
          continue;
        }

        // create new wd config
        wd = LCEC_ALLOCATE(lcec_slave_watchdog_t);

        // initialize wd conf
        wd->divider = wd_conf->divider;
        wd->intervals = wd_conf->intervals;

        // add to slave
        slave->wd_conf = wd;
        break;

      case lcecConfTypeSyncManager:
        // get config token
        sm_conf = (LCEC_CONF_SYNCMANAGER_T *)conf;
        conf += sizeof(LCEC_CONF_SYNCMANAGER_T);

        // check for syncmanager
        if (generic_sync_managers == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Sync manager for generic device missing\n");
          goto fail2;
        }

        // check for pdos
        if (generic_pdos == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "PDOs for generic device missing\n");
          goto fail2;
        }

        // initialize sync manager
        generic_sync_managers->index = sm_conf->index;
        generic_sync_managers->dir = sm_conf->dir;
        generic_sync_managers->n_pdos = sm_conf->pdoCount;
        generic_sync_managers->pdos = sm_conf->pdoCount == 0 ? NULL : generic_pdos;

        // get hal direction
        switch (sm_conf->dir) {
          case EC_DIR_INPUT:
            generic_hal_dir = HAL_OUT;
            break;
          case EC_DIR_OUTPUT:
            generic_hal_dir = HAL_IN;
            break;
          default:
            generic_hal_dir = HAL_DIR_UNSPECIFIED;
        }

        // next syncmanager
        generic_sync_managers++;
        generic_sync_managers->index = 0xff;
        break;

      case lcecConfTypePdo:
        // get config token
        pdo_conf = (LCEC_CONF_PDO_T *)conf;
        conf += sizeof(LCEC_CONF_PDO_T);

        // check for pdos
        if (generic_pdos == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "PDOs for generic device missing\n");
          goto fail2;
        }

        // check for pdos entries
        if (generic_pdo_entries == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "PDO entries for generic device missing\n");
          goto fail2;
        }

        // initialize pdo
        generic_pdos->index = pdo_conf->index;
        generic_pdos->n_entries = pdo_conf->pdoEntryCount;
        generic_pdos->entries = pdo_conf->pdoEntryCount == 0 ? NULL : generic_pdo_entries;

        // next pdo
        generic_pdos++;
        break;

      case lcecConfTypePdoEntry:
        // get config token
        pe_conf = (LCEC_CONF_PDOENTRY_T *)conf;
        conf += sizeof(LCEC_CONF_PDOENTRY_T);

        // check for pdos entries
        if (generic_pdo_entries == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "PDO entries for generic device missing\n");
          goto fail2;
        }

        // check for hal data
        if (generic_hal_data == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "HAL data for generic device missing\n");
          goto fail2;
        }

        // check for hal dir
        if (generic_hal_dir == HAL_DIR_UNSPECIFIED) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "HAL direction for generic device missing\n");
          goto fail2;
        }

        // initialize pdo entry
        generic_pdo_entries->index = pe_conf->index;
        generic_pdo_entries->subindex = pe_conf->subindex;
        generic_pdo_entries->bit_length = pe_conf->bitLength;

        // initialize hal data
        if (pe_conf->halPin[0] != 0) {
          strncpy(generic_hal_data->name, pe_conf->halPin, LCEC_CONF_STR_MAXLEN);
          generic_hal_data->name[LCEC_CONF_STR_MAXLEN - 1] = 0;
          generic_hal_data->type = pe_conf->halType;
          generic_hal_data->subType = pe_conf->subType;
          generic_hal_data->floatScale = pe_conf->floatScale;
          generic_hal_data->floatOffset = pe_conf->floatOffset;
          generic_hal_data->bitOffset = 0;
          generic_hal_data->bitLength = pe_conf->bitLength;
          generic_hal_data->dir = generic_hal_dir;
          generic_hal_data->pdo_idx = pe_conf->index;
          generic_hal_data->pdo_sidx = pe_conf->subindex;
          generic_hal_data++;
        }

        // next pdo entry
        generic_pdo_entries++;
        break;

      case lcecConfTypeComplexEntry:
        // get config token
        ce_conf = (LCEC_CONF_COMPLEXENTRY_T *)conf;
        conf += sizeof(LCEC_CONF_COMPLEXENTRY_T);

        // check for pdoEntry
        if (pe_conf == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "pdoEntry for generic device missing\n");
          goto fail2;
        }

        // check for hal data
        if (generic_hal_data == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "HAL data for generic device missing\n");
          goto fail2;
        }

        // initialize hal data
        if (ce_conf->halPin[0] != 0) {
          strncpy(generic_hal_data->name, ce_conf->halPin, LCEC_CONF_STR_MAXLEN);
          generic_hal_data->name[LCEC_CONF_STR_MAXLEN - 1] = 0;
          generic_hal_data->type = ce_conf->halType;
          generic_hal_data->subType = ce_conf->subType;
          generic_hal_data->floatScale = ce_conf->floatScale;
          generic_hal_data->floatOffset = ce_conf->floatOffset;
          generic_hal_data->bitOffset = ce_conf->bitOffset;
          generic_hal_data->bitLength = ce_conf->bitLength;
          generic_hal_data->dir = generic_hal_dir;
          generic_hal_data->pdo_idx = pe_conf->index;
          generic_hal_data->pdo_sidx = pe_conf->subindex;
          generic_hal_data++;
        }
        break;

      case lcecConfTypeSdoConfig:
        // get config token
        sdo_conf = (LCEC_CONF_SDOCONF_T *)conf;
        conf += sizeof(LCEC_CONF_SDOCONF_T) + sdo_conf->length;

        if (sdo_config == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "sdo_config is NULL\n");
          goto fail2;
        }

        // copy attributes
        sdo_config->index = sdo_conf->index;
        sdo_config->subindex = sdo_conf->subindex;
        sdo_config->length = sdo_conf->length;

        // copy data
        memcpy(sdo_config->data, sdo_conf->data, sdo_config->length);

        sdo_config = (lcec_slave_sdoconf_t *)&sdo_config->data[sdo_config->length];
        sdo_config->index = 0xffff;
        break;

      case lcecConfTypeIdnConfig:
        // get config token
        idn_conf = (LCEC_CONF_IDNCONF_T *)conf;
        conf += sizeof(LCEC_CONF_IDNCONF_T) + idn_conf->length;

        if (idn_config == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "idn_config is NULL\n");
          goto fail2;
        }

        // copy attributes
        idn_config->drive = idn_conf->drive;
        idn_config->idn = idn_conf->idn;
        idn_config->state = idn_conf->state;
        idn_config->length = idn_conf->length;

        // copy data
        memcpy(idn_config->data, idn_conf->data, idn_config->length);

        idn_config = (lcec_slave_idnconf_t *)&idn_config->data[idn_config->length];
        break;

      case lcecConfTypeModParam:
        // get config token
        modparam_conf = (LCEC_CONF_MODPARAM_T *)conf;
        conf += sizeof(LCEC_CONF_MODPARAM_T);

        // check for slave
        if (slave == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Slave node for modparam config missing\n");
          goto fail2;
        }

        if (modparams == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "modparams is nullg\n");
          goto fail2;
        }

        // copy attributes
        modparams->id = modparam_conf->id;
        modparams->value = modparam_conf->value;
        modparams->name = modparam_conf->name;

        // next entry
        modparams++;
        break;

      default:
        rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Unknown config item type\n");
        goto fail2;
    }
  }

  // close shmem
  rtapi_shmem_delete(shmem_id, lcec_comp_id);

  // allocate PDO entity memory
  for (master = first_master; master != NULL; master = master->next) {
    // stage 1 preinit: process all but FSOE logic devices
    for (slave = master->first_slave; slave != NULL; slave = slave->next) {
      if (!slave->is_fsoe_logic && slave->proc_preinit != NULL) {
        if (slave->proc_preinit(slave) < 0) {
          goto fail2;
        }
      }
    }

    // stage 2 preinit: process only FSOE logic devices (this depends on initialized fsoeConf data)
    for (slave = master->first_slave; slave != NULL; slave = slave->next) {
      if (slave->is_fsoe_logic && slave->proc_preinit != NULL) {
        if (slave->proc_preinit(slave) < 0) {
          goto fail2;
        }
      }
    }
  }

  return slave_count;

fail2:
  lcec_clear_config();
fail1:
  rtapi_shmem_delete(shmem_id, lcec_comp_id);
fail0:
  return -1;
}

/// @brief Clear configuration.
void lcec_clear_config(void) {
  lcec_master_t *master, *prev_master;
  lcec_slave_t *slave, *prev_slave;

  // iterate all masters
  master = last_master;
  while (master != NULL) {
    prev_master = master->prev;

    // iterate all masters
    slave = master->last_slave;
    while (slave != NULL) {
      prev_slave = slave->prev;

      // cleanup slave
      if (slave->proc_cleanup != NULL) {
        slave->proc_cleanup(slave);
      }

      slave = prev_slave;
    }

    // release master
    if (master->master) {
      ecrt_release_master(master->master);
    }

    master = prev_master;
  }
}

#ifdef __KERNEL__
/// @brief Lock LCEC.
static void lcec_request_lock(void *data) {
  lcec_master_t *master = (lcec_master_t *)data;
  rtapi_mutex_get(&master->mutex);
}

/// @brief Unlock LCEC.
static void lcec_release_lock(void *data) {
  lcec_master_t *master = (lcec_master_t *)data;
  rtapi_mutex_give(&master->mutex);
}
#endif

/// @brief Initialize LinuxCNC HAL pins for the master device.
lcec_master_data_t *lcec_init_master_hal(const char *pfx, int global) {
  lcec_master_data_t *hal_data;

  // alloc hal data
  if ((hal_data = LCEC_HAL_ALLOCATE(lcec_master_data_t)) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for %s failed\n", pfx);
    return NULL;
  }
  memset(hal_data, 0, sizeof(lcec_master_data_t));

  // export pins
  if (lcec_pin_newf_list(hal_data, master_global_pins, pfx) != 0) {
    return NULL;
  }
  if (!global) {
    if (lcec_pin_newf_list(hal_data, master_pins, pfx) != 0) {
      return NULL;
    }
    if (lcec_param_newf_list(hal_data, master_params, pfx) != 0) {
      return NULL;
    }
  }

  return hal_data;
}

/// @brief Initialize generic LinuxCNC HAL pins for a slave
lcec_slave_state_t *lcec_init_slave_state_hal(char *master_name, char *slave_name) {
  lcec_slave_state_t *hal_data;

  // alloc hal data
  if ((hal_data = LCEC_HAL_ALLOCATE(lcec_slave_state_t)) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for %s.%s.%s failed\n", LCEC_MODULE_NAME, master_name, slave_name);
    return NULL;
  }
  memset(hal_data, 0, sizeof(lcec_slave_state_t));

  // export pins
  if (lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master_name, slave_name) != 0) {
    return NULL;
  }

  return hal_data;
}

/// @brief Update HAL pins for the master.
void lcec_update_master_hal(lcec_master_data_t *hal_data, ec_master_state_t *ms) {
  *(hal_data->slaves_responding) = ms->slaves_responding;
  *(hal_data->state_init) = (ms->al_states & 0x01) != 0;
  *(hal_data->state_preop) = (ms->al_states & 0x02) != 0;
  *(hal_data->state_safeop) = (ms->al_states & 0x04) != 0;
  *(hal_data->state_op) = (ms->al_states & 0x08) != 0;
  *(hal_data->link_up) = ms->link_up;
  *(hal_data->all_op) = (ms->al_states == 0x08);
}

/// @brief Update generic HAL pins for a slave.
void lcec_update_slave_state_hal(lcec_slave_state_t *hal_data, ec_slave_config_state_t *ss) {
  *(hal_data->online) = ss->online;
  *(hal_data->operational) = ss->operational;
  *(hal_data->state_init) = (ss->al_state & 0x01) != 0;
  *(hal_data->state_preop) = (ss->al_state & 0x02) != 0;
  *(hal_data->state_safeop) = (ss->al_state & 0x04) != 0;
  *(hal_data->state_op) = (ss->al_state & 0x08) != 0;
}

/// @brief Update all input pins across all masters and slaves.
void lcec_read_all(void *arg, long period) {
  lcec_master_t *master;

  // initialize global state
  global_ms.slaves_responding = 0;
  global_ms.al_states = 0;
  global_ms.link_up = (first_master != NULL);

  // process slaves
  for (master = first_master; master != NULL; master = master->next) {
    lcec_read_master(master, period);
  }

  // update global state pins
  lcec_update_master_hal(global_hal_data, &global_ms);
}

/// @brief Update all output pins across all masters and slaves.
void lcec_write_all(void *arg, long period) {
  lcec_master_t *master;

  // process slaves
  for (master = first_master; master != NULL; master = master->next) {
    lcec_write_master(master, period);
  }
}

/// @brief Activate master on first call (delayed activation)
/// This ensures activation happens inside the real-time thread,
/// minimizing the delay between activation and cyclic communication.
static int lcec_activate_master(lcec_master_t *master) {
  struct timeval tv;
  
  uint64_t initial_app_time;
  
  if (master->activated) {
    return 0;  // Already activated
  }
  
  rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "Activating master %s (delayed activation in RT thread)\n", master->name);
  
  // Initialize application time base (now we're in the RT thread context)
  lcec_gettimeofday(&tv);
  master->app_time_base = EC_TIMEVAL2NANO(tv);
  
#ifdef RTAPI_TASK_PLL_SUPPORT
  master->dc_time_valid_last = 0;
  master->dc_ref = 0;
  if (!master->sync_to_ref_clock) {
    master->app_time_base -= rtapi_get_time();
  }
  // Calculate initial app_time (same formula as in cyclic write)
  initial_app_time = master->app_time_base + (master->sync_to_ref_clock ? 0 : rtapi_get_time());
#else
  master->app_time_base -= rtapi_get_time();
  initial_app_time = master->app_time_base + rtapi_get_time();
  if (master->sync_to_ref_clock) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "unable to sync master %s cycle to reference clock, RTAPI_TASK_PLL_SUPPORT not present\n",
        master->name);
  }
#endif

  // Set application time BEFORE activate
  // This sets dc_ref_time in the kernel, which is needed for DC SYNC0 phase calculation
  ecrt_master_application_time(master->master, initial_app_time);
#ifdef RTAPI_TASK_PLL_SUPPORT
  master->dc_ref_time = initial_app_time;  // Record the same value we sent to kernel
#endif
  rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "Initial app_time set to %llu\n", (unsigned long long)initial_app_time);

  // Activate master
  if (ecrt_master_activate(master->master)) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to activate master %s\n", master->name);
    return -1;
  }
  
  // Get internal process data for domain
  master->process_data = ecrt_domain_data(master->domain);
  master->process_data_len = ecrt_domain_size(master->domain);
  
  master->activated = 1;
  rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "Master %s activated successfully\n", master->name);
  
  return 0;
}

/// @brief Read all input pins on a master and its slaves.
void lcec_read_master(void *arg, long period) {
  lcec_master_t *master = (lcec_master_t *)arg;
  lcec_slave_t *slave;
  int check_states;

  // check period
  if (period != master->period_last) {
    master->period_last = period;
    if (master->app_time_period != period) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "Invalid appTimePeriod of %u for master %s (should be %ld).\n", master->app_time_period,
          master->name, period);
    }
  }

  // get state check flag
  if (master->state_update_timer > 0) {
    check_states = 0;
    master->state_update_timer -= period;
  } else {
    check_states = 1;
    master->state_update_timer = LCEC_STATE_UPDATE_PERIOD;
  }

  // receive process data & master state
  rtapi_mutex_get(&master->mutex);
  ecrt_master_receive(master->master);
  ecrt_domain_process(master->domain);
  if (check_states) {
    ecrt_master_state(master->master, &master->ms);
  }
  rtapi_mutex_give(&master->mutex);

  // update state pins
  lcec_update_master_hal(master->hal_data, &master->ms);

  // update global state
  global_ms.slaves_responding += master->ms.slaves_responding;
  global_ms.al_states |= master->ms.al_states;
  global_ms.link_up = global_ms.link_up && master->ms.link_up;

  // process slaves
  for (slave = master->first_slave; slave != NULL; slave = slave->next) {
    // get slaves state
    rtapi_mutex_get(&master->mutex);
    if (check_states) {
      ecrt_slave_config_state(slave->config, &slave->state);
    }
    rtapi_mutex_give(&master->mutex);
    if (check_states) {
      lcec_update_slave_state_hal(slave->hal_state_data, &slave->state);
    }

    // process read function
    if (slave->proc_read != NULL) {
      slave->proc_read(slave, period);
    }
  }
}

/// @brief Write all output pins on a master and its slaves.
void lcec_write_master(void *arg, long period) {
  lcec_master_t *master = (lcec_master_t *)arg;
  lcec_slave_t *slave;
  uint64_t app_time;
  long long now;
#ifdef RTAPI_TASK_PLL_SUPPORT
  long long ref;
  lcec_master_data_t *hal_data;
#endif

  // Skip if master not yet activated (should have been activated in read)
  if (!master->activated) {
    return;
  }

  // process slaves
  for (slave = master->first_slave; slave != NULL; slave = slave->next) {
    if (slave->proc_write != NULL) {
      slave->proc_write(slave, period);
    }
  }

#ifdef RTAPI_TASK_PLL_SUPPORT
  // get reference time
  ref = rtapi_task_pll_get_reference();
#endif

  // send process data
  rtapi_mutex_get(&master->mutex);
  ecrt_domain_queue(master->domain);

  // update application time
  now = rtapi_get_time();
#ifdef RTAPI_TASK_PLL_SUPPORT
  if (!master->sync_to_ref_clock) {
    app_time = master->app_time_base + now;
  } else {
    // Note: dc_ref is incremented AFTER calculating app_time
    // so first call gives app_time = app_time_base + 0 + (now - ref)
    // which is consistent with initial_app_time set at activation
    app_time = master->app_time_base + master->dc_ref + (now - ref);
    master->dc_ref += period;
  }
#else
  app_time = master->app_time_base + now;
#endif

  ecrt_master_application_time(master->master, app_time);

  // sync ref clock to master
  if (!master->sync_to_ref_clock) {
    if (master->sync_ref_cnt == 0) {
      master->sync_ref_cnt = master->sync_ref_cycles;
      ecrt_master_sync_reference_clock(master->master);
    }
    master->sync_ref_cnt--;
  }

  // sync slaves to ref clock
  ecrt_master_sync_slave_clocks(master->master);

  // send domain data
  ecrt_master_send(master->master);
  rtapi_mutex_give(&master->mutex);

#ifdef RTAPI_TASK_PLL_SUPPORT
  // BANG-BANG controller for master thread PLL sync
  // this part is done after ecrt_master_send() to reduce jitter
  hal_data = master->hal_data;
  *(hal_data->pll_err) = 0;
  *(hal_data->pll_out) = 0;
  *(hal_data->dc_phased) = 0;

  // Calculate app_phase: our execution position in local cycle
  // This is relative to dc_ref_time (the time we set at activation)
  // app_phase = (app_time - dc_ref_time) % period
  // This represents where we are within the current cycle since activation
  int32_t current_app_phase = (int32_t)((app_time - master->dc_ref_time) % master->app_time_period);
  *(hal_data->app_phase) = current_app_phase;
  int32_t app_period = (int32_t)master->app_time_period;
  
  // When sync_to_ref_clock = false: adjust app_phase to a stable position using PLL
  // This is needed because app_phase is random at startup
  if (!master->sync_to_ref_clock) {
    #define PHASE_MEASURE_CYCLES 100
    
    if (!hal_data->phase_calibrated) {
      // Phase 1: Measure app_phase jitter over PHASE_MEASURE_CYCLES cycles
      if (hal_data->phase_measure_cnt == 0) {
        // First measurement - initialize
        hal_data->phase_min = current_app_phase;
        hal_data->phase_max = current_app_phase;
        hal_data->phase_last = current_app_phase;
        hal_data->phase_measure_cnt = 1;
      } else if (hal_data->phase_measure_cnt < PHASE_MEASURE_CYCLES) {
        // Detect boundary crossing: if difference > app_period/2, phase wrapped around
        int32_t diff = current_app_phase - hal_data->phase_last;
        int32_t adjusted_phase = current_app_phase;
        
        // Unwrap: if phase jumped by more than half period, adjust for continuity
        if (diff > app_period / 2) {
          // Jumped from low to high (e.g., 10000 -> 990000), adjust down
          adjusted_phase = current_app_phase - app_period;
        } else if (diff < -app_period / 2) {
          // Jumped from high to low (e.g., 990000 -> 10000), adjust up
          adjusted_phase = current_app_phase + app_period;
        }
        
        // Update min/max with adjusted phase
        if (adjusted_phase < hal_data->phase_min) {
          hal_data->phase_min = adjusted_phase;
        }
        if (adjusted_phase > hal_data->phase_max) {
          hal_data->phase_max = adjusted_phase;
        }
        
        hal_data->phase_last = current_app_phase;
        hal_data->phase_measure_cnt++;
      } else {
        // Phase 2: Calculate jitter and target position
        hal_data->phase_jitter = hal_data->phase_max - hal_data->phase_min;
        *(hal_data->phase_jitter_out) = hal_data->phase_jitter;  // Output jitter for debugging
        
        // Target position: jitter + jitter/2 = jitter * 1.5
        int32_t target = hal_data->phase_jitter + hal_data->phase_jitter / 2;
        
        // Limit target to app_period/8
        int32_t max_target = app_period / 8;
        if (target > max_target) {
          target = max_target;
        }
        
        hal_data->phase_target = target;
        hal_data->phase_calibrated = 1;
        
        rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "Phase calibration complete: jitter=%d target=%d\n",
            hal_data->phase_jitter, hal_data->phase_target);
      }
    } else {
      // Phase 3: Use PLL to move app_phase towards target
      // Calculate error: how far are we from target?
      // Positive error (app_phase > target) means we need to speed up to reduce app_phase
      // Negative error (app_phase < target) means we need to slow down to increase app_phase
      int32_t phase_error = current_app_phase - hal_data->phase_target;
      
      // Handle wrap-around: if error > app_period/2, adjust
      if (phase_error > app_period / 2) {
        phase_error -= app_period;
      } else if (phase_error < -app_period / 2) {
        phase_error += app_period;
      }
      
      // Set pll_err for monitoring
      *(hal_data->pll_err) = phase_error;
      
      // Check if phased (within 10% of jitter or 1% of app_period, whichever is larger)
      int32_t lock_threshold = hal_data->phase_jitter;
      if (lock_threshold < app_period / 100) {
        lock_threshold = app_period / 100;
      }
      if (abs(phase_error) < lock_threshold) {
        // Increment lock counter - require 100 consecutive cycles within threshold
        hal_data->phase_lock_cnt++;
        if (hal_data->phase_lock_cnt >= 100) {
          *(hal_data->dc_phased) = 1;
        }
      } else {
        // Reset lock counter if we fall out of threshold
        hal_data->phase_lock_cnt = 0;
        *(hal_data->dc_phased) = 0;
      }
      
      // BANG-BANG control: small steps to move towards target
      // Positive pll_out = slow down = app_phase increases
      // Negative pll_out = speed up = app_phase decreases
      if (*(hal_data->dc_phased)) {
        *(hal_data->pll_out) = 0;
      } else {
        if (phase_error > 0) {
          *(hal_data->pll_out) = -(hal_data->pll_step);  // Speed up to reduce app_phase
        } else if (phase_error < 0) {
          *(hal_data->pll_out) = hal_data->pll_step;  // Slow down to increase app_phase
        }
      }

    }
  }
  
  // Read DC reference clock time (local variable)
  uint32_t dc_time = 0;
  int dc_time_valid = (ecrt_master_reference_clock_time(master->master, &dc_time) == 0);
  
  // the first read dc_time value seems to be invalid, so wait for two successive successful reads
  if (dc_time_valid && master->dc_time_valid_last) {
    // Raw offset between app_time and dc_time (this is what varies at each startup)
    int32_t raw_offset = master->app_time_last - dc_time;
    
    // Apply drift compensation based on drift-mode:
    //   0 = simple: (app_period - app_phase) % app_period
    //   1 = manual: use pll-drift pin value
    //   other = same as 1 (manual)
    int32_t drift = 0;
    int32_t mode = *(hal_data->drift_mode);
    if (master->sync_to_ref_clock) {
      if (mode == 0) {
        // Mode 0: simple - (app_period - app_phase) % app_period
        int32_t calc_val = (app_period - current_app_phase) % app_period;
        if (calc_val < 0) calc_val += app_period;
        if (hal_data->auto_drift_delay < 100) {
          hal_data->auto_drift_delay++;
        } else {
          drift = calc_val;
        }
      }
    }
    // Mode 1 or other: use manual pll-drift value
    
    *(hal_data->pll_err) = raw_offset + drift;
    
    // PLL is considered phased if error is within 10% of period
    int32_t lock_threshold = master->app_time_period / 10;
    if (abs(*(hal_data->pll_err)) < lock_threshold) {
      *(hal_data->dc_phased) = 1;
    }
    
    // Only run automatic PLL adjustment when sync_to_ref_clock is enabled
    // When sync_to_ref_clock = false, master is the clock source, DC syncs to us
    // When sync_to_ref_clock = true, DC is the clock source, we sync to DC
    if (master->sync_to_ref_clock) {
      // check for invalid error values
      if (abs(*(hal_data->pll_err)) > hal_data->pll_max_err) {
        // force resync of master time
        master->dc_ref -= *(hal_data->pll_err);
        // skip next control cycle to allow resync
        dc_time_valid = 0;
        // increment reset counter to document this event
        (*(hal_data->pll_reset_cnt))++;
        // Reset auto-drift delay on resync
        if (*(hal_data->drift_mode) == 0) {
          hal_data->auto_drift_delay = 0;
        }
      } else {
        if (*(hal_data->dc_phased)) {
          *(hal_data->pll_out) = 0;
        } else {
          *(hal_data->pll_out) = (*(hal_data->pll_err) < 0) ? -(hal_data->pll_step) : (hal_data->pll_step);
        }
      }
    }
    // Note: When sync_to_ref_clock = false, pll_out is set in the phase calibration code above
  }

  // Apply PLL correction with debug offset
  // pll_out is set by PLL controller (both sync_to_ref_clock modes now use PLL)
  // pll_drift is user-provided offset for debugging
  // Once phased, stop adjusting to maintain stability
  int32_t pll_correction;
  if (*(hal_data->dc_phased)) {
    pll_correction = *(hal_data->pll_drift);  // Stop adjusting once phased
  } else {
    pll_correction = *(hal_data->pll_out) + *(hal_data->pll_drift);
  }
  *(hal_data->pll_final) = pll_correction;
  rtapi_task_pll_set_correction(pll_correction);
  
  master->app_time_last = (uint32_t)app_time;
  master->dc_time_valid_last = dc_time_valid;
#endif
}


#ifndef __KERNEL__
#define BACKTRACE_SIZE 100

// Print a stack backtrace.  This presumably won't work in kernel
// mode.  It doesn't work *quite* right, in that it doesn't pick up
// symbols from lcec.so, but the offsets are enough to debug many
// problems.  Use `objdump --syms /usr/lib/linuxcnc/modules/lcec.so |
// sort | less` to extract a sorted list of symbols if needed.
static void sigsegv_handler(int sig) {
  struct sigaction act;
  int nptrs, i;
  void *buffer[BACKTRACE_SIZE];
  char **strings;

  fprintf(stderr, "LinuxCNC Ethercat crashed with signal %d\n", sig);

  fprintf(stderr, "-- STACK TRACE START\n");
  nptrs = backtrace(buffer, BACKTRACE_SIZE);
  strings = backtrace_symbols(buffer, nptrs);
  if (strings == NULL) {
    return;
  }

  for (i = 0; i < nptrs; ++i) {
    fprintf(stderr, "%d\t%s\n", i, strings[i]);
  }

  free(strings);

  fprintf(stderr, "-- STACK TRACE END\n");

  /* Restore the default behavior for the traped signal */
  sigemptyset(&act.sa_mask);
  act.sa_flags = SA_NODEFER | SA_ONSTACK | SA_RESETHAND;
  act.sa_handler = SIG_DFL;
  sigaction(sig, &act, NULL);
  kill(getpid(), sig);
}
#endif
