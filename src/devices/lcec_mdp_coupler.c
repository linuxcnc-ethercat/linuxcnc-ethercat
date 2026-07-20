//
//    Copyright (C) 2026 linuxcnc-ethercat contributors
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
/// @brief Generic driver for ETG.5001 MDP modular bus couplers.
///
/// See lcec_mdp_coupler.h for the design.  Module tables are generated from
/// vendor ESI files by scripts/esi2coupler.py; per-family hardware quirks
/// are declared in the family registration table below.

#include "lcec_mdp_coupler.h"

#include <stdio.h>
#include <string.h>

#include "../lcec.h"
#include "lcec_mdp_gl20.h"
#include "lcec_mdp_uc20.h"

static int lcec_mdp_coupler_init(int comp_id, lcec_slave_t *slave);
static void lcec_mdp_coupler_read(lcec_slave_t *slave, long period);
static void lcec_mdp_coupler_write(lcec_slave_t *slave, long period);

/// @brief One registered coupler family: generated table + quirks.
typedef struct {
  const lcec_mdp_family_t *family;
  uint64_t quirks;
} lcec_mdp_registration_t;

/// Families supported by this driver.  Quirks are hardware findings that the
/// ESI cannot express; document each with the observed behavior.
static const lcec_mdp_registration_t registrations[] = {
    // UTRIO UC20-RTU-EC-A: accepts the master's SM config attempt with
    // "does not support changing the PDO mapping" warnings (its layout is
    // fixed by the 0xF030 ident list), then exchanges correctly.  No quirks
    // needed; tested on hardware 2026-07.
    {&uc20_family, 0},
    // Inovance GL20(S)-RTU-ECT32: same MDP pattern per ESI; supports DC
    // sync0 (AssignActivate 0x300).  Untested on hardware so far.
    {&gl20_family, 0},
    {NULL, 0},
};

// The typelist is built from `registrations` at load time, one entry per
// family, with `flags` = index into `registrations`.
static lcec_typelist_t types[3];  // registrations count + terminator

static void AddTypesMdpCoupler(void) __attribute__((constructor));
static void AddTypesMdpCoupler(void) {
  int i;

  for (i = 0; registrations[i].family != NULL; i++) {
    const lcec_mdp_family_t *fam = registrations[i].family;

    // count modules, build the parser-facing submodule descriptor table
    int n = 0;
    while (fam->modules[n].name != NULL) n++;
    lcec_submodule_desc_t *subs = LCEC_ALLOCATE_ARRAY(lcec_submodule_desc_t, (n + 1));
    for (int m = 0; m < n; m++) {
      subs[m].name = fam->modules[m].name;
      subs[m].ident = fam->modules[m].ident;
      subs[m].modparams = NULL;  // modparams not supported yet (see docs)
    }
    subs[n].name = NULL;

    types[i].name = fam->name;
    types[i].vid = fam->vid;
    types[i].pid = fam->pid;
    types[i].proc_init = lcec_mdp_coupler_init;
    types[i].modules = subs;
    types[i].flags = (uint64_t)i;
    types[i].sourcefile = __FILE__;
  }
  types[i].name = NULL;

  lcec_addtypes(types, __FILE__);
}

/// @brief Look up a module definition by ident within a family.
static const lcec_mdp_module_t *lcec_mdp_find_module(const lcec_mdp_family_t *fam, uint32_t ident) {
  for (const lcec_mdp_module_t *m = fam->modules; m->name != NULL; m++) {
    if (m->ident == ident) return m;
  }
  return NULL;
}

/// @brief Allocate a persistent "<base>-<kind>-<ch>" HAL pin name.
static char *lcec_mdp_name(const char *base, const char *kind, int ch) {
  char buf[HAL_NAME_LEN];
  snprintf(buf, sizeof(buf), "%s-%s-%d", base, kind, ch);
  char *s = LCEC_HAL_ALLOCATE_STRING(strlen(buf) + 1);
  strcpy(s, buf);
  return s;
}

/// @brief Entry's object index for a given slot.
static uint16_t lcec_mdp_entry_index(const lcec_mdp_family_t *fam, const lcec_mdp_pdo_entry_t *e, uint8_t slot) {
  return e->index + (e->index_dos ? slot * fam->slot_index_incr : 0);
}

/// @brief Append one direction of a slot's mapping to the syncs builder.
static void lcec_mdp_append_pdos(lcec_syncs_t *syncs, const lcec_mdp_family_t *fam, uint8_t slot, uint16_t pdo_base, uint8_t pdo_dos,
    const lcec_mdp_pdo_entry_t *entries, int count) {
  if (pdo_base == 0 || count == 0) return;
  lcec_syncs_add_pdo_info(syncs, pdo_base + (pdo_dos ? slot * fam->slot_pdo_incr : 0));
  for (int i = 0; i < count; i++) {
    lcec_syncs_add_pdo_entry(syncs, lcec_mdp_entry_index(fam, &entries[i], slot), entries[i].subindex, entries[i].bitlen);
  }
}

/// @brief Build the dynamic SM/PDO layout for the configured slots.
/// @return 0 on success, -EINVAL if the layout overflowed the sync builder.
static int lcec_mdp_build_syncs(lcec_slave_t *slave, const lcec_mdp_family_t *fam, lcec_syncs_t *syncs) {
  lcec_syncs_init(slave, syncs);

  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);  // SM0 mailbox out
  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DEFAULT);   // SM1 mailbox in

  lcec_syncs_add_sync(syncs, EC_DIR_OUTPUT, EC_WD_DEFAULT);  // SM2 outputs
  for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next) {
    const lcec_mdp_module_t *def = lcec_mdp_find_module(fam, s->ident);
    if (def != NULL) lcec_mdp_append_pdos(syncs, fam, s->id, def->rx_pdo, def->rx_pdo_dos, def->rx_entries, def->rx_entry_count);
  }

  lcec_syncs_add_sync(syncs, EC_DIR_INPUT, EC_WD_DEFAULT);  // SM3 inputs
  for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next) {
    const lcec_mdp_module_t *def = lcec_mdp_find_module(fam, s->ident);
    if (def != NULL) lcec_mdp_append_pdos(syncs, fam, s->id, def->tx_pdo, def->tx_pdo_dos, def->tx_entries, def->tx_entry_count);
  }

  if (syncs->error) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s.%s: PDO/sync layout overflow: too many modules/channels configured\n",
        slave->master->name, slave->name);
    return -EINVAL;
  }
  slave->sync_info = &syncs->syncs[0];
  return 0;
}

/// @brief Register HAL pins for one slot from its entry tables.
///
/// Digital channels are BOOL (bitlen 1) non-padding entries; analog channels
/// are 16-bit non-padding entries.  Padding/diagnostic entries are mapped in
/// the PDO layout but get no pins.
static int lcec_mdp_register_slot(lcec_slave_t *slave, const lcec_mdp_family_t *fam, lcec_mdp_slot_t *slot, const char *base) {
  const lcec_mdp_module_t *def = slot->def;

  // digital inputs
  if (def->kind == LCEC_MDP_MOD_DIN || def->kind == LCEC_MDP_MOD_DIO) {
    int n = 0;
    for (int i = 0; i < def->tx_entry_count; i++)
      if (!def->tx_entries[i].padding && def->tx_entries[i].bitlen == 1) n++;
    if (n > 0) {
      slot->din = lcec_din_allocate_channels(n);
      int ch = 0;
      for (int i = 0; i < def->tx_entry_count; i++) {
        const lcec_mdp_pdo_entry_t *e = &def->tx_entries[i];
        if (e->padding || e->bitlen != 1) continue;
        slot->din->channels[ch] =
            lcec_din_register_channel_named(slave, lcec_mdp_entry_index(fam, e, slot->id), e->subindex, lcec_mdp_name(base, "din", ch));
        if (slot->din->channels[ch] == NULL) return -EIO;
        ch++;
      }
    }
  }

  // digital outputs
  if (def->kind == LCEC_MDP_MOD_DOUT || def->kind == LCEC_MDP_MOD_DIO) {
    int n = 0;
    for (int i = 0; i < def->rx_entry_count; i++)
      if (!def->rx_entries[i].padding && def->rx_entries[i].bitlen == 1) n++;
    if (n > 0) {
      slot->dout = lcec_dout_allocate_channels(n);
      int ch = 0;
      for (int i = 0; i < def->rx_entry_count; i++) {
        const lcec_mdp_pdo_entry_t *e = &def->rx_entries[i];
        if (e->padding || e->bitlen != 1) continue;
        slot->dout->channels[ch] =
            lcec_dout_register_channel_named(slave, lcec_mdp_entry_index(fam, e, slot->id), e->subindex, lcec_mdp_name(base, "dout", ch));
        if (slot->dout->channels[ch] == NULL) return -EIO;
        ch++;
      }
    }
  }

  // analog inputs (16-bit value entries)
  if (def->kind == LCEC_MDP_MOD_AIN) {
    int n = 0;
    for (int i = 0; i < def->tx_entry_count; i++)
      if (!def->tx_entries[i].padding && def->tx_entries[i].bitlen == 16) n++;
    if (n > 0) {
      slot->ain = lcec_ain_allocate_channels(n);
      int ch = 0;
      for (int i = 0; i < def->tx_entry_count; i++) {
        const lcec_mdp_pdo_entry_t *e = &def->tx_entries[i];
        if (e->padding || e->bitlen != 16) continue;
        lcec_class_ain_options_t *opt = lcec_ain_options();
        opt->name_prefix = lcec_mdp_name(base, "ain", ch);
        opt->valueonly = 1;
        opt->value_idx = lcec_mdp_entry_index(fam, e, slot->id);
        opt->value_sidx = e->subindex;
        slot->ain->channels[ch] = lcec_ain_register_channel(slave, ch, opt->value_idx, opt);
        if (slot->ain->channels[ch] == NULL) return -EIO;
        ch++;
      }
    }
  }

  // analog outputs (16-bit value entries)
  if (def->kind == LCEC_MDP_MOD_AOUT) {
    int n = 0;
    for (int i = 0; i < def->rx_entry_count; i++)
      if (!def->rx_entries[i].padding && def->rx_entries[i].bitlen == 16) n++;
    if (n > 0) {
      slot->aout = lcec_aout_allocate_channels(n);
      int ch = 0;
      for (int i = 0; i < def->rx_entry_count; i++) {
        const lcec_mdp_pdo_entry_t *e = &def->rx_entries[i];
        if (e->padding || e->bitlen != 16) continue;
        lcec_class_aout_options_t *opt = lcec_aout_options();
        opt->name_prefix = lcec_mdp_name(base, "aout", ch);
        opt->value_sidx = e->subindex;
        slot->aout->channels[ch] = lcec_aout_register_channel(slave, ch, lcec_mdp_entry_index(fam, e, slot->id), opt);
        if (slot->aout->channels[ch] == NULL) return -EIO;
        ch++;
      }
    }
  }

  return 0;
}

/// @brief Write the configured module ident list (0xF030).
///
/// Sub 0 is the USINT count; subs 1..N are UDINT module idents (slot id + 1).
/// Gap subindices inside the declared count are cleared to 0 so a sparse
/// config cannot leave stale idents that fail the coupler's configured-vs-
/// detected check at the SafeOp transition.
static int lcec_mdp_write_module_list(lcec_slave_t *slave, const lcec_mdp_family_t *fam) {
  int count = 0;
  int err;

  for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next) {
    if (s->id + 1 > count) count = s->id + 1;
  }

  for (int sub = 1; sub <= count; sub++) {
    uint32_t ident = 0;
    for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next) {
      if (s->id + 1 == sub) ident = s->ident;
    }
    if ((err = lcec_write_sdo32(slave, LCEC_MDP_CONFMODULES, sub, ident)) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s.%s: failed writing module ident 0x%08x to 0x%04x:%02x\n", slave->master->name,
          slave->name, ident, LCEC_MDP_CONFMODULES, sub);
      return err;
    }
  }
  // Some couplers maintain the count themselves and mark 0xF030:00 read-only
  // (UTRIO UC20 aborts with 0x06010002 "attempt to write a read-only
  // object").  Not fatal: the ident subindices above are what the
  // configured-vs-detected check consumes.
  if ((err = lcec_write_sdo8(slave, LCEC_MDP_CONFMODULES, 0x00, count)) != 0) {
    rtapi_print_msg(RTAPI_MSG_WARN,
        LCEC_MSG_PFX "%s.%s: module count write to 0x%04x:00 rejected (read-only on some couplers), continuing\n", slave->master->name,
        slave->name, LCEC_MDP_CONFMODULES);
  }
  return 0;
}

/// @brief Explicitly write the SM2/SM3 PDO assignment (0x1C12/0x1C13).
/// Only used for families with LCEC_MDP_QUIRK_EXPLICIT_SM_ASSIGN.
static int lcec_mdp_assign_pdos(lcec_slave_t *slave, lcec_syncs_t *syncs) {
  ec_sync_info_t *sm_out = &syncs->syncs[LCEC_MDP_SM_OUT];
  ec_sync_info_t *sm_in = &syncs->syncs[LCEC_MDP_SM_IN];
  int err;

  if ((err = lcec_write_sdo8(slave, LCEC_MDP_SM_PDO_ASSIGN(LCEC_MDP_SM_OUT), 0, 0)) != 0) return err;
  if ((err = lcec_write_sdo8(slave, LCEC_MDP_SM_PDO_ASSIGN(LCEC_MDP_SM_IN), 0, 0)) != 0) return err;
  for (unsigned int i = 0; i < sm_out->n_pdos; i++) {
    if ((err = lcec_write_sdo16(slave, LCEC_MDP_SM_PDO_ASSIGN(LCEC_MDP_SM_OUT), i + 1, sm_out->pdos[i].index)) != 0) return err;
  }
  for (unsigned int i = 0; i < sm_in->n_pdos; i++) {
    if ((err = lcec_write_sdo16(slave, LCEC_MDP_SM_PDO_ASSIGN(LCEC_MDP_SM_IN), i + 1, sm_in->pdos[i].index)) != 0) return err;
  }
  if ((err = lcec_write_sdo8(slave, LCEC_MDP_SM_PDO_ASSIGN(LCEC_MDP_SM_OUT), 0, sm_out->n_pdos)) != 0) return err;
  if ((err = lcec_write_sdo8(slave, LCEC_MDP_SM_PDO_ASSIGN(LCEC_MDP_SM_IN), 0, sm_in->n_pdos)) != 0) return err;
  return 0;
}

static int lcec_mdp_coupler_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  const lcec_mdp_registration_t *reg = &registrations[slave->flags];
  const lcec_mdp_family_t *fam = reg->family;
  int err;

  lcec_mdp_coupler_data_t *hal_data = LCEC_HAL_ALLOCATE(lcec_mdp_coupler_data_t);
  slave->hal_data = hal_data;
  hal_data->family = fam;
  hal_data->quirks = reg->quirks;

  // validate the configured slots before building anything: slot bounds,
  // duplicate slot ids, duplicate names (all fatal misconfigurations)
  int slot_count = 0;
  for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next, slot_count++) {
    if (s->id >= fam->max_slots) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s.%s: subModule slot %d exceeds max %u for %s\n", master->name, slave->name, s->id,
          fam->max_slots, fam->name);
      return -EINVAL;
    }
    for (lcec_slave_submodule_t *t = s->next; t != NULL; t = t->next) {
      if (t->id == s->id) {
        rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s.%s: duplicate subModule slot id %d\n", master->name, slave->name, s->id);
        return -EINVAL;
      }
      if (strcmp(t->name, s->name) == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s.%s: duplicate subModule name \"%s\"\n", master->name, slave->name, s->name);
        return -EINVAL;
      }
    }
  }

  hal_data->slot_count = slot_count;
  hal_data->slots = slot_count > 0 ? LCEC_HAL_ALLOCATE_ARRAY(lcec_mdp_slot_t, slot_count) : NULL;

  // build the dynamic SM/PDO layout (skipped entirely for NO_PDO_ASSIGN
  // couplers, whose layout is fixed by the 0xF030 ident list)
  lcec_syncs_t *syncs = NULL;
  if (!(reg->quirks & LCEC_MDP_QUIRK_NO_PDO_ASSIGN)) {
    syncs = LCEC_HAL_ALLOCATE(lcec_syncs_t);
    if ((err = lcec_mdp_build_syncs(slave, fam, syncs)) != 0) {
      return err;
    }
  }

  // register HAL pins per slot
  int i = 0;
  for (lcec_slave_submodule_t *s = slave->submodules; s != NULL; s = s->next, i++) {
    lcec_mdp_slot_t *slot = &hal_data->slots[i];
    slot->id = s->id;
    slot->def = lcec_mdp_find_module(fam, s->ident);
    if (slot->def == NULL) {
      // the parser validates idents against the same table; defensive only
      rtapi_print_msg(
          RTAPI_MSG_ERR, LCEC_MSG_PFX "%s.%s: unknown submodule ident 0x%08x in slot %d\n", master->name, slave->name, s->ident, s->id);
      return -EINVAL;
    }
    if ((err = lcec_mdp_register_slot(slave, fam, slot, s->name)) != 0) {
      return err;
    }
  }

  if (fam->download_ident_list && (err = lcec_mdp_write_module_list(slave, fam)) != 0) {
    return err;
  }
  if ((reg->quirks & LCEC_MDP_QUIRK_EXPLICIT_SM_ASSIGN) && syncs != NULL && (err = lcec_mdp_assign_pdos(slave, syncs)) != 0) {
    return err;
  }

  slave->proc_read = lcec_mdp_coupler_read;
  slave->proc_write = lcec_mdp_coupler_write;
  return 0;
}

static void lcec_mdp_coupler_read(lcec_slave_t *slave, long period) {
  lcec_mdp_coupler_data_t *hal_data = (lcec_mdp_coupler_data_t *)slave->hal_data;

  if (!slave->state.operational) return;

  for (int i = 0; i < hal_data->slot_count; i++) {
    lcec_mdp_slot_t *slot = &hal_data->slots[i];
    if (slot->din != NULL) lcec_din_read_all(slave, slot->din);
    if (slot->ain != NULL) lcec_ain_read_all(slave, slot->ain);
  }
}

static void lcec_mdp_coupler_write(lcec_slave_t *slave, long period) {
  lcec_mdp_coupler_data_t *hal_data = (lcec_mdp_coupler_data_t *)slave->hal_data;

  if (!slave->state.operational) return;

  for (int i = 0; i < hal_data->slot_count; i++) {
    lcec_mdp_slot_t *slot = &hal_data->slots[i];
    if (slot->dout != NULL) lcec_dout_write_all(slave, slot->dout);
    if (slot->aout != NULL) lcec_aout_write_all(slave, slot->aout);
  }
}
