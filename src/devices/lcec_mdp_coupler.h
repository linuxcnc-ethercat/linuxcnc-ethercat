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
/// Modular couplers (one EtherCAT slave hosting N backplane I/O modules)
/// differ only in data: slot geometry, module ident lists, and per-module
/// PDO layouts.  All of that is declared in the vendor's ESI file, so this
/// driver consumes tables generated from the ESI by scripts/esi2coupler.py
/// instead of hand-written per-brand C.
///
/// Supporting a new coupler brand is:
///   1. generate its table:  scripts/esi2coupler.py --esi X.xml --family y
///   2. add the family to the list in lcec_mdp_coupler.c
///   3. if the hardware misbehaves in a way the ESI cannot express, add a
///      quirk flag to its registration (see lcec_mdp_quirks_t)
///
/// Config usage is the generic `<subModule>` grammar; the type name is the
/// family name in uppercase (documentation/mdp_coupler.md):
///
///   <slave idx="4" type="UC20" name="io">
///     <subModule id="0" ident="12" name="do1"/>
///     <subModule id="2" ident="1"  name="di3"/>
///   </slave>

#ifndef _LCEC_MDP_COUPLER_H_
#define _LCEC_MDP_COUPLER_H_

#include <stdint.h>

#include "../lcec.h"
#include "lcec_class_ain.h"
#include "lcec_class_aout.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"

/// @brief Behavioral quirks that ESI files cannot express.
///
/// These are per-family, discovered on hardware, and documented at the
/// registration site in lcec_mdp_coupler.c.
typedef enum {
  /// Explicitly write the SM PDO assignment objects (0x1C12/0x1C13) via SDO.
  /// Some couplers (Leadshine R2EC/R3EC) do not activate the input SM
  /// assignment from the master's automatic configuration.
  LCEC_MDP_QUIRK_EXPLICIT_SM_ASSIGN = 1 << 0,
  /// Do not pass a sync/PDO configuration to the master at all: the coupler
  /// derives its PDO layout purely from the 0xF030 module ident list and
  /// rejects CoE PDO (re)assignment (UTRIO UC20: "does not support assigning
  /// PDOs").  The layout tables are then only used for HAL pin registration.
  LCEC_MDP_QUIRK_NO_PDO_ASSIGN = 1 << 1,
} lcec_mdp_quirks_t;

/// @brief Kind of module; selects the HAL pin class for its channels.
typedef enum {
  LCEC_MDP_MOD_OTHER = 0,  // mapped for process-data completeness, no pins
  LCEC_MDP_MOD_DIN,
  LCEC_MDP_MOD_DOUT,
  LCEC_MDP_MOD_DIO,
  LCEC_MDP_MOD_AIN,
  LCEC_MDP_MOD_AOUT,
  LCEC_MDP_MOD_ENC,  // not yet pin-mapped; layout only
} lcec_mdp_modkind_t;

/// @brief One PDO entry of a module's default mapping, from the ESI.
typedef struct {
  uint16_t index;     ///< object index for slot 0
  uint8_t index_dos;  ///< index is DependOnSlot (add slot * slot_index_incr)
  uint8_t subindex;
  uint8_t bitlen;
  uint8_t padding;   ///< alignment/reserved entry: mapped, but no HAL pin
  const char *name;  ///< entry name per ESI (used in pin naming diagnostics)
} lcec_mdp_pdo_entry_t;

/// @brief One PDO the coupler activates for a module, from the ESI.
///
/// A module contributes every mapping the coupler assigns by itself, not just
/// its data mapping: an Inovance GL20 builds 0x1C12/0x1C13 from its detected
/// module list and includes each module's diagnosis mapping, so a master that
/// configured only the data mappings would compute every following byte
/// offset against a shorter image than the slave actually produces.
typedef struct {
  uint16_t index;     ///< PDO index for slot 0
  uint8_t index_dos;  ///< index is DependOnSlot (add slot * slot_pdo_incr)
  uint8_t io;         ///< entries are process I/O (ETG.5001 0x6000/0x7000 ranges)
                      ///< and may become HAL pins; 0 for diagnosis (0xA000) and
                      ///< other status mappings, which are mapped but get no pins
  const lcec_mdp_pdo_entry_t *entries;
  uint16_t entry_count;
} lcec_mdp_pdo_t;

/// @brief One module type, from the ESI `<Module>` list.
typedef struct {
  uint32_t ident;    ///< ModuleIdent, matched against <subModule ident=...>
  const char *name;  ///< type name per ESI
  lcec_mdp_modkind_t kind;
  const lcec_mdp_pdo_t *tx_pdos;  ///< inputs, in ESI order (NULL = none)
  uint16_t tx_pdo_count;
  const lcec_mdp_pdo_t *rx_pdos;  ///< outputs, in ESI order (NULL = none)
  uint16_t rx_pdo_count;
} lcec_mdp_module_t;

/// @brief One coupler family: everything scripts/esi2coupler.py extracts
/// from the vendor ESI, plus quirks added at registration.
typedef struct {
  const char *name;
  uint32_t vid;
  uint32_t pid;
  uint32_t revision;
  uint32_t slot_index_incr;  ///< CoE object index increment per slot
  uint32_t slot_pdo_incr;    ///< PDO index increment per slot
  uint32_t max_slots;
  uint8_t download_ident_list;       ///< write 0xF030 (per ESI DownloadModuleIdentList)
  const lcec_mdp_module_t *modules;  ///< ident-0-terminated
} lcec_mdp_family_t;

/// @brief Per-slot runtime state.
typedef struct {
  uint8_t id;
  const lcec_mdp_module_t *def;
  lcec_class_din_channels_t *din;
  lcec_class_dout_channels_t *dout;
  lcec_class_ain_channels_t *ain;
  lcec_class_aout_channels_t *aout;
} lcec_mdp_slot_t;

/// @brief Per-slave HAL data.
typedef struct {
  const lcec_mdp_family_t *family;
  uint64_t quirks;
  int slot_count;
  lcec_mdp_slot_t *slots;
  lcec_syncs_t *syncs;  ///< layout built in _init (NULL for NO_PDO_ASSIGN), reused for the PDO assignment on re-init
} lcec_mdp_coupler_data_t;

// CoE objects common to all MDP couplers.
#define LCEC_MDP_CONFMODULES       0xF030
#define LCEC_MDP_SM_PDO_ASSIGN(sm) (0x1C10 + (sm))

// Mailbox couplers: process data on SM2 (outputs) / SM3 (inputs).
#define LCEC_MDP_SM_OUT 2
#define LCEC_MDP_SM_IN  3

#endif
