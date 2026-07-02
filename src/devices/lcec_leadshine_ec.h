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
/// @brief Driver for the Leadshine R2EC / R3EC EtherCAT bus couplers and
/// their modular I/O sub-devices (issue #434).
///
/// The R2EC/R3EC is a single EtherCAT slave that hosts up to 32 (R2EC) or
/// 64 (R3EC) modules on a passive backplane.  Each populated slot is
/// described in the LinuxCNC XML config as a `<subModule>` (see the generic
/// submodule support in lcec_conf.c / lcec_main.c):
///
///   <slave idx="0" type="R2EC" name="io">
///     <subModule id="0" ident="0x61100025" name="in0">
///       <modParam name="inputFilter0" value="20"/>
///     </subModule>
///     <subModule id="1" ident="0x61100205" name="out0">
///       <modParam name="safeState" value="0"/>
///     </subModule>
///   </slave>
///
/// The coupler reports the modules it actually detects in 0xF050; the master
/// mirrors the configured list into 0xF030 so the coupler accepts the PDO
/// mapping at the SafeOp transition.
///
/// Addressing (per ESI / MDP conventions), for slot N (= `<subModule id>`):
///   CoE objects : inputs 0x6000 + N*SLOT_INCR, outputs 0x7000 + N*SLOT_INCR
///   PDOs        : TxPdo (in)  0x1A00 + N*PdoIncr
///                 RxPdo (out) 0x1600 + N*PdoIncr
///   SLOT_INCR is 0x10 for both couplers.  PdoIncr is 0x10 for R2EC and
///   0x08 for R3EC (packed into the type's `flags`, see LEADSHINE_EC_FLAG).

#ifndef _LCEC_LEADSHINE_EC_H_
#define _LCEC_LEADSHINE_EC_H_

#include <stdint.h>

#include "../lcec.h"
#include "lcec_class_ain.h"
#include "lcec_class_aout.h"
#include "lcec_class_din.h"
#include "lcec_class_dout.h"
#include "lcec_class_enc.h"

// CoE objects the coupler exposes.
#define LEADSHINE_EC_READMODULES 0xF050  // detected module ident list (read)
#define LEADSHINE_EC_CONFMODULES 0xF030  // configured module ident list (written by us)
#define LEADSHINE_EC_INOBJ       0x6000  // per-slot input object base
#define LEADSHINE_EC_OUTOBJ      0x7000  // per-slot output object base
#define LEADSHINE_EC_TXPDO       0x1A00  // per-slot input (Tx) PDO base
#define LEADSHINE_EC_RXPDO       0x1600  // per-slot output (Rx) PDO base
#define LEADSHINE_EC_SLOT_INCR   0x10    // CoE object index increment per slot

// The coupler is a CoE device with mailbox sync managers, so process-data
// PDOs live on SM2 (outputs) / SM3 (inputs).
#define LEADSHINE_EC_SM_MBOX_OUT 0  // SM0: mailbox out (empty)
#define LEADSHINE_EC_SM_MBOX_IN  1  // SM1: mailbox in (empty)
#define LEADSHINE_EC_SM_OUT      2  // SM2: RxPDO / outputs
#define LEADSHINE_EC_SM_IN       3  // SM3: TxPDO / inputs

// SM PDO-assignment objects: 0x1C10 + SM index (0x1C12 = SM2, 0x1C13 = SM3).
// Subindex 0 is the USINT count; subindices 1..N are UINT PDO indices.
#define LEADSHINE_EC_SM_PDO_ASSIGN(sm) (0x1C10 + (sm))

// Per-slot config object base (0x8000 + slot*SLOT_INCR) and the analog/temperature
// diagnostic object base (0xA000).  All addresses below are taken from the R3EC
// ESI (documentation/R3EC-v2.4.xml) and are DependOnSlot, i.e. base + slot*incr.
#define LEADSHINE_EC_CFGOBJ  0x8000  // per-slot config object base
#define LEADSHINE_EC_DIAGOBJ 0xA000  // analog/temp diagnostic (input) object base

// Config-object subindices (per the ESI DT8000 layouts).
#define LEADSHINE_EC_SUB_SAFESTATE_LO 1  // DO/relay: output state on link loss, bits 0-15  (UINT16)
#define LEADSHINE_EC_SUB_SAFESTATE_HI 2  // DO 32-ch: output state on link loss, bits 16-31 (UINT16)
#define LEADSHINE_EC_SUB_FILTER_BASE  3  // DI: input filter, subindices 3..6 per 8-channel group (UINT16)
#define LEADSHINE_EC_SUB_ANALOG_BASE  1  // AIN/AOUT: per-channel range/mode config, sub 1..4 (USINT8)
#define LEADSHINE_EC_SUB_ENC_MODE     1  // encoder: operation mode (USINT8)
#define LEADSHINE_EC_SUB_ENC_ABPHASE  2  // encoder: AB phase (USINT8)
#define LEADSHINE_EC_SUB_ENC_MIN      4  // encoder: minimum value (DINT32)
#define LEADSHINE_EC_SUB_ENC_MAX      5  // encoder: maximum value (DINT32)
#define LEADSHINE_EC_SUB_ENC_COUNTMODE 8  // encoder: count mode (USINT8)
#define LEADSHINE_EC_SUB_ENC_FILTER    9  // encoder: input filter (UINT16)

/// @brief Pack per-type geometry into the typelist `flags` field.
/// @param max_slots Number of backplane slots the coupler supports.
/// @param pdo_incr  PDO index increment per slot (0x10 R2EC, 0x08 R3EC).
#define LEADSHINE_EC_FLAG(max_slots, pdo_incr) (((uint64_t)(max_slots) & 0xff) | (((uint64_t)(pdo_incr) & 0xff) << 8))
#define LEADSHINE_EC_MAX_SLOTS(f)              ((int)((f) & 0xff))
#define LEADSHINE_EC_PDO_INCR(f)               ((int)(((f) >> 8) & 0xff))

/// @brief Kind of module in a slot; selects the PDO shape and HAL class.
typedef enum {
  MODULE_UNKNOWN = 0,
  MODULE_DIN,      // digital input
  MODULE_DOUT,     // digital output (incl. relay)
  MODULE_DIO,      // mixed digital input + output
  MODULE_AIN,      // analog input
  MODULE_AOUT,     // analog output
  MODULE_ENCODER,  // encoder
} leadshine_ec_modkind_t;

/// @brief Static description of a supported module type.
///
/// `in` / `out` are the channel counts (digital: bits; analog/encoder:
/// channels).  This single table is the source of truth: the driver builds
/// the `lcec_submodule_desc_t[]` (attached to `types[].modules`) from it at
/// load time, so the XML parser can validate `<subModule ident=...>` and its
/// child `<modParam>`s.
typedef struct {
  uint32_t ident;                         // module ident (matches <subModule ident=...>)
  const char *name;                       // module name (per ESI)
  leadshine_ec_modkind_t type;            // module kind
  uint8_t in;                             // input channel count
  uint8_t out;                            // output channel count
  const lcec_modparam_desc_t *modparams;  // modparams supported by this module type
} leadshine_ec_module_def_t;

/// @brief Runtime state for one populated backplane slot.
///
/// Only the class container(s) relevant to the module kind are allocated; the
/// rest stay NULL.  Encoders don't use a class container (class_enc owns its
/// own struct), so we keep an array plus per-channel position PDO offsets.
typedef struct {
  uint8_t id;                        // backplane slot (= <subModule id>)
  uint32_t ident;                    // module ident
  leadshine_ec_modkind_t type;       // module kind
  lcec_class_din_channels_t *din;    // DIN / DIO inputs, or NULL
  lcec_class_dout_channels_t *dout;  // DOUT / DIO outputs, or NULL
  lcec_class_ain_channels_t *ain;    // AIN inputs, or NULL
  lcec_class_aout_channels_t *aout;  // AOUT outputs, or NULL
  int enc_count;                     // number of encoder channels
  lcec_class_enc_data_t *enc;        // encoder channel array, or NULL
  unsigned int *enc_pos_os;          // per-encoder position PDO offset, or NULL
} leadshine_ec_slot_t;

/// @brief Per-slave HAL data: one entry per configured `<subModule>`.
typedef struct {
  int slot_count;
  leadshine_ec_slot_t *slots;
} lcec_leadshine_ec_data_t;

// Modparam ids.  These are the keys passed to lcec_submodule_modparam_get();
// they are per-module-type, so the same numeric id may recur across the
// digital/analog/encoder tables without conflict.
#define LEADSHINE_EC_MP_SAFESTATE      1  // digital output safe value on link loss
#define LEADSHINE_EC_MP_INPUTFILTER(g) (3 + (g))  // digital input filter, group g (0..3)
#define LEADSHINE_EC_MP_ANALOG_CFG(ch) (2 + (ch))  // analog channel config, ch (0..3)
#define LEADSHINE_EC_MP_ENC_MODE       2
#define LEADSHINE_EC_MP_ENC_ABPHASE    3
#define LEADSHINE_EC_MP_ENC_MINVALUE   4
#define LEADSHINE_EC_MP_ENC_MAXVALUE   5
#define LEADSHINE_EC_MP_ENC_COUNTMODE  6
#define LEADSHINE_EC_MP_ENC_FILTER     7

static const lcec_modparam_desc_t leadshine_ec_digital_params[] = {
    {"safeState", LEADSHINE_EC_MP_SAFESTATE, MODPARAM_TYPE_U32, "0", "Output value when link is lost (0 = all off)"},
    {"inputFilter0", LEADSHINE_EC_MP_INPUTFILTER(0), MODPARAM_TYPE_U32, "10", "Input filter time group 0 (channels 0-7), 0-255 ms"},
    {"inputFilter1", LEADSHINE_EC_MP_INPUTFILTER(1), MODPARAM_TYPE_U32, "10", "Input filter time group 1 (channels 8-15), 0-255 ms"},
    {"inputFilter2", LEADSHINE_EC_MP_INPUTFILTER(2), MODPARAM_TYPE_U32, "10", "Input filter time group 2 (channels 16-23), 0-255 ms"},
    {"inputFilter3", LEADSHINE_EC_MP_INPUTFILTER(3), MODPARAM_TYPE_U32, "10", "Input filter time group 3 (channels 24-31), 0-255 ms"},
    {NULL},
};

static const lcec_modparam_desc_t leadshine_ec_analog_params[] = {
    {"ch0Config", LEADSHINE_EC_MP_ANALOG_CFG(0), MODPARAM_TYPE_U32, "0", "Channel 0 configuration (AD/DA config)"},
    {"ch1Config", LEADSHINE_EC_MP_ANALOG_CFG(1), MODPARAM_TYPE_U32, "0", "Channel 1 configuration (AD/DA config)"},
    {"ch2Config", LEADSHINE_EC_MP_ANALOG_CFG(2), MODPARAM_TYPE_U32, "0", "Channel 2 configuration (AD/DA config)"},
    {"ch3Config", LEADSHINE_EC_MP_ANALOG_CFG(3), MODPARAM_TYPE_U32, "0", "Channel 3 configuration (AD/DA config)"},
    {NULL},
};

static const lcec_modparam_desc_t leadshine_ec_encoder_params[] = {
    {"encoderMode", LEADSHINE_EC_MP_ENC_MODE, MODPARAM_TYPE_U32, "0", "Encoder operation mode"},
    {"abPhase", LEADSHINE_EC_MP_ENC_ABPHASE, MODPARAM_TYPE_U32, "0", "AB phase configuration"},
    {"minValue", LEADSHINE_EC_MP_ENC_MINVALUE, MODPARAM_TYPE_S32, "-100000", "Minimum encoder value"},
    {"maxValue", LEADSHINE_EC_MP_ENC_MAXVALUE, MODPARAM_TYPE_S32, "100000", "Maximum encoder value"},
    {"countMode", LEADSHINE_EC_MP_ENC_COUNTMODE, MODPARAM_TYPE_U32, "0", "Counting mode"},
    {"encoderFilter", LEADSHINE_EC_MP_ENC_FILTER, MODPARAM_TYPE_U32, "2", "Encoder input filter"},
    {NULL},
};

/// @brief The one supported-module table (source of truth for `types[].modules`).
static const leadshine_ec_module_def_t leadshine_ec_module_table[] = {
    // ---- Digital input ----
    {0x61100025, "PM-1600", MODULE_DIN, 16, 0, leadshine_ec_digital_params},
    {0x61100026, "R3-1600-V20", MODULE_DIN, 16, 0, leadshine_ec_digital_params},
    {0x81100025, "R3-1600", MODULE_DIN, 16, 0, leadshine_ec_digital_params},
    {0x61100045, "PM-3200", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x61100046, "R3-3200-V20", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x61101045, "PM-3200-1", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x61101046, "R3-3200-1-V20", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x61102045, "PM-3200-2", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x61102046, "R3-3200-2-V20", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x81100045, "R3-3200", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x81101045, "R3-3200-1", MODULE_DIN, 32, 0, leadshine_ec_digital_params},
    {0x81102045, "R3-3200-2", MODULE_DIN, 32, 0, leadshine_ec_digital_params},

    // ---- Digital output (incl. relay) ----
    {0x61100205, "PM-0016-N", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x61100206, "R3-0016-N-V20", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x81100205, "R3-0016-N", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x61110205, "PM-0016-P", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x61110206, "R3-0016-P-V20", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x81110205, "R3-0016-P", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x61100405, "PM-0032-N", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61100406, "R3-0032-N-V20", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61101405, "PM-0032-N-1", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61101406, "R3-0032-N-1-V20", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61102405, "PM-0032-N-2", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61102406, "R3-0032-N-2-V20", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x81100405, "R3-0032-N", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x81101405, "R3-0032-N-1", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x81102405, "R3-0032-N-2", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61110406, "R3-0032-P-V20", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x81110405, "R3-0032-P", MODULE_DOUT, 0, 32, leadshine_ec_digital_params},
    {0x61900106, "R3-0008-R-V20", MODULE_DOUT, 0, 8, leadshine_ec_digital_params},
    {0x81900105, "R3-0008-R", MODULE_DOUT, 0, 8, leadshine_ec_digital_params},
    {0x61900205, "PM-0016-R", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x61900206, "R3-0016-R-V20", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},
    {0x81900205, "R3-0016-R", MODULE_DOUT, 0, 16, leadshine_ec_digital_params},

    // ---- Mixed digital input + output ----
    {0x61100116, "R3-0808-N-V20", MODULE_DIO, 8, 8, leadshine_ec_digital_params},
    {0x81100115, "R3-0808-N", MODULE_DIO, 8, 8, leadshine_ec_digital_params},
    {0x61100225, "PM-1616-N", MODULE_DIO, 16, 16, leadshine_ec_digital_params},
    {0x61100226, "R3-1616-N-V20", MODULE_DIO, 16, 16, leadshine_ec_digital_params},
    {0x81100225, "R3-1616-N", MODULE_DIO, 16, 16, leadshine_ec_digital_params},
    {0x61110226, "R3-1616-P-V20", MODULE_DIO, 16, 16, leadshine_ec_digital_params},
    {0x81110225, "R3-1616-P", MODULE_DIO, 16, 16, leadshine_ec_digital_params},
    {0x61101446, "R3-3232-N-1-V20", MODULE_DIO, 32, 32, leadshine_ec_digital_params},
    {0x81101445, "R3-3232-N-1", MODULE_DIO, 32, 32, leadshine_ec_digital_params},

    // ---- Analog input ----
    {0x61000025, "PM-A0400-IV", MODULE_AIN, 4, 0, leadshine_ec_analog_params},
    {0x61000026, "R3-A0400-IV-V20", MODULE_AIN, 4, 0, leadshine_ec_analog_params},
    {0x81000025, "R3-A0400-IV", MODULE_AIN, 4, 0, leadshine_ec_analog_params},

    // ---- Analog output ----
    {0x61000205, "PM-A0004-IV", MODULE_AOUT, 0, 4, leadshine_ec_analog_params},
    {0x61000206, "R3-A0004-IV-V20", MODULE_AOUT, 0, 4, leadshine_ec_analog_params},
    {0x81000205, "R3-A0004-IV", MODULE_AOUT, 0, 4, leadshine_ec_analog_params},

    // ---- Encoder ----
    {0x61300025, "PM-E0200-S", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},
    {0x61300026, "R3-E0200-S-V20", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},
    {0x61300125, "PM-E0200-D", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},
    {0x61300126, "R3-E0200-D-V20", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},
    {0x81300025, "R3-E0200-S", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},
    {0x81300125, "R3-E0200-D", MODULE_ENCODER, 2, 0, leadshine_ec_encoder_params},

    {0, NULL, MODULE_UNKNOWN, 0, 0, NULL},
};

#endif
