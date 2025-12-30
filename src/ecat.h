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
/// @brief Header file for LinuxCNC-Ethercat

#ifndef _ECAT_H_
#define _ECAT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ecrt.h"
#include "hal.h"
#include "ecat_xml.h"
#include "ecat_rtapi.h"
#include "rtapi_ctype.h"
#include "rtapi_math.h"
#include "rtapi_string.h"

#ifdef __cplusplus
}
#endif

// list macros
#define ECAT_LIST_APPEND(first, last, item) \
  do {                                      \
    (item)->prev = (last);                  \
    if ((item)->prev != NULL) {             \
      (item)->prev->next = (item);          \
    } else {                                \
      (first) = (item);                     \
    }                                       \
    (last) = (item);                        \
  } while (0);

#define ECAT_MSG_PFX "ECAT: "

// init macro; this will make GCC run calls to AddTypes() before
// main() is called.  This is used to register new slave types
// dynamically without needing a giant list in ecat_main.c.
#define ADD_TYPES(types)                                          \
  static void AddTypes##types(void) __attribute__((constructor)); \
  static void AddTypes##types(void) { ecat_addtypes(types, __FILE__); }

// vendor ids, please keep sorted.
#define ECAT_BECKHOFF_VID   0x00000002
#define ECAT_OMRON_VID      0x00000083
#define ECAT_STOEBER_VID    0x000000b9
#define ECAT_DS5_VID        ECAT_STOEBER_VID  // Alias for DS5 series (Stoeber)
#define ECAT_SMC_VID        0x00000114
#define ECAT_DELTA_VID      0x000001dd
#define ECAT_INOVANCE_VID   0x00100000  // 汇川 Inovance
#define ECAT_XINJE_VID      0x00000e52  // 信捷 Xinje
#define ECAT_ABET_VID       0x0000079A
#define ECAT_MODUSOFT_VID   0x00000907
#define ECAT_RTELLIGENT_VID 0x00000a88

// State update period (ns)
#define ECAT_STATE_UPDATE_PERIOD 1000000000LL

// IDN builder
#define ECAT_IDN_TYPE_P 0x8000
#define ECAT_IDN_TYPE_S 0x0000

#define ECAT_IDN(type, set, block) (type | ((set & 0x07) << 12) | (block & 0x0fff))

#define ECAT_FSOE_CMD_LEN    1
#define ECAT_FSOE_CRC_LEN    2
#define ECAT_FSOE_CONNID_LEN 2

#define ECAT_FSOE_SIZE(ch_count, data_len) (ECAT_FSOE_CMD_LEN + ch_count * (data_len + ECAT_FSOE_CRC_LEN) + ECAT_FSOE_CONNID_LEN)

#define ECAT_MAX_PDO_REG_COUNT   256  ///< The maximum number of calls to ecat_pdo_init() for a single driver.
#define ECAT_MAX_PDO_ENTRY_COUNT 128  ///< The maximum number of PDO entries in a PDO in a sync.
#define ECAT_MAX_PDO_INFO_COUNT  16   ///< The maximum number of PDOs in a sync.
#define ECAT_MAX_SYNC_COUNT      4    ///< The maximum number of syncs.

// Memory allocation macros.  These differ from malloc in a couple
// substantial ways.  First, they check for NULL and call exit(1), so
// there's no point in comparing their return value to NULL.  If
// memory allocation fails, then we're broken and there's no point in
// going on.  Next, they always 0 the allocated memory, so there's no
// point in calling memset() on the result.  Finally, they take a
// *type*, not a size_t, and cast the result into a pointer-to-type,
// so compiling this as C++ doesn't cause untold numbers of errors.

/// Allocate memory for an `expr`.  This zeros out the allocated memory automatically, and exits if malloc fails.
#define ECAT_HAL_ALLOCATE(expr) ((__typeof__(expr) *)ecat_hal_malloc(sizeof(expr), __FILE__, __func__, __LINE__))

/// Allocate memory for a string of `len` bytes.  This zeros out the allocated memory automatically, and exits if malloc fails.
#define ECAT_HAL_ALLOCATE_STRING(len) ((char *)ecat_hal_malloc(len, __FILE__, __func__, __LINE__))

/// Allocate memory for an array of `count` `expr`s.  This zeros out the allocated memory automatically, and exits if malloc fails.
#define ECAT_HAL_ALLOCATE_ARRAY(expr, count) ((__typeof__(expr) *)ecat_hal_malloc(sizeof(expr) * count, __FILE__, __func__, __LINE__))

/// Allocate memory for an `expr`.  This zeros out the allocated memory automatically, and exits if malloc fails.
#define ECAT_ALLOCATE(expr) ((__typeof__(expr) *)ecat_malloc(sizeof(expr), __FILE__, __func__, __LINE__))

/// Allocate memory for a string of `len` bytes.  This zeros out the allocated memory automatically, and exits if malloc fails.
#define ECAT_ALLOCATE_STRING(len) ((char *)ecat_malloc(len, __FILE__, __func__, __LINE__))

/// Allocate memory for an array of `count` `expr`s.  This zeros out the allocated memory automatically, and exits if malloc fails.
#define ECAT_ALLOCATE_ARRAY(expr, count) ((__typeof__(expr) *)ecat_malloc(sizeof(expr) * count, __FILE__, __func__, __LINE__))

typedef struct ecat_master ecat_master_t;
typedef struct ecat_slave ecat_slave_t;

typedef int (*ecat_slave_preinit_t)(ecat_slave_t *slave);
typedef int (*ecat_slave_init_t)(int comp_id, ecat_slave_t *slave);
typedef void (*ecat_slave_cleanup_t)(ecat_slave_t *slave);
typedef void (*ecat_slave_rw_t)(ecat_slave_t *slave, long period);

typedef enum {
  MODPARAM_TYPE_BIT,    ///< Modparam value is a single bit.
  MODPARAM_TYPE_U32,    ///< Modparam value is an unsigned 32-bit int.
  MODPARAM_TYPE_S32,    ///< Modparam value is a signed 32-bit int.
  MODPARAM_TYPE_FLOAT,  ///< Modparam value is a float.
  MODPARAM_TYPE_STRING  ///< Modparam value is a string.
} ecat_modparam_type_t;

typedef struct {
  const char *name;            ///< the name that appears in the XML.
  int id;                      ///< Numeric ID, should be unique per device driver.
  ecat_modparam_type_t type;   ///< The type (bit, int, float, string) of this modParam.
  const char *config_value;    ///< The default value (as a string), for use in ecat_configgen.
  const char *config_comment;  ///< A comment to added to the output in ecat_configgen.
} ecat_modparam_desc_t;

typedef struct {
  const char *name;            ///< the name that appears in the XML.
  const char *config_value;    ///< The default value (as a string), for use in ecat_configgen.
  const char *config_comment;  ///< A comment to added to the output in ecat_configgen.
} ecat_modparam_doc_t;

/// @brief Definition of a device that LinuxCNC-Ethercat supports.
typedef struct {
  const char *name;                       ///< The device's name ("EL1008")
  uint32_t vid;                           ///< The EtherCAT vendor ID
  uint32_t pid;                           ///< The EtherCAT product ID
  int is_fsoe_logic;                      ///< Does this device use Safety-over-EtherCAT?
  ecat_slave_preinit_t proc_preinit;      ///< pre-init function, if any
  ecat_slave_init_t proc_init;            ///< init function.  Sets up the device.
  const ecat_modparam_desc_t *modparams;  ///< XML modparams, if any
  uint64_t flags;                         ///< Flags, passed through to `proc_init` as `slave->flags`.
  const char *sourcefile;                 ///< Source filename, autopopulated.
} ecat_typelist_t;

/// @brief Linked list for holding device type definitions.
typedef struct ecat_typelinkedlist {
  const ecat_typelist_t *type;       ///< The type definition.
  struct ecat_typelinkedlist *next;  ///< Pointer to the next `ecat_typelinkedlist` in the linked list.
} ecat_typelinkedlist_t;

typedef struct {
  int slave_data_len;   ///< Length of slave data.
  int master_data_len;  ///< Length of master data.
  int data_channels;    ///< Number of data channels.
} ECAT_CONF_FSOE_T;

typedef struct ecat_master_data {
  hal_u32_t *slaves_responding;
  hal_bit_t *state_init;
  hal_bit_t *state_preop;
  hal_bit_t *state_safeop;
  hal_bit_t *state_op;
  hal_bit_t *link_up;
  hal_bit_t *all_op;
#ifdef RTAPI_TASK_PLL_SUPPORT
  hal_s32_t *pll_err;
  hal_s32_t *pll_out;
  hal_u32_t pll_step;
  hal_u32_t pll_max_err;
  hal_u32_t *pll_reset_cnt;
  hal_s32_t *app_phase;            // Our execution phase in local cycle (ns, real-time)
  hal_bit_t *pll_locked;           // PLL lock status indicator
  hal_s32_t *phase_jitter_out;     // Output: measured app_phase jitter amplitude (ns)
  hal_s32_t *drift_mode;            // Input: 0=simple, 1=manual
  hal_s32_t *pll_drift;            // Input: debug offset added to PLL correction (ns)
  hal_s32_t *pll_final;            // Output: final PLL correction value sent to rtapi (ns)
#endif
  int32_t auto_drift_delay;        // Internal: delay counter before applying auto drift
  // Phase calibration for sync_to_ref_clock=false mode
  int32_t phase_measure_cnt;       // Internal: measurement cycle counter
  int32_t phase_min;               // Internal: minimum app_phase during measurement
  int32_t phase_max;               // Internal: maximum app_phase during measurement
  int32_t phase_last;              // Internal: last app_phase value (for boundary detection)
  int32_t phase_jitter;            // Internal: calculated jitter amplitude
  int32_t phase_target;            // Internal: target app_phase position
  int32_t phase_calibrated;        // Internal: 0=measuring, 1=calibrated
} ecat_master_data_t;

typedef struct ecat_slave_state {
  hal_bit_t *online;        ///< Is device online?  Equivalent to the `.slave-online` HAL pin.
  hal_bit_t *operational;   ///< Is device operational?  Equivalent to the `.slave-oper` HAL pin.
  hal_bit_t *state_init;    ///< Is the device in state `INIT`?  Equivalant to the `.slave-state-init` HAL pin.
  hal_bit_t *state_preop;   ///< Is the device in state `PREOP`?  Equivalant to the `.slave-state-preop` HAL pin.
  hal_bit_t *state_safeop;  ///< Is the device in state `SAFEOP`?  Equivalant to the `.slave-state-safeop` HAL pin.
  hal_bit_t *state_op;      ///< Is the device in state `OP`?  Equivalant to the `.slave-state-op` HAL pin.
} ecat_slave_state_t;

typedef struct ecat_master {
  ecat_master_t *prev;              ///< Next master.
  ecat_master_t *next;              ///< Previous master.
  int index;                        ///< Index of this mater.
  char name[ECAT_CONF_STR_MAXLEN];  ///< Name of master.
  ec_master_t *master;              ///< EtherCAT master structure.
  unsigned long mutex;              ///< Mutex for locking operations.
  ec_pdo_entry_reg_t *pdo_entry_regs;
  ec_domain_t *domain;
  uint8_t *process_data;
  int process_data_len;
  ecat_slave_t *first_slave;
  ecat_slave_t *last_slave;
  ecat_master_data_t *hal_data;
  uint64_t app_time_base;
  uint32_t app_time_period;
  long period_last;
  int sync_ref_cnt;
  int sync_ref_cycles;
  int sync_to_ref_clock;
  long long state_update_timer;
  ec_master_state_t ms;
  int activated;                    // Flag: master has been activated (0=not yet, 1=activated)
#ifdef RTAPI_TASK_PLL_SUPPORT
  uint64_t dc_ref;
  uint64_t dc_ref_time;          // DC reference time (epoch) - set on first app_time call
  int32_t sync0_shift;            // sync0Shift from first DC slave (for auto-drift calculation)
  uint32_t app_time_last;
  int dc_time_valid_last;         // Previous cycle's dc_time_valid (for detecting consecutive valid reads)
#endif
} ecat_master_t;

typedef struct ecat_pdo_entry_reg {
  int current;
  int max;
  ec_pdo_entry_reg_t *pdo_entry_regs;
} ecat_pdo_entry_reg_t;

/// @brief Slave Distributed Clock configuration.
typedef struct {
  uint16_t assignActivate;
  uint32_t sync0Cycle;
  int32_t sync0Shift;
  uint32_t sync1Cycle;
  int32_t sync1Shift;
} ecat_slave_dc_t;

/// @brief Slave Watchdog configuration.
typedef struct {
  uint16_t divider;
  uint16_t intervals;
} ecat_slave_watchdog_t;

/// @brief Slave SDO configuration.
typedef struct {
  uint16_t index;
  int16_t subindex;
  size_t length;
  uint8_t data[];
} ecat_slave_sdoconf_t;

/// @brief Slave IDN configuration.
typedef struct {
  uint8_t drive;
  uint16_t idn;
  ec_al_state_t state;
  size_t length;
  uint8_t data[];
} ecat_slave_idnconf_t;

/// @brief ModParam definition.
typedef struct {
  int id;                          /// The integer ID from the modparam definition.  Use this as the key for comparison.
  const char *name;                /// The actual name used in the XML file.  Only use for error messages.
  ECAT_CONF_MODPARAM_VAL_T value;  /// The value set in `<modparam name="..." value="..."/>`
} ecat_slave_modparam_t;

/// @brief EtherCAT slave.
typedef struct ecat_slave {
  ecat_slave_t *prev;                        ///< Next slave
  ecat_slave_t *next;                        ///< Previous slave
  ecat_master_t *master;                     ///< Master for this slave
  int index;                                 ///< Index of this slave.
  char name[ECAT_CONF_STR_MAXLEN];           ///< Slave name.
  uint32_t vid;                              ///< Slave's vendor ID
  uint32_t pid;                              ///< Slave's EtherCAT PID/device ID.
  ec_sync_info_t *sync_info;                 ///< Sync Manager configuration.
  ec_slave_config_t *config;                 ///< Configuration data.
  ec_slave_config_state_t state;             ///< Slave state.
  ecat_slave_dc_t *dc_conf;                  ///< Distributed Clock configuration.
  ecat_slave_watchdog_t *wd_conf;            ///< Watchdog configuration.
  ecat_slave_preinit_t proc_preinit;         ///< Callback for pre-init, if any.
  ecat_slave_init_t proc_init;               ///< Callback for initializing device.
  ecat_slave_cleanup_t proc_cleanup;         ///< Calback for cleaning up the device.
  ecat_slave_rw_t proc_read;                 ///< Callback for reading from the device.
  ecat_slave_rw_t proc_write;                ///< Callback for writing to the device.
  ecat_slave_state_t *hal_state_data;        ///< HAL state data.
  void *hal_data;                            ///< HAL data, device driver specific.
  int generic_pdo_entry_count;               ///< The number of generic PDO entries.
  ec_pdo_entry_info_t *generic_pdo_entries;  ///< Generic PDO entries.
  ec_pdo_info_t *generic_pdos;               ///< Generic PDOs.
  ec_sync_info_t *generic_sync_managers;     ///< Generic sync managers.
  ecat_slave_sdoconf_t *sdo_config;          ///< SDO config.
  ecat_slave_idnconf_t *idn_config;          ///< IDN config.
  ecat_slave_modparam_t *modparams;          ///< modParams.
  const ECAT_CONF_FSOE_T *fsoeConf;          ///< Safety config.
  int is_fsoe_logic;                         ///< Device supports FSoE safety logic.
  unsigned int *fsoe_slave_offset;           ///< FSoE slave offset.
  unsigned int *fsoe_master_offset;          ///< FSoE master offset.
  uint64_t flags;                            ///< Flags, as defined by the driver itself.
  ecat_pdo_entry_reg_t *regs;
} ecat_slave_t;

/// @brief HAL pin description.
typedef struct {
  hal_type_t type;    ///< HAL type of this pin (`HAL_BIT`, `HAL_FLOAT`, `HAL_S32`, or `HAL_U32`).
  hal_pin_dir_t dir;  ///< Direction for this pin (`HAL_IN`, `HAL_OUT`, or `HAL_IO`).
  int offset;         ///< Offset for this pin's data in `hal_data`.
  const char *fmt;    ///< Format string for generating pin names via sprintf().
} ecat_pindesc_t;

/// @brief HAL pin description.
typedef struct {
  hal_type_t type;      ///< HAL type of this pin (`HAL_BIT`, `HAL_FLOAT`, `HAL_S32`, or `HAL_U32`).
  hal_param_dir_t dir;  ///< Direction for this pin (`HAL_IN`, `HAL_OUT`, or `HAL_IO`).
  int offset;           ///< Offset for this pin's data in `hal_data`.
  const char *fmt;      ///< Format string for generating pin names via sprintf().
} ecat_paramdesc_t;

/// @brief Sync manager configuration.
typedef struct {
  ecat_slave_t *slave;                            ///< For debugging messages
  int sync_count;                                 ///< Number of syncs.
  ec_sync_info_t *curr_sync;                      ///< Current sync.
  ec_sync_info_t syncs[ECAT_MAX_SYNC_COUNT + 1];  ///< Sync definitions.

  int pdo_info_count;                                    ///< Number of PDO infos.
  ec_pdo_info_t *curr_pdo_info;                          ///< Current PDO info.
  ec_pdo_info_t pdo_infos[ECAT_MAX_PDO_INFO_COUNT + 1];  ///< PDO info definitions.

  int pdo_entry_count;                                            ///< Total number of PDO entries for slave.
  ec_pdo_entry_info_t *curr_pdo_entry;                            ///< Current PDO entry.
  ec_pdo_entry_info_t pdo_entries[ECAT_MAX_PDO_ENTRY_COUNT + 1];  ///< PDO entry definitions.

  int autoflow;         ///< If true, then adding new PDO entries will automatically start a new PDO at `pdo_entry_limit`
  int pdo_entry_limit;  ///< Device limit on the number of PDO entries per PDO
  int pdo_limit;        ///< Device limit on the number of PDOs per sync.
  int pdo_increment;    ///< Number to increment PDO number when overflowing.
} ecat_syncs_t;

/// @brief Lookup table mapping string to int
typedef struct {
  const char *key;
  const int value;
} ecat_lookuptable_int_t;

/// @brief Lookup table mapping string to double
typedef struct {
  const char *key;
  const double value;
} ecat_lookuptable_double_t;

ecat_slave_t *ecat_slave_by_index(ecat_master_t *master, int index) __attribute__((nonnull));

int ecat_read_sdo(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint8_t *target, size_t size);
int ecat_read_sdo8(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint8_t *result);
int ecat_read_sdo8_pin_U32(ecat_slave_t *slave, uint16_t index, uint8_t subindex, volatile uint32_t *result);
int ecat_read_sdo8_pin_S32(ecat_slave_t *slave, uint16_t index, uint8_t subindex, volatile int32_t *result);
int ecat_read_sdo16(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint16_t *result);
int ecat_read_sdo16_pin_U32(ecat_slave_t *slave, uint16_t index, uint8_t subindex, volatile uint32_t *result);
int ecat_read_sdo16_pin_S32(ecat_slave_t *slave, uint16_t index, uint8_t subindex, volatile int32_t *result);
int ecat_read_sdo32(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint32_t *result);
int ecat_read_sdo32_pin_U32(ecat_slave_t *slave, uint16_t index, uint8_t subindex, volatile uint32_t *result);
int ecat_read_sdo32_pin_S32(ecat_slave_t *slave, uint16_t index, uint8_t subindex, volatile int32_t *result);
int ecat_read_idn(ecat_slave_t *slave, uint8_t drive_no, uint16_t idn, uint8_t *target, size_t size);
int ecat_write_sdo(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint8_t *value, size_t size);
int ecat_write_sdo8(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint8_t value);
int ecat_write_sdo16(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint16_t value);
int ecat_write_sdo32(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint32_t value);
int ecat_write_sdo8_modparam(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint8_t value, const char *mpname);
int ecat_write_sdo16_modparam(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint16_t value, const char *mpname);
int ecat_write_sdo32_modparam(ecat_slave_t *slave, uint16_t index, uint8_t subindex, uint32_t value, const char *mpname);

int ecat_pin_newf(hal_type_t type, hal_pin_dir_t dir, void **data_ptr_addr, const char *fmt, ...);
int ecat_pin_newf_list(void *base, const ecat_pindesc_t *list, ...);
int ecat_param_newf(hal_type_t type, hal_param_dir_t dir, void *data_addr, const char *fmt, ...);
int ecat_param_newf_list(void *base, const ecat_paramdesc_t *list, ...);

void copy_fsoe_data(ecat_slave_t *slave, unsigned int slave_offset, unsigned int master_offset) __attribute__((nonnull));
void ecat_syncs_init(ecat_slave_t *slave, ecat_syncs_t *syncs) __attribute__((nonnull));
void ecat_syncs_enable_autoflow(ecat_slave_t *slave, ecat_syncs_t *syncs, int pdo_limit, int pdo_entry_limit, int pdo_increment);
void ecat_syncs_add_sync(ecat_syncs_t *syncs, ec_direction_t dir, ec_watchdog_mode_t watchdog_mode);
void ecat_syncs_add_pdo_info(ecat_syncs_t *syncs, uint16_t index);
void ecat_syncs_add_pdo_entry(ecat_syncs_t *syncs, uint16_t index, uint8_t subindex, uint8_t bit_length);

const ecat_typelist_t *ecat_findslavetype(const char *name) __attribute__((nonnull));
void ecat_addtype(ecat_typelist_t *type, const char *sourcefile) __attribute__((nonnull));
void ecat_addtypes(ecat_typelist_t types[], const char *sourcefile) __attribute__((nonnull));

int ecat_lookupint(const ecat_lookuptable_int_t *table, const char *key, int default_value) __attribute__((nonnull));
int ecat_lookupint_i(const ecat_lookuptable_int_t *table, const char *key, int default_value) __attribute__((nonnull));
double ecat_lookupdouble(const ecat_lookuptable_double_t *table, const char *key, double default_value) __attribute__((nonnull));
double ecat_lookupdouble_i(const ecat_lookuptable_double_t *table, const char *key, double default_value) __attribute__((nonnull));

ECAT_CONF_MODPARAM_VAL_T *ecat_modparam_get(ecat_slave_t *slave, int id) __attribute__((nonnull));
int ecat_modparam_desc_len(const ecat_modparam_desc_t *mp);
ecat_modparam_desc_t *ecat_modparam_desc_concat(ecat_modparam_desc_t const *a, ecat_modparam_desc_t const *b);
ecat_modparam_desc_t *ecat_modparam_desc_merge_docs(ecat_modparam_desc_t const *a, ecat_modparam_doc_t const *b);

ecat_pdo_entry_reg_t *ecat_allocate_pdo_entry_reg(int size);
int ecat_pdo_init(ecat_slave_t *slave, uint16_t idx, uint16_t sidx, unsigned int *os, unsigned int *bp);
int ecat_pdo_entry_reg_len(ecat_pdo_entry_reg_t *reg);
int ecat_append_pdo_entry_reg(ecat_pdo_entry_reg_t *dest, ecat_pdo_entry_reg_t *src);

void *ecat_hal_malloc(size_t size, const char *file, const char *func, int line);
void *ecat_malloc(size_t size, const char *file, const char *func, int line);

// Compatibility macro for old API - maps to new ecat_pdo_init function
// Old: ECAT_PDO_INIT(pdo, pos, vid, pid, idx, sidx, off, bpos)
// New: ecat_pdo_init(slave, idx, sidx, off, bpos)
// Note: pdo, vid, pid parameters are ignored since they're already in slave struct
// The 'pos' parameter is used as the slave pointer
#define ECAT_PDO_INIT(pdo, slavevar, vid, pid, idx, sidx, off, bpos) \
    ecat_pdo_init((slavevar), idx, sidx, off, bpos)

// Compatibility: allow ecat_pindesc_t to be used as ecat_paramdesc_t
// These types are structurally identical (both have type, dir, offset, fmt)
// The only difference is dir field type (hal_pin_dir_t vs hal_param_dir_t)
// which are both int-sized enums with compatible values
#define ecat_param_newf_list_compat(base, list, ...) \
    ecat_param_newf_list(base, (const ecat_paramdesc_t *)(list), ##__VA_ARGS__)

#endif
