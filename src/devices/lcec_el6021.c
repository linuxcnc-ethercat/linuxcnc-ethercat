//
//    Copyright (C) 2026 The LinuxCNC EtherCAT authors
//
//    EL60xx serial protocol logic ported from the IgH EtherCAT Master
//    tty example (examples/tty/serial.c), Copyright (C) 2006-2008
//    Florian Pose, Ingenieurgemeinschaft IgH.
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

/// @file  lcec_el6021.c
/// @brief Driver for Beckhoff EL6001/EL6021 serial interface terminals
///
/// Exposes the terminal's serial port as a userspace character device
/// (/dev/<ttyName>) via CUSE, so existing serial software (mb2hal,
/// pymodbus, minicom, ...) can talk to the attached device without any
/// EtherCAT knowledge.  Baud rate, data frame and RTS/CTS settings
/// requested through termios ioctls are applied to the terminal via
/// SDO writes at runtime.
///
/// Throughput is limited to 22 bytes per process data window, so one
/// window per cycle (or less) is available, depending on the servo
/// period.

#include "lcec_el6021.h"
#include "lcec_el6021_cfg.h"

#include <ecrt.h>
#include <errno.h>
#include <pthread.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/eventfd.h>
#include <termios.h>
#include <unistd.h>

#include "../lcec.h"

#ifdef LCEC_HAVE_CUSE
#define FUSE_USE_VERSION 31

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/uio.h>

#include <cuse_lowlevel.h>
#include <fuse_opt.h>
#endif

// termios2 ioctl numbers (asm-generic), to avoid <asm/termbits.h>
// conflicts with <termios.h>
#define LCEC_TCGETS2 0x802c542a
#define LCEC_TCSETS2 0x402c542b
#define LCEC_TCSETSW2 0x402c542c
#define LCEC_TCSETSF2 0x402c542d
#define LCEC_TCFLSH 0x540b
#define LCEC_TCSBRK   0x5409  // tcdrain() is TCSBRK with a non-zero arg
#define LCEC_TCSBRKP  0x5425

typedef enum {
  LCEC_EL6021_STATE_REQUEST_INIT,
  LCEC_EL6021_STATE_WAIT_INIT_RESPONSE,
  LCEC_EL6021_STATE_READY,
  LCEC_EL6021_STATE_SET_RTSCTS,
  LCEC_EL6021_STATE_SET_BAUD,
  LCEC_EL6021_STATE_SET_FRAME,
} lcec_el6021_state_t;

/// The terminal has RTS/CTS (0x8000:01); only the RS232 EL6001 does
#define LCEC_EL6021_FLAG_RTSCTS 1

typedef struct {
  // HAL pins
  hal_bit_t *reset;
  hal_bit_t *cfg_error;
  hal_bit_t *tty_open;
  hal_u32_t *ser_state;
  hal_u32_t *rx_bytes;
  hal_u32_t *tx_bytes;
  hal_u32_t *rx_dropped;
  hal_u32_t *baud;

  // PDO offsets
  unsigned int ctrl_os;
  unsigned int ctrl_bp;  // bit 0 of a byte: the word starts there
  unsigned int status_bp;
  unsigned int tx_os;
  unsigned int status_os;
  unsigned int rx_os;

  // protocol state machine
  int state;
  uint8_t tx_req_tgl;
  uint8_t tx_ack_tgl;
  uint8_t rx_req_tgl;
  uint8_t rx_ack_tgl;
  uint16_t control;
  uint16_t last_status;
  uint8_t tx_buf[LCEC_EL6021_DATA_MAX];
  uint8_t tx_len;
  uint32_t rx_byte_count;
  uint32_t tx_byte_count;
  uint32_t rx_drop_count;

  // serial port configuration (SDO indices, not raw values).
  // req_* is the requested configuration (written by modparams and the
  // CUSE thread), pend_* is the snapshot the RT state machine is
  // applying, cur_* is what the terminal currently runs with.
  uint8_t has_rtscts;
  uint8_t req_rtscts;
  uint8_t cur_rtscts;
  uint8_t req_baud;
  uint8_t cur_baud;
  uint8_t req_frame;
  uint8_t cur_frame;
  uint8_t pend_rtscts;
  uint8_t pend_baud;
  uint8_t pend_frame;
  int cfg_error_state;
  volatile int cfg_changed;

  // runtime SDO requests
  ec_sdo_request_t *sdo_rtscts;
  ec_sdo_request_t *sdo_baud;
  ec_sdo_request_t *sdo_frame;

  // SPSC ring buffers between the RT thread and the CUSE thread.
  // rx: RT produces, CUSE consumes.  tx: CUSE produces, RT consumes.
  uint8_t rx_ring[LCEC_EL6021_RING_SIZE];
  volatile uint32_t rx_w;
  volatile uint32_t rx_r;
  uint8_t tx_ring[LCEC_EL6021_RING_SIZE];
  volatile uint32_t tx_w;
  volatile uint32_t tx_r;

  // CUSE character device
  char tty_name[LCEC_CONF_STR_MAXLEN];
  int tty_enabled;
  int efd;
  int stop_efd;  // wakes the CUSE thread's blocking read on shutdown
  volatile int thread_stop;
  pthread_t cuse_thread;
  pthread_t notify_thread;
  void *cuse_se;
  void *cuse_ph;
  pthread_mutex_t lock;  // protects cuse_ph and req_* writes from the CUSE thread
  lcec_el6021_ktermios_t tio;
  volatile int open_count;
} lcec_el6021_data_t;

static const lcec_pindesc_t slave_pins[] = {
    {HAL_BIT, HAL_IN, offsetof(lcec_el6021_data_t, reset), "%s.%s.%s.ser-reset"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el6021_data_t, cfg_error), "%s.%s.%s.ser-cfg-error"},
    {HAL_BIT, HAL_OUT, offsetof(lcec_el6021_data_t, tty_open), "%s.%s.%s.ser-tty-open"},
    {HAL_U32, HAL_OUT, offsetof(lcec_el6021_data_t, ser_state), "%s.%s.%s.ser-state"},
    {HAL_U32, HAL_OUT, offsetof(lcec_el6021_data_t, rx_bytes), "%s.%s.%s.ser-rx-bytes"},
    {HAL_U32, HAL_OUT, offsetof(lcec_el6021_data_t, tx_bytes), "%s.%s.%s.ser-tx-bytes"},
    {HAL_U32, HAL_OUT, offsetof(lcec_el6021_data_t, rx_dropped), "%s.%s.%s.ser-rx-dropped"},
    {HAL_U32, HAL_OUT, offsetof(lcec_el6021_data_t, baud), "%s.%s.%s.ser-baud"},
    {HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL},
};

// The terminals' fixed 22-byte COM maps.  The control/status bits and the
// length byte form one little-endian 16-bit word at the first entry, which is
// how the handshake below reads and writes them.
static ec_pdo_entry_info_t lcec_el6021_pdo_entries_out[] = {
    {0x7000, 0x01, 1},  // Transmit request
    {0x7000, 0x02, 1},  // Receive accepted
    {0x7000, 0x03, 1},  // Init request
    {0x7000, 0x04, 1},  // Send continuous
    {0x0000, 0x00, 4},  // gap
    {0x7000, 0x09, 8},  // Output length
    {0x7000, 0x11, 8},  // Data Out 0
    {0x7000, 0x12, 8},  // Data Out 1
    {0x7000, 0x13, 8},  // Data Out 2
    {0x7000, 0x14, 8},  // Data Out 3
    {0x7000, 0x15, 8},  // Data Out 4
    {0x7000, 0x16, 8},  // Data Out 5
    {0x7000, 0x17, 8},  // Data Out 6
    {0x7000, 0x18, 8},  // Data Out 7
    {0x7000, 0x19, 8},  // Data Out 8
    {0x7000, 0x1a, 8},  // Data Out 9
    {0x7000, 0x1b, 8},  // Data Out 10
    {0x7000, 0x1c, 8},  // Data Out 11
    {0x7000, 0x1d, 8},  // Data Out 12
    {0x7000, 0x1e, 8},  // Data Out 13
    {0x7000, 0x1f, 8},  // Data Out 14
    {0x7000, 0x20, 8},  // Data Out 15
    {0x7000, 0x21, 8},  // Data Out 16
    {0x7000, 0x22, 8},  // Data Out 17
    {0x7000, 0x23, 8},  // Data Out 18
    {0x7000, 0x24, 8},  // Data Out 19
    {0x7000, 0x25, 8},  // Data Out 20
    {0x7000, 0x26, 8},  // Data Out 21
};

static ec_pdo_entry_info_t lcec_el6021_pdo_entries_in[] = {
    {0x6000, 0x01, 1},  // Transmit accepted
    {0x6000, 0x02, 1},  // Receive request
    {0x6000, 0x03, 1},  // Init accepted
    {0x6000, 0x04, 1},  // Buffer full
    {0x6000, 0x05, 1},  // Parity error
    {0x6000, 0x06, 1},  // Framing error
    {0x6000, 0x07, 1},  // Overrun error
    {0x0000, 0x00, 1},  // gap
    {0x6000, 0x09, 8},  // Input length
    {0x6000, 0x11, 8},  // Data In 0
    {0x6000, 0x12, 8},  // Data In 1
    {0x6000, 0x13, 8},  // Data In 2
    {0x6000, 0x14, 8},  // Data In 3
    {0x6000, 0x15, 8},  // Data In 4
    {0x6000, 0x16, 8},  // Data In 5
    {0x6000, 0x17, 8},  // Data In 6
    {0x6000, 0x18, 8},  // Data In 7
    {0x6000, 0x19, 8},  // Data In 8
    {0x6000, 0x1a, 8},  // Data In 9
    {0x6000, 0x1b, 8},  // Data In 10
    {0x6000, 0x1c, 8},  // Data In 11
    {0x6000, 0x1d, 8},  // Data In 12
    {0x6000, 0x1e, 8},  // Data In 13
    {0x6000, 0x1f, 8},  // Data In 14
    {0x6000, 0x20, 8},  // Data In 15
    {0x6000, 0x21, 8},  // Data In 16
    {0x6000, 0x22, 8},  // Data In 17
    {0x6000, 0x23, 8},  // Data In 18
    {0x6000, 0x24, 8},  // Data In 19
    {0x6000, 0x25, 8},  // Data In 20
    {0x6000, 0x26, 8},  // Data In 21
};

static ec_pdo_info_t lcec_el6021_pdos[] = {
    {0x1604, 28, lcec_el6021_pdo_entries_out},  // COM RxPDO-Map Outputs
    {0x1a04, 31, lcec_el6021_pdo_entries_in},   // COM TxPDO-Map Inputs
};

static ec_sync_info_t lcec_el6021_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, lcec_el6021_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, lcec_el6021_pdos + 1, EC_WD_DISABLE},
    {0xff},
};

static int lcec_el6021_init(int comp_id, lcec_slave_t *slave);
static int lcec_el6021_apply_config(lcec_slave_t *slave);
static void lcec_el6021_cleanup(lcec_slave_t *slave);
static void lcec_el6021_read(lcec_slave_t *slave, long period);
static void lcec_el6021_write(lcec_slave_t *slave, long period);
#ifdef LCEC_HAVE_CUSE
static int lcec_el6021_cuse_start(lcec_slave_t *slave);
static void lcec_el6021_cuse_stop(lcec_slave_t *slave);
#endif

static lcec_modparam_desc_t lcec_el6021_modparams[] = {
    {"baud", LCEC_EL6021_PARAM_BAUD, MODPARAM_TYPE_U32, "9600",
        "Startup baud rate (overridden by the app via termios on open). One of 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200."},
    {"dataFrame", LCEC_EL6021_PARAM_FRAME, MODPARAM_TYPE_STRING, "8N1",
        "Startup data frame (overridden by the app via termios on open), e.g. 8N1, 8E1, 7E1."},
    {"rtsCts", LCEC_EL6021_PARAM_RTSCTS, MODPARAM_TYPE_BIT, "0", "Enable RTS/CTS hardware handshake."},
    {"ttyName", LCEC_EL6021_PARAM_TTYNAME, MODPARAM_TYPE_STRING, NULL,
        "Name of the /dev/ character device node. Default: lcec-<master>-<slave>."},
    {NULL},
};

static lcec_typelist_t types[] = {
    // clang-format off
    {"EL6001", LCEC_BECKHOFF_VID, 0x17713052, 0, NULL, lcec_el6021_init, lcec_el6021_modparams, LCEC_EL6021_FLAG_RTSCTS,
        NULL, NULL, lcec_el6021_apply_config},
    {"EL6021", LCEC_BECKHOFF_VID, 0x17853052, 0, NULL, lcec_el6021_init, lcec_el6021_modparams, 0, NULL, NULL,
        lcec_el6021_apply_config},
    // clang-format on
    {NULL},
};
ADD_TYPES(types);

// ring buffer helpers (single producer, single consumer, power-of-2 size)

static uint32_t ring_free(volatile uint32_t *w, volatile uint32_t *r) {
  return LCEC_EL6021_RING_SIZE - (*w - *r);
}

static uint32_t ring_avail(volatile uint32_t *w, volatile uint32_t *r) {
  return *w - *r;
}

static void ring_write(uint8_t *ring, volatile uint32_t *w, const uint8_t *data, uint32_t len) {
  uint32_t pos = *w & (LCEC_EL6021_RING_SIZE - 1);
  uint32_t first = LCEC_EL6021_RING_SIZE - pos;
  if (first > len) {
    first = len;
  }
  memcpy(ring + pos, data, first);
  memcpy(ring, data + first, len - first);
  __sync_synchronize();
  *w += len;
}

static void ring_read(uint8_t *ring, volatile uint32_t *r, uint8_t *data, uint32_t len) {
  uint32_t pos = *r & (LCEC_EL6021_RING_SIZE - 1);
  uint32_t first = LCEC_EL6021_RING_SIZE - pos;
  if (first > len) {
    first = len;
  }
  memcpy(data, ring + pos, first);
  memcpy(data + first, ring, len - first);
  __sync_synchronize();
  *r += len;
}

// build the kernel termios reported back to applications from the
// current requested configuration
static void config_to_ktermios(lcec_el6021_data_t *hal_data, lcec_el6021_ktermios_t *tio) {
  const lcec_el6021_baud_t *b = lcec_el6021_baud_by_idx(hal_data->req_baud);
  const lcec_el6021_frame_t *f = lcec_el6021_frame_by_idx(hal_data->req_frame);

  memset(tio, 0, sizeof(*tio));
  tio->c_cflag = b->speed | CREAD | CLOCAL;
  tio->c_ispeed = b->baud;
  tio->c_ospeed = b->baud;
  if (f != NULL) {
    tio->c_cflag |= (f->data_bits == 7) ? CS7 : CS8;
    if (f->parity != 'N') {
      tio->c_cflag |= PARENB;
      if (f->parity == 'O') {
        tio->c_cflag |= PARODD;
      }
    }
    if (f->stop_bits == 2) {
      tio->c_cflag |= CSTOPB;
    }
  }
  if (hal_data->req_rtscts) {
    tio->c_cflag |= CRTSCTS;
  }
}

// mark a new serial port configuration for the RT thread to apply.
// Returns 0 if the requested configuration is supported.
static int request_config(lcec_el6021_data_t *hal_data, tcflag_t cflag, unsigned int ospeed) {
  uint8_t baud_idx, frame_idx, rtscts;

  if (lcec_el6021_cflag_to_config(cflag, ospeed, &baud_idx, &frame_idx, &rtscts) != 0) {
    return -1;
  }
  if (rtscts && !hal_data->has_rtscts) {
    return -1;  // RS422/RS485 terminals have no RTS/CTS
  }

  pthread_mutex_lock(&hal_data->lock);
  hal_data->req_baud = baud_idx;
  hal_data->req_frame = frame_idx;
  hal_data->req_rtscts = rtscts;
  config_to_ktermios(hal_data, &hal_data->tio);
  pthread_mutex_unlock(&hal_data->lock);
  hal_data->cfg_changed = 1;
  return 0;
}

/// @brief Apply the requested serial configuration (0x8000:01/11/15) and
/// resync cur_/pend_.  Shared by _init and proc_reinit: a power-cycled
/// terminal reverts to factory defaults, so re-apply the last termios
/// request (req_* survives in hal_data).  Safe against the RT thread,
/// which returns early while the slave is not operational.
static int lcec_el6021_apply_config(lcec_slave_t *slave) {
  lcec_el6021_data_t *hal_data = (lcec_el6021_data_t *)slave->hal_data;
  uint8_t rtscts, baud, frame;

  pthread_mutex_lock(&hal_data->lock);
  rtscts = hal_data->req_rtscts;
  baud = hal_data->req_baud;
  frame = hal_data->req_frame;
  pthread_mutex_unlock(&hal_data->lock);

  // 0x8000:01 is RTS/CTS on the EL6001 but a padding bit on the EL6021
  if (hal_data->has_rtscts && lcec_write_sdo8(slave, 0x8000, 0x01, rtscts) != 0) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo RtsCts\n", slave->master->name, slave->name);
    return -1;
  }
  if (lcec_write_sdo8(slave, 0x8000, 0x11, baud) != 0) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo BaudRate\n", slave->master->name, slave->name);
    return -1;
  }
  if (lcec_write_sdo8(slave, 0x8000, 0x15, frame) != 0) {
    rtapi_print_msg(
        RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo DataFrame\n", slave->master->name, slave->name);
    return -1;
  }
  hal_data->cur_rtscts = hal_data->pend_rtscts = rtscts;
  hal_data->cur_baud = hal_data->pend_baud = baud;
  hal_data->cur_frame = hal_data->pend_frame = frame;
  return 0;
}

static int lcec_el6021_init(int comp_id, lcec_slave_t *slave) {
  lcec_master_t *master = slave->master;
  (void)comp_id;
  lcec_slave_modparam_t *p;
  lcec_el6021_data_t *hal_data;
  const lcec_el6021_baud_t *baud;
  const lcec_el6021_frame_t *frame;
  int err;
  char *c;

  // alloc hal memory
  hal_data = LCEC_HAL_ALLOCATE(lcec_el6021_data_t);
  slave->hal_data = hal_data;
  memset(hal_data, 0, sizeof(*hal_data));
  hal_data->efd = -1;
  hal_data->stop_efd = -1;
  pthread_mutex_init(&hal_data->lock, NULL);

  // defaults: 9600 8N1, no handshake
  hal_data->req_baud = lcec_el6021_baud_by_value(9600)->idx;
  hal_data->req_frame = lcec_el6021_frame_by_string("8N1")->idx;
  hal_data->req_rtscts = 0;
  snprintf(hal_data->tty_name, sizeof(hal_data->tty_name), "lcec-%s-%s", master->name, slave->name);

  // parse modparams
  for (p = slave->modparams; p != NULL && p->id >= 0; p++) {
    switch (p->id) {
      case LCEC_EL6021_PARAM_BAUD:
        baud = lcec_el6021_baud_by_value(p->value.u32);
        if (baud == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid baud rate %u for slave %s.%s\n", p->value.u32, master->name,
              slave->name);
          return -1;
        }
        hal_data->req_baud = baud->idx;
        break;
      case LCEC_EL6021_PARAM_FRAME:
        frame = lcec_el6021_frame_by_string(p->value.str);
        if (frame == NULL) {
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "invalid dataFrame \"%s\" for slave %s.%s\n", p->value.str,
              master->name, slave->name);
          return -1;
        }
        hal_data->req_frame = frame->idx;
        break;
      case LCEC_EL6021_PARAM_RTSCTS:
        hal_data->req_rtscts = p->value.bit ? 1 : 0;
        break;
      case LCEC_EL6021_PARAM_TTYNAME:
        snprintf(hal_data->tty_name, sizeof(hal_data->tty_name), "%s", p->value.str);
        break;
    }
  }

  // sanitize device node name
  for (c = hal_data->tty_name; *c != 0; c++) {
    if (!(*c >= 'a' && *c <= 'z') && !(*c >= 'A' && *c <= 'Z') && !(*c >= '0' && *c <= '9') && *c != '-' && *c != '_' &&
        *c != '.') {
      *c = '-';
    }
  }

  // apply serial configuration to the terminal (PREOP SDO writes)
  hal_data->has_rtscts = (slave->flags & LCEC_EL6021_FLAG_RTSCTS) != 0;
  if (!hal_data->has_rtscts) {
    hal_data->req_rtscts = 0;
  }
  if ((err = lcec_el6021_apply_config(slave)) != 0) {
    return err;
  }
  config_to_ktermios(hal_data, &hal_data->tio);

  // runtime SDO requests for configuration changes while running
  if (hal_data->has_rtscts) {
    hal_data->sdo_rtscts = ecrt_slave_config_create_sdo_request(slave->config, 0x8000, 0x01, 1);
  }
  hal_data->sdo_baud = ecrt_slave_config_create_sdo_request(slave->config, 0x8000, 0x11, 1);
  hal_data->sdo_frame = ecrt_slave_config_create_sdo_request(slave->config, 0x8000, 0x15, 1);
  if ((hal_data->has_rtscts && hal_data->sdo_rtscts == NULL) || hal_data->sdo_baud == NULL || hal_data->sdo_frame == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to create SDO requests for slave %s.%s\n", master->name,
        slave->name);
    return -1;
  }

  // initialize callbacks
  slave->proc_read = lcec_el6021_read;
  slave->proc_write = lcec_el6021_write;
  slave->proc_cleanup = lcec_el6021_cleanup;

  // initialize sync info
  slave->sync_info = lcec_el6021_syncs;

  // initialize PDO entries (data windows are contiguous from 0x?000:11)
  lcec_pdo_init(slave, 0x7000, 0x01, &hal_data->ctrl_os, &hal_data->ctrl_bp);
  lcec_pdo_init(slave, 0x7000, 0x11, &hal_data->tx_os, NULL);
  lcec_pdo_init(slave, 0x6000, 0x01, &hal_data->status_os, &hal_data->status_bp);
  lcec_pdo_init(slave, 0x6000, 0x11, &hal_data->rx_os, NULL);

  // export pins
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  // initialize state
  hal_data->state = LCEC_EL6021_STATE_REQUEST_INIT;

#ifdef LCEC_HAVE_CUSE
  err = lcec_el6021_cuse_start(slave);
  if (err != 0) {
    rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "slave %s.%s: CUSE device /dev/%s not available, serial port is "
        "not exposed to userspace\n", master->name, slave->name, hal_data->tty_name);
  }
#else
  rtapi_print_msg(RTAPI_MSG_WARN, LCEC_MSG_PFX "slave %s.%s: built without CUSE support, serial port is not exposed "
      "to userspace\n", master->name, slave->name);
#endif

  return 0;
}

static void lcec_el6021_read(lcec_slave_t *slave, long period) {
  (void)period;
  lcec_el6021_data_t *hal_data = (lcec_el6021_data_t *)slave->hal_data;
  uint8_t *pd = lcec_slave_pd(slave);
  uint16_t status;
  uint8_t rx_request_toggle, rx_len;

  if (!slave->state.operational) {
    hal_data->last_status = 0;
    return;
  }

  status = EC_READ_U16(&pd[hal_data->status_os]);
  hal_data->last_status = status;

  // receive data: toggle on status bit 1 signals new data
  rx_request_toggle = (status >> 1) & 1;
  if (hal_data->state == LCEC_EL6021_STATE_READY && rx_request_toggle != hal_data->rx_req_tgl) {
    hal_data->rx_req_tgl = rx_request_toggle;
    rx_len = status >> 8;
    if (rx_len > LCEC_EL6021_DATA_MAX) {
      rx_len = LCEC_EL6021_DATA_MAX;
    }
    if (rx_len > 0) {
      if (ring_free(&hal_data->rx_w, &hal_data->rx_r) >= rx_len) {
        ring_write(hal_data->rx_ring, &hal_data->rx_w, &pd[hal_data->rx_os], rx_len);
        hal_data->rx_byte_count += rx_len;
      } else {
        hal_data->rx_drop_count += rx_len;
      }
    }
    hal_data->rx_ack_tgl = !hal_data->rx_ack_tgl;

    // wake the CUSE notify thread
    if (hal_data->efd >= 0) {
      uint64_t one = 1;
      ssize_t unused = write(hal_data->efd, &one, sizeof(one));
      (void)unused;
    }
  }

  // handle pin-driven counter reset
  if (LCEC_PIN_BIT_GET(hal_data->reset)) {
    hal_data->rx_byte_count = 0;
    hal_data->tx_byte_count = 0;
    hal_data->rx_drop_count = 0;
  }

  // update pins
  LCEC_PIN_U32_SET(hal_data->ser_state, hal_data->state);
  LCEC_PIN_U32_SET(hal_data->rx_bytes, hal_data->rx_byte_count);
  LCEC_PIN_U32_SET(hal_data->tx_bytes, hal_data->tx_byte_count);
  LCEC_PIN_U32_SET(hal_data->rx_dropped, hal_data->rx_drop_count);
  {
    const lcec_el6021_baud_t *b = lcec_el6021_baud_by_idx(hal_data->cur_baud);
    LCEC_PIN_U32_SET(hal_data->baud, b != NULL ? b->baud : 0);
  }
  LCEC_PIN_BIT_SET(hal_data->cfg_error, hal_data->cfg_error_state);
  LCEC_PIN_BIT_SET(hal_data->tty_open, hal_data->open_count > 0);
}

static void lcec_el6021_write(lcec_slave_t *slave, long period) {
  (void)period;
  lcec_el6021_data_t *hal_data = (lcec_el6021_data_t *)slave->hal_data;
  uint8_t *pd = lcec_slave_pd(slave);
  uint16_t status = hal_data->last_status;
  uint8_t tx_accepted_toggle;
  uint32_t avail, n;

  if (!slave->state.operational) {
    hal_data->state = LCEC_EL6021_STATE_REQUEST_INIT;
    hal_data->tx_req_tgl = 0;
    hal_data->rx_ack_tgl = 0;
    hal_data->tx_len = 0;
    hal_data->control = 0;
    return;
  }

  switch (hal_data->state) {
    case LCEC_EL6021_STATE_READY:
      // check if the serial port configuration has to be updated
      if (hal_data->cfg_changed) {
        hal_data->cfg_changed = 0;
        pthread_mutex_lock(&hal_data->lock);
        hal_data->pend_rtscts = hal_data->req_rtscts;
        hal_data->pend_baud = hal_data->req_baud;
        hal_data->pend_frame = hal_data->req_frame;
        pthread_mutex_unlock(&hal_data->lock);
      }
      if (!hal_data->cfg_error_state && hal_data->pend_rtscts != hal_data->cur_rtscts) {
        EC_WRITE_U8(ecrt_sdo_request_data(hal_data->sdo_rtscts), hal_data->pend_rtscts);
        ecrt_sdo_request_write(hal_data->sdo_rtscts);
        hal_data->state = LCEC_EL6021_STATE_SET_RTSCTS;
        break;
      }
      if (!hal_data->cfg_error_state && hal_data->pend_baud != hal_data->cur_baud) {
        EC_WRITE_U8(ecrt_sdo_request_data(hal_data->sdo_baud), hal_data->pend_baud);
        ecrt_sdo_request_write(hal_data->sdo_baud);
        hal_data->state = LCEC_EL6021_STATE_SET_BAUD;
        break;
      }
      if (!hal_data->cfg_error_state && hal_data->pend_frame != hal_data->cur_frame) {
        EC_WRITE_U8(ecrt_sdo_request_data(hal_data->sdo_frame), hal_data->pend_frame);
        ecrt_sdo_request_write(hal_data->sdo_frame);
        hal_data->state = LCEC_EL6021_STATE_SET_FRAME;
        break;
      }

      // send data: toggle on status bit 0 signals the terminal accepted
      // the previous window
      tx_accepted_toggle = status & 1;
      hal_data->tx_len = 0;
      if (tx_accepted_toggle != hal_data->tx_ack_tgl) {
        avail = ring_avail(&hal_data->tx_w, &hal_data->tx_r);
        n = avail > LCEC_EL6021_DATA_MAX ? LCEC_EL6021_DATA_MAX : avail;
        if (n > 0) {
          ring_read(hal_data->tx_ring, &hal_data->tx_r, hal_data->tx_buf, n);
          hal_data->tx_len = n;
          hal_data->tx_byte_count += n;
          hal_data->tx_req_tgl = !hal_data->tx_req_tgl;
          hal_data->tx_ack_tgl = tx_accepted_toggle;
        }
      }

      hal_data->control = hal_data->tx_req_tgl | (hal_data->rx_ack_tgl << 1) | ((uint16_t)hal_data->tx_len << 8);
      break;

    case LCEC_EL6021_STATE_REQUEST_INIT:
      if (status & (1 << 2)) {
        hal_data->control = 0x0000;
        hal_data->state = LCEC_EL6021_STATE_WAIT_INIT_RESPONSE;
      } else {
        hal_data->control = 1 << 2;  // CW.2, request initialization
      }
      break;

    case LCEC_EL6021_STATE_WAIT_INIT_RESPONSE:
      if (!(status & (1 << 2))) {
        // Init successful. The terminal restarts its handshake with SW.0 and
        // SW.1 at 0 (and saw CW.0/CW.1 at 0 during the init), so restart
        // ours: a stale request toggle left at 1 by an odd number of TX
        // windows before a re-init (baud/frame change) would make the next
        // request look like no request, and TX stalls forever.
        hal_data->tx_req_tgl = 0;
        hal_data->rx_req_tgl = 0;
        hal_data->rx_ack_tgl = 0;
        // first TX window is free
        hal_data->tx_ack_tgl = 1;
        hal_data->control = 0x0000;
        hal_data->state = LCEC_EL6021_STATE_READY;
      }
      break;

    case LCEC_EL6021_STATE_SET_RTSCTS:
      switch (ecrt_sdo_request_state(hal_data->sdo_rtscts)) {
        case EC_REQUEST_SUCCESS:
          hal_data->cur_rtscts = hal_data->pend_rtscts;
          hal_data->state = LCEC_EL6021_STATE_REQUEST_INIT;
          break;
        case EC_REQUEST_ERROR:
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s: failed to set RTS/CTS\n", hal_data->tty_name);
          hal_data->state = LCEC_EL6021_STATE_REQUEST_INIT;
          hal_data->cfg_error_state = 1;
          break;
        default:
          break;
      }
      break;

    case LCEC_EL6021_STATE_SET_BAUD:
      switch (ecrt_sdo_request_state(hal_data->sdo_baud)) {
        case EC_REQUEST_SUCCESS:
          hal_data->cur_baud = hal_data->pend_baud;
          hal_data->state = LCEC_EL6021_STATE_REQUEST_INIT;
          break;
        case EC_REQUEST_ERROR:
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s: failed to set baud rate\n", hal_data->tty_name);
          hal_data->state = LCEC_EL6021_STATE_REQUEST_INIT;
          hal_data->cfg_error_state = 1;
          break;
        default:
          break;
      }
      break;

    case LCEC_EL6021_STATE_SET_FRAME:
      switch (ecrt_sdo_request_state(hal_data->sdo_frame)) {
        case EC_REQUEST_SUCCESS:
          hal_data->cur_frame = hal_data->pend_frame;
          hal_data->state = LCEC_EL6021_STATE_REQUEST_INIT;
          break;
        case EC_REQUEST_ERROR:
          rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s: failed to set data frame\n", hal_data->tty_name);
          hal_data->state = LCEC_EL6021_STATE_REQUEST_INIT;
          hal_data->cfg_error_state = 1;
          break;
        default:
          break;
      }
      break;
  }

  EC_WRITE_U16(&pd[hal_data->ctrl_os], hal_data->control);
  if (hal_data->tx_len > 0) {
    memcpy(&pd[hal_data->tx_os], hal_data->tx_buf, hal_data->tx_len);
  }
}

static void lcec_el6021_cleanup(lcec_slave_t *slave) {
#ifdef LCEC_HAVE_CUSE
  lcec_el6021_cuse_stop(slave);
#endif
}

#ifdef LCEC_HAVE_CUSE

// CUSE character device glue.  Runs in its own (non-realtime) thread and
// exchanges bytes with the realtime thread through the ring buffers.

static ssize_t cuse_io_writev(int fd, struct iovec *iov, int count, void *userdata) {
  (void)userdata;
  return writev(fd, iov, count);
}

// fuse_session_exit() only sets a flag; the session loop sits in this read
// until the kernel sends a request.  Wait on the stop eventfd as well, so
// lcec_el6021_cuse_stop() can end the loop (EINTR after exit ends it).
static ssize_t cuse_io_read(int fd, void *buf, size_t buf_len, void *userdata) {
  lcec_el6021_data_t *hal_data = userdata;
  struct pollfd pfd[2] = {{fd, POLLIN, 0}, {hal_data->stop_efd, POLLIN, 0}};

  for (;;) {
    if (poll(pfd, 2, -1) < 0) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    }
    if (pfd[1].revents) {
      errno = EINTR;
      return -1;
    }
    if (pfd[0].revents) {
      return read(fd, buf, buf_len);
    }
  }
}

static const struct fuse_custom_io cuse_io = {
    .writev = cuse_io_writev,
    .read = cuse_io_read,
};

static void cuse_open(fuse_req_t req, struct fuse_file_info *fi) {
  lcec_el6021_data_t *hal_data = fuse_req_userdata(req);
  hal_data->open_count++;
  fuse_reply_open(req, fi);
}

static void cuse_release(fuse_req_t req, struct fuse_file_info *fi) {
  lcec_el6021_data_t *hal_data = fuse_req_userdata(req);
  (void)fi;
  if (hal_data->open_count > 0) {
    hal_data->open_count--;
  }
  fuse_reply_err(req, 0);
}

static void cuse_read(fuse_req_t req, size_t size, off_t off, struct fuse_file_info *fi) {
  lcec_el6021_data_t *hal_data = fuse_req_userdata(req);
  uint8_t buf[256];
  uint32_t avail, n;
  (void)off;
  (void)fi;

  if (size > sizeof(buf)) {
    size = sizeof(buf);
  }
  avail = ring_avail(&hal_data->rx_w, &hal_data->rx_r);
  n = avail < size ? avail : (uint32_t)size;
  if (n == 0) {
    fuse_reply_buf(req, NULL, 0);
    return;
  }
  ring_read(hal_data->rx_ring, &hal_data->rx_r, buf, n);
  fuse_reply_buf(req, (const char *)buf, n);
}

static void cuse_write(fuse_req_t req, const char *buf, size_t size, off_t off, struct fuse_file_info *fi) {
  lcec_el6021_data_t *hal_data = fuse_req_userdata(req);
  uint32_t free, n;
  (void)off;
  (void)fi;

  free = ring_free(&hal_data->tx_w, &hal_data->tx_r);
  n = free < size ? free : (uint32_t)size;
  if (n > 0) {
    ring_write(hal_data->tx_ring, &hal_data->tx_w, (const uint8_t *)buf, n);
  }
  fuse_reply_write(req, n);
}

static void cuse_poll(fuse_req_t req, struct fuse_file_info *fi, struct fuse_pollhandle *ph) {
  lcec_el6021_data_t *hal_data = fuse_req_userdata(req);
  unsigned revents = 0;
  (void)fi;

  if (ring_avail(&hal_data->rx_w, &hal_data->rx_r) > 0) {
    revents |= POLLIN | POLLRDNORM;
  }
  if (ring_free(&hal_data->tx_w, &hal_data->tx_r) > 0) {
    revents |= POLLOUT | POLLWRNORM;
  }

  pthread_mutex_lock(&hal_data->lock);
  if (ph != NULL) {
    if (hal_data->cuse_ph != NULL) {
      fuse_lowlevel_notify_poll(hal_data->cuse_ph);
    }
    hal_data->cuse_ph = ph;
  }
  pthread_mutex_unlock(&hal_data->lock);

  fuse_reply_poll(req, revents);
}

static void cuse_ioctl(fuse_req_t req, int cmd, void *arg, struct fuse_file_info *fi, unsigned int flags,
    const void *in_buf, size_t in_bufsz, size_t out_bufsz) {
  lcec_el6021_data_t *hal_data = fuse_req_userdata(req);
  struct iovec iov;
  lcec_el6021_ktermios_t tio;
  size_t tio_size;
  uint32_t avail;
  int mstate;
  (void)fi;

  if (flags & FUSE_IOCTL_COMPAT) {
    fuse_reply_err(req, ENOSYS);
    return;
  }

  switch (cmd) {
    case TCGETS:
    case LCEC_TCGETS2:
      tio_size = (cmd == TCGETS) ? LCEC_EL6021_KTERMIOS_SIZE : LCEC_EL6021_KTERMIOS2_SIZE;
      if (!out_bufsz) {
        iov.iov_base = arg;
        iov.iov_len = tio_size;
        fuse_reply_ioctl_retry(req, NULL, 0, &iov, 1);
        return;
      }
      pthread_mutex_lock(&hal_data->lock);
      tio = hal_data->tio;
      pthread_mutex_unlock(&hal_data->lock);
      fuse_reply_ioctl(req, 0, &tio, tio_size);
      return;

    case TCSETS:
    case TCSETSW:
    case TCSETSF:
    case LCEC_TCSETS2:
    case LCEC_TCSETSW2:
    case LCEC_TCSETSF2:
      tio_size = (cmd == TCSETS || cmd == TCSETSW || cmd == TCSETSF) ? LCEC_EL6021_KTERMIOS_SIZE : LCEC_EL6021_KTERMIOS2_SIZE;
      if (!in_bufsz) {
        iov.iov_base = arg;
        iov.iov_len = tio_size;
        fuse_reply_ioctl_retry(req, &iov, 1, NULL, 0);
        return;
      }
      memset(&tio, 0, sizeof(tio));
      memcpy(&tio, in_buf, in_bufsz < tio_size ? in_bufsz : tio_size);
      if (request_config(hal_data, tio.c_cflag, tio.c_ospeed) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s: unsupported serial configuration requested\n",
            hal_data->tty_name);
        fuse_reply_err(req, EINVAL);
        return;
      }
      fuse_reply_ioctl(req, 0, NULL, 0);
      return;

    case LCEC_TCFLSH:
      // arg is the queue selector passed by value
      if ((long)arg == TCIFLUSH || (long)arg == TCIOFLUSH) {
        hal_data->rx_r = hal_data->rx_w;
      }
      if ((long)arg == TCOFLUSH || (long)arg == TCIOFLUSH) {
        hal_data->tx_w = hal_data->tx_r;
      }
      fuse_reply_ioctl(req, 0, NULL, 0);
      return;

    case LCEC_TCSBRK:
    case LCEC_TCSBRKP:
      // tcdrain(): wait until the RT side has handed every queued byte to
      // the terminal (bounded, so a stalled bus cannot hang the caller).
      // A real break (arg 0) is not supported by the EL6021 and is accepted
      // as a no-op like TIOCSBRK.
      if ((long)arg != 0 || cmd == LCEC_TCSBRKP) {
        for (int i = 0; i < 2000 && ring_avail(&hal_data->tx_w, &hal_data->tx_r) > 0; i++) {
          usleep(1000);
        }
      }
      fuse_reply_ioctl(req, 0, NULL, 0);
      return;

    case FIONREAD:
      if (!out_bufsz) {
        iov.iov_base = arg;
        iov.iov_len = sizeof(int);
        fuse_reply_ioctl_retry(req, NULL, 0, &iov, 1);
        return;
      }
      avail = ring_avail(&hal_data->rx_w, &hal_data->rx_r);
      fuse_reply_ioctl(req, 0, &avail, sizeof(avail));
      return;

    case TIOCMGET:
      if (!out_bufsz) {
        iov.iov_base = arg;
        iov.iov_len = sizeof(int);
        fuse_reply_ioctl_retry(req, NULL, 0, &iov, 1);
        return;
      }
      mstate = TIOCM_RTS | TIOCM_CTS | TIOCM_DSR | TIOCM_CD;
      fuse_reply_ioctl(req, 0, &mstate, sizeof(mstate));
      return;

    case TIOCMSET:
    case TIOCMBIS:
    case TIOCMBIC:
    case TIOCSBRK:
    case TIOCCBRK:
      // accepted, but there is no physical modem control behind this port
      fuse_reply_ioctl(req, 0, NULL, 0);
      return;

    default:
      fuse_reply_err(req, ENOSYS);
      return;
  }
}

static const struct cuse_lowlevel_ops cuse_clop = {
    .open = cuse_open,
    .read = cuse_read,
    .write = cuse_write,
    .release = cuse_release,
    .ioctl = cuse_ioctl,
    .poll = cuse_poll,
};

// waits on the eventfd signalled by the RT thread when RX data arrived
// and wakes up pollers of the character device
static void *lcec_el6021_notify_main(void *arg) {
  lcec_el6021_data_t *hal_data = arg;
  uint64_t val;
  void *ph;
  ssize_t ret;

  while (!hal_data->thread_stop) {
    ret = read(hal_data->efd, &val, sizeof(val));
    if (ret <= 0) {
      if (ret < 0 && errno == EINTR) {
        continue;
      }
      break;
    }
    pthread_mutex_lock(&hal_data->lock);
    ph = hal_data->cuse_ph;
    pthread_mutex_unlock(&hal_data->lock);
    if (ph != NULL) {
      fuse_lowlevel_notify_poll(ph);
    }
  }
  return NULL;
}

static void *lcec_el6021_cuse_main(void *arg) {
  lcec_slave_t *slave = arg;
  lcec_el6021_data_t *hal_data = (lcec_el6021_data_t *)slave->hal_data;
  lcec_master_t *master = slave->master;
  char devinfo[HAL_NAME_LEN + 16];
  const char *ci_argv[1];
  struct cuse_info ci;
  char *argv[1];
  struct fuse_args fargs;
  struct fuse_session *se;
  char progname[16];
  int fd;

  snprintf(devinfo, sizeof(devinfo), "DEVNAME=%s", hal_data->tty_name);
  ci_argv[0] = devinfo;
  memset(&ci, 0, sizeof(ci));
  ci.dev_info_argc = 1;
  ci.dev_info_argv = ci_argv;
  ci.flags = CUSE_UNRESTRICTED_IOCTL;

  snprintf(progname, sizeof(progname), "lcec-el6021");
  argv[0] = progname;
  fargs = (struct fuse_args)FUSE_ARGS_INIT(1, argv);

  se = cuse_lowlevel_new(&fargs, &ci, &cuse_clop, hal_data);
  if (se == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s.%s: failed to create CUSE session\n", master->name, slave->name);
    return NULL;
  }

  fd = open("/dev/cuse", O_RDWR);
  if (fd < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s.%s: failed to open /dev/cuse: %s (cuse module loaded?)\n",
        master->name, slave->name, strerror(errno));
    fuse_session_destroy(se);
    return NULL;
  }

  if (fuse_session_custom_io(se, &cuse_io, fd) != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s.%s: failed to wire CUSE session\n", master->name, slave->name);
    close(fd);
    fuse_session_destroy(se);
    return NULL;
  }

  hal_data->cuse_se = se;
  rtapi_print_msg(RTAPI_MSG_INFO, LCEC_MSG_PFX "%s.%s: serial port available as /dev/%s\n", master->name, slave->name,
      hal_data->tty_name);

  fuse_session_loop(se);

  hal_data->cuse_se = NULL;
  fuse_session_destroy(se);
  fuse_opt_free_args(&fargs);
  return NULL;
}

static int lcec_el6021_cuse_start(lcec_slave_t *slave) {
  lcec_el6021_data_t *hal_data = (lcec_el6021_data_t *)slave->hal_data;

  hal_data->efd = eventfd(0, 0);
  hal_data->stop_efd = eventfd(0, 0);
  if (hal_data->efd < 0 || hal_data->stop_efd < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "%s: eventfd failed: %s\n", hal_data->tty_name, strerror(errno));
    return -1;
  }

  if (pthread_create(&hal_data->cuse_thread, NULL, lcec_el6021_cuse_main, slave) != 0) {
    close(hal_data->efd);
    hal_data->efd = -1;
    return -1;
  }

  if (pthread_create(&hal_data->notify_thread, NULL, lcec_el6021_notify_main, hal_data) != 0) {
    hal_data->thread_stop = 1;
    if (hal_data->cuse_se != NULL) {
      fuse_session_exit(hal_data->cuse_se);
    }
    pthread_join(hal_data->cuse_thread, NULL);
    close(hal_data->efd);
    hal_data->efd = -1;
    return -1;
  }

  hal_data->tty_enabled = 1;
  return 0;
}

static void lcec_el6021_cuse_stop(lcec_slave_t *slave) {
  lcec_el6021_data_t *hal_data = (lcec_el6021_data_t *)slave->hal_data;
  uint64_t one = 1;
  ssize_t unused;

  if (!hal_data->tty_enabled) {
    return;
  }

  hal_data->thread_stop = 1;
  if (hal_data->cuse_se != NULL) {
    fuse_session_exit(hal_data->cuse_se);
  }
  unused = write(hal_data->efd, &one, sizeof(one));
  unused = write(hal_data->stop_efd, &one, sizeof(one));
  (void)unused;

  pthread_join(hal_data->cuse_thread, NULL);
  pthread_join(hal_data->notify_thread, NULL);
  close(hal_data->efd);
  hal_data->efd = -1;
  close(hal_data->stop_efd);
  hal_data->stop_efd = -1;
  hal_data->tty_enabled = 0;
}

#endif  // LCEC_HAVE_CUSE
