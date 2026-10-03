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

/// @file
/// @brief Driver for Beckhoff EL6001/EL6021 serial interface terminals

#ifndef _LCEC_EL6021_H_
#define _LCEC_EL6021_H_

#include "../lcec.h"

// modparam IDs
#define LCEC_EL6021_PARAM_BAUD    1
#define LCEC_EL6021_PARAM_FRAME   2
#define LCEC_EL6021_PARAM_RTSCTS  3
#define LCEC_EL6021_PARAM_TTYNAME 4

/// number of data bytes transferred per process data window
#define LCEC_EL6021_DATA_MAX 22

/// size of the RX/TX ring buffers between the realtime thread and the CUSE thread
#define LCEC_EL6021_RING_SIZE 1024

#endif
