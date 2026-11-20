//
//    Copyright (C) 2014 Sascha Ittner <sascha.ittner@modusoft.de>
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
/// @brief Driver for Delta ASDA servo controllers

/*#############################################################################

Description of Pins/Parameters:

Pins:
  FLOAT OUT srv-vel-fb
    Actual velocity feddback in scale units per second.

  FLOAT OUT srv-vel-fb-rpm
    Actual velocity feddback in revolutions per minute.

  FLOAT OUT srv-vel-fb-rpm-abs
    Actual absolute value of velocity feddback in
    revolutions per minute.

  FLOAT OUT srv-vel-rpm
    Current velocity command send to motor driver in
    revolutions per minute.

  BIT OUT srv-ready
    ready status flag from motor driver

  BIT OUT srv-switched-on
    switched-on status flag from motor driver

  BIT OUT srv-oper-enabled
    operation-enabled status flag from motor driver

  BIT OUT srv-fault
    fault status flag from motor driver

  U32 OUT srv-error-code
    CiA 402 error code of the current alarm (0x603F), 0 without alarm

  U32 OUT srv-alarm-code
    alarm code as shown on the display of the drive (P0-01), 0 without
    alarm

  BIT OUT srv-volt-enabled
    voltage-enabled status flag from motor driver

  BIT OUT srv-quick-stoped
    quick-stoped status flag from motor driver

  BIT OUT srv-on-disabled
    on-disabled status flag from motor driver

  BIT OUT srv-warning
    warning status flag from motor driver

  BIT OUT srv-remote
    motor driver is remote controlled

  BIT OUT srv-at-speed
    motor has reached requested velocity

  BIT OUT srv-limit-active
    motor driver has reached some limit (statusword bit 11; on A3 and B3
    also the positive and negative limit in statusword bits 14 and 15).

  BIT OUT srv-zero-speed
    motor has stopped: the actual speed is below the zero speed detection
    level of the drive (0x606F, drive parameter P1.038, default 10 rpm)

  S32 OUT srv-enc-raw
    raw value from motor encoder without offset calculation. This
    value is the unmodified one ad repoted by the motor driver.

  U32 OUT srv-pos-raw-hi
    Upper 32 bits of range extended 64 bit encoder value. This is a relative
    value wich is reset on restart/reset pin trigger.

  U32 OUT srv-pos-raw-lo
    Lower 32 bits of range extended 64 bit encoder value. This is a relative
    value wich is reset on restart/reset pin trigger.

  FLOAT OUT srv-pos-fb
    Actual encoder position

  BIT OUT srv-on-home-neg
    Motor encoder is below virtual home switch position. (See parameter
    srv-home-raw)

  BIT OUT srv-on-home-pos
    Motor encoder is above virtual home switch position. (See parameter
    srv-home-raw)

  BIT IN srv-pos-reset
    Reset relative position counter (srv-pos-raw-hi/srv-pos-raw-lo)

  BIT IN srv-switch-on
    Switch motor driver on

  BIT IN srv-enable-volt
    Enable motor driver voltage

  BIT IN srv-quick-stop
    Enable motor quick stop

  BIT IN srv-enable
    Enable motor driver operation. This signal is internaly delayed
    until driver reports switched-on.

  BIT IN srv-fault-reset
    Reset motor driver error

  BIT IN srv-fault-autoreset
    If set to 1 errors get automatically reset if drive is disabled (enable = 0)

  BIT IN srv-halt
    Set motor driver halt flag

  FLOAT IN srv-vel-cmd
    Velocity command input in scale units per second.

Pins with <modParam name="enableHM" value="true"/>:
  BIT IO srv-home
    Start drive internal homing. Gets reset by the driver as soon as
    homing is complete, and only then: a fault aborts the run but leaves
    the pin on. Could be connected to joint.N.index-enable.

  BIT OUT srv-homed
    Drive reports that homing is complete.

  BIT OUT srv-homing
    Homing is running.

  BIT OUT srv-homing-error
    Drive reports a homing error.

  S32 OUT srv-opmode-display
    Active mode of operation of the drive (6 = homing, 8 = CSP, 9 = CSV).

  FLOAT OUT srv-pos-fb
    Actual position of the drive in units of srv-pos-cmd.
    Follows srv-pos-cmd while srv-home is set, to prevent a following
    error while the drive is homing. Connect to joint.N.motor-pos-fb.

  FLOAT IN srv-pos-cmd
    CSV mode only (in CSP mode this is the regular command pin).
    Position command of the joint. It is not sent to the drive, it is
    only used for srv-pos-fb while the drive is homing.

  S32 IN srv-home-method
    Homing method (0x6098).

  U32 IN srv-home-velocity-fast
    Speed during search for switch (0x6099:01).

  U32 IN srv-home-velocity-slow
    Speed during search for zero (0x6099:02).

  U32 IN srv-home-accel
    Homing acceleration (0x609A).

  BIT OUT srv-sw-limit-armed
    The software limits of the drive (0x607D) are set from
    srv-sw-limit-min / srv-sw-limit-max. Only while the drive is homed.

Parameters with <modParam name="enableHM" value="true"/>:
  RW FLOAT srv-sw-limit-min
  RW FLOAT srv-sw-limit-max
    Limits of the joint, in joint coordinates ([JOINT_n]MIN_LIMIT /
    MAX_LIMIT). The software limits of the drive are not used while both
    are equal.

  RW FLOAT srv-sw-limit-offset
    Joint position of the zero of the drive ([JOINT_n]HOME_OFFSET).

  RW FLOAT srv-sw-limit-margin
    How far the limits of the drive are outside of the limits of the joint.

Parameters:
  RW FLOAT srv-pos-scale
    Scale for position/velocity values (1.0 -> 1 revolution per second)

  RW S32 srv-home-raw
    Absolute position of virtual home switch. This could be used to implement
    a home switch on motors with abolute encoders without a hardware switch.

#############################################################################*/

#ifndef _LCEC_DEASDA_H_
#define _LCEC_DEASDA_H_

#include "../lcec.h"

#endif
