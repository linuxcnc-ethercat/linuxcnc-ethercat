//
//    Copyright (C) 2026 Luca Toniolo
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
/// @brief Compatibility layer for the LinuxCNC HAL getter/setter API transition.
///
/// LinuxCNC PR #4099 introduced typed opaque pin/param references
/// (`hal_bool_t`, `hal_sint_t`, `hal_uint_t`, `hal_real_t`) with
/// `hal_get_*()`/`hal_set_*()` inline accessors.  The old API
/// (`hal_pin_bit_new()` and direct dereferencing of `hal_bit_t *` etc.)
/// remains available until upstream performs the announced "API break"
/// (see draft PR #4247), at which point `HAL_S32`/`HAL_U32`/`HAL_S64`/
/// `HAL_U64` and direct data access disappear.
///
/// The `LCEC_PIN_*` macros below let lcec code access pin data through the
/// new getter/setter inlines when building against a new-enough LinuxCNC,
/// and fall back to direct dereferencing on older LinuxCNC (2.9.x).  Both
/// variants interoperate on shared HAL signals: the new setters always write
/// the full 64-bit storage slot (sign-extended), and both reader styles use
/// the low bytes of the same little-endian slot.
///
/// Detection relies on `HAL_BOOL`, which is only defined by post-#4099 hal.h.

#ifndef _LCEC_HAL_COMPAT_H_
#define _LCEC_HAL_COMPAT_H_

#include "hal.h"

#ifdef HAL_BOOL
#define LCEC_HAL_NEW_API 1

// New API: access through the typed inline accessors.  Pin storage pointers
// in lcec hal_data structs are still declared with the old `hal_*_t *`
// pointer types; the casts are safe because the opaque reference types are
// plain pointers to the same little-endian storage slot, and the accessors
// reach it through volatile union members.
#define LCEC_PIN_BIT_SET(p, v) hal_set_bool((hal_bool_t)(p), (v))
#define LCEC_PIN_BIT_GET(p) hal_get_bool((hal_bool_t)(p))
#define LCEC_PIN_FLOAT_SET(p, v) hal_set_real((hal_real_t)(p), (v))
#define LCEC_PIN_FLOAT_GET(p) hal_get_real((hal_real_t)(p))
#define LCEC_PIN_S32_SET(p, v) hal_set_si32((hal_sint_t)(p), (v))
#define LCEC_PIN_S32_GET(p) hal_get_si32((hal_sint_t)(p))
#define LCEC_PIN_U32_SET(p, v) hal_set_ui32((hal_uint_t)(p), (v))
#define LCEC_PIN_U32_GET(p) hal_get_ui32((hal_uint_t)(p))
#else
// Old API (LinuxCNC 2.9.x): direct dereference.
#define LCEC_PIN_BIT_SET(p, v) (*(p) = (v))
#define LCEC_PIN_BIT_GET(p) (*(p))
#define LCEC_PIN_FLOAT_SET(p, v) (*(p) = (v))
#define LCEC_PIN_FLOAT_GET(p) (*(p))
#define LCEC_PIN_S32_SET(p, v) (*(p) = (v))
#define LCEC_PIN_S32_GET(p) (*(p))
#define LCEC_PIN_U32_SET(p, v) (*(p) = (v))
#define LCEC_PIN_U32_GET(p) (*(p))
#endif

#endif  // _LCEC_HAL_COMPAT_H_
