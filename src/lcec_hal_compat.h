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
/// Detection: upstream will add `#define HAL_API_VERSION 1` to hal.h once the
/// getter/setter + query API are in place (per B. Stultiens).  Until that
/// lands, fall back to COMPONENT_TYPE_* (define aliases added to hal.h with
/// the query API) or HAL_BOOL (temporary, will become an enum entry).
/// Pre-#4099 hal.h defines none of these.

#ifndef _LCEC_HAL_COMPAT_H_
#define _LCEC_HAL_COMPAT_H_

#include <hal.h>

#if defined(HAL_API_VERSION) && HAL_API_VERSION != 1
#error "Unsupported HAL_API_VERSION detected"
#endif

#if defined(HAL_API_VERSION) || defined(COMPONENT_TYPE_USER) || defined(HAL_BOOL)
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

// Params. Storage is HAL-owned on the new API (the creators return an
// opaque reference), but caller-provided struct fields on the old API.
// The typedefs below make param fields a reference on the new API and a
// value on the old API; all access must go through LCEC_PARAM_*.
typedef hal_bool_t lcec_param_bit_t;
typedef hal_real_t lcec_param_float_t;
typedef hal_sint_t lcec_param_s32_t;
typedef hal_uint_t lcec_param_u32_t;
#define LCEC_PARAM_BIT_SET(f, v) hal_set_bool((f), (v))
#define LCEC_PARAM_BIT_GET(f) hal_get_bool((f))
#define LCEC_PARAM_FLOAT_SET(f, v) hal_set_real((f), (v))
#define LCEC_PARAM_FLOAT_GET(f) hal_get_real((f))
#define LCEC_PARAM_S32_SET(f, v) hal_set_si32((f), (v))
#define LCEC_PARAM_S32_GET(f) hal_get_si32((f))
#define LCEC_PARAM_U32_SET(f, v) hal_set_ui32((f), (v))
#define LCEC_PARAM_U32_GET(f) hal_get_ui32((f))
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

// Old API: params are plain value fields in the component's hal_data.
typedef hal_bit_t lcec_param_bit_t;
typedef hal_float_t lcec_param_float_t;
typedef hal_s32_t lcec_param_s32_t;
typedef hal_u32_t lcec_param_u32_t;
#define LCEC_PARAM_BIT_SET(f, v) ((f) = (v))
#define LCEC_PARAM_BIT_GET(f) (f)
#define LCEC_PARAM_FLOAT_SET(f, v) ((f) = (v))
#define LCEC_PARAM_FLOAT_GET(f) (f)
#define LCEC_PARAM_S32_SET(f, v) ((f) = (v))
#define LCEC_PARAM_S32_GET(f) (f)
#define LCEC_PARAM_U32_SET(f, v) ((f) = (v))
#define LCEC_PARAM_U32_GET(f) (f)
#endif

// Type-dispatching variants for macro-generated code where the pin type
// varies per instantiation (e.g. lcec_class_cia402).  Dispatch is on the
// declared field pointer type; must be revisited if the field declarations
// move to the opaque new-API reference types.
#define LCEC_PIN_GET(p)                         \
  _Generic((p),                                 \
      hal_bit_t *: LCEC_PIN_BIT_GET(p),         \
      hal_float_t *: LCEC_PIN_FLOAT_GET(p),     \
      hal_s32_t *: LCEC_PIN_S32_GET(p),         \
      hal_u32_t *: LCEC_PIN_U32_GET(p))
#define LCEC_PIN_SET(p, v)                      \
  _Generic((p),                                 \
      hal_bit_t *: LCEC_PIN_BIT_SET(p, v),      \
      hal_float_t *: LCEC_PIN_FLOAT_SET(p, v),  \
      hal_s32_t *: LCEC_PIN_S32_SET(p, v),      \
      hal_u32_t *: LCEC_PIN_U32_SET(p, v))

#endif  // _LCEC_HAL_COMPAT_H_
