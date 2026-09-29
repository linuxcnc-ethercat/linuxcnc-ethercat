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
/// Detection - three HAL API states exist in the wild:
///
///   1. LinuxCNC 2.9.x: none of the markers below are defined.  Direct
///      dereference of `hal_bit_t *` etc., legacy pin/param creators.
///   2. Transitional 2.10 master (roughly 2026-07..2026-09, post-#4099,
///      pre-break): defines COMPONENT_TYPE_* and HAL_BOOL as macros, no
///      HAL_API_VERSION.  Typed creators and opaque references exist, but
///      the legacy `hal_*_t` data typedefs are still present (deprecated).
///   3. LinuxCNC 2.10.0~pre2 and later: defines HAL_API_VERSION 1.  The
///      legacy `hal_*_t` data typedefs, the legacy creators and the
///      `uses_fp` argument of hal_export_funct() are gone; HAL_BOOL is an
///      enum entry, not a macro.
///
/// State 3 removed only declarations; the getter/setter accessors and the
/// typed creators are identical to state 2, so one code path covers both.

#ifndef _LCEC_HAL_COMPAT_H_
#define _LCEC_HAL_COMPAT_H_

#include <hal.h>

#if defined(HAL_API_VERSION) && HAL_API_VERSION != 1
#error "Unsupported HAL_API_VERSION detected"
#endif

#if defined(HAL_API_VERSION) || defined(COMPONENT_TYPE_USER) || defined(HAL_BOOL)
#define LCEC_HAL_NEW_API 1

#if defined(HAL_API_VERSION)
// HAL_API_VERSION >= 1 removed the legacy data typedefs from hal.h.
// Reinstate them locally: lcec hal_data structs keep declaring pin storage
// with the old `hal_*_t *` pointer types (they hold opaque references via
// the casts below), and config structs/locals use the scalar types with
// their legacy widths.  These definitions match the pre-break upstream
// typedefs exactly (volatile bool / rtapi_s32 / rtapi_u32 / rtapi_real).
typedef volatile rtapi_bool hal_bit_t;
typedef volatile rtapi_s32 hal_s32_t;
typedef volatile rtapi_u32 hal_u32_t;
typedef volatile rtapi_real hal_float_t;

// hal_export_funct() lost its uses_fp argument at the same break.
#define LCEC_HAL_EXPORT_FUNCT(name, funct, arg, uses_fp, reentrant, comp_id) \
  hal_export_funct((name), (funct), (arg), (reentrant), (comp_id))
#else
#define LCEC_HAL_EXPORT_FUNCT(name, funct, arg, uses_fp, reentrant, comp_id) \
  hal_export_funct((name), (funct), (arg), (uses_fp), (reentrant), (comp_id))
#endif

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
#define LCEC_HAL_EXPORT_FUNCT(name, funct, arg, uses_fp, reentrant, comp_id) \
  hal_export_funct((name), (funct), (arg), (uses_fp), (reentrant), (comp_id))
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
