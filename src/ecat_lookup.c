//
//    Copyright (C) 2024 Scott Laird <scott@sigkill.org>
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

#include <strings.h>

#include "ecat.h"

/// @file
/// @brief Code for implementing string lookup tables.
///
/// These are intended to make `<modParam>` decoding easier by
/// allowing generic string->int and string->double mapping, with
/// configurable default values.

/// @brief Perform a lookup in a lookup table.
///
/// This maps a string onto an int, looking in a predefined table.
/// This is intended to be used for looking up string->value maps in
/// `<modParam>`s, for example figuring out what the correct hardware
/// sensor type for a EL32xx is for a "Pt100" sensor.
///
/// @param table The table to do lookups in.
/// @param key The string to look for.
/// @param default_value The value to return if the lookup fails.
/// @return The value that matches the key, or default if it is not found.
int ecat_lookupint(const ecat_lookuptable_int_t *table, const char *key, int default_value) {
  while (table && table->key) {
    if (!strcmp(table->key, key)) {
      return table->value;
    }
    table++;
  }

  return default_value;
}

/// @brief Perform a case-insensitive lookup in a lookup table.
///
/// This maps a string onto an int, looking in a predefined table.
/// This is intended to be used for looking up string->value maps in
/// `<modParam>`s, for example figuring out what the correct hardware
/// sensor type for a EL32xx is for a "Pt100" sensor.
///
/// This version is case-insensitive.
///
/// @param table The table to do lookups in.
/// @param key The string to look for.
/// @param default_value The value to return if the lookup fails.
/// @return The value that matches the key, or default if it is not found.
int ecat_lookupint_i(const ecat_lookuptable_int_t *table, const char *key, int default_value) {
  while (table && table->key) {
    if (!strcasecmp(table->key, key)) {
      return table->value;
    }
    table++;
  }

  return default_value;
}

/// @brief Perform a lookup in a lookup table.
///
/// This maps a string onto an double, looking in a predefined table.
/// This is intended to be used for looking up string->value maps in
/// `<modParam>`s, for example figuring out what the correct hardware
/// sensor type for a EL32xx is for a "Pt100" sensor.
///
/// @param table The table to do lookups in.
/// @param key The string to look for.
/// @param default_value The value to return if the lookup fails.
/// @return The value that matches the key, or default if it is not found.
double ecat_lookupdouble(const ecat_lookuptable_double_t *table, const char *key, double default_value) {
  while (table && table->key) {
    if (!strcmp(table->key, key)) {
      return table->value;
    }
    table++;
  }

  return default_value;
}

/// @brief Perform a case-insensitive lookup in a lookup table.
///
/// This maps a string onto an double, looking in a predefined table.
/// This is intended to be used for looking up string->value maps in
/// `<modParam>`s, for example figuring out what the correct hardware
/// sensor type for a EL32xx is for a "Pt100" sensor.
///
/// This version is case-insensitive.
///
/// @param table The table to do lookups in.
/// @param key The string to look for.
/// @param default_value The value to return if the lookup fails.
/// @return The value that matches the key, or default if it is not found.
double ecat_lookupdouble_i(const ecat_lookuptable_double_t *table, const char *key, double default_value) {
  while (table && table->key) {
    if (!strcasecmp(table->key, key)) {
      return table->value;
    }
    table++;
  }

  return default_value;
}
