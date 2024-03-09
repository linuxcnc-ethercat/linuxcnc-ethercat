//
//    Copyright (C) 2023 Scott Laird <scott@sigkill.org>
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
/// @brief Code for manipulating device definitions and finding devices by name

#include "lcec.h"

lcec_typelinkedlist_t *typeslist = NULL;

/// @brief Register a single slave type with LinuxCNC-Ethercat.
/// @param[in] type the definition of the device type to add.
void lcec_addtype(lcec_typelist_t *type, const char *sourcefile) {
  lcec_typelinkedlist_t *t, *l;

  // using malloc instead of hal_malloc because this can be called
  // from either lcec.so (inside of LinuxCNC) or lcec_conf (a
  // standalone binary).
  t = LCEC_ALLOCATE(lcec_typelinkedlist_t);

  type->sourcefile = sourcefile;
  t->type = type;
  t->next = NULL;

  if (typeslist == NULL) {
    typeslist = t;
  } else {
    // This should really validate that we don't have duplicate names,
    // but there's no good way to print error messages here.  If we're
    // running in lcec_main, then we would need to use different code
    // from lcec_conf.
    //
    // Note that if there are duplicate names, then the first match
    // found will win in lcec_findslavetype, below.
    for (l = typeslist; l->next != NULL; l = l->next)
      ;
    l->next = t;
  }
}

/// @brief Register an array of new slave types with LinuxCNC-Ethercat.
/// @param[in] types A list of types to add, terminated with a `NULL`.
void lcec_addtypes(lcec_typelist_t types[], const char *sourcefile) {
  lcec_typelist_t *type;

  for (type = types; type->name != NULL; type++) {
    lcec_addtype(type, sourcefile);
  }
}

/// @brief Find a slave type by name.
/// @param[in] name the name to find.
/// @returns a pointer to the `lcec_typelist_t` for the slave, or NULL if the type is not found.
const lcec_typelist_t *lcec_findslavetype(const char *name) {
  lcec_typelinkedlist_t *tl;

  for (tl = typeslist; tl != NULL && tl->type != NULL && strcmp(tl->type->name, name); tl = tl->next)
    ;

  if (tl != NULL) {
    return tl->type;
  }

  // Not found
  return NULL;
}
