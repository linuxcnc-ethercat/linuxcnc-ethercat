//
//    Copyright (C) 2012 Sascha Ittner <sascha.ittner@modusoft.de>
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
/// @brief Modparam-handling library code

#include "lcec.h"

/// @brief Cound the number of entries in a `lcec_modparam_desc_t[]`.
int lcec_modparam_desc_len(const lcec_modparam_desc_t *mp) {
  int l;

  if (mp == NULL) return 0;

  for (l = 0; mp[l].name != NULL; l++)
    ;

  return l;
}

lcec_modparam_desc_t *lcec_modparam_desc_concat(lcec_modparam_desc_t const *a, lcec_modparam_desc_t const *b) {
  int a_len, b_len, i;
  lcec_modparam_desc_t *c;

  a_len = lcec_modparam_desc_len(a);
  b_len = lcec_modparam_desc_len(b);

  c = LCEC_ALLOCATE_ARRAY(lcec_modparam_desc_t, (a_len + b_len + 1));

  for (i = 0; i < a_len; i++) {
    c[i] = a[i];
  }
  for (i = 0; i < b_len; i++) {
    c[a_len + i] = b[i];
  }
  c[a_len + b_len] = a[a_len];  // Copy terminator

  return c;
}

/// @brief Override the config value for entries in a with values from b.
lcec_modparam_desc_t *lcec_modparam_desc_override(lcec_modparam_desc_t const *a, lcec_modparam_desc_t const *b) {
  int a_len, b_len, i, j;
  lcec_modparam_desc_t const *aa, *bb;
  lcec_modparam_desc_t *c, *cc;

  a_len = lcec_modparam_desc_len(a);
  b_len = lcec_modparam_desc_len(b);

  // Duplicate a into c
  c = LCEC_ALLOCATE_ARRAY(lcec_modparam_desc_t, a_len + 1);

  for (j = 0; j < a_len; j++) {
    aa = a + j;
    cc = c + j;
    cc->name = aa->name;
    cc->id = aa->id;
    cc->type = aa->type;
    if (aa->config_value) cc->config_value = aa->config_value;
    if (aa->config_comment) cc->config_comment = aa->config_comment;
  }

  for (i = 0; i < b_len; i++) {
    bb = b + i;
    if (bb->config_value || bb->config_comment) {
      for (j = 0; j < a_len; j++) {
        cc = c + j;
        if (strcmp(cc->name, bb->name) == 0) {
          if (bb->config_value) cc->config_value = bb->config_value;
          if (bb->config_comment) cc->config_comment = bb->config_comment;
        }
      }
    }
  }

  return c;
}
