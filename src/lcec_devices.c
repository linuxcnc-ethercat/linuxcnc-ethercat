//
//  Copyright (C) 2012 Sascha Ittner <sascha.ittner@modusoft.de>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

/// @file
/// @brief Code for `lcec_devices' tool, which just dumps the list of supported devices to stdout.

#include <ctype.h>
#include <expat.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include "hal.h"
#include "lcec.h"
#include "lcec_conf.h"
#include "lcec_conf_priv.h"
#include "lcec_rtapi.h"
#include "rtapi.h"

extern lcec_typelinkedlist_t *typeslist;

int main(int argc, char **argv) {
  for (lcec_typelinkedlist_t *t = typeslist; t != NULL; t = t->next) {
    printf("%s\t0x%08x\t0x%08x\t%s\t", t->type->name, t->type->vid, t->type->pid, t->type->sourcefile);
    for (const lcec_modparam_desc_t *m = t->type->modparams; m && m->name != NULL; m++) {
      if (m->config_comment) printf("<!-- %s --> ", m->config_comment);
      if (m->config_value) printf("<modParam name=\"%s\" value=\"%s\"/> ", m->name, m->config_value);
    }
    printf("\n");
  }
}
