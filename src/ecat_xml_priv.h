//
//    Copyright (C) 2011 Sascha Ittner <sascha.ittner@modusoft.de>
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

#ifndef _ECAT_CONF_PRIV_H_
#define _ECAT_CONF_PRIV_H_

#include <expat.h>

#define BUFFSIZE 8192

struct ECAT_CONF_XML_HANLDER;

typedef struct ECAT_CONF_XML_INST {
  XML_Parser parser;
  const struct ECAT_CONF_XML_HANLDER *states;
  int state;
} ECAT_CONF_XML_INST_T;

typedef struct ECAT_CONF_XML_HANLDER {
  const char *el;
  int state_from;
  int state_to;
  void (*start_handler)(struct ECAT_CONF_XML_INST *inst, int next, const char **attr);
  void (*end_handler)(struct ECAT_CONF_XML_INST *inst, int next);
} ECAT_CONF_XML_HANLDER_T;

typedef struct ECAT_CONF_OUTBUF_ITEM {
  size_t len;
  struct ECAT_CONF_OUTBUF_ITEM *next;
} ECAT_CONF_OUTBUF_ITEM_T;

typedef struct {
  ECAT_CONF_OUTBUF_ITEM_T *head;
  ECAT_CONF_OUTBUF_ITEM_T *tail;
  size_t len;
} ECAT_CONF_OUTBUF_T;

extern const char *modname;

#define ADD_OUTPUT_BUFFER(buf, type) ((type *)addOutputBuffer(buf, sizeof(type)))

void initOutputBuffer(ECAT_CONF_OUTBUF_T *buf);
void *addOutputBuffer(ECAT_CONF_OUTBUF_T *buf, size_t len);
void copyFreeOutputBuffer(ECAT_CONF_OUTBUF_T *buf, char *dest);

int parseIcmds(ECAT_CONF_SLAVE_T *slave, ECAT_CONF_OUTBUF_T *outputBuf, const char *filename);

int initXmlInst(ECAT_CONF_XML_INST_T *inst, const ECAT_CONF_XML_HANLDER_T *states);

int parseHex(const char *s, int slen, uint8_t *buf);

#endif
