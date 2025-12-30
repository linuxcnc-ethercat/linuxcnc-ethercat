//
//  Copyright (C) 2018 Sascha Ittner <sascha.ittner@modusoft.de>
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

#include <ctype.h>
#include <expat.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "ecat_xml.h"
#include "ecat_xml_priv.h"

typedef enum {
  icmdTypeNone = 0,
  icmdTypeMailbox,
  icmdTypeCoe,
  icmdTypeCoeIcmds,
  icmdTypeCoeIcmd,
  icmdTypeCoeIcmdTrans,
  icmdTypeCoeIcmdComment,
  icmdTypeCoeIcmdTimeout,
  icmdTypeCoeIcmdCcs,
  icmdTypeCoeIcmdIndex,
  icmdTypeCoeIcmdSubindex,
  icmdTypeCoeIcmdData,
  icmdTypeSoe,
  icmdTypeSoeIcmds,
  icmdTypeSoeIcmd,
  icmdTypeSoeIcmdTrans,
  icmdTypeSoeIcmdComment,
  icmdTypeSoeIcmdTimeout,
  icmdTypeSoeIcmdOpcode,
  icmdTypeSoeIcmdDriveno,
  icmdTypeSoeIcmdIdn,
  icmdTypeSoeIcmdElements,
  icmdTypeSoeIcmdAttribute,
  icmdTypeSoeIcmdData
} ECAT_ICMD_TYPE_T;

typedef struct {
  ECAT_CONF_XML_INST_T xml;

  ECAT_CONF_SLAVE_T *currSlave;
  ECAT_CONF_OUTBUF_T *outputBuf;

  ECAT_CONF_SDOCONF_T *currSdoConf;
  ECAT_CONF_IDNCONF_T *currIdnConf;
} ECAT_CONF_ICMDS_STATE_T;

static void xml_data_handler(void *data, const XML_Char *s, int len);

static void icmdTypeCoeIcmdStart(ECAT_CONF_XML_INST_T *inst, int next, const char **attr);
static void icmdTypeCoeIcmdEnd(ECAT_CONF_XML_INST_T *inst, int next);
static void icmdTypeSoeIcmdStart(ECAT_CONF_XML_INST_T *inst, int next, const char **attr);
static void icmdTypeSoeIcmdEnd(ECAT_CONF_XML_INST_T *inst, int next);

static const ECAT_CONF_XML_HANLDER_T xml_states[] = {
    {"EtherCATMailbox", icmdTypeNone, icmdTypeMailbox, NULL, NULL},
    {"CoE", icmdTypeMailbox, icmdTypeCoe, NULL, NULL},
    {"InitCmds", icmdTypeCoe, icmdTypeCoeIcmds, NULL, NULL},
    {"InitCmd", icmdTypeCoeIcmds, icmdTypeCoeIcmd, icmdTypeCoeIcmdStart, icmdTypeCoeIcmdEnd},
    {"Transition", icmdTypeCoeIcmd, icmdTypeCoeIcmdTrans, NULL, NULL},
    {"Comment", icmdTypeCoeIcmd, icmdTypeCoeIcmdComment, NULL, NULL},
    {"Timeout", icmdTypeCoeIcmd, icmdTypeCoeIcmdTimeout, NULL, NULL},
    {"Ccs", icmdTypeCoeIcmd, icmdTypeCoeIcmdCcs, NULL, NULL},
    {"Index", icmdTypeCoeIcmd, icmdTypeCoeIcmdIndex, NULL, NULL},
    {"SubIndex", icmdTypeCoeIcmd, icmdTypeCoeIcmdSubindex, NULL, NULL},
    {"Data", icmdTypeCoeIcmd, icmdTypeCoeIcmdData, NULL, NULL},
    {"SoE", icmdTypeMailbox, icmdTypeSoe, NULL, NULL},
    {"InitCmds", icmdTypeSoe, icmdTypeSoeIcmds, NULL, NULL},
    {"InitCmd", icmdTypeSoeIcmds, icmdTypeSoeIcmd, icmdTypeSoeIcmdStart, icmdTypeSoeIcmdEnd},
    {"Transition", icmdTypeSoeIcmd, icmdTypeSoeIcmdTrans, NULL, NULL},
    {"Comment", icmdTypeSoeIcmd, icmdTypeSoeIcmdComment, NULL, NULL},
    {"Timeout", icmdTypeSoeIcmd, icmdTypeSoeIcmdTimeout, NULL, NULL},
    {"OpCode", icmdTypeSoeIcmd, icmdTypeSoeIcmdOpcode, NULL, NULL},
    {"DriveNo", icmdTypeSoeIcmd, icmdTypeSoeIcmdDriveno, NULL, NULL},
    {"IDN", icmdTypeSoeIcmd, icmdTypeSoeIcmdIdn, NULL, NULL},
    {"Elements", icmdTypeSoeIcmd, icmdTypeSoeIcmdElements, NULL, NULL},
    {"Attribute", icmdTypeSoeIcmd, icmdTypeSoeIcmdAttribute, NULL, NULL},
    {"Data", icmdTypeSoeIcmd, icmdTypeSoeIcmdData, NULL, NULL},
    {"NULL", -1, -1, NULL, NULL},
};

static long int parse_int(ECAT_CONF_ICMDS_STATE_T *state, const char *s, unsigned int len, long int min, long int max);
static int parse_data(ECAT_CONF_ICMDS_STATE_T *state, const char *s, int len);

int parseIcmds(ECAT_CONF_SLAVE_T *slave, ECAT_CONF_OUTBUF_T *outputBuf, const char *filename) {
  int ret = 1;
  int done;
  char buffer[BUFFSIZE];
  FILE *file;
  ECAT_CONF_ICMDS_STATE_T state;

  // open file
  file = fopen(filename, "r");
  if (file == NULL) {
    fprintf(stderr, "%s: ERROR: unable to open config file %s\n", modname, filename);
    goto fail1;
  }

  // create xml parser
  memset(&state, 0, sizeof(state));
  if (initXmlInst((ECAT_CONF_XML_INST_T *)&state, xml_states)) {
    fprintf(stderr, "%s: ERROR: Couldn't allocate memory for parser\n", modname);
    goto fail2;
  }

  // setup handlers
  XML_SetCharacterDataHandler(state.xml.parser, xml_data_handler);

  state.currSlave = slave;
  state.outputBuf = outputBuf;
  for (done = 0; !done;) {
    // read block
    int len = fread(buffer, 1, BUFFSIZE, file);
    if (ferror(file)) {
      fprintf(stderr, "%s: ERROR: Couldn't read from file %s\n", modname, filename);
      goto fail3;
    }

    // check for EOF
    done = feof(file);

    // parse current block
    if (!XML_Parse(state.xml.parser, buffer, len, done)) {
      fprintf(stderr, "%s: ERROR: Parse error at line %u: %s\n", modname, (unsigned int)XML_GetCurrentLineNumber(state.xml.parser),
          XML_ErrorString(XML_GetErrorCode(state.xml.parser)));
      goto fail3;
    }
  }

  // everything is fine
  ret = 0;

fail3:
  XML_ParserFree(state.xml.parser);
fail2:
  fclose(file);
fail1:
  return ret;
}

static void xml_data_handler(void *data, const XML_Char *s, int len) {
  ECAT_CONF_XML_INST_T *inst = (ECAT_CONF_XML_INST_T *)data;
  ECAT_CONF_ICMDS_STATE_T *state = (ECAT_CONF_ICMDS_STATE_T *)inst;

  switch (inst->state) {
    case icmdTypeCoeIcmdTrans:
      if (len == 2) {
        if (strncmp("IP", s, len) == 0) {
          return;
        }
        if (strncmp("PS", s, len) == 0) {
          return;
        }
      }
      fprintf(stderr, "%s: ERROR: Invalid Transition state\n", modname);
      XML_StopParser(inst->parser, 0);
      return;
    case icmdTypeCoeIcmdIndex:
      state->currSdoConf->index = parse_int(state, s, len, 0, 0xffff);
      return;
    case icmdTypeCoeIcmdSubindex:
      if (state->currSdoConf->subindex != ECAT_CONF_SDO_COMPLETE_SUBIDX) {
        state->currSdoConf->subindex = parse_int(state, s, len, 0, 0xff);
      }
      return;
    case icmdTypeCoeIcmdData:
      state->currSdoConf->length += parse_data(state, s, len);
      return;

    case icmdTypeSoeIcmdTrans:
      if (len == 2) {
        if (strncmp("IP", s, len) == 0) {
          state->currIdnConf->state = EC_AL_STATE_PREOP;
          return;
        }
        if (strncmp("PS", s, len) == 0) {
          state->currIdnConf->state = EC_AL_STATE_PREOP;
          return;
        }
        if (strncmp("SO", s, len) == 0) {
          state->currIdnConf->state = EC_AL_STATE_SAFEOP;
          return;
        }
      }
      fprintf(stderr, "%s: ERROR: Invalid Transition state\n", modname);
      XML_StopParser(inst->parser, 0);
      return;
    case icmdTypeSoeIcmdDriveno:
      state->currIdnConf->drive = parse_int(state, s, len, 0, 7);
      return;
    case icmdTypeSoeIcmdIdn:
      state->currIdnConf->idn = parse_int(state, s, len, 0, 0xffff);
      return;
    case icmdTypeSoeIcmdData:
      state->currIdnConf->length += parse_data(state, s, len);
      return;
  }
}

static void icmdTypeCoeIcmdStart(ECAT_CONF_XML_INST_T *inst, int next, const char **attr) {
  ECAT_CONF_ICMDS_STATE_T *state = (ECAT_CONF_ICMDS_STATE_T *)inst;

  state->currSdoConf = ADD_OUTPUT_BUFFER(state->outputBuf, ECAT_CONF_SDOCONF_T);

  if (state->currSdoConf == NULL) {
    XML_StopParser(inst->parser, 0);
    return;
  }

  state->currSdoConf->confType = lcecConfTypeSdoConfig;
  state->currSdoConf->index = 0xffff;
  state->currSdoConf->subindex = 0xff;

  while (*attr) {
    const char *name = *(attr++);
    const char *val = *(attr++);

    // parse CompleteAccess
    if (strcmp(name, "CompleteAccess") == 0) {
      if (atoi(val)) {
        state->currSdoConf->subindex = ECAT_CONF_SDO_COMPLETE_SUBIDX;
      }
      continue;
    }
  }
}

static void icmdTypeCoeIcmdEnd(ECAT_CONF_XML_INST_T *inst, int next) {
  ECAT_CONF_ICMDS_STATE_T *state = (ECAT_CONF_ICMDS_STATE_T *)inst;

  if (state->currSdoConf->index == 0xffff) {
    fprintf(stderr, "%s: ERROR: sdoConfig has no idx attribute\n", modname);
    XML_StopParser(inst->parser, 0);
    return;
  }

  if (state->currSdoConf->subindex == 0xff) {
    fprintf(stderr, "%s: ERROR: sdoConfig has no subIdx attribute\n", modname);
    XML_StopParser(inst->parser, 0);
    return;
  }

  state->currSlave->sdoConfigLength += sizeof(ECAT_CONF_SDOCONF_T) + state->currSdoConf->length;
}

static void icmdTypeSoeIcmdStart(ECAT_CONF_XML_INST_T *inst, int next, const char **attr) {
  ECAT_CONF_ICMDS_STATE_T *state = (ECAT_CONF_ICMDS_STATE_T *)inst;

  state->currIdnConf = ADD_OUTPUT_BUFFER(state->outputBuf, ECAT_CONF_IDNCONF_T);

  if (state->currIdnConf == NULL) {
    XML_StopParser(inst->parser, 0);
    return;
  }

  state->currIdnConf->confType = lcecConfTypeIdnConfig;
  state->currIdnConf->drive = 0;
  state->currIdnConf->idn = 0xffff;
  state->currIdnConf->state = (ec_al_state_t)0;
}

static void icmdTypeSoeIcmdEnd(ECAT_CONF_XML_INST_T *inst, int next) {
  ECAT_CONF_ICMDS_STATE_T *state = (ECAT_CONF_ICMDS_STATE_T *)inst;

  if (state->currIdnConf->idn == 0xffff) {
    fprintf(stderr, "%s: ERROR: idnConfig has no idn attribute\n", modname);
    XML_StopParser(inst->parser, 0);
    return;
  }

  if (state->currIdnConf->state == 0) {
    fprintf(stderr, "%s: ERROR: idnConfig has no state attribute\n", modname);
    XML_StopParser(inst->parser, 0);
    return;
  }

  state->currSlave->idnConfigLength += sizeof(ECAT_CONF_IDNCONF_T) + state->currIdnConf->length;
}

static long int parse_int(ECAT_CONF_ICMDS_STATE_T *state, const char *s, unsigned int len, long int min, long int max) {
  char buf[32];
  char *end;
  long int ret;

  if (s == NULL || len == 0) {
    fprintf(stderr, "%s: ERROR: Missing number value\n", modname);
    XML_StopParser(state->xml.parser, 0);
    return 0;
  }

  if (len >= sizeof(buf)) {
    fprintf(stderr, "%s: ERROR: Number value size exceeded\n", modname);
    XML_StopParser(state->xml.parser, 0);
    return 0;
  }

  strncpy(buf, s, len);
  buf[len] = 0;

  ret = strtol(buf, &end, 0);
  if (*end != 0 || ret < min || ret > max) {
    fprintf(stderr, "%s: ERROR: Invalid number value '%s'\n", modname, s);
    XML_StopParser(state->xml.parser, 0);
    return 0;
  }

  return ret;
}

static int parse_data(ECAT_CONF_ICMDS_STATE_T *state, const char *s, int len) {
  uint8_t *p;
  int size;

  // get size
  size = parseHex(s, len, NULL);
  if (size < 0) {
    fprintf(stderr, "%s: ERROR: Invalid data\n", modname);
    XML_StopParser(state->xml.parser, 0);
    return 0;
  }

  // allocate memory
  p = (uint8_t *)addOutputBuffer(state->outputBuf, size);
  if (p == NULL) {
    XML_StopParser(state->xml.parser, 0);
    return 0;
  }

  // parse data
  parseHex(s, len, p);
  return size;
}
