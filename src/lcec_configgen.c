//
//    Copyright (C) 2026 Luca Toniolo
//    Based on the Go implementation by Scott Laird.
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//

/// @file
/// @brief Generates a LinuxCNC-Ethercat XML config skeleton from a live bus.
///
/// Calls `ethercat slaves`, `ethercat sdos`, `ethercat pdos`, and
/// `ethercat upload` as subprocesses, parses their output, looks up
/// known drivers in the in-process device registry (linked from
/// `liblcecdevices.a`), and emits XML to stdout.
///
/// This is a C port of the previous Go implementation
/// (`src/configgen/lcec_configgen.go`).

#include <ctype.h>
#include <getopt.h>
#include <regex.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "hal.h"
#include "lcec.h"
#include "lcec_conf.h"
#include "lcec_rtapi.h"
#include "rtapi.h"

extern lcec_typelinkedlist_t *typeslist;

// ===== flags =====
static int flag_typedb = 1;
static int flag_extra_cia_modparams = 0;
static int flag_generic_pdos = 1;

// ===== dynamic helpers =====

static void *xmalloc(size_t n) {
  void *p = calloc(1, n);
  if (!p) {
    fprintf(stderr, "lcec_configgen: out of memory\n");
    exit(1);
  }
  return p;
}

static char *xstrdup(const char *s) {
  if (!s) return NULL;
  char *r = strdup(s);
  if (!r) {
    fprintf(stderr, "lcec_configgen: out of memory\n");
    exit(1);
  }
  return r;
}

static char *xasprintf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  char *r = NULL;
  if (vasprintf(&r, fmt, ap) < 0) r = NULL;
  va_end(ap);
  if (!r) {
    fprintf(stderr, "lcec_configgen: vasprintf failed\n");
    exit(1);
  }
  return r;
}

// trim trailing whitespace in place
static void rtrim(char *s) {
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r' || s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = 0;
}

// case-insensitive lowercase ascii in place
static void to_lower(char *s) {
  for (; *s; s++) *s = (char)tolower((unsigned char)*s);
}

// ===== data model =====

typedef struct {
  char *idx;   // "0x6040:00"
  char *type;  // "uint16"
} sdo_t;

typedef struct {
  char *master;
  char *slave;
  char *vendor_id;
  char *product_id;
  char *revision_no;
  char *device_name;
  sdo_t *sdos;
  size_t sdo_count, sdo_cap;
} ec_slave_t;

// SDO map lookup: returns type string or "" if absent.
static const char *sdo_get(const ec_slave_t *s, const char *idx) {
  for (size_t i = 0; i < s->sdo_count; i++) {
    if (strcmp(s->sdos[i].idx, idx) == 0) return s->sdos[i].type;
  }
  return "";
}

static void sdo_put(ec_slave_t *s, const char *idx, const char *type) {
  if (s->sdo_count == s->sdo_cap) {
    s->sdo_cap = s->sdo_cap ? s->sdo_cap * 2 : 16;
    s->sdos = realloc(s->sdos, s->sdo_cap * sizeof(sdo_t));
    if (!s->sdos) {
      fprintf(stderr, "lcec_configgen: oom\n");
      exit(1);
    }
  }
  s->sdos[s->sdo_count].idx = xstrdup(idx);
  s->sdos[s->sdo_count].type = xstrdup(type);
  s->sdo_count++;
}

typedef enum { MP_PARAM, MP_COMMENT, MP_BLANK } mp_kind_t;

typedef struct {
  mp_kind_t kind;
  char *name;     // for PARAM
  char *value;    // for PARAM
  char *comment;  // for COMMENT
} modparam_t;

typedef struct {
  char *idx;     // "6000"
  char *subidx;  // "01"
  char *bitlen;
  char *halpin;
  char *haltype;
  char *comment;  // unused after fixup, kept for parity
} pdo_entry_t;

typedef struct {
  char *idx;   // "1600"
  char *comment;
  pdo_entry_t **entries;
  size_t entry_count, entry_cap;
} pdo_t;

typedef struct {
  char *idx;  // "0".."3"
  char *dir;  // "in"/"out"/""
  pdo_t **pdos;
  size_t pdo_count, pdo_cap;
} sm_t;

typedef struct {
  char *idx;
  char *type;
  char *vid;       // empty unless generic/basic_cia402
  char *pid;       // empty unless generic/basic_cia402
  char *name;      // "D1", "D2", ...
  char *comment;   // device name, only set for generic/basic_cia402
  modparam_t **modparams;
  size_t mp_count, mp_cap;
  sm_t **sms;
  size_t sm_count, sm_cap;
} slave_config_t;

typedef struct {
  char *idx;
  slave_config_t **slaves;
  size_t slave_count, slave_cap;
} master_config_t;

// ===== compiled regexes =====

static regex_t re_slave_master, re_slave_vendor, re_slave_product, re_slave_revision, re_slave_devname;
static regex_t re_sdo;
static regex_t re_pdo_sm, re_pdo_pdo, re_pdo_entry;
static regex_t re_pin_chars;

static void compile_re(regex_t *r, const char *pat) {
  int rc = regcomp(r, pat, REG_EXTENDED);
  if (rc) {
    char buf[256];
    regerror(rc, r, buf, sizeof(buf));
    fprintf(stderr, "lcec_configgen: regcomp failed for %s: %s\n", pat, buf);
    exit(1);
  }
}

static void compile_all_regexes(void) {
  compile_re(&re_slave_master, "^=== Master ([0-9]+), Slave ([0-9]+) ===$");
  compile_re(&re_slave_vendor, "^  Vendor Id: +(0x[0-9a-fA-F]+)");
  compile_re(&re_slave_product, "^  Product code: +(0x[0-9a-fA-F]+)");
  compile_re(&re_slave_revision, "^  Revision number: +(0x[0-9a-fA-F]+)");
  compile_re(&re_slave_devname, "^  Device name: (.*)");
  compile_re(&re_sdo, "^  (0x[0-9a-fA-F]{4}:[0-9a-fA-F]{2}), [rw-]+, ([^,]+),");
  compile_re(&re_pdo_sm, "^SM([0-9]+): PhysAddr (0x[0-9a-f]+).*");
  compile_re(&re_pdo_pdo, "^  ([RT]xPDO) (0x[0-9a-f]+) \"(.*)\"");
  compile_re(&re_pdo_entry, "^    PDO entry (0x[0-9a-f]+):([0-9a-f]+), +([0-9]+) bit, \"(.*)\"");
  compile_re(&re_pin_chars, "[^a-z0-9]+");
}

// returns malloced substring of haystack [m.rm_so, m.rm_eo)
static char *re_capture(const char *line, regmatch_t m) {
  size_t len = (size_t)(m.rm_eo - m.rm_so);
  char *out = xmalloc(len + 1);
  memcpy(out, line + m.rm_so, len);
  out[len] = 0;
  return out;
}

// ===== subprocess helpers =====

static FILE *run_ethercat(const char *args) {
  // Build argv via /bin/sh for simplicity. Caller passes whitespace-quoted args.
  char *cmd = xasprintf("ethercat %s", args);
  FILE *f = popen(cmd, "r");
  if (!f) {
    fprintf(stderr, "lcec_configgen: popen failed: %s\n", cmd);
    free(cmd);
    exit(1);
  }
  free(cmd);
  return f;
}

// ===== readers =====

static ec_slave_t *slaves_arr = NULL;
static size_t slaves_count = 0, slaves_cap = 0;

static void slaves_push(const ec_slave_t *s) {
  if (slaves_count == slaves_cap) {
    slaves_cap = slaves_cap ? slaves_cap * 2 : 8;
    slaves_arr = realloc(slaves_arr, slaves_cap * sizeof(ec_slave_t));
    if (!slaves_arr) exit(1);
  }
  slaves_arr[slaves_count++] = *s;
}

static void read_slaves(void) {
  FILE *f = run_ethercat("-v slaves");
  char *line = NULL;
  size_t cap = 0;
  ssize_t n;
  ec_slave_t cur = {0};
  regmatch_t m[3];

  while ((n = getline(&line, &cap, f)) >= 0) {
    rtrim(line);

    if (!regexec(&re_slave_master, line, 3, m, 0)) {
      if (cur.slave) {
        // previous slave had no Device name line; flush it.
        slaves_push(&cur);
        memset(&cur, 0, sizeof(cur));
      }
      cur.master = re_capture(line, m[1]);
      cur.slave = re_capture(line, m[2]);
    } else if (!regexec(&re_slave_vendor, line, 2, m, 0)) {
      cur.vendor_id = re_capture(line, m[1]);
    } else if (!regexec(&re_slave_product, line, 2, m, 0)) {
      cur.product_id = re_capture(line, m[1]);
    } else if (!regexec(&re_slave_revision, line, 2, m, 0)) {
      cur.revision_no = re_capture(line, m[1]);
    } else if (!regexec(&re_slave_devname, line, 2, m, 0)) {
      cur.device_name = re_capture(line, m[1]);
      slaves_push(&cur);
      memset(&cur, 0, sizeof(cur));
    }
  }
  free(line);
  pclose(f);
}

static void read_sdos(ec_slave_t *s) {
  char *args = xasprintf("-m %s sdos -p %s", s->master, s->slave);
  FILE *f = run_ethercat(args);
  free(args);

  char *line = NULL;
  size_t cap = 0;
  ssize_t n;
  regmatch_t m[3];

  while ((n = getline(&line, &cap, f)) >= 0) {
    rtrim(line);
    if (!regexec(&re_sdo, line, 3, m, 0)) {
      char *idx = re_capture(line, m[1]);
      char *type = re_capture(line, m[2]);
      sdo_put(s, idx, type);
      free(idx);
      free(type);
    }
  }
  free(line);
  pclose(f);
}

// Read a single SDO via `ethercat upload`. Returns the integer value, or -1 on error.
static int64_t read_sdo_int(const char *master, const char *slave, int idx, int sub) {
  char *args = xasprintf("-m %s upload -p %s 0x%04x 0x%02x", master, slave, idx, sub);
  FILE *f = run_ethercat(args);
  free(args);

  char buf[256];
  if (!fgets(buf, sizeof(buf), f)) {
    pclose(f);
    return -1;
  }
  pclose(f);
  // Expected format: "0xXXXX NNN" (hex space decimal)
  return strtoll(buf, NULL, 0);
}

// ===== CiA 402 helpers =====

static bool is_cia402(const ec_slave_t *s) {
  return *sdo_get(s, "0x6040:00") && *sdo_get(s, "0x6041:00") && *sdo_get(s, "0x6502:00");
}

static int cia_channels(const ec_slave_t *s) {
  static const char *probes[] = {"0x6502:00", "0x6d02:00", "0x7502:00", "0x7d02:00",
                                 "0x8502:00", "0x8d02:00", "0x9502:00", "0x9d02:00"};
  int n = 0;
  for (size_t i = 0; i < sizeof(probes) / sizeof(probes[0]); i++) {
    if (*sdo_get(s, probes[i]))
      n++;
    else
      break;
  }
  return n;
}

static int cia_pdo_entries(const ec_slave_t *s, int base_idx) {
  for (int e = 32; e > 8; e--) {
    char buf[16];
    snprintf(buf, sizeof(buf), "0x%04x:%02d", base_idx, e);
    if (*sdo_get(s, buf)) return e;
  }
  return 8;
}

typedef struct {
  const char *name;
  int offset;
  int subindex;
} enable_sdo_t;

static const enable_sdo_t enable_sdos[] = {
    {"enableActualCurrent", 0x78, 0},
    {"enableActualFollowingError", 0xf4, 0},
    {"enableActualTorque", 0x77, 0},
    {"enableActualVelocitySensor", 0x69, 0},
    {"enableActualVoltage", 0x79, 0},
    {"enableControlEffort", 0xfa, 0},
    {"enableDigitalInput", 0xfd, 0},
    {"enableDigitalOutput", 0xfe, 1},
    {"enableErrorCode", 0x3f, 0},
    {"enableFollowingErrorTimeout", 0x66, 0},
    {"enableFollowingErrorWindow", 0x65, 0},
    {"enableHomeAccel", 0x9a, 0},
    {"enableInterpolationTimePeriod", 0xc2, 1},
    {"enableMaximumAcceleration", 0xc6, 0},
    {"enableMaximumCurrent", 0x73, 0},
    {"enableMaximumDeceleration", 0xc6, 0},
    {"enableMaximumMotorRPM", 0x80, 0},
    {"enableMaximumSlippage", 0xf8, 0},
    {"enableMaximumTorque", 0x72, 0},
    {"enableMotorRatedCurrent", 0x75, 0},
    {"enableMotorRatedTorque", 0x76, 0},
    {"enablePolarity", 0x7e, 0},
    {"enablePositionDemand", 0x62, 0},
    {"enablePositioningTime", 0x68, 0},
    {"enablePositioningWindow", 0x67, 0},
    {"enableProbeStatus", 0xb9, 0},
    {"enableProfileAccel", 0x83, 0},
    {"enableProfileDecel", 0x84, 0},
    {"enableProfileEndVelocity", 0x82, 0},
    {"enableProfileMaxVelocity", 0x7f, 0},
    {"enableProfileVelocity", 0x81, 0},
    {"enableTargetTorque", 0x71, 0},
    {"enableTargetVL", 0x42, 0},
    {"enableTorqueDemand", 0x74, 0},
    {"enableTorqueProfileType", 0x88, 0},
    {"enableTorqueSlope", 0x87, 0},
    {"enableVLAccel", 0x48, 0},
    {"enableVLDecel", 0x49, 0},
    {"enableVLDemand", 0x43, 0},
    {"enableVLMaximum", 0x46, 2},
    {"enableVLMinimum", 0x46, 1},
    {"enableVelocityDemand", 0x6b, 0},
    {"enableVelocityErrorTime", 0x6e, 0},
    {"enableVelocityErrorWindow", 0x6d, 0},
    {"enableVelocitySensorSelector", 0x6a, 0},
    {"enableVelocityThresholdTime", 0x70, 0},
    {"enableVelocityThresholdWindow", 0x6f, 0},
};

// ===== modparam list helpers =====

static void mp_push(slave_config_t *sc, modparam_t *mp) {
  if (sc->mp_count == sc->mp_cap) {
    sc->mp_cap = sc->mp_cap ? sc->mp_cap * 2 : 8;
    sc->modparams = realloc(sc->modparams, sc->mp_cap * sizeof(modparam_t *));
  }
  sc->modparams[sc->mp_count++] = mp;
}

static modparam_t *mp_param(const char *name, const char *value) {
  modparam_t *m = xmalloc(sizeof(*m));
  m->kind = MP_PARAM;
  m->name = xstrdup(name);
  m->value = xstrdup(value);
  return m;
}
static modparam_t *mp_comment(const char *text) {
  modparam_t *m = xmalloc(sizeof(*m));
  m->kind = MP_COMMENT;
  m->comment = xstrdup(text);
  return m;
}
static modparam_t *mp_blank(void) {
  modparam_t *m = xmalloc(sizeof(*m));
  m->kind = MP_BLANK;
  return m;
}

static void cia_enable_modparams(const ec_slave_t *s, slave_config_t *sc) {
  int channels = cia_channels(s);

  if (channels > 1) mp_push(sc, mp_param("ciaChannels", (const char *)xasprintf("%d", channels)));

  char tmp[32];
  snprintf(tmp, sizeof(tmp), "%d", cia_pdo_entries(s, 0x1600));
  mp_push(sc, mp_param("ciaRxPDOEntryLimit", tmp));
  snprintf(tmp, sizeof(tmp), "%d", cia_pdo_entries(s, 0x1a00));
  mp_push(sc, mp_param("ciaTxPDOEntryLimit", tmp));

  for (int ch = 0; ch < (channels > 0 ? channels : 1); ch++) {
    char prefix[16] = "";
    if (channels > 1) snprintf(prefix, sizeof(prefix), "ch%d", ch + 1);
    int base = 0x6000 + 0x800 * ch;

    int64_t abilities = read_sdo_int(s->master, s->slave, base + 0x502, 0);

    static const struct {
      int bit;
      const char *name;
      const char *value;
    } abil[] = {
        {0, "enablePP", "true"},     {1, "enableVL", "true"},  {2, "enablePV", "true"},
        {3, "enableTQ", "true"},     {5, "enableHM", "true"},  {6, "enableIP", "disabled"},
        {7, "enableCSP", "true"},    {8, "enableCSV", "true"}, {9, "enableCST", "true"},
    };
    for (size_t i = 0; i < sizeof(abil) / sizeof(abil[0]); i++) {
      if (abilities & ((int64_t)1 << abil[i].bit)) {
        char *full = xasprintf("%s%s", prefix, abil[i].name);
        mp_push(sc, mp_param(full, abil[i].value));
        free(full);
      }
    }

    for (size_t i = 0; i < sizeof(enable_sdos) / sizeof(enable_sdos[0]); i++) {
      char buf[32];
      snprintf(buf, sizeof(buf), "0x%04x:%02x", enable_sdos[i].offset + base, enable_sdos[i].subindex);
      if (*sdo_get(s, buf)) {
        char *full = xasprintf("%s%s", prefix, enable_sdos[i].name);
        mp_push(sc, mp_param(full, "true"));
        free(full);
      }
      if (!strcmp(enable_sdos[i].name, "enableDigitalInput")) {
        char *full = xasprintf("%sdigitalInChannels", prefix);
        mp_push(sc, mp_param(full, "16"));
        free(full);
      }
      if (!strcmp(enable_sdos[i].name, "enableDigitalOutput")) {
        char *full = xasprintf("%sdigitalOutChannels", prefix);
        mp_push(sc, mp_param(full, "16"));
        free(full);
      }
    }
  }
}

// ===== driver registry lookup =====

// Returns the matching type (by VID/PID), or NULL.
// NB: does not respect flag_typedb — driver modparams attach independent of
// type inference (matches Go ConfigModParams behavior).
static const lcec_typelist_t *find_driver(const char *vid_str, const char *pid_str) {
  uint32_t vid = (uint32_t)strtoul(vid_str, NULL, 0);
  uint32_t pid = (uint32_t)strtoul(pid_str, NULL, 0);
  for (lcec_typelinkedlist_t *t = typeslist; t; t = t->next) {
    if (t->type && t->type->vid == vid && t->type->pid == pid) return t->type;
  }
  return NULL;
}

static const char *infer_type(const ec_slave_t *s) {
  if (flag_typedb) {
    const lcec_typelist_t *t = find_driver(s->vendor_id, s->product_id);
    if (t) return t->name;
  }
  if (is_cia402(s)) return "basic_cia402";
  return "generic";
}

// Driver-registry modparams: append { COMMENT, PARAM, BLANK } per registry entry.
static void config_modparams_from_driver(const ec_slave_t *s, slave_config_t *sc) {
  const lcec_typelist_t *t = find_driver(s->vendor_id, s->product_id);
  if (!t || !t->modparams) return;

  for (const lcec_modparam_desc_t *m = t->modparams; m && m->name != NULL; m++) {
    if (m->config_comment) mp_push(sc, mp_comment(m->config_comment));
    if (m->config_value) {
      mp_push(sc, mp_param(m->name, m->config_value));
      mp_push(sc, mp_blank());
    }
  }
}

// ===== PDO walk =====

static char *xml_format_hex(const char *in) {
  // strip "0x" prefix
  if (strncmp(in, "0x", 2) == 0) return xstrdup(in + 2);
  return xstrdup(in);
}

static char *pin_format_comment(const char *in) {
  // lowercase + replace [^a-z0-9]+ with single '-'
  char *low = xstrdup(in);
  to_lower(low);
  // POSIX regex doesn't do replace; do it manually.
  size_t cap = strlen(low) + 1;
  char *out = xmalloc(cap);
  size_t oi = 0;
  bool prev_dash = false;
  for (size_t i = 0; low[i]; i++) {
    char c = low[i];
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
      out[oi++] = c;
      prev_dash = false;
    } else {
      if (!prev_dash) out[oi++] = '-';
      prev_dash = true;
    }
  }
  out[oi] = 0;
  free(low);
  return out;
}

static sm_t *sm_new(const char *idx, const char *dir) {
  sm_t *sm = xmalloc(sizeof(*sm));
  sm->idx = xstrdup(idx);
  sm->dir = xstrdup(dir);
  return sm;
}
static void sc_push_sm(slave_config_t *sc, sm_t *sm) {
  if (sc->sm_count == sc->sm_cap) {
    sc->sm_cap = sc->sm_cap ? sc->sm_cap * 2 : 4;
    sc->sms = realloc(sc->sms, sc->sm_cap * sizeof(sm_t *));
  }
  sc->sms[sc->sm_count++] = sm;
}
static void sm_push_pdo(sm_t *sm, pdo_t *p) {
  if (sm->pdo_count == sm->pdo_cap) {
    sm->pdo_cap = sm->pdo_cap ? sm->pdo_cap * 2 : 4;
    sm->pdos = realloc(sm->pdos, sm->pdo_cap * sizeof(pdo_t *));
  }
  sm->pdos[sm->pdo_count++] = p;
}
static void pdo_push_entry(pdo_t *p, pdo_entry_t *e) {
  if (p->entry_count == p->entry_cap) {
    p->entry_cap = p->entry_cap ? p->entry_cap * 2 : 8;
    p->entries = realloc(p->entries, p->entry_cap * sizeof(pdo_entry_t *));
  }
  p->entries[p->entry_count++] = e;
}

static const char *infer_haltype(const char *sdotype, unsigned long bits) {
  if (bits < 8) return "bit";
  if (!strcmp(sdotype, "uint8") || !strcmp(sdotype, "uint16") || !strcmp(sdotype, "uint32")) return "u32";
  if (!strcmp(sdotype, "int8") || !strcmp(sdotype, "int16") || !strcmp(sdotype, "int32")) return "s32";
  if (!strcmp(sdotype, "bool")) return bits == 1 ? "bit" : "u32";
  if (!strcmp(sdotype, "uint64")) return "u64";
  if (!strcmp(sdotype, "int64")) return "s64";
  if (!strcmp(sdotype, "float") || !strcmp(sdotype, "double"))
    return bits == 32 ? "float-ieee" : "float-double-ieee";
  if (!*sdotype) return "BLANK";
  // unrecognized: "!!" + sdotype + "!!"
  static __thread char buf[64];
  snprintf(buf, sizeof(buf), "!!%s!!", sdotype);
  return buf;
}

static void build_pdos(ec_slave_t *s, slave_config_t *sc) {
  char *args = xasprintf("-m %s pdos -p %s", s->master, s->slave);
  FILE *f = run_ethercat(args);
  free(args);

  char *line = NULL;
  size_t cap = 0;
  ssize_t n;
  regmatch_t m[5];
  sm_t *cur_sm = NULL;
  pdo_t *cur_pdo = NULL;

  while ((n = getline(&line, &cap, f)) >= 0) {
    rtrim(line);
    if (!regexec(&re_pdo_sm, line, 3, m, 0)) {
      char *sm_idx = re_capture(line, m[1]);
      const char *dir = "";
      if (!strcmp(sm_idx, "0"))
        dir = "in";
      else if (!strcmp(sm_idx, "1"))
        dir = "out";
      cur_sm = sm_new(sm_idx, dir);
      sc_push_sm(sc, cur_sm);
      free(sm_idx);
      cur_pdo = NULL;
    } else if (!regexec(&re_pdo_pdo, line, 4, m, 0)) {
      char *kind = re_capture(line, m[1]);
      char *idx_full = re_capture(line, m[2]);
      char *cmt = re_capture(line, m[3]);
      cur_pdo = xmalloc(sizeof(*cur_pdo));
      cur_pdo->idx = xml_format_hex(idx_full);
      cur_pdo->comment = cmt;
      sm_push_pdo(cur_sm, cur_pdo);
      if (!strcmp(kind, "RxPDO")) {
        free(cur_sm->dir);
        cur_sm->dir = xstrdup("out");
      } else if (!strcmp(kind, "TxPDO")) {
        free(cur_sm->dir);
        cur_sm->dir = xstrdup("in");
      }
      free(kind);
      free(idx_full);
    } else if (!regexec(&re_pdo_entry, line, 5, m, 0)) {
      char *e_idx = re_capture(line, m[1]);
      char *e_sub = re_capture(line, m[2]);
      char *e_bits = re_capture(line, m[3]);
      char *e_cmt = re_capture(line, m[4]);
      pdo_entry_t *e = xmalloc(sizeof(*e));
      e->idx = xml_format_hex(e_idx);
      e->subidx = xstrdup(e_sub);
      e->bitlen = xstrdup(e_bits);
      free(e_idx);
      free(e_sub);
      free(e_bits);
      // skip "gap" entries (0x0000)
      if (!strcmp(e->idx, "0000")) {
        free(e->idx);
        free(e->subidx);
        free(e->bitlen);
        free(e);
        free(e_cmt);
        continue;
      }
      // halpin: from comment if any, else "pin-IDX-SUB"
      if (*e_cmt) {
        e->halpin = pin_format_comment(e_cmt);
      } else {
        e->halpin = xasprintf("pin-%s-%s", e->idx, e->subidx);
      }
      free(e_cmt);
      // haltype: from SDO type if known
      char sdo_key[32];
      snprintf(sdo_key, sizeof(sdo_key), "0x%s:%s", e->idx, e->subidx);
      const char *sdotype = sdo_get(s, sdo_key);
      unsigned long bits = strtoul(e->bitlen, NULL, 10);
      e->haltype = xstrdup(infer_haltype(sdotype, bits));
      pdo_push_entry(cur_pdo, e);
    }
  }
  free(line);
  pclose(f);
}

// Append "-pdoname" prefix to halPins that collide within a slave.
static void fixup_pin_names(slave_config_t *sc) {
  // count pin name occurrences
  typedef struct {
    char *name;
    int count;
  } pn_t;
  pn_t *names = NULL;
  size_t names_n = 0, names_cap = 0;
  for (size_t i = 0; i < sc->sm_count; i++) {
    sm_t *sm = sc->sms[i];
    for (size_t j = 0; j < sm->pdo_count; j++) {
      pdo_t *p = sm->pdos[j];
      for (size_t k = 0; k < p->entry_count; k++) {
        const char *pn = p->entries[k]->halpin;
        size_t f;
        for (f = 0; f < names_n; f++)
          if (!strcmp(names[f].name, pn)) break;
        if (f == names_n) {
          if (names_n == names_cap) {
            names_cap = names_cap ? names_cap * 2 : 16;
            names = realloc(names, names_cap * sizeof(pn_t));
          }
          names[names_n].name = xstrdup(pn);
          names[names_n].count = 1;
          names_n++;
        } else {
          names[f].count++;
        }
      }
    }
  }
  // for each entry whose pin appears more than once, prefix from PDO comment last word (or sm.dir)
  for (size_t i = 0; i < sc->sm_count; i++) {
    sm_t *sm = sc->sms[i];
    for (size_t j = 0; j < sm->pdo_count; j++) {
      pdo_t *p = sm->pdos[j];
      // last whitespace-separated word of comment, else sm->dir
      const char *prefix = NULL;
      if (p->comment && *p->comment) {
        const char *sp = strrchr(p->comment, ' ');
        prefix = sp ? sp + 1 : p->comment;
        // Go: split on " " — only uses last component if split count > 1.
        // Re-check: `if len(split) > 1 { prefix = last } else { prefix = sm.Dir }`
        // So if comment has no space, use sm.Dir.
        if (!sp) prefix = sm->dir;
      } else {
        prefix = sm->dir;
      }
      for (size_t k = 0; k < p->entry_count; k++) {
        pdo_entry_t *e = p->entries[k];
        for (size_t f = 0; f < names_n; f++) {
          if (!strcmp(names[f].name, e->halpin) && names[f].count > 1) {
            char *combined = xasprintf("%s %s", prefix, e->halpin);
            char *newpin = pin_format_comment(combined);
            free(combined);
            free(e->halpin);
            e->halpin = newpin;
            break;
          }
        }
      }
    }
  }
  for (size_t i = 0; i < names_n; i++) free(names[i].name);
  free(names);
}

// ===== XML emit =====

// XML attribute escape — spec values from this generator are already safe
// (hex numbers, name tokens). Skip escape for parity with Go output.
static void emit_attr(FILE *out, const char *name, const char *value) {
  fprintf(out, " %s=\"%s\"", name, value);
}

static bool slave_has_children(const slave_config_t *sc) {
  return (sc->comment && *sc->comment) || sc->mp_count || sc->sm_count;
}

static void emit_modparam(FILE *out, const modparam_t *m) {
  switch (m->kind) {
    case MP_PARAM:
      fprintf(out, "      <modParam name=\"%s\" value=\"%s\"/>\n", m->name, m->value);
      break;
    case MP_COMMENT:
      fprintf(out, "      <!-- %s -->\n", m->comment);
      break;
    case MP_BLANK:
      fprintf(out, "      \n");
      break;
  }
}

static void emit_pdo(FILE *out, const pdo_t *p) {
  fprintf(out, "        <pdo idx=\"%s\">\n", p->idx);
  if (p->comment && *p->comment) fprintf(out, "          <!--%s-->\n", p->comment);
  for (size_t i = 0; i < p->entry_count; i++) {
    pdo_entry_t *e = p->entries[i];
    fprintf(out, "          <pdoEntry idx=\"%s\" subIdx=\"%s\" bitLen=\"%s\" halPin=\"%s\" halType=\"%s\"/>\n",
            e->idx, e->subidx, e->bitlen, e->halpin, e->haltype);
  }
  fprintf(out, "        </pdo>\n");
}

static void emit_sm(FILE *out, const sm_t *sm) {
  if (sm->pdo_count == 0) {
    fprintf(out, "      <syncManager idx=\"%s\" dir=\"%s\"/>\n", sm->idx, sm->dir);
    return;
  }
  fprintf(out, "      <syncManager idx=\"%s\" dir=\"%s\">\n", sm->idx, sm->dir);
  for (size_t i = 0; i < sm->pdo_count; i++) emit_pdo(out, sm->pdos[i]);
  fprintf(out, "      </syncManager>\n");
}

static void emit_slave(FILE *out, const slave_config_t *sc) {
  fputs("    <slave", out);
  emit_attr(out, "idx", sc->idx);
  emit_attr(out, "type", sc->type);
  if (sc->vid && *sc->vid) emit_attr(out, "vid", sc->vid);
  if (sc->pid && *sc->pid) emit_attr(out, "pid", sc->pid);
  emit_attr(out, "name", sc->name);

  if (!slave_has_children(sc)) {
    fputs("/>\n", out);
    return;
  }
  fputs(">\n", out);
  if (sc->comment && *sc->comment) fprintf(out, "      <!--%s-->\n", sc->comment);
  for (size_t i = 0; i < sc->sm_count; i++) emit_sm(out, sc->sms[i]);
  for (size_t i = 0; i < sc->mp_count; i++) emit_modparam(out, sc->modparams[i]);
  fputs("    </slave>\n", out);
}

// ===== master sort =====

static int master_cmp(const void *a, const void *b) {
  const master_config_t *const *pa = a;
  const master_config_t *const *pb = b;
  return strcmp((*pa)->idx, (*pb)->idx);
}

// ===== main =====

static master_config_t *find_master(master_config_t **masters, size_t n, const char *idx) {
  for (size_t i = 0; i < n; i++)
    if (!strcmp(masters[i]->idx, idx)) return masters[i];
  return NULL;
}

static void usage(const char *argv0) {
  fprintf(stderr,
          "Usage: %s [options]\n"
          "  -typedb              Use built-in driver db (default true)\n"
          "  -extra_cia_modparams Add CiA modParams to all CiA 402 devices\n"
          "  -generic_pdos        Walk PDOs for generic devices (default true)\n",
          argv0);
}

static int parse_bool(const char *s) {
  if (!s || !*s) return 1;
  return (!strcasecmp(s, "1") || !strcasecmp(s, "true") || !strcasecmp(s, "t") || !strcasecmp(s, "yes"));
}

int main(int argc, char **argv) {
  // Parse Go-style "-flag=value" / "-flag" args (matches existing tool's CLI).
  for (int i = 1; i < argc; i++) {
    char *a = argv[i];
    if (a[0] != '-') {
      usage(argv[0]);
      return 1;
    }
    a++;
    if (a[0] == '-') a++;  // accept --flag too
    char *eq = strchr(a, '=');
    char *val = NULL;
    if (eq) {
      *eq = 0;
      val = eq + 1;
    }
    if (!strcmp(a, "typedb"))
      flag_typedb = parse_bool(val);
    else if (!strcmp(a, "extra_cia_modparams"))
      flag_extra_cia_modparams = parse_bool(val);
    else if (!strcmp(a, "generic_pdos"))
      flag_generic_pdos = parse_bool(val);
    else if (!strcmp(a, "h") || !strcmp(a, "help")) {
      usage(argv[0]);
      return 0;
    } else {
      fprintf(stderr, "%s: unknown flag -%s\n", argv[0], a);
      return 1;
    }
  }

  compile_all_regexes();
  read_slaves();

  master_config_t **masters = NULL;
  size_t mc = 0, mcap = 0;
  int dev_seq = 0;

  for (size_t i = 0; i < slaves_count; i++) {
    ec_slave_t *s = &slaves_arr[i];
    master_config_t *mm = find_master(masters, mc, s->master);
    if (!mm) {
      if (mc == mcap) {
        mcap = mcap ? mcap * 2 : 4;
        masters = realloc(masters, mcap * sizeof(*masters));
      }
      mm = xmalloc(sizeof(*mm));
      mm->idx = xstrdup(s->master);
      masters[mc++] = mm;
    }

    read_sdos(s);

    slave_config_t *sc = xmalloc(sizeof(*sc));
    sc->idx = xstrdup(s->slave);
    sc->type = xstrdup(infer_type(s));
    dev_seq++;
    sc->name = xasprintf("D%d", dev_seq);

    config_modparams_from_driver(s, sc);

    if (!strcmp(sc->type, "basic_cia402") || (flag_extra_cia_modparams && is_cia402(s))) {
      cia_enable_modparams(s, sc);
    }

    if (!strcmp(sc->type, "generic") || !strcmp(sc->type, "basic_cia402")) {
      sc->vid = xstrdup(s->vendor_id);
      sc->pid = xstrdup(s->product_id);
      sc->comment = xstrdup(s->device_name);
    } else {
      sc->vid = xstrdup("");
      sc->pid = xstrdup("");
      sc->comment = xstrdup("");
    }

    if (!strcmp(sc->type, "generic") && flag_generic_pdos) {
      build_pdos(s, sc);
    }

    fixup_pin_names(sc);

    if (mm->slave_count == mm->slave_cap) {
      mm->slave_cap = mm->slave_cap ? mm->slave_cap * 2 : 4;
      mm->slaves = realloc(mm->slaves, mm->slave_cap * sizeof(*mm->slaves));
    }
    mm->slaves[mm->slave_count++] = sc;
  }

  // Sort masters by idx for deterministic output.
  qsort(masters, mc, sizeof(*masters), master_cmp);

  printf("<masters>\n");
  for (size_t i = 0; i < mc; i++) {
    printf("  <master idx=\"%s\">\n", masters[i]->idx);
    for (size_t j = 0; j < masters[i]->slave_count; j++) emit_slave(stdout, masters[i]->slaves[j]);
    printf("  </master>\n");
  }
  printf("</masters>\n");
  return 0;
}
