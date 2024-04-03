#include <stdio.h>

#include "../../src/lcec.h"
#include "tests.h"

TESTGLOBALSETUP;

static const lcec_modparam_desc_t mp_3[] = {
    {"positionLimitMin", 5, MODPARAM_TYPE_S32},
    {"positionLimitMax", 6, MODPARAM_TYPE_S32},
    {"swPositionLimitMin", 7, MODPARAM_TYPE_S32},
    {NULL},
};

static const lcec_modparam_desc_t mp_1[] = {
    {"positionLimitMax", 8, MODPARAM_TYPE_S32},
    {NULL},
};

static const lcec_modparam_desc_t mp_0[] = {
    {NULL},
};

TESTFUNC(test_modparam_len) {
  TESTSETUP;

  TESTINT(lcec_modparam_desc_len(mp_3), 3);
  TESTINT(lcec_modparam_desc_len(mp_1), 1);
  TESTINT(lcec_modparam_desc_len(mp_0), 0);
  TESTINT(lcec_modparam_desc_len(NULL), 0);

  TESTRESULTS;
}

TESTFUNC(test_modparam_concat) {
  TESTSETUP;
  TESTINT(lcec_modparam_desc_len(lcec_modparam_desc_concat(mp_1, mp_3)), 4);
  TESTINT(lcec_modparam_desc_len(lcec_modparam_desc_concat(mp_1, mp_0)), 1);
  TESTINT(lcec_modparam_desc_len(lcec_modparam_desc_concat(mp_1, NULL)), 1);

  TESTRESULTS;
}

TESTMAIN
