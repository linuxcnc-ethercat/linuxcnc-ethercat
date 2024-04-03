#include <stdio.h>

#include "../../src/devices/lcec_class_cia402.h"
#include "../../src/lcec.h"
#include "tests.h"

TESTGLOBALSETUP;

static const lcec_modparam_desc_t per_channel_mps[] = {
    {"aaa", 0x1000, MODPARAM_TYPE_S32},
    {"bbb", 0x1010, MODPARAM_TYPE_S32},
    {"ccc", 0x1020, MODPARAM_TYPE_S32},
    {NULL},
};

static const lcec_modparam_desc_t device_mps[] = {
    {"ddd", 0x1, MODPARAM_TYPE_S32},
    {NULL},
};

static const lcec_modparam_desc_t device_mps2[] = {
    {"eee", 0x100, MODPARAM_TYPE_S32, "42"},
    {NULL},
};

static const lcec_modparam_desc_t override_mps[] = {
    {"aaa", 0, 0, "12"},
    {NULL},
};

TESTFUNC(test_cia402_modparam_len) {
  TESTSETUP;
  lcec_modparam_desc_t *channelized_mps;

  TESTINT(lcec_modparam_desc_len(per_channel_mps), 3);
  TESTINT(lcec_modparam_desc_len(device_mps), 1);

  channelized_mps = lcec_cia402_channelized_modparams(per_channel_mps);
  TESTNOTNULL(channelized_mps);

  TESTINT(lcec_modparam_desc_len(channelized_mps), 27);

  lcec_modparam_desc_t *all_mps = lcec_modparam_desc_concat(channelized_mps, device_mps);
  TESTNOTNULL(all_mps);

  TESTINT(lcec_modparam_desc_len(all_mps), 28);
  TESTSTRING(all_mps[0].name, "aaa");
  TESTSTRING(all_mps[1].name, "ch1aaa");
  TESTSTRING(all_mps[2].name, "ch2aaa");
  TESTSTRING(all_mps[3].name, "ch3aaa");
  TESTSTRING(all_mps[4].name, "ch4aaa");
  TESTSTRING(all_mps[5].name, "ch5aaa");
  TESTSTRING(all_mps[6].name, "ch6aaa");
  TESTSTRING(all_mps[7].name, "ch7aaa");
  TESTSTRING(all_mps[8].name, "ch8aaa");
  TESTSTRING(all_mps[9].name, "bbb");
  TESTSTRING(all_mps[10].name, "ch1bbb");
  TESTSTRING(all_mps[11].name, "ch2bbb");
  TESTSTRING(all_mps[12].name, "ch3bbb");
  TESTSTRING(all_mps[13].name, "ch4bbb");
  TESTSTRING(all_mps[14].name, "ch5bbb");
  TESTSTRING(all_mps[15].name, "ch6bbb");
  TESTSTRING(all_mps[16].name, "ch7bbb");
  TESTSTRING(all_mps[17].name, "ch8bbb");

  TESTINT(all_mps[0].id, 0x1000);
  TESTINT(all_mps[1].id, 0x1000);
  TESTINT(all_mps[2].id, 0x1001);
  TESTINT(all_mps[3].id, 0x1002);
  TESTINT(all_mps[4].id, 0x1003);
  TESTINT(all_mps[5].id, 0x1004);
  TESTINT(all_mps[6].id, 0x1005);
  TESTINT(all_mps[7].id, 0x1006);
  TESTINT(all_mps[8].id, 0x1007);
  TESTINT(all_mps[9].id, 0x1010);
  TESTINT(all_mps[10].id, 0x1010);
  TESTINT(all_mps[11].id, 0x1011);
  TESTINT(all_mps[12].id, 0x1012);
  TESTINT(all_mps[13].id, 0x1013);
  TESTINT(all_mps[14].id, 0x1014);
  TESTINT(all_mps[15].id, 0x1015);
  TESTINT(all_mps[16].id, 0x1016);
  TESTINT(all_mps[17].id, 0x1017);

  TESTRESULTS;
}

TESTFUNC(test_cia402_modparam_defaults) {
  TESTSETUP;

  int a = lcec_modparam_desc_len(lcec_cia402_modparams(NULL, NULL, NULL));

  TESTINT(lcec_modparam_desc_len(lcec_cia402_modparams(per_channel_mps, NULL, NULL)), a + 27);
  TESTINT(lcec_modparam_desc_len(lcec_cia402_modparams(per_channel_mps, device_mps, NULL)), a + 28);
  TESTINT(lcec_modparam_desc_len(lcec_cia402_modparams(per_channel_mps, device_mps, override_mps)), a + 28);

  lcec_modparam_desc_t *all_mps = lcec_modparam_desc_concat(per_channel_mps, device_mps2);

  TESTINT(lcec_modparam_desc_len(all_mps), 4);
  TESTSTRING(all_mps[3].config_value, "42");

  TESTRESULTS;
}

TESTFUNC(test_cia402_modparam_override) {
  TESTSETUP;
  lcec_modparam_desc_t *merged_mps = lcec_modparam_desc_override(per_channel_mps, override_mps);

  TESTINT(lcec_modparam_desc_len(merged_mps), 3);
  TESTSTRING(merged_mps[0].config_value, "12");

  TESTRESULTS;
}

TESTFUNC(test_cia402_modparam_ratio) {
  TESTSETUP;

  int max_denom = 1 << 15;

  lcec_ratio ratio;
  ratio = lcec_cia402_decode_ratio_modparam("1/2", max_denom);
  TESTINT(ratio.numerator, 1);
  TESTINT(ratio.denominator, 2);

  ratio = lcec_cia402_decode_ratio_modparam("3:4", max_denom);
  TESTINT(ratio.numerator, 3);
  TESTINT(ratio.denominator, 4);

  ratio = lcec_cia402_decode_ratio_modparam("86400/3600", max_denom);
  TESTINT(ratio.numerator, 86400);
  TESTINT(ratio.denominator, 3600);

  ratio = lcec_cia402_decode_ratio_modparam("3", max_denom);
  TESTINT(ratio.numerator, 3);
  TESTINT(ratio.denominator, 1);

  ratio = lcec_cia402_decode_ratio_modparam("5.2", max_denom);
  TESTINT(ratio.numerator, 26);
  TESTINT(ratio.denominator, 5);

  ratio = lcec_cia402_decode_ratio_modparam(
      "3355443.2", max_denom);  // From https://github.com/linuxcnc-ethercat/linuxcnc-ethercat/issues/286#issuecomment-1962819925
  TESTINT(ratio.numerator, 16777216);
  TESTINT(ratio.denominator, 5);

  TESTRESULTS;
}

TESTMAIN
