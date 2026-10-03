#include <stdio.h>

#include "../devices/lcec_el6021_cfg.h"
#include "tests.h"

TESTGLOBALSETUP;

TESTFUNC(test_el6021_baud_table) {
  TESTSETUP;

  // every supported baud rate resolves by value, by SDO index, and back
  static const uint32_t bauds[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
  for (size_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
    const lcec_el6021_baud_t *b = lcec_el6021_baud_by_value(bauds[i]);
    TESTNOTNULL(b);
    TESTINT(lcec_el6021_baud_by_idx(b->idx)->baud, bauds[i]);
  }
  TESTINT(lcec_el6021_baud_by_value(115200)->idx, 10);

  // unsupported rates and indices must be rejected
  TESTINT(lcec_el6021_baud_by_value(14400) == NULL, 1);
  TESTINT(lcec_el6021_baud_by_idx(0) == NULL, 1);
  TESTINT(lcec_el6021_baud_by_idx(11) == NULL, 1);

  TESTRESULTS;
}

TESTFUNC(test_el6021_frame_table) {
  TESTSETUP;

  TESTNOTNULL(lcec_el6021_frame_by_string("8N1"));
  TESTINT(lcec_el6021_frame_by_string("8N1")->idx, 0x03);
  TESTINT(lcec_el6021_frame_by_string("7E1")->idx, 0x01);
  TESTINT(lcec_el6021_frame_by_string("7O2")->idx, 0x0a);
  TESTINT(lcec_el6021_frame_by_string("8E2")->idx, 0x0c);
  TESTINT(lcec_el6021_frame_by_string("8O1")->idx, 0x05);

  // malformed specs must be rejected
  TESTINT(lcec_el6021_frame_by_string(NULL) == NULL, 1);
  TESTINT(lcec_el6021_frame_by_string("") == NULL, 1);
  TESTINT(lcec_el6021_frame_by_string("8N11") == NULL, 1);
  TESTINT(lcec_el6021_frame_by_string("6N1") == NULL, 1);
  TESTINT(lcec_el6021_frame_by_string("8X1") == NULL, 1);
  TESTINT(lcec_el6021_frame_by_string("8N3") == NULL, 1);
  // 7N1 is not in the EL600x frame table
  TESTINT(lcec_el6021_frame_by_string("7N1") == NULL, 1);

  TESTRESULTS;
}

// kernel CBAUD codes (asm-generic/termbits.h), not glibc B* values
TESTFUNC(test_el6021_cflag_kernel_baud) {
  TESTSETUP;
  uint8_t baud_idx, frame_idx, rtscts;

  // 9600 8N1: kernel code 0x000d
  TESTINT(lcec_el6021_cflag_to_config(0x000d | CS8 | CREAD | CLOCAL, 0, &baud_idx, &frame_idx, &rtscts), 0);
  TESTINT(baud_idx, 6);
  TESTINT(frame_idx, 0x03);
  TESTINT(rtscts, 0);

  // 115200 8N1: kernel code 0x1002
  TESTINT(lcec_el6021_cflag_to_config(0x1002 | CS8, 0, &baud_idx, &frame_idx, &rtscts), 0);
  TESTINT(baud_idx, 10);

  // 300 7E1: kernel code 0x0007
  TESTINT(lcec_el6021_cflag_to_config(0x0007 | CS7 | PARENB, 0, &baud_idx, &frame_idx, &rtscts), 0);
  TESTINT(baud_idx, 1);
  TESTINT(frame_idx, 0x01);

  TESTRESULTS;
}

TESTFUNC(test_el6021_cflag_bother) {
  TESTSETUP;
  uint8_t baud_idx, frame_idx, rtscts;

  // glibc >= 2.42 cfsetospeed() sends BOTHER + c_ospeed via TCSETS2
  TESTINT(lcec_el6021_cflag_to_config(LCEC_EL6021_KBOTHER | CS8, 19200, &baud_idx, &frame_idx, &rtscts), 0);
  TESTINT(baud_idx, 7);

  // BOTHER with an unsupported rate must be rejected
  TESTINT(lcec_el6021_cflag_to_config(LCEC_EL6021_KBOTHER | CS8, 14400, &baud_idx, &frame_idx, &rtscts), -1);

  TESTRESULTS;
}

TESTFUNC(test_el6021_cflag_frames) {
  TESTSETUP;
  uint8_t baud_idx, frame_idx, rtscts;
  tcflag_t b9600 = 0x000d;

  // 9600 7O2
  TESTINT(lcec_el6021_cflag_to_config(b9600 | CS7 | PARENB | PARODD | CSTOPB, 0, &baud_idx, &frame_idx, &rtscts), 0);
  TESTINT(frame_idx, 0x0a);

  // 9600 8N2
  TESTINT(lcec_el6021_cflag_to_config(b9600 | CS8 | CSTOPB, 0, &baud_idx, &frame_idx, &rtscts), 0);
  TESTINT(frame_idx, 0x0b);

  // RTS/CTS flag decodes
  TESTINT(lcec_el6021_cflag_to_config(b9600 | CS8 | CRTSCTS, 0, &baud_idx, &frame_idx, &rtscts), 0);
  TESTINT(rtscts, 1);

  // unsupported data sizes must be rejected
  TESTINT(lcec_el6021_cflag_to_config(b9600 | CS6, 0, &baud_idx, &frame_idx, &rtscts), -1);
  TESTINT(lcec_el6021_cflag_to_config(b9600 | CS5, 0, &baud_idx, &frame_idx, &rtscts), -1);

  TESTRESULTS;
}

TESTMAIN
