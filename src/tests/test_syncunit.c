#include <stdio.h>
#include <stdlib.h>

#include "../../src/lcec_syncunit.h"
#include "tests.h"

TESTGLOBALSETUP;

#define T 125000  // master cycle, 8 kHz

TESTFUNC(test_su_mod) {
  TESTSETUP;

  TESTINT((int)lcec_su_mod(5, 4), 1);
  TESTINT((int)lcec_su_mod(-1, 4), 3);
  TESTINT((int)lcec_su_mod(-4, 4), 0);

  TESTRESULTS;
}

TESTFUNC(test_su_round_div) {
  TESTSETUP;

  // M2R resyncs move app_time by whole periods
  TESTINT((int)lcec_su_round_div(3 * T, T), 3);
  TESTINT((int)lcec_su_round_div(-2 * T, T), -2);
  // a sub-period resync does not move the grid
  TESTINT((int)lcec_su_round_div(T / 3, T), 0);
  TESTINT((int)lcec_su_round_div(-T / 3, T), 0);
  TESTINT((int)lcec_su_round_div(T - 1000, T), 1);

  TESTRESULTS;
}

TESTFUNC(test_su_due) {
  TESTSETUP;

  TESTINT(lcec_su_due(7, 1, 0), 1);
  TESTINT(lcec_su_due(0, 4, 0), 1);
  TESTINT(lcec_su_due(4, 4, 0), 1);
  TESTINT(lcec_su_due(1, 4, 0), 0);
  TESTINT(lcec_su_due(-4, 4, 0), 1);
  TESTINT(lcec_su_due(3, 4, 3), 1);
  TESTINT(lcec_su_due(-1, 4, 3), 1);
  TESTINT(lcec_su_due(0, 4, 3), 0);

  TESTRESULTS;
}

TESTMAIN
