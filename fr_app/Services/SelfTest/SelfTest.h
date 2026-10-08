/*
 * SelfTest.h
 *
 * Boot-time peripheral smoke tests. Called once from INIT_vInitialization
 * after all normal bring-up is done. Each check is memoized; a failure is
 * indicated live on the green LED (see SELFTEST_vRunAndReport) and can
 * also be queried later over FrKernel ("tag <ID> selftest [gps|acc|flash]").
 *
 * Never fatal: init always proceeds to the normal run state whatever the
 * test outcome. The point is a visual "this unit has a hardware problem"
 * signal at the bench, not a boot barrier.
 */

#ifndef SERVICES_SELFTEST_SELFTEST_H_
#define SERVICES_SELFTEST_SELFTEST_H_

#include <stdbool.h>

/* Green-LED flash codes. Ordering here is also the order used when
 * multiple failures are reported one after the other. */
typedef enum {
    SELFTEST_ERR_GPS   = 1,   /* secondary only; not run on primary  */
    SELFTEST_ERR_ACC   = 2,   /* secondary only; not run on primary  */
    SELFTEST_ERR_FLASH = 3,
} SelfTestErr_e;

/* Runs the checks that apply to this role, records each outcome, and if any
 * failed flashes the green LED with the code sequence:
 *   for each failed test (in the SelfTestErr_e order above):
 *     flash the code N times, pause 2 s, flash again, pause 2 s, flash a
 *     third time. If more than one test failed, pause 3 s between codes.
 *   Green LED off at the end.
 * Blocking; returns once the sequence is done. */
void SELFTEST_vRunAndReport(void);

/* Query API for FrKernel. GPS and ACC read the memoized boot-time result.
 * FLASH is answered LIVE from the driver gate instead: that gate can close and
 * re-open at run time, so a boot snapshot would keep reporting OK on a unit
 * whose log and FOTA staging had since stopped working — the exact failure this
 * check is meant to catch. A "not applicable" test (GPS or ACC on primary)
 * counts as OK so callers can decide with the *Applicable() functions. */
bool SELFTEST_bGpsOk(void);
bool SELFTEST_bAccOk(void);
bool SELFTEST_bFlashOk(void);

bool SELFTEST_bGpsApplicable(void);
/* True on a secondary only: no accelerometer is fitted on a V3 RevA primary. */
bool SELFTEST_bAccApplicable(void);
bool SELFTEST_bFlashApplicable(void);

/* The flash self-test's reason code from boot, as a short string
 * ("PASS", "FAIL:wrong-part", "FAIL:no-comms", ...). SELFTEST_bFlashOk() says
 * whether the flash works NOW; this says what was wrong with it at boot, which
 * is the part an operator can act on. Never NULL. */
const char *SELFTEST_pcFlashDetail(void);

#endif /* SERVICES_SELFTEST_SELFTEST_H_ */
