/*
 * Flash_SelfTest.h
 *
 * Bring-up and diagnostic tests for the AT25XE041B external NOR flash.
 *
 * Two things live here, and they answer different questions:
 *
 *   FLASH_eSelfTest()  - "does this board talk to its flash correctly?"
 *                        Runs at boot from SELFTEST_vRunAndReport(). Walks
 *                        identification, status register, erase and program,
 *                        and CLASSIFIES a failure instead of reporting a bare
 *                        pass/fail. What it reports is the input to a repair
 *                        decision: a wrong-but-plausible JEDEC ID means the
 *                        wrong part is fitted, all-00h means the bus is dead,
 *                        a bit-shifted pattern means a clock or sampling
 *                        problem, and each of those sends a technician
 *                        somewhere different.
 *
 *   FLASH_bStressTest() - "does it STAY correct while something else is
 *                        happening?" Console-driven, loops a read of a known
 *                        page and takes a callback to run between iterations.
 *                        This is the harness for the open defect: flash reads
 *                        returning corrupt data shortly after a LoRa TX. The
 *                        callback is the whole point - the radio is not named
 *                        anywhere in this module, so the same loop can be
 *                        driven by a TX, by a power-down cycle, or by nothing
 *                        at all, and the three results can be compared.
 *
 * Both need the reserved scratch sector at FLASH_SELFTEST_ADDR; see
 * Flash_Config.h for why the log gave up its top 4 KB for it.
 */

#ifndef DEVICE_FLASH_FLASH_SELFTEST_H_
#define DEVICE_FLASH_FLASH_SELFTEST_H_

#include <stdint.h>
#include <stdbool.h>

/* Why the self-test failed, in the order the test runs. A caller that only
 * wants pass/fail can compare against FLASH_ST_PASS; everything else exists so
 * a failure says what to go and look at. */
typedef enum
{
    FLASH_ST_PASS = 0,
    FLASH_ST_FAIL_NO_COMMS,     /* ID all 00h or all FFh - nothing is talking  */
    FLASH_ST_FAIL_WRONG_PART,   /* ID read cleanly and is not 1F 44 02         */
    FLASH_ST_FAIL_BUSY_STUCK,   /* RDY/BSY never cleared                       */
    FLASH_ST_FAIL_PROTECTED,    /* still protected after a Global Unprotect    */
    FLASH_ST_FAIL_ERASE,        /* scratch sector did not read back all FFh    */
    FLASH_ST_FAIL_PROGRAM,      /* pattern read back wrong                     */
} Flash_SelfTestResult_t;

/* How a mismatched buffer was wrong. Shared by the self-test and the stress
 * harness so a fault classified at boot and the same fault classified under
 * load are described in the same words. */
typedef enum
{
    FLASH_MISMATCH_NONE = 0,
    FLASH_MISMATCH_ALL_FF,      /* not responding: still parked, or still busy */
    FLASH_MISMATCH_ALL_00,      /* MISO stuck low, or CS never asserted        */
    FLASH_MISMATCH_SHIFTED,     /* right data, wrong alignment - clock glitch
                                 * or a sampling-phase problem                 */
    FLASH_MISMATCH_NOISE,       /* none of the above                           */
} Flash_Mismatch_t;

/* Run the boot self-test. Safe to call once the flash driver is initialised
 * (FLASH_vInit) and before anything else is using the bus. Logs every step
 * over DBG_LOG whether it passes or not - a self-test that only speaks up on
 * failure cannot be used to confirm a repair. */
Flash_SelfTestResult_t FLASH_eSelfTest(void);

/* Human-readable form of the above, for a log line or an over-air reply. */
const char *FLASH_pcSelfTestResult(Flash_SelfTestResult_t eResult);
const char *FLASH_pcMismatchName(Flash_Mismatch_t eKind);

/* What a stress run saw. Counts rather than log lines: a 1000-cycle run that
 * logged every read would perturb the timing being measured, which is the
 * same reason FLASH_u16GetAndClearReadFails exists. */
typedef struct
{
    uint32_t u32Cycles;         /* iterations actually completed              */
    uint32_t u32Mismatch;       /* iterations whose bytes were wrong          */
    uint32_t u32AllFF;
    uint32_t u32All00;
    uint32_t u32Shifted;
    uint32_t u32Noise;
    uint32_t u32ReadFail;       /* FLASH_vRead returned false - a transport
                                 * fault, NOT corruption. Kept separate on
                                 * purpose: conflating "the bus failed" with
                                 * "the bus worked and lied" is what made the
                                 * 2026-08-31 events undiagnosable.           */
    uint32_t u32FirstBadCycle;  /* 0 if none                                  */
    uint16_t u16DriverRetries;  /* recovered by the driver read-verify        */
    uint16_t u16DriverFails;    /* not recovered by the driver read-verify    */
} Flash_StressStats_t;

/* Read the self-test page u32Cycles times, classifying every mismatch, calling
 * pfnBetweenCycles (may be NULL) between iterations.
 *
 * Requires the pattern the self-test writes to be present at
 * FLASH_SELFTEST_ADDR; run FLASH_eSelfTest() first, or use the console command
 * that does both. Returns false if the pattern is not there to begin with -
 * otherwise every cycle would "fail" and say nothing about the bus.
 *
 * Always linkable: without ENABLE_FLASH_STRESS_TEST it zeroes the stats and
 * returns false, so callers need no #ifdef around the call site. */
bool FLASH_bStressTest(uint32_t u32Cycles,
                       void (*pfnBetweenCycles)(void),
                       Flash_StressStats_t *ptStats);

#endif /* DEVICE_FLASH_FLASH_SELFTEST_H_ */
