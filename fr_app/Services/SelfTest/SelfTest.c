/*
 * SelfTest.c — see SelfTest.h for the contract.
 */

#include "SelfTest.h"

#include "build_config.h"
#include "cmsis_os2.h"
#include "hal_bsp.h"
#include "dbg_log.h"

#include "DeviceDiscovery.h"       /* DEVICE_DISCOVERY_eGetDeviceRole */
#include "Acc.h"                   /* ACC_bDeviceIdOkEx              */
#include "Acc_Config.h"            /* ACC_WHO_AM_I_VALUE             */
#include "Gps.h"                   /* GPS_bSelfTest                  */

#include "Flash.h"                 /* FLASH_bRecoverDevice, health   */
#include "Flash_Config.h"          /* FLASH_MANUFACTURER_ID          */
#include "Flash_SelfTest.h"        /* FLASH_eSelfTest                */

/* Result flags default to true so a not-applicable test naturally reports
 * OK (e.g. GPS on primary). */
static bool bGpsOk       = true;
static bool bAccOk       = true;
static bool bFlashOk     = true;

static bool bGpsRan      = false;   /* whether the test actually executed */
static bool bFlashRan    = false;

/* Kept so a human asking later gets the same verdict the boot log carried, not
 * just a bool. "flash=FAIL" tells an operator to do something; "FAIL:wrong-part"
 * tells them what. */
static Flash_SelfTestResult_t eFlashResult = FLASH_ST_PASS;

/* --- LED flash sequence ---------------------------------------------------
 * User spec: for each failed test, flash the code (N short flashes) three
 * times with a 2 s gap between repeats. If more than one test failed, put
 * a 3 s gap between codes. Yellow LED off at the end so the existing
 * BSP_LED_Off(LED_YELLOW) at the init tail has nothing left to clean up.
 * -------------------------------------------------------------------------- */
#define SELFTEST_FLASH_ON_MS       200U
#define SELFTEST_FLASH_OFF_MS      200U
#define SELFTEST_GAP_REPEAT_MS     2000U
#define SELFTEST_GAP_BETWEEN_MS    3000U
#define SELFTEST_REPEATS           3U

static void selftest_vFlashCode(uint8_t u8Code)
{
    for (uint8_t r = 0; r < SELFTEST_REPEATS; r++)
    {
        for (uint8_t i = 0; i < u8Code; i++)
        {
            BSP_LED_On(LED_YELLOW);
            osDelay(SELFTEST_FLASH_ON_MS);
            BSP_LED_Off(LED_YELLOW);
            osDelay(SELFTEST_FLASH_OFF_MS);
        }
        if (r + 1U < SELFTEST_REPEATS)
            osDelay(SELFTEST_GAP_REPEAT_MS);
    }
}

void SELFTEST_vRunAndReport(void)
{
    /* GPS — secondary only. Primary has no GPS fitted; leave bGpsOk=true
     * (its default) so query cmds report the (n/a) OK state cleanly. */
    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
    {
        bGpsRan = true;
        bGpsOk  = GPS_bSelfTest(1500U);   /* > one 1 Hz NMEA epoch */
    }

    /* Accelerometer — fitted on both roles. Retried and raw-byte reported for
     * the same reason as the flash JEDEC read below: one bad WHO_AM_I is a
     * rail transient, and a bare pass/fail can't tell that from a real fault.
     * This is what was reporting acc=FAIL on primaries whose ACC was fine. */
    {
        uint8_t u8Id       = 0U;
        uint8_t u8Attempts = 0U;

        bAccOk = ACC_bDeviceIdOkEx(&u8Id, &u8Attempts);

        if (bAccOk && u8Attempts > 1U)
            DBG_LOG("SelfTest: acc WHO_AM_I %02X OK on attempt %u (first read(s) transient)\r\n",
                    u8Id, (unsigned)u8Attempts);
        else if (!bAccOk)
            DBG_LOG("SelfTest: acc WHO_AM_I read FAILED %ux - last byte %02X (expected %02X)\r\n",
                    (unsigned)u8Attempts, u8Id, (unsigned)ACC_WHO_AM_I_VALUE);
    }

    /* Ext flash. Reports the DRIVER GATE (FLASH_bDevicePresent), not merely whether a
     * JEDEC read can be coaxed through right now. Those are different
     * questions, and answering the easy one is what made this check actively
     * harmful: on 2026-08-31 a tag printed "flash=OK" at every boot for 18 h
     * while the gate was shut, the log was frozen and every FOTA erase was
     * failing on its first call. The one line an operator watches must mean
     * "the flash driver is usable", or it is worse than no check at all.
     *
     * FLASH_bRecoverDevice rather than a passive probe, because a gate found
     * shut here is very often a boot-time transient that a retry clears — and
     * a self-test that repairs the fault it was about to report is strictly
     * better than one that only names it, on a unit nobody can reach by wire. */
    {
        bFlashRan = true;

        /* Delegated to FLASH_eSelfTest, which does what this block used to do
         * (a retried, evidence-reporting JEDEC probe) and then keeps going:
         * status register by name, and - where the build asks for it - an
         * erase, a program and a byte-for-byte compare with the mismatch
         * CLASSIFIED. It logs every step itself, so nothing is duplicated
         * here; what comes back is the one verdict this file has to turn into
         * an LED code and a summary line.
         *
         * The driver gate is still what "flash=OK" means, exactly as before -
         * see SELFTEST_bFlashOk below and the 18-hour false-OK it exists to
         * prevent. FLASH_eSelfTest opens with FLASH_bRecoverDevice, so a gate
         * found shut at boot still gets one repair attempt before it is
         * reported. */
        eFlashResult = FLASH_eSelfTest();
        bFlashOk     = (eFlashResult == FLASH_ST_PASS) && FLASH_bDevicePresent();

        if (!bFlashOk)
            DBG_LOG("SelfTest: flash %s - writes/erases may be DISABLED "
                    "(log + FOTA staging) until this is resolved\r\n",
                    FLASH_pcSelfTestResult(eFlashResult));

        /* Anything the driver has had to survive gets said out loud, even when
         * the verdict is OK — a unit that recovered, or that is running on a
         * chip whose protection bits had to be cleared, is not the same as one
         * that was clean, and the distinction is invisible from "flash=OK". */
        Flash_Health_t tHealth;
        FLASH_vGetHealth(&tHealth);
        /* bWriteProtected is deliberately NOT a trigger any more. On the
         * AT25XE041B the array's power-on default is protected and every
         * ultra-deep-power-down exit restores that default, so bring-up clears
         * it on literally every boot - printing a health line for it would
         * make the one line that means "something is wrong" fire every time
         * and stop meaning anything. What still matters is an unprotect that
         * did NOT take. */
        if (tHealth.bEverAbsent || tHealth.bUnprotectFailed ||
            tHealth.bEraseVerifyFail || tHealth.bEpeSeen ||
            (tHealth.u16ReadCrcFails > 0U))
            DBG_LOG("SelfTest: flash health - absent=%u recoveries=%u probeFails=%u "
                    "wprot=%u unprotFail=%u eraseVfyFails=%u epe=%u "
                    "crcRetries=%u crcFails=%u\r\n",
                    (unsigned)tHealth.bEverAbsent,
                    (unsigned)tHealth.u16Recoveries,
                    (unsigned)tHealth.u16ProbeFailures,
                    (unsigned)tHealth.bWriteProtected,
                    (unsigned)tHealth.bUnprotectFailed,
                    (unsigned)tHealth.u16EraseVerifyFails,
                    (unsigned)tHealth.u16EpeFails,
                    (unsigned)tHealth.u16ReadCrcRetries,
                    (unsigned)tHealth.u16ReadCrcFails);
    }

    DBG_LOG("SelfTest: gps=%s acc=%s flash=%s\r\n",
            bGpsRan   ? (bGpsOk   ? "OK" : "FAIL") : "n/a",
            bAccOk    ? "OK" : "FAIL",
            bFlashRan ? (bFlashOk ? "OK" : FLASH_pcSelfTestResult(eFlashResult))
                      : "n/a");

    /* Collect the failure codes in enum order so a multi-failure sequence
     * is deterministic. */
    uint8_t au8Codes[3];
    uint8_t u8Count = 0U;
    if (bGpsRan   && !bGpsOk)   au8Codes[u8Count++] = (uint8_t)SELFTEST_ERR_GPS;
    if              (!bAccOk)   au8Codes[u8Count++] = (uint8_t)SELFTEST_ERR_ACC;
    if (bFlashRan && !bFlashOk) au8Codes[u8Count++] = (uint8_t)SELFTEST_ERR_FLASH;

    for (uint8_t i = 0; i < u8Count; i++)
    {
        selftest_vFlashCode(au8Codes[i]);
        if (i + 1U < u8Count)
            osDelay(SELFTEST_GAP_BETWEEN_MS);
    }

    BSP_LED_Off(LED_YELLOW);
}

/* --- Query API ------------------------------------------------------------ */

bool SELFTEST_bGpsOk(void)   { return bGpsOk;   }
bool SELFTEST_bAccOk(void)   { return bAccOk;   }

/* Live, not memoized — deliberately unlike its two neighbours. The flash gate
 * can now both close and re-open at run time (see FLASH_bEnsurePresent), so a
 * boot-time snapshot answers a question nobody is asking: an operator running
 * "tag <ID> selftest" wants to know whether the log and FOTA staging work NOW.
 * Answering from a cached boot result is the same mistake that let a gated-off
 * tag report flash=OK for 18 h. */
bool SELFTEST_bFlashOk(void)
{
    return bFlashRan ? FLASH_bDevicePresent() : bFlashOk;
}

bool SELFTEST_bGpsApplicable(void)
{
    return DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY;
}

/* The boot verdict in words. Live pass/fail still comes from
 * SELFTEST_bFlashOk(); this is the extra detail an operator needs to act on it,
 * which is why it is a separate accessor rather than a widened return. */
const char *SELFTEST_pcFlashDetail(void)
{
    return FLASH_pcSelfTestResult(eFlashResult);
}

bool SELFTEST_bFlashApplicable(void)
{
    return true;
}
