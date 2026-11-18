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
#include "LoraRadio.h"             /* noise floor, radio id          */
#include "Battery.h"               /* BAT_u16GetVoltage              */
#include "SolarPower.h"            /* SOLAR_u16GetVSolarMV           */
#include "Power.h"                 /* POWER_CLASS_NORMAL             */
#include "Identity.h"              /* serial number                  */
#include "version_config.h"

#include <stdio.h>

/* Result flags default to true so a not-applicable test naturally reports
 * OK (GPS and ACC on primary). */
static bool bGpsOk       = true;
static bool bAccOk       = true;
static bool bFlashOk     = true;

static bool bGpsRan      = false;   /* whether the test actually executed */
static bool bAccRan      = false;
static bool bFlashRan    = false;

/* Kept so a human asking later gets the same verdict the boot log carried, not
 * just a bool. "flash=FAIL" tells an operator to do something; "FAIL:wrong-part"
 * tells them what. */
static Flash_SelfTestResult_t eFlashResult = FLASH_ST_PASS;

/* --- LED flash sequence ---------------------------------------------------
 * User spec: for each failed test, flash the code (N short flashes) three
 * times with a 2 s gap between repeats. If more than one test failed, put
 * a 3 s gap between codes. Green LED off at the end so the existing
 * BSP_LED_Off(LED_GREEN) at the init tail has nothing left to clean up.
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
            BSP_LED_On(LED_GREEN);
            osDelay(SELFTEST_FLASH_ON_MS);
            BSP_LED_Off(LED_GREEN);
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

    /* Accelerometer — secondary only. A Tagskaap V3 RevA primary has no
     * accelerometer fitted and never starts its SPI bus, so a WHO_AM_I read
     * there could only ever fail: that is what used to report acc=FAIL and
     * flash error code 2 (about 6 s of blocking LED sequence) on every
     * primary boot. On a primary bAccOk keeps its true default, the n/a
     * state, exactly like GPS.
     *
     * Retried and raw-byte reported for the same reason as the flash JEDEC
     * read below: a lone bad WHO_AM_I has been seen from a healthy part (cause
     * unproven - see ACC_bDeviceIdOkEx), and a bare pass/fail can't tell that
     * from a real fault. */
    if (SELFTEST_bAccApplicable())
    {
        uint8_t u8Id       = 0U;
        uint8_t u8Attempts = 0U;

        bAccRan = true;
        bAccOk  = ACC_bDeviceIdOkEx(&u8Id, &u8Attempts);

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
        /* bWriteProtected IS a trigger again. On the AT25XE041B (briefly
         * targeted during V3 bring-up) the array's power-on default was
         * protected and every ultra-deep-power-down exit restored it, so it
         * fired on every boot and had to be excluded. The AT25EU0041A that
         * V3 RevA actually fits powers up unprotected (Table 3) with
         * non-volatile protection bits, so a protected state seen at bring-up
         * means something wrote the status register - worth the line even
         * when the unprotect then succeeded. (The XE041B's EPE counter is
         * gone: this part has no such bit.) */
        if (tHealth.bEverAbsent || tHealth.bWriteProtected ||
            tHealth.bUnprotectFailed || tHealth.bEraseVerifyFail ||
            (tHealth.u16ReadCrcFails > 0U))
            DBG_LOG("SelfTest: flash health - absent=%u recoveries=%u probeFails=%u "
                    "wprot=%u unprotFail=%u eraseVfyFails=%u "
                    "crcRetries=%u crcFails=%u\r\n",
                    (unsigned)tHealth.bEverAbsent,
                    (unsigned)tHealth.u16Recoveries,
                    (unsigned)tHealth.u16ProbeFailures,
                    (unsigned)tHealth.bWriteProtected,
                    (unsigned)tHealth.bUnprotectFailed,
                    (unsigned)tHealth.u16EraseVerifyFails,
                    (unsigned)tHealth.u16ReadCrcRetries,
                    (unsigned)tHealth.u16ReadCrcFails);
    }

    DBG_LOG("SelfTest: gps=%s acc=%s flash=%s\r\n",
            bGpsRan   ? (bGpsOk   ? "OK" : "FAIL") : "n/a",
            bAccRan   ? (bAccOk   ? "OK" : "FAIL") : "n/a",
            bFlashRan ? (bFlashOk ? "OK" : FLASH_pcSelfTestResult(eFlashResult))
                      : "n/a");

    /* Collect the failure codes in enum order so a multi-failure sequence
     * is deterministic. A test that did not run never contributes a code, so
     * a clean primary flashes nothing. */
    uint8_t au8Codes[3];
    uint8_t u8Count = 0U;
    if (bGpsRan   && !bGpsOk)   au8Codes[u8Count++] = (uint8_t)SELFTEST_ERR_GPS;
    if (bAccRan   && !bAccOk)   au8Codes[u8Count++] = (uint8_t)SELFTEST_ERR_ACC;
    if (bFlashRan && !bFlashOk) au8Codes[u8Count++] = (uint8_t)SELFTEST_ERR_FLASH;

    for (uint8_t i = 0; i < u8Count; i++)
    {
        selftest_vFlashCode(au8Codes[i]);
        if (i + 1U < u8Count)
            osDelay(SELFTEST_GAP_BETWEEN_MS);
    }

    BSP_LED_Off(LED_GREEN);
}

/* --- Query API ------------------------------------------------------------ */

bool SELFTEST_bGpsOk(void)   { return bGpsOk;   }
bool SELFTEST_bAccOk(void)   { return bAccOk;   }

/* Live, not memoized — deliberately unlike its two neighbours. The flash gate
 * can now both close and re-open at run time (see FLASH_bEnsurePresent), so a
 * boot-time snapshot answers a question nobody is asking: an operator running
 * kernel command 'e' wants to know whether the log and FOTA staging work NOW.
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

/* Same rule as GPS: the accelerometer is fitted on a secondary only, and the
 * FRKERNEL_INTERFACE_LORA_CONSOLE build never starts SPI1 or runs ACC_vInit on
 * either role (init.c), so a WHO_AM_I read there could only fail - n/a, as
 * GPS_bSelfTest reports for the same build. Also what SELFTEST_vRunAndReport
 * uses to decide whether to run the WHO_AM_I test, so the boot run and
 * FrKernel's later "n/a" cannot disagree. */
bool SELFTEST_bAccApplicable(void)
{
#ifdef FRKERNEL_INTERFACE_LORA_CONSOLE
    return false;
#else
    return DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY;
#endif
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

/* --------------------------------------------------------------------------
 * Production test - see SELFTEST_bRunProductionTest in SelfTest.h.
 *
 * Everything here is measured LIVE, which is the difference from the boot
 * snapshot behind 'e': a production station must see the board as it is now,
 * on the station's supply, not as it was at power-up a minute earlier.
 * -------------------------------------------------------------------------- */

/* GPS: the boot test's 1500 ms is one NMEA epoch with little margin; at the
 * station a failed unit costs a re-test, so give the module twice that. */
#define SELFTEST_PROD_GPS_TIMEOUT_MS    3000U

/* Radio: noise-floor samples are taken on the radio task's 50 ms idle passes;
 * 500 ms is ten of them. A working receiver on a bench reads somewhere around
 * -120 to -90 dBm; the window below only rejects readings that cannot come
 * from a live front end (a dead SPI reads 0 or garbage, an RSSI register that
 * was never updated reads the invalid marker). */
#define SELFTEST_PROD_NF_SETTLE_MS      500U
#define SELFTEST_PROD_NF_MIN_DBM        (-140)
#define SELFTEST_PROD_NF_MAX_DBM        (-40)

bool SELFTEST_bRunProductionTest(uint32_t u32HoldS, SelfTestReply_f pfReply, void *pvCtx)
{
    char acLine[200];
    const bool bSecondary = (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY);

    if (u32HoldS > SELFTEST_PROD_HOLD_MAX_S)
        u32HoldS = SELFTEST_PROD_HOLD_MAX_S;

    /* ---- flash: live gate, plus the boot detail for the operator ---- */
    const bool bFlash = SELFTEST_bFlashOk();

    /* ---- accelerometer: fresh WHO_AM_I ---- */
    bool bAcc = true;
    if (SELFTEST_bAccApplicable())
        bAcc = ACC_bDeviceIdOkEx(NULL, NULL);

    /* ---- GPS: NMEA on the wire. Any session is shut first so the test has
     * the module and its UART to itself. ---- */
    bool bGps = true;
    if (SELFTEST_bGpsApplicable())
    {
        GPS_vShutdown();
        bGps = GPS_bSelfTest(SELFTEST_PROD_GPS_TIMEOUT_MS);
    }

    /* ---- radio: awake, in RX, producing plausible RSSI ---- */
    LORARADIO_vSetKeepAwake(true);
    LORARADIO_vWakeUp();
    LORARADIO_vSetNoiseFloorSampling(true);
    osDelay(SELFTEST_PROD_NF_SETTLE_MS);
    const int16_t i16Nf   = LORARADIO_i16GetNoiseFloor();
    const bool    bRadio  = (i16Nf != LORA_NOISE_FLOOR_INVALID) &&
                            (i16Nf >= SELFTEST_PROD_NF_MIN_DBM) &&
                            (i16Nf <= SELFTEST_PROD_NF_MAX_DBM);

    const bool bPass = bFlash && bAcc && bGps && bRadio;

    char acNf[8], acBat[8], acSol[8];
    if (i16Nf == LORA_NOISE_FLOOR_INVALID) (void)snprintf(acNf, sizeof(acNf), "none");
    else                                   (void)snprintf(acNf, sizeof(acNf), "%d", (int)i16Nf);
    if (bSecondary)
    {
        (void)snprintf(acBat, sizeof(acBat), "%u", (unsigned)BAT_u16GetVoltage());
        (void)snprintf(acSol, sizeof(acSol), "%u", (unsigned)SOLAR_u16GetVSolarMV());
    }
    else
    {
        (void)snprintf(acBat, sizeof(acBat), "na");
        (void)snprintf(acSol, sizeof(acSol), "na");
    }

    (void)snprintf(acLine, sizeof(acLine),
                   "PT v1 role=%s result=%s flash=%s flashdetail=%s acc=%s gps=%s radio=%s "
                   "nf=%s bat=%s sol=%s sn=%lu id=%08lX fw=%u.%u.%u\r\n",
                   bSecondary ? "secondary" : "primary",
                   bPass ? "PASS" : "FAIL",
                   bFlash ? "ok" : "FAIL",
                   SELFTEST_pcFlashDetail(),
                   SELFTEST_bAccApplicable() ? (bAcc ? "ok" : "FAIL") : "na",
                   SELFTEST_bGpsApplicable() ? (bGps ? "ok" : "FAIL") : "na",
                   bRadio ? "ok" : "FAIL",
                   acNf, acBat, acSol,
                   (unsigned long)IDENTITY_u32GetSerial(),
                   (unsigned long)LORARADIO_u32GetUniqueId(),
                   (unsigned)VERSION_SW_MAJOR, (unsigned)VERSION_SW_MINOR,
                   (unsigned)VERSION_SW_PATCH);
    pfReply(pvCtx, acLine);
    DBG_LOG("SelfTest: production test %s\r\n", bPass ? "PASS" : "FAIL");

    /* ---- all-on hold ---- */
    if (u32HoldS > 0U)
    {
        /* GPS powered and acquiring, no auto-shutdown; bForce so a GPS-disabled
         * system setting cannot leave the module off during the measurement.
         * GPS_vRequestFix refuses outside POWER_CLASS_NORMAL, which on a
         * station supply means the supply is set too low - say so, because
         * the all-on current would then be missing the GPS. */
        if (bSecondary)
        {
            if (POWER_tGetState() & POWER_CLASS_NORMAL)
                GPS_vRequestFix(false, 0U, true);
            else
                pfReply(pvCtx, "PT note gps not powered - supply below POWER_CLASS_NORMAL\r\n");
        }
        FLASH_vInhibitDeepPowerDown(true);

        (void)snprintf(acLine, sizeof(acLine), "PT hold %lu s\r\n", (unsigned long)u32HoldS);
        pfReply(pvCtx, acLine);

        osDelay(u32HoldS * 1000U);

        if (bSecondary)
            GPS_vShutdown();
        FLASH_vInhibitDeepPowerDown(false);
    }

    /* Hand the radio back to its normal owners; the session's end puts it to
     * sleep the usual way. */
    LORARADIO_vSetNoiseFloorSampling(false);
    LORARADIO_vSetKeepAwake(false);

    pfReply(pvCtx, "PT done\r\n");
    return bPass;
}
