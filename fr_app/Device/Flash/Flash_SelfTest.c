/*
 * Flash_SelfTest.c — see Flash_SelfTest.h for the contract.
 *
 * The design rule for everything here: a diagnostic that says "failed" is
 * barely better than no diagnostic. This board has a documented history of
 * healthy parts being declared dead by a single bad read, and of a gated-off
 * driver reporting flash=OK for 18 hours, so a test that cannot distinguish
 * "the chip is absent" from "the chip answered and the bytes were wrong" would
 * be repeating a mistake this codebase has already paid for twice.
 *
 * Hence: every raw value is printed whether it matched or not, every mismatch
 * is classified, and every failure is re-checked once so a transient and a
 * hard fault are never reported the same way.
 */

#include "Flash_SelfTest.h"

#include "Flash.h"
#include "Flash_Config.h"
#include "build_config.h"
#include "hal_spi.h"               /* FLASH_SPI_CLOCK_HZ, for the failure hint */
#include "dbg_log.h"
#include "cmsis_os2.h"
#include <string.h>

/* Everything from here to the stress harness is shared by the destructive half
 * of the self-test and by the stress loop. With neither built in, none of it is
 * reachable, so it is compiled out rather than left as unused statics. */
#if defined(ENABLE_FLASH_SELFTEST_DESTRUCTIVE) || defined(ENABLE_FLASH_STRESS_TEST)
#  define FLASH_SELFTEST_NEED_PATTERN  1
#endif

#ifdef FLASH_SELFTEST_NEED_PATTERN

/* --------------------------------------------------------------------------
 * The test pattern
 *
 * A 32-bit xorshift with a fixed seed, generated on the fly rather than stored.
 * Two properties are doing work here:
 *
 *  - Fixed seed, so the same bytes appear at the same offsets on every boot and
 *    on every unit. A pattern that differed per run could not be checked by the
 *    stress harness later, and two units could not be compared.
 *  - Pseudorandom rather than a counting ramp. A ramp survives a one-bit shift
 *    or a one-byte rotation looking almost right, which is exactly the failure
 *    this test most wants to catch; a random pattern turns the same fault into
 *    an obvious, and detectable, misalignment.
 * -------------------------------------------------------------------------- */
static uint32_t FLASH_u32Xorshift(uint32_t u32State)
{
    u32State ^= u32State << 13;
    u32State ^= u32State >> 17;
    u32State ^= u32State << 5;
    return u32State;
}

static void FLASH_vFillPattern(uint8_t *pu8Buf, uint16_t u16Len)
{
    uint32_t u32State = FLASH_SELFTEST_PRNG_SEED;

    for (uint16_t i = 0U; i < u16Len; i++)
    {
        u32State = FLASH_u32Xorshift(u32State);
        pu8Buf[i] = (uint8_t)(u32State & 0xFFU);
    }
}

/* Working buffers. Static, not stack: this runs on the init task at boot and on
 * the FrKernel task later, and 512 bytes of stack is not something to demand of
 * either. Nothing here is re-entrant and nothing needs it to be. */
static uint8_t au8Expected[FLASH_SELFTEST_PATTERN_LEN];
static uint8_t au8Actual[FLASH_SELFTEST_PATTERN_LEN];

/* --------------------------------------------------------------------------
 * Mismatch classification
 *
 * The four outcomes point at four different physical causes, which is the
 * whole reason for classifying rather than counting:
 *
 *   all FFh   the device is not driving MISO. Still in ultra-deep-power-down
 *             (an unhonoured tXUDPD), still busy, absent, or CS never went low
 *             - the pull-up on PA5 is what makes an undriven line read as FFh.
 *   all 00h   MISO is being held low: a short, a pin still parked from STOP2
 *             and never restored to its alternate function, or CS stuck
 *             asserted so the device never started the transaction.
 *   shifted   the right bytes arrived in the wrong alignment. That is a clock
 *             or sampling-phase fault - a glitch on SCK adding or losing an
 *             edge, or CPOL/CPHA disagreeing with the device - and it is the
 *             one classification that says "the data is fine, the transport
 *             is not".
 *   noise     none of the above. Individual bytes wrong in no pattern, which
 *             is what a supply-rail disturbance during the transfer looks
 *             like, and is the classification the TX-corruption hypothesis
 *             predicts.
 * -------------------------------------------------------------------------- */

/* Is the received buffer the expected one, displaced? Tries whole-byte
 * rotations either way and whole-buffer bit shifts either way. Bounded
 * deliberately: a displacement larger than this is indistinguishable from
 * noise, and searching further would start finding coincidences. */
static bool FLASH_bLooksShifted(const uint8_t *pu8Exp, const uint8_t *pu8Act,
                                uint16_t u16Len)
{
    /* Byte rotations, +/- 1..4. Compare the overlapping region only. */
    for (uint16_t u16Off = 1U; u16Off <= 4U; u16Off++)
    {
        if (u16Len <= u16Off)
            break;
        uint16_t u16Cmp = (uint16_t)(u16Len - u16Off);
        if (memcmp(pu8Exp + u16Off, pu8Act, u16Cmp) == 0) return true;
        if (memcmp(pu8Exp, pu8Act + u16Off, u16Cmp) == 0) return true;
    }

    /* Bit shifts, 1..7, applied across the whole buffer in both directions.
     * A dropped or duplicated SCK edge shows up exactly like this. */
    for (uint8_t u8Bits = 1U; u8Bits <= 7U; u8Bits++)
    {
        bool bLeft  = true;
        bool bRight = true;

        for (uint16_t i = 0U; (i + 1U) < u16Len; i++)
        {
            uint8_t u8Carry = (i > 0U)
                            ? (uint8_t)(pu8Exp[i - 1U] << (8U - u8Bits))
                            : (uint8_t)0U;
            uint8_t u8L = (uint8_t)((pu8Exp[i] << u8Bits) |
                                    (pu8Exp[i + 1U] >> (8U - u8Bits)));
            uint8_t u8R = (uint8_t)((pu8Exp[i] >> u8Bits) | u8Carry);
            if (pu8Act[i] != u8L) bLeft  = false;
            if (pu8Act[i] != u8R) bRight = false;
            if (!bLeft && !bRight) break;
        }

        if (bLeft || bRight) return true;
    }

    return false;
}

static Flash_Mismatch_t FLASH_eClassify(const uint8_t *pu8Exp,
                                        const uint8_t *pu8Act,
                                        uint16_t u16Len)
{
    if (memcmp(pu8Exp, pu8Act, u16Len) == 0)
        return FLASH_MISMATCH_NONE;

    bool bAllFF = true;
    bool bAll00 = true;
    for (uint16_t i = 0U; i < u16Len; i++)
    {
        if (pu8Act[i] != 0xFFU) bAllFF = false;
        if (pu8Act[i] != 0x00U) bAll00 = false;
        if (!bAllFF && !bAll00) break;
    }

    if (bAllFF) return FLASH_MISMATCH_ALL_FF;
    if (bAll00) return FLASH_MISMATCH_ALL_00;
    if (FLASH_bLooksShifted(pu8Exp, pu8Act, u16Len)) return FLASH_MISMATCH_SHIFTED;
    return FLASH_MISMATCH_NOISE;
}

#endif /* FLASH_SELFTEST_NEED_PATTERN */

const char *FLASH_pcMismatchName(Flash_Mismatch_t eKind)
{
    switch (eKind)
    {
        case FLASH_MISMATCH_NONE:    return "none";
        case FLASH_MISMATCH_ALL_FF:  return "all-FF (not responding: parked, busy or CS never asserted)";
        case FLASH_MISMATCH_ALL_00:  return "all-00 (MISO stuck low or CS never released)";
        case FLASH_MISMATCH_SHIFTED: return "shifted (clock glitch or sampling phase)";
        default:                     return "noise";
    }
}

const char *FLASH_pcSelfTestResult(Flash_SelfTestResult_t eResult)
{
    switch (eResult)
    {
        case FLASH_ST_PASS:             return "PASS";
        case FLASH_ST_FAIL_NO_COMMS:    return "FAIL:no-comms";
        case FLASH_ST_FAIL_WRONG_PART:  return "FAIL:wrong-part";
        case FLASH_ST_FAIL_BUSY_STUCK:  return "FAIL:busy-stuck";
        case FLASH_ST_FAIL_PROTECTED:   return "FAIL:protected";
        case FLASH_ST_FAIL_ERASE:       return "FAIL:erase";
        case FLASH_ST_FAIL_PROGRAM:     return "FAIL:program";
        default:                        return "?";
    }
}

/* First 16 bytes of each, side by side. Enough to see the shape of a fault
 * without turning a boot log into a hex dump. */
#ifdef ENABLE_FLASH_SELFTEST_DESTRUCTIVE
static void FLASH_vLogFirstBytes(const uint8_t *pu8Exp, const uint8_t *pu8Act)
{
    _DBG_LOG_("  expected: %02X %02X %02X %02X %02X %02X %02X %02X "
              "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
              pu8Exp[0], pu8Exp[1], pu8Exp[2],  pu8Exp[3],  pu8Exp[4],  pu8Exp[5],
              pu8Exp[6], pu8Exp[7], pu8Exp[8],  pu8Exp[9],  pu8Exp[10], pu8Exp[11],
              pu8Exp[12], pu8Exp[13], pu8Exp[14], pu8Exp[15]);
    _DBG_LOG_("  actual:   %02X %02X %02X %02X %02X %02X %02X %02X "
              "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
              pu8Act[0], pu8Act[1], pu8Act[2],  pu8Act[3],  pu8Act[4],  pu8Act[5],
              pu8Act[6], pu8Act[7], pu8Act[8],  pu8Act[9],  pu8Act[10], pu8Act[11],
              pu8Act[12], pu8Act[13], pu8Act[14], pu8Act[15]);
}
#endif /* ENABLE_FLASH_SELFTEST_DESTRUCTIVE */

/* --------------------------------------------------------------------------
 * The self-test
 * -------------------------------------------------------------------------- */

Flash_SelfTestResult_t FLASH_eSelfTest(void)
{
    /* ---- 1. Identification -------------------------------------------------
     *
     * The raw bytes are printed unconditionally, matched or not. A verdict
     * alone cannot tell a technician whether to reflow a joint, replace the
     * part or look at the firmware, and those bytes can: 00s and FFs are
     * wiring, a plausible-but-wrong ID is the wrong part, anything else is
     * usually timing. */
    uint8_t au8Id[3]   = {0};
    uint8_t u8Attempts = 0U;
    bool    bIdOk      = FLASH_bRecoverDevice(au8Id, &u8Attempts);

    _DBG_LOG_("FlashTest: JEDEC %02X %02X %02X (expected %02X %02X %02X) "
              "after %u attempt(s)\r\n",
              au8Id[0], au8Id[1], au8Id[2],
              (unsigned)FLASH_MANUFACTURER_ID,
              (unsigned)FLASH_DEVICE_ID1,
              (unsigned)FLASH_DEVICE_ID2,
              (unsigned)u8Attempts);

    if (!bIdOk)
    {
        bool bAllSame = (au8Id[0] == au8Id[1]) && (au8Id[1] == au8Id[2]);

        if (bAllSame && ((au8Id[0] == 0x00U) || (au8Id[0] == 0xFFU)))
        {
            /* Nothing is driving the bus. Not a flash fault as such - this is
             * what an unpopulated footprint, a dry joint, a CS that never goes
             * low, or SPI2 pins left parked analog after a STOP2 wake all look
             * like from here. */
            DBG_LOG("FlashTest: no communication - ID read back all %02X. Check "
                    "CS#/SCK/MOSI/MISO, the part being fitted, and that "
                    "HAL_SPI_FLASH_vEnsureAwake restored the bus\r\n", au8Id[0]);
            return FLASH_ST_FAIL_NO_COMMS;
        }

        if (au8Id[0] == FLASH_MANUFACTURER_ID)
        {
            /* Reads cleanly, is a Renesas part, is not this one. The bus is
             * fine; the BOM is not. Worth its own verdict because every other
             * failure here sends you to the wrong place. */
            DBG_LOG("FlashTest: WRONG PART fitted - manufacturer %02X is right "
                    "but device ID %02X %02X is not %02X %02X\r\n",
                    au8Id[0], au8Id[1], au8Id[2],
                    (unsigned)FLASH_DEVICE_ID1, (unsigned)FLASH_DEVICE_ID2);
            return FLASH_ST_FAIL_WRONG_PART;
        }

        DBG_LOG("FlashTest: unrecognised ID - bus is active but the bytes are "
                "wrong. Suspect SPI mode (this part needs mode 0 or 3), clock "
                "rate, or signal integrity at %lu Hz\r\n",
                (unsigned long)FLASH_SPI_CLOCK_HZ);
        return FLASH_ST_FAIL_WRONG_PART;
    }

    /* ---- 2. Status register ------------------------------------------------
     *
     * Reported by name, not as a hex byte to be decoded by hand. WEL set while
     * idle is a finding in itself: it means a previous command sequence was
     * interrupted after its WREN, and on this board - WP# grounded, SPRL a
     * one-way latch - an armed device is one stray WRSR away from being
     * permanently write-locked. */
    uint8_t u8Sr1 = FLASH_u8ReadStatusReg();

    _DBG_LOG_("FlashTest: SR1=%02X SPRL=%u SPM=%u EPE=%u WPP=%u(WP#%s) SWP=%u "
              "WEL=%u BSY=%u\r\n",
              u8Sr1,
              (unsigned)((u8Sr1 & FLASH_SR1_SPRL)    ? 1U : 0U),
              (unsigned)((u8Sr1 & FLASH_SR1_SPM)     ? 1U : 0U),
              (unsigned)((u8Sr1 & FLASH_SR1_EPE)     ? 1U : 0U),
              (unsigned)((u8Sr1 & FLASH_SR1_WPP)     ? 1U : 0U),
              (u8Sr1 & FLASH_SR1_WPP) ? "=high" : "=GND",
              (unsigned)((u8Sr1 & FLASH_SR1_SWP_MASK) >> 2),
              (unsigned)((u8Sr1 & FLASH_SR1_WEL)     ? 1U : 0U),
              (unsigned)((u8Sr1 & FLASH_SR1_RDY_BSY) ? 1U : 0U));

    if (u8Sr1 & FLASH_SR1_WEL)
        DBG_LOG("FlashTest: WEL is SET at idle - a command sequence was "
                "interrupted after its WREN. With WP# grounded an armed device "
                "is one stray WRSR from a permanent write-lock\r\n");

    if (u8Sr1 & FLASH_SR1_RDY_BSY)
    {
        /* Busy long after boot, with no operation outstanding, means the
         * status read itself is not working - which would make every
         * subsequent step wait out a timeout for nothing. */
        DBG_LOG("FlashTest: RDY/BSY still set with no operation outstanding - "
                "the status read is not returning real data\r\n");
        return FLASH_ST_FAIL_BUSY_STUCK;
    }

    if (u8Sr1 & FLASH_STATUS_PROTECTED)
    {
        /* FLASH_bRecoverDevice above has already run bring-up, which performs
         * a Global Unprotect. Still protected here means the unprotect did not
         * take - and with WP# grounded and SPRL latched, cannot. */
        DBG_LOG("FlashTest: array still protected after bring-up (SWP=%u "
                "SPRL=%u) - writes and erases will be silently refused\r\n",
                (unsigned)((u8Sr1 & FLASH_SR1_SWP_MASK) >> 2),
                (unsigned)((u8Sr1 & FLASH_SR1_SPRL) ? 1U : 0U));
        return FLASH_ST_FAIL_PROTECTED;
    }

#ifndef ENABLE_FLASH_SELFTEST_DESTRUCTIVE
    _DBG_LOG_("FlashTest: read-only checks passed; erase/program test not "
              "built in (ENABLE_FLASH_SELFTEST_DESTRUCTIVE)\r\n");
    return FLASH_ST_PASS;
#else

    /* ---- 3. Erase the reserved sector, and verify ALL of it -----------------
     *
     * The driver's own erase verify samples 32 bytes at each end, which is the
     * right trade on a hot path. This is not a hot path: it runs once per boot
     * on a sector nobody else owns, so it checks every byte. A partial erase
     * that leaves a hole in the middle is exactly the fault a two-ended sample
     * is blind to. */
    if (!FLASH_vSectorErase(FLASH_SELFTEST_ADDR))
    {
        DBG_LOG("FlashTest: sector erase @%06lX FAILED\r\n",
                (unsigned long)FLASH_SELFTEST_ADDR);
        return FLASH_ST_FAIL_ERASE;
    }

    for (uint32_t u32Off = 0UL; u32Off < FLASH_SELFTEST_LEN;
         u32Off += FLASH_SELFTEST_PATTERN_LEN)
    {
        if (!FLASH_vRead(FLASH_SELFTEST_ADDR + u32Off, au8Actual,
                         FLASH_SELFTEST_PATTERN_LEN))
        {
            DBG_LOG("FlashTest: read-back of erased sector FAILED at +%04lX\r\n",
                    (unsigned long)u32Off);
            return FLASH_ST_FAIL_ERASE;
        }

        for (uint16_t i = 0U; i < FLASH_SELFTEST_PATTERN_LEN; i++)
        {
            if (au8Actual[i] != FLASH_ERASED_BYTE)
            {
                DBG_LOG("FlashTest: erase did not blank @%06lX: byte %02X at "
                        "offset +%04lX\r\n",
                        (unsigned long)(FLASH_SELFTEST_ADDR + u32Off + i),
                        au8Actual[i], (unsigned long)(u32Off + i));
                return FLASH_ST_FAIL_ERASE;
            }
        }
    }

    /* ---- 4. Program a known pattern and read it back ----------------------- */
    FLASH_vFillPattern(au8Expected, FLASH_SELFTEST_PATTERN_LEN);

    if (!FLASH_vPageWrite(FLASH_SELFTEST_ADDR, au8Expected,
                          FLASH_SELFTEST_PATTERN_LEN))
    {
        DBG_LOG("FlashTest: page program @%06lX FAILED\r\n",
                (unsigned long)FLASH_SELFTEST_ADDR);
        return FLASH_ST_FAIL_PROGRAM;
    }

    memset(au8Actual, 0, sizeof(au8Actual));
    if (!FLASH_vRead(FLASH_SELFTEST_ADDR, au8Actual, FLASH_SELFTEST_PATTERN_LEN))
    {
        DBG_LOG("FlashTest: read-back after program FAILED\r\n");
        return FLASH_ST_FAIL_PROGRAM;
    }

    Flash_Mismatch_t eKind = FLASH_eClassify(au8Expected, au8Actual,
                                             FLASH_SELFTEST_PATTERN_LEN);
    if (eKind == FLASH_MISMATCH_NONE)
    {
        _DBG_LOG_("FlashTest: erase + program + verify OK (%u bytes @%06lX)\r\n",
                  (unsigned)FLASH_SELFTEST_PATTERN_LEN,
                  (unsigned long)FLASH_SELFTEST_ADDR);
        return FLASH_ST_PASS;
    }

    /* ---- 5/6. Classify, then ask again ------------------------------------
     *
     * The second read is the most valuable line in this whole test. A fault
     * that repeats identically is in the stored bytes or in a fixed property
     * of the bus; a fault that clears, or comes back DIFFERENT, is a transient
     * - and those two point at completely different investigations. This board
     * has a documented case of exactly the second kind: a read right after a
     * TX was wrong, an immediate re-read was also wrong AND DIFFERENT, and a
     * third read ~65 ms later was correct. */
    DBG_LOG("FlashTest: pattern MISMATCH - %s\r\n", FLASH_pcMismatchName(eKind));
    FLASH_vLogFirstBytes(au8Expected, au8Actual);

    static uint8_t au8Second[FLASH_SELFTEST_PATTERN_LEN];
    memset(au8Second, 0, sizeof(au8Second));

    if (!FLASH_vRead(FLASH_SELFTEST_ADDR, au8Second, FLASH_SELFTEST_PATTERN_LEN))
    {
        DBG_LOG("FlashTest: second read FAILED outright\r\n");
        return FLASH_ST_FAIL_PROGRAM;
    }

    if (memcmp(au8Expected, au8Second, FLASH_SELFTEST_PATTERN_LEN) == 0)
        DBG_LOG("FlashTest: second read is CORRECT - the first was a transient. "
                "The stored bytes are fine; the read path is disturbed by "
                "something. Use 'flash stress' to find out by what\r\n");
    else if (memcmp(au8Actual, au8Second, FLASH_SELFTEST_PATTERN_LEN) == 0)
        DBG_LOG("FlashTest: second read repeats the same wrong bytes - a HARD "
                "fault in the stored data or in the bus, not a transient\r\n");
    else
        DBG_LOG("FlashTest: second read is wrong AGAIN and DIFFERENTLY (%s) - "
                "the classic signature of a disturbance during the transfer\r\n",
                FLASH_pcMismatchName(FLASH_eClassify(au8Expected, au8Second,
                                                     FLASH_SELFTEST_PATTERN_LEN)));

    return FLASH_ST_FAIL_PROGRAM;
#endif /* ENABLE_FLASH_SELFTEST_DESTRUCTIVE */
}

/* --------------------------------------------------------------------------
 * The stress harness
 * -------------------------------------------------------------------------- */

#ifdef ENABLE_FLASH_STRESS_TEST

bool FLASH_bStressTest(uint32_t u32Cycles,
                       void (*pfnBetweenCycles)(void),
                       Flash_StressStats_t *ptStats)
{
    if (ptStats == NULL)
        return false;

    memset(ptStats, 0, sizeof(*ptStats));
    FLASH_vFillPattern(au8Expected, FLASH_SELFTEST_PATTERN_LEN);

    /* Establish that the pattern is actually there before measuring anything.
     * Without this a run on a blank sector would report 100% mismatch and say
     * nothing whatsoever about the bus - a confident, precise, meaningless
     * number, which is worse than an error. */
    if (!FLASH_vRead(FLASH_SELFTEST_ADDR, au8Actual, FLASH_SELFTEST_PATTERN_LEN) ||
        (memcmp(au8Expected, au8Actual, FLASH_SELFTEST_PATTERN_LEN) != 0))
    {
        DBG_LOG("FlashStress: reference page @%06lX does not hold the test "
                "pattern - run the self-test first\r\n",
                (unsigned long)FLASH_SELFTEST_ADDR);
        return false;
    }

    /* Clear the driver-level read-verify tally so what this run reports is
     * what this run caused. */
    (void)FLASH_u32GetAndClearReadCrcStats(NULL, NULL);

    for (uint32_t u32i = 1UL; u32i <= u32Cycles; u32i++)
    {
        memset(au8Actual, 0, sizeof(au8Actual));

        if (!FLASH_vRead(FLASH_SELFTEST_ADDR, au8Actual, FLASH_SELFTEST_PATTERN_LEN))
        {
            ptStats->u32ReadFail++;
            if (ptStats->u32FirstBadCycle == 0UL) ptStats->u32FirstBadCycle = u32i;
        }
        else
        {
            Flash_Mismatch_t eKind = FLASH_eClassify(au8Expected, au8Actual,
                                                     FLASH_SELFTEST_PATTERN_LEN);
            if (eKind != FLASH_MISMATCH_NONE)
            {
                ptStats->u32Mismatch++;
                if (ptStats->u32FirstBadCycle == 0UL) ptStats->u32FirstBadCycle = u32i;

                switch (eKind)
                {
                    case FLASH_MISMATCH_ALL_FF:  ptStats->u32AllFF++;   break;
                    case FLASH_MISMATCH_ALL_00:  ptStats->u32All00++;   break;
                    case FLASH_MISMATCH_SHIFTED: ptStats->u32Shifted++; break;
                    default:                     ptStats->u32Noise++;   break;
                }
            }
        }

        ptStats->u32Cycles = u32i;

        if (pfnBetweenCycles != NULL)
            pfnBetweenCycles();

        /* Yield regardless of the callback. A tight 1000-cycle loop on the
         * FrKernel task would otherwise starve the heartbeat and the watchdog
         * feed for seconds at a time, and a stress test that resets the unit
         * measures nothing. */
        osDelay(1);
    }

    (void)FLASH_u32GetAndClearReadCrcStats(&ptStats->u16DriverRetries,
                                           &ptStats->u16DriverFails);
    return true;
}

#else  /* !ENABLE_FLASH_STRESS_TEST */

bool FLASH_bStressTest(uint32_t u32Cycles,
                       void (*pfnBetweenCycles)(void),
                       Flash_StressStats_t *ptStats)
{
    (void)u32Cycles;
    (void)pfnBetweenCycles;
    if (ptStats != NULL)
        memset(ptStats, 0, sizeof(*ptStats));
    return false;
}

#endif /* ENABLE_FLASH_STRESS_TEST */
