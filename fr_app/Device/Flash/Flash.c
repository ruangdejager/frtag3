/*
 * Flash.c
 *
 * AT25XE041B SPI NOR flash device driver.
 *
 * All SPI transactions use the driver macros in Flash_Driver.h which
 * route through the HAL SPI2 layer in hal_spi.h. Every opcode, register bit
 * and timing constant lives in Flash_Config.h with its datasheet section;
 * nothing numeric should appear in this file without one.
 *
 * ---- What changed when the board moved from the AT25EU0041A ----
 *
 * The part is not a drop-in and three of the differences are load-bearing:
 *
 *  1. The status register is a different register. No BP bits, no SRP0, no
 *     separate SR2 opcode. The old FLASH_STATUS_PROTECTED mask read as clean
 *     on a healthy chip by pure coincidence - see Flash_Config.h.
 *  2. The array's default state is PROTECTED. A Global Unprotect is part of
 *     normal bring-up here, not a recovery step.
 *  3. The idle park is Ultra-Deep Power-Down (79h), and exiting it resets
 *     every internal register to its power-on default - which puts (2) back.
 *     So a wake re-arms the write protection and the driver has to undo it
 *     again before the next program or erase. That is what bNeedsUnprotect
 *     below exists for, and getting it wrong reproduces the exact historical
 *     failure this driver already carries scar tissue for: reads fine, every
 *     write and erase silently refused, log frozen.
 */

#include "Flash.h"
#include "Flash_Config.h"
#include "Flash_Driver.h"
#include "build_config.h"
#include "hal_system.h"        /* HAL_SYSTEM_vBusyWaitUs - sub-ms settle waits */
#include "dbg_log.h"
#include "cmsis_os2.h"
#include <string.h>

/* Ultra-deep-power-down state. The flash is parked whenever idle (the DbgLog
 * consumer issues FLASH_vDeepPowerDown() after draining its queue) so it draws
 * ~200 nA through the long STOP2 sleep periods instead of the ~25 µA standby
 * current. Any access transparently wakes it first via FLASH_vEnsureAwake(). */
static bool bInDpd = false;

/* When true, FLASH_vDeepPowerDown() is suppressed and the chip is kept
 * awake. Set for the duration of an OTA session — see
 * FLASH_vInhibitDeepPowerDown() in Flash.h. */
static bool bDpdInhibited = false;

/* Set by every UDPD wake, cleared by a successful Global Unprotect.
 *
 * Datasheet §12.5.3: "Upon recovery from Ultra Deep Power Down, all internal
 * registers are in the Power-On default state." Table 3: the Sector Protection
 * Register power-on default is 1, protected. So a wake silently re-locks the
 * whole array for writes while leaving reads working perfectly - the exact
 * signature of the 2026-08-31 field failure, arriving through a completely
 * different door.
 *
 * Deliberately lazy rather than unprotecting inside the wake itself: the
 * overwhelming majority of wakes serve a read (the log consumer draining, a
 * FOTA verify pass), and reads are unaffected by protection. Paying a WREN +
 * WRSR + ready-wait on every one of them would put a self-timed
 * status-register write into the hottest path in the driver for no benefit.
 * The cost lands instead on the first program or erase after a wake, which is
 * already a self-timed operation measured in milliseconds. */
static bool bNeedsUnprotect = false;

/* This device is shared: Log.c (driven by DBGLOG_vConsumerTask, its own
 * thread) and OtaStore.c (driven by the AppTask) both call straight into
 * this driver's public functions. Every one of those functions is a
 * multi-step SPI transaction bracketed by CS select/deselect; on a
 * preemptive scheduler, a task switch mid-transaction lets a second task
 * start its own Select/Write/Read/Deselect sequence on the same physical
 * CS/MOSI/MISO lines while the first transaction is still open, corrupting
 * both. Splitting the address space between the log partition and the OTA
 * scratchpad (the assumption an earlier version of this file's comment
 * made) does NOT protect against this — SPI bus ownership, not the target
 * address, is what needs to be exclusive. Recursive because the internal
 * helpers below (FLASH_bWaitReady -> FLASH_bDeviceBusy ->
 * FLASH_u8ReadStatusReg, etc.) call back into other public entry points
 * from the same task while it already holds the lock. */
static osMutexId_t xFlashMutex = NULL;

static void FLASH_vLock(void)
{
    if (xFlashMutex == NULL)
    {
        const osMutexAttr_t attr = { .attr_bits = osMutexRecursive };
        xFlashMutex = osMutexNew(&attr);
    }
    osMutexAcquire(xFlashMutex, osWaitForever);
}

static void FLASH_vUnlock(void)
{
    osMutexRelease(xFlashMutex);
}

/* Read-failure telemetry — see Flash_ReadFailReason_t in Flash.h for why a
 * bare false return was not enough. Recorded under the flash mutex by
 * FLASH_vRead, read out (and cleared) by whoever is diagnosing. Details are
 * kept for the FIRST failure of a batch, so a later cascade cannot bury what
 * actually went wrong first. */
static volatile uint16_t               u16RdFailCount  = 0U;
static volatile Flash_ReadFailReason_t tRdFailReason   = FLASH_RDFAIL_NONE;
static volatile uint32_t               u32RdFailAddr   = 0U;
static volatile uint8_t                u8RdFailHal     = 0U;

/* Call with the flash mutex held. */
static void FLASH_vRecordReadFail(Flash_ReadFailReason_t tReason,
                                  uint32_t u32Addr, uint8_t u8Hal)
{
    if (u16RdFailCount == 0U)
    {
        tRdFailReason = tReason;
        u32RdFailAddr = u32Addr;
        u8RdFailHal   = u8Hal;
    }
    if (u16RdFailCount < UINT16_MAX) u16RdFailCount++;
}

uint16_t FLASH_u16GetAndClearReadFails(Flash_ReadFailReason_t *ptReason,
                                       uint32_t *pu32Addr,
                                       uint8_t  *pu8HalStatus)
{
    FLASH_vLock();
    uint16_t u16Count = u16RdFailCount;
    if (ptReason     != NULL) *ptReason     = tRdFailReason;
    if (pu32Addr     != NULL) *pu32Addr     = u32RdFailAddr;
    if (pu8HalStatus != NULL) *pu8HalStatus = u8RdFailHal;
    u16RdFailCount = 0U;
    tRdFailReason  = FLASH_RDFAIL_NONE;
    u32RdFailAddr  = 0U;
    u8RdFailHal    = 0U;
    FLASH_vUnlock();
    return u16Count;
}

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

/* Wake from ultra-deep-power-down on demand. No-op when already awake, so only
 * the first access after a park pays the exit sequence.
 *
 * UDPD is exited by PULSING CS, not by an opcode (datasheet §12.5.1): assert
 * CS for at least tCSLU, deassert, then wait tXUDPD before clocking anything
 * at the device. A command issued inside tXUDPD is accepted onto the bus and
 * then discarded - "that operation is ignored and nothing is done" - which
 * from the host's side looks exactly like a read returning garbage.
 *
 * Busy-wait, not osDelay, for two reasons: 70 us is below osDelay's 1 ms
 * resolution, and this runs from the chip-select chokepoint where yielding the
 * scheduler could let another task open its own transaction on the same bus.
 * The previous part's resume paid a flat osDelay(1) against a tRDPD of 8 us -
 * 125x the requirement - so if the DPD-wake read corruption this driver
 * documents is real, an unhonoured recovery time was never the explanation
 * for it. Worth remembering when reading that history.
 *
 * The wake re-arms write protection; see bNeedsUnprotect. */
static void FLASH_vEnsureAwake(void)
{
    if (!bInDpd) return;

    FLASH_DRIVER_vSelect();
    HAL_SYSTEM_vBusyWaitUs(FLASH_TCSLU_US);    /* >= tCSLU (20 ns spec min) */
    FLASH_DRIVER_vDeselect();
    HAL_SYSTEM_vBusyWaitUs(FLASH_TXUDPD_US);   /* tXUDPD - device in standby */

    bInDpd          = false;
    bNeedsUnprotect = true;
}

/* FLASH_WAIT_READY_TIMEOUT_MS / _BLOCK_ERASE_ / _CHIP_ERASE_ now live in
 * Flash_Config.h alongside the datasheet tPP / tBLKE / tCHPE figures they are
 * sized against, so the margin each one carries is visible at the point the
 * number is chosen rather than asserted here. */

/* Bounded by real elapsed time, not loop iterations: FLASH_bDeviceBusy()
 * issues two SPI transactions, each individually bounded by SPI_TIMEOUT
 * (1000 ms). If the SPI2 bus is stalled (e.g. contention with another
 * board's SPI sharing these lines), a single FLASH_bDeviceBusy() call can
 * itself take ~2 s and FLASH_u8ReadStatusReg defaults to "busy" on failure
 * - an iteration-counted loop would then take ~u32TimeoutMs * 2s to expire
 * (3000 ms -> ~100 minutes), which looks indistinguishable from a hang. */
static bool FLASH_bWaitReadyTimeout(uint32_t u32TimeoutMs)
{
    uint32_t u32Start = osKernelGetTickCount();
    while (FLASH_bDeviceBusy())
    {
        if ((osKernelGetTickCount() - u32Start) >= u32TimeoutMs)
        {
            DBG("FLASH: busy >%lu ms - aborting operation\r\n", (unsigned long)u32TimeoutMs);
            return false;
        }
        osDelay(1);
    }
    return true;
}

static bool FLASH_bWaitReady(void)
{
    return FLASH_bWaitReadyTimeout(FLASH_WAIT_READY_TIMEOUT_MS);
}

static void FLASH_vWriteEnable(void)
{
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
}

/* Clear the Write Enable Latch (WRDI). Program, Erase and Write-Status-Register
 * are ONLY accepted while WEL=1, so the chip must never be left idle with WEL
 * set: a single failed/garbled command transaction otherwise leaves the device
 * armed, and any subsequent stray WRSR on a glitchy bus could latch SRP0/BP -
 * which, because WP# is grounded on this board, permanently bricks the part for
 * writes. Calling this on every write error path keeps the armed window down to
 * the few microseconds between our WREN and our own command. */
static void FLASH_vWriteDisable(void)
{
    uint8_t cmd = FLASH_CMD_WRITE_DISABLE;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
}

/* Clear all sector protection (Global Unprotect): WREN, then WRSR (01h) with a
 * SINGLE data byte of 00h. Datasheet §9.5 / Table 4, row WP#=0, current
 * SPRL=0, data 0x0000xx: "Global Unprotect - all Sector Protection Registers
 * reset to 0", leaving SPRL at 0.
 *
 * This is bring-up, not just recovery. Table 3: a Sector Protection Register's
 * power-on default is 1 (protected), so a virgin part - and a part that has
 * just exited UDPD, see bNeedsUnprotect - refuses every program and erase
 * until this runs. Reads are unaffected throughout, which is what makes the
 * protected state so quiet: the log stops growing and nothing reports an error.
 *
 * ONE data byte, not two. The previous part took SR1 and SR2 in one bracket;
 * here WRSR is a one-byte command and §11.3 says any further bytes are
 * ignored - so the old three-byte version happened to work, but only by luck.
 *
 * THE DATA BYTE'S BIT 7 MUST NEVER BE SET. WP# is grounded on this board with
 * no MCU GPIO, and §11.3 states that while WP# is asserted SPRL can only be
 * driven 0 -> 1, never back. One WRSR with bit 7 set therefore locks that
 * unit's protection registers permanently: reads keep working, writes and
 * erases are refused forever, and nothing short of physically lifting pin 3
 * recovers it. FLASH_WRSR_GLOBAL_UNPROTECT is 00h for exactly this reason, and
 * FLASH_vWriteDisable() on every error path (WEL=0 at idle, so a stray WRSR on
 * a glitchy bus is refused outright) is the second line of that defence.
 *
 * Returns true once the device reports SWP = none and SPRL clear. */
static bool FLASH_bGlobalUnprotect(void)
{
    uint8_t cmd[2] = { FLASH_CMD_WRITE_STATUS, FLASH_WRSR_GLOBAL_UNPROTECT };

    if (!FLASH_bWaitReady())
        return false;
    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    bool bOk = (FLASH_DRIVER_vWrite(cmd, 2) == HAL_OK);
    FLASH_DRIVER_vDeselect();

    if (!bOk)
    {
        FLASH_vWriteDisable();    /* never leave the device armed (WEL=1) */
        return false;
    }

    if (!FLASH_bWaitReady())      /* WRSR is itself a self-timed write */
        return false;

    bool bClear = (FLASH_u8ReadStatusReg() & FLASH_STATUS_PROTECTED) == 0U;
    if (bClear)
        bNeedsUnprotect = false;
    return bClear;
}


/* The driver's usability gate. When false (no chip fitted, or SPI2 floating
 * with nothing attached), every operation below returns immediately without
 * touching SPI - avoiding the bounded-but-still-3s FLASH_bWaitReady() wait on
 * every single call (LOG_vInit alone probes every sector, so that wait
 * multiplied by FLASH_NUM_SECTORS would turn boot into a multi-minute stall).
 *
 * NOT latched for the life of the boot. It used to be: assigned once, from one
 * unretried JEDEC read in FLASH_vInit, and never revisited. A single transient
 * bad read therefore disabled logging and every FOTA erase/write until the unit
 * was physically power-cycled - which is what happened on 2026-08-31 (log
 * frozen mid-campaign for 18 h, FWDONE=ERR with 0 B delivered on every
 * subsequent session, while SelfTest kept printing flash=OK). Recovery now runs
 * from FLASH_bEnsurePresent() below, off ordinary traffic. */
static bool bDevicePresent = false;

/* Health/telemetry - see Flash_Health_t in Flash.h. Deliberately RAM + backup
 * register only: when the flash is the thing that is broken, the flash log is
 * not available to record why. */
static bool     bEverAbsent         = false;
static bool     bWriteProtectedSeen = false;
static bool     bUnprotectFailed    = false;
static bool     bEraseVerifyFail    = false;
static uint8_t  au8LastId[3]        = {0};
static uint8_t  u8LastAttempts      = 0U;
static uint16_t u16ProbeFailures    = 0U;
static uint16_t u16Recoveries       = 0U;
static uint16_t u16EraseVerifyFails = 0U;
static bool     bEpeSeen            = false;
static uint16_t u16EpeFails         = 0U;
static uint16_t u16ReadCrcRetries   = 0U;
static uint16_t u16ReadCrcFails     = 0U;

/* Cooldown bookkeeping for the automatic re-probe. bProbedOnce distinguishes
 * "never probed" from "probed at tick 0", so the first recovery attempt is not
 * gated behind a full interval. */
static uint32_t u32LastProbeTick    = 0U;
static bool     bProbedOnce         = false;

/* Undo the write protection a UDPD wake re-armed, at the last moment that can
 * still matter. Call with the mutex held, from every path that is about to
 * program or erase - and from nowhere else, because reads do not need it and
 * paying for it on the read path would put a self-timed status-register write
 * into the driver's hottest loop.
 *
 * The flag is cleared before the attempt, not after, so a chip that cannot be
 * unprotected does not re-enter this on every single write; the erase-verify
 * path in FLASH_bEraseRegionLocked is what catches that case and reports it. */
static void FLASH_vEnsureUnprotectedLocked(void)
{
    if (!bNeedsUnprotect)
        return;

    bNeedsUnprotect = false;

    if (!FLASH_bGlobalUnprotect())
    {
        bUnprotectFailed = true;
        DBG("FLASH: post-wake unprotect FAILED, SR1=%02X - the array is still "
            "write-locked from the UDPD exit\r\n", FLASH_u8ReadStatusReg());
    }
}

/* Read both status register bytes in one CS bracket. Call with the mutex held
 * and the device awake.
 *
 * There is no second opcode on this part. A single 05h makes the device emit
 * byte 1, byte 2, byte 1, ... for as long as CS stays asserted (datasheet
 * §11.1), so two bytes clocked after one opcode gets both. The previous part
 * used 35h for SR2; 35h is not a command here and unsupported opcodes are
 * simply ignored (§6), so the old code was reading whatever the bus happened
 * to idle at and reporting it as a register value.
 *
 * Fails safe to busy: a short or failed transfer must never be mistaken for
 * "ready", or FLASH_bWaitReady would issue the next command onto a device that
 * is still mid-program. */
static void FLASH_vReadStatusLocked(uint8_t *pu8Sr1, uint8_t *pu8Sr2)
{
    uint8_t cmd      = FLASH_CMD_READ_STATUS;
    uint8_t au8Sr[2] = { FLASH_SR1_RDY_BSY, 0U };

    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    if (FLASH_DRIVER_vRead(au8Sr, 2) != HAL_OK)
    {
        au8Sr[0] = FLASH_SR1_RDY_BSY;
        au8Sr[1] = 0U;
    }
    FLASH_DRIVER_vDeselect();

    if (pu8Sr1 != NULL) *pu8Sr1 = au8Sr[0];
    if (pu8Sr2 != NULL) *pu8Sr2 = au8Sr[1];
}

/* Did the device itself report that the last erase or program failed?
 *
 * EPE (SR1 bit 5) is new on this part and is the one fault signal that does
 * not depend on reading the array back. §11.1.3: it is set when at least one
 * byte did not erase or program properly, and is updated after every erase and
 * program. Note what it deliberately does NOT cover - an operation that never
 * started, because the region was protected or WEL was clear, leaves EPE at 0.
 * That case is caught by FLASH_bVerifyErasedLocked's read-back instead, so the
 * two checks are complementary rather than redundant: EPE catches "tried and
 * failed", the read-back catches "never tried". */
static bool FLASH_bEraseProgramErrorLocked(void)
{
    uint8_t sr1 = 0U;
    FLASH_vReadStatusLocked(&sr1, NULL);
    return (sr1 & FLASH_SR1_EPE) != 0U;
}

/* Mirror the fault state into a TAMP backup register. Survives reset and
 * brownout, needs no external flash, and is readable by the next boot - the
 * only durable record available when the log device itself is the casualty.
 * The magic in the top bits keeps an uninitialised or unrelated value from
 * being mistaken for health data. */
static void FLASH_vPersistHealth(void)
{
    uint32_t u32Word = FLASH_HEALTH_BKP_MAGIC
                     | (bDevicePresent      ? FLASH_HEALTH_BIT_PRESENT     : 0UL)
                     | (bEverAbsent         ? FLASH_HEALTH_BIT_EVERABSENT  : 0UL)
                     | (bWriteProtectedSeen ? FLASH_HEALTH_BIT_PROTECTED   : 0UL)
                     | (bUnprotectFailed    ? FLASH_HEALTH_BIT_UNPROT_FAIL : 0UL)
                     | (bEraseVerifyFail    ? FLASH_HEALTH_BIT_ERASEVFAIL  : 0UL)
                     | (bEpeSeen            ? FLASH_HEALTH_BIT_EPE         : 0UL)
                     | ((uint32_t)au8LastId[0] << 8);

    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP4R = u32Word;
}

/* Retried JEDEC-ID probe. Call with the flash mutex held and the device awake.
 * Single source of truth for "is the right part answering" - FLASH_vInit,
 * FLASH_bVerifyDeviceEx and the recovery path all route through this, so none
 * of them can drift back to a one-shot read. */
static bool FLASH_bProbeLocked(uint8_t *pu8Id, uint8_t *pu8Attempts)
{
    uint8_t au8Raw[FLASH_JEDEC_ID_BYTES] = {0};
    uint8_t id[3] = {0};
    bool    bOk   = false;
    uint8_t u8Try = 0U;

    /* A single JEDEC read can come back wrong while the part is perfectly
     * healthy - the same transient this codebase already documents elsewhere
     * (see OTA_LORA_CHUNK_GAP_MS: on frtag2, a flash read issued too soon after
     * a TX came back corrupt). That note used to blame supply-rail sag; the
     * mechanism was never confirmed, and noise on the long MISO/MOSI runs over
     * a non-continuous ground plane is the likelier cause. Reporting FAIL off
     * one bad read was declaring every unit's flash dead while it was
     * demonstrably reading, writing and holding the log fine.
     *
     * The raw bytes are handed back so the caller can log what actually came
     * off the wire, which is what distinguishes the cases:
     *   00 00 00 -> no clock/data (AF, wiring, bus not restored)
     *   FF FF FF -> MISO idle / CS never asserted
     *   1F xx xx -> a Renesas part, but not THIS one (see below)
     *   other    -> CPOL/CPHA, timing, or genuinely the wrong part
     *
     * ALL THREE meaningful bytes are compared, not just the manufacturer.
     * 1F alone passes any Renesas flash of any density, so a mis-stuffed reel
     * would have been accepted here and failed much later, at an address the
     * fitted part does not have - which presents as data corruption, not as an
     * identification fault. Expected: 1F 44 02 (datasheet Table 14). A fourth
     * byte, the Extended Device Information String Length, is clocked as well
     * so the transaction matches the datasheet's Figure 29 exactly; it must be
     * 00h and is reported but not gated on, since nothing depends on it. */
    for (u8Try = 1U; u8Try <= FLASH_JEDEC_READ_ATTEMPTS; u8Try++)
    {
        uint8_t cmd = FLASH_CMD_JEDEC_ID;

        memset(au8Raw, 0, sizeof(au8Raw));
        FLASH_DRIVER_vSelect();
        FLASH_DRIVER_vWrite(&cmd, 1);
        FLASH_DRIVER_vRead(au8Raw, FLASH_JEDEC_ID_BYTES);
        FLASH_DRIVER_vDeselect();

        id[0] = au8Raw[0];
        id[1] = au8Raw[1];
        id[2] = au8Raw[2];

        if ((id[0] == FLASH_MANUFACTURER_ID) &&
            (id[1] == FLASH_DEVICE_ID1)      &&
            (id[2] == FLASH_DEVICE_ID2))     { bOk = true; break; }

        osDelay(2);   /* short gap before re-reading */
    }

    if (u8Try > FLASH_JEDEC_READ_ATTEMPTS) u8Try = FLASH_JEDEC_READ_ATTEMPTS;

    au8LastId[0]   = id[0];
    au8LastId[1]   = id[1];
    au8LastId[2]   = id[2];
    u8LastAttempts = u8Try;
    if (!bOk && (u16ProbeFailures < UINT16_MAX)) u16ProbeFailures++;

    if (pu8Id != NULL) { pu8Id[0] = id[0]; pu8Id[1] = id[1]; pu8Id[2] = id[2]; }
    if (pu8Attempts != NULL) *pu8Attempts = u8Try;

    return bOk;
}

/* Post-identification bring-up: report protection state, self-heal a chip stuck
 * write-protected, and leave the device disarmed. Call with the mutex held,
 * only once the part has answered. Factored out of FLASH_vInit so the recovery
 * path gets it too - a device whose gate re-opens mid-run has not been through
 * init and may never have had its protection bits cleared. */
static void FLASH_vBringUpLocked(void)
{
    /* Sector protection blocks program and erase while leaving reads working
     * perfectly, so a device left in that state freezes its log and ignores
     * chip-erase with nothing anywhere reporting an error. On this part that
     * is not an exotic fault state - it is the POWER-ON DEFAULT (Table 3), and
     * it is restored by every ultra-deep-power-down exit, so clearing it is
     * routine bring-up rather than recovery.
     *
     * Both status bytes come from one 05h bracket; there is no SR2 opcode. */
    uint8_t sr1 = 0U;
    uint8_t sr2 = 0U;
    FLASH_vReadStatusLocked(&sr1, &sr2);

    /* WPP is reported rather than assumed. This board ties WP# to GND, so a
     * healthy part must read WPP=0; a 1 here means the strap is not what
     * Flash_Config.h says it is, and that changes what is recoverable - with
     * WP# high SPRL can be cleared again, with WP# low it cannot. Worth one
     * line at boot, because nothing else in the system can see that pin. */
    DBG("FLASH: SR1=%02X (SPRL=%u SPM=%u EPE=%u WPP=%u[WP#=%s] SWP=%u WEL=%u BSY=%u) "
        "SR2=%02X (RSTE=%u)\r\n",
        sr1,
        (unsigned)((sr1 & FLASH_SR1_SPRL)    ? 1U : 0U),
        (unsigned)((sr1 & FLASH_SR1_SPM)     ? 1U : 0U),
        (unsigned)((sr1 & FLASH_SR1_EPE)     ? 1U : 0U),
        (unsigned)((sr1 & FLASH_SR1_WPP)     ? 1U : 0U),
        (sr1 & FLASH_SR1_WPP) ? "deasserted" : "asserted/GND",
        (unsigned)((sr1 & FLASH_SR1_SWP_MASK) >> 2),
        (unsigned)((sr1 & FLASH_SR1_WEL)     ? 1U : 0U),
        (unsigned)((sr1 & FLASH_SR1_RDY_BSY) ? 1U : 0U),
        sr2,
        (unsigned)((sr2 & FLASH_SR2_RSTE)    ? 1U : 0U));

    if (sr1 & FLASH_STATUS_PROTECTED)
    {
        /* Recorded, but NOT treated as a fault on its own: a part that has
         * only ever been powered up, or that has just come out of UDPD, lands
         * here every single time by design. The health flag distinguishes a
         * unit that needed unprotecting from one that did not, which is still
         * worth knowing - it should be every unit, every boot, and a unit that
         * reports otherwise is telling us something about its wake path. */
        bWriteProtectedSeen = true;
        DBG("FLASH: array protected (SWP=%u SPRL=%u) - performing global "
            "unprotect\r\n",
            (unsigned)((sr1 & FLASH_SR1_SWP_MASK) >> 2),
            (unsigned)((sr1 & FLASH_SR1_SPRL) ? 1U : 0U));

        if (FLASH_bGlobalUnprotect())
        {
            bUnprotectFailed = false;
            DBG("FLASH: unprotected, SR1=%02X\r\n", FLASH_u8ReadStatusReg());
        }
        else
        {
            bUnprotectFailed = true;
            /* With SPRL latched and WP# grounded this is terminal for writes
             * on this unit: datasheet 11.3, SPRL cannot be cleared while WP#
             * is asserted, and WP# has no MCU GPIO. Reads keep working, which
             * is exactly what makes it easy to miss. */
            DBG("FLASH: unprotect FAILED, SR1=%02X - SPRL latched with WP# "
                "grounded; writes are locked until pin 3 is lifted\r\n",
                FLASH_u8ReadStatusReg());
        }
    }
    else
    {
        /* Already clear, so the wake path owes nothing. */
        bNeedsUnprotect = false;
    }

    /* Start from a known-safe, disarmed state: with WP# grounded the only thing
     * standing between us and a permanent write-lock is never accepting a stray
     * WRSR, which requires WEL=0 at idle. */
    FLASH_vWriteDisable();
}

/* The gate, with self-recovery. Call with the mutex held. Cheap in the normal
 * case (one bool test); when the gate is shut it re-probes at most once per
 * FLASH_REPROBE_INTERVAL_MS and, on success, brings the device back up and
 * re-opens the gate.
 *
 * This is what makes a transient survivable without a power cycle - the whole
 * point, given a field unit cannot be reprogrammed by wire and depends on FOTA
 * to ever be fixed again. A genuinely absent chip still costs only one
 * throttled probe per interval, which is what the gate was protecting against
 * in the first place. */
static bool FLASH_bEnsurePresent(void)
{
    if (bDevicePresent)
        return true;

    uint32_t u32Now = osKernelGetTickCount();
    if (bProbedOnce && ((u32Now - u32LastProbeTick) < FLASH_REPROBE_INTERVAL_MS))
        return false;

    u32LastProbeTick = u32Now;
    bProbedOnce      = true;

    FLASH_vEnsureAwake();
    if (!FLASH_bProbeLocked(NULL, NULL))
    {
        FLASH_vPersistHealth();
        return false;
    }

    bDevicePresent = true;
    if (u16Recoveries < UINT16_MAX) u16Recoveries++;
    DBG("FLASH: device recovered - JEDEC %02X %02X %02X, gate re-opened "
        "(%u failed probes since boot)\r\n",
        au8LastId[0], au8LastId[1], au8LastId[2], (unsigned)u16ProbeFailures);

    FLASH_vBringUpLocked();
    FLASH_vPersistHealth();
    return true;
}

/* One Read Array transaction, no retry, no telemetry. Call with the mutex held
 * and the device present and ready. Factored out so the read-verify below can
 * re-read without recursing through the whole entry point, and so the erase
 * verifier has a raw path that does not pollute the read-failure tally. */
static bool FLASH_bRawReadLocked(uint32_t addr, uint8_t *buf, uint16_t len,
                                 HAL_StatusTypeDef *ptCmd,
                                 HAL_StatusTypeDef *ptData)
{
    uint8_t cmd[4] = {
        FLASH_CMD_READ,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };

    /* Command and payload transfers kept separate rather than &&-chained, so
     * the tally can say which half failed: a command failure means the bus
     * refused a 4-byte write, a payload failure means it died partway through
     * a read the device had already accepted. Different faults. */
    FLASH_DRIVER_vSelect();
    HAL_StatusTypeDef tCmd  = FLASH_DRIVER_vWrite(cmd, 4);
    HAL_StatusTypeDef tData = (tCmd == HAL_OK) ? FLASH_DRIVER_vRead(buf, len) : HAL_OK;
    FLASH_DRIVER_vDeselect();

    if (ptCmd  != NULL) *ptCmd  = tCmd;
    if (ptData != NULL) *ptData = tData;
    return (tCmd == HAL_OK) && (tData == HAL_OK);
}

/* Confirm an erase actually blanked the region. Call with the mutex held, after
 * the erase has completed. Uses a raw read rather than FLASH_vRead so a verify
 * failure does not pollute the read-failure telemetry with something that is
 * not a read fault. Returns false if the bytes are not all-ones, or if the
 * read-back itself could not be performed - in both cases the erase must not be
 * reported as successful. */
static bool FLASH_bVerifyErasedLocked(uint32_t u32Addr, uint32_t u32Len)
{
    uint8_t  au8Buf[FLASH_ERASE_VERIFY_BYTES];
    uint16_t u16Chunk = (uint16_t)((u32Len < (uint32_t)FLASH_ERASE_VERIFY_BYTES)
                                   ? u32Len : (uint32_t)FLASH_ERASE_VERIFY_BYTES);

    /* Sample the first and last chunk of the region. A protected device fails
     * uniformly so one sample would do, but a partially-completed erase (a
     * power dip mid-operation) leaves the tail unerased - and the tail is
     * exactly what a sequential writer fills last and would notice last. */
    uint32_t au32Offsets[2];
    au32Offsets[0] = 0UL;
    au32Offsets[1] = (u32Len > (uint32_t)u16Chunk) ? (u32Len - (uint32_t)u16Chunk) : 0UL;

    if (u16Chunk == 0U)
        return true;

    for (uint8_t i = 0U; i < 2U; i++)
    {
        uint32_t u32At = u32Addr + au32Offsets[i];

        if (!FLASH_bRawReadLocked(u32At, au8Buf, u16Chunk, NULL, NULL))
            return false;

        for (uint16_t j = 0U; j < u16Chunk; j++)
        {
            if (au8Buf[j] != FLASH_ERASED_BYTE)
                return false;
        }

        if (au32Offsets[1] == au32Offsets[0])
            break;   /* region smaller than one chunk - one sample covers it */
    }

    return true;
}

/* Issue one erase command and wait for it to complete. Call with the mutex held
 * and the device known present. Shared by all three erase entry points so the
 * WREN / command / disarm-on-failure / wait sequence exists in exactly one
 * place - and so the retry below re-issues an identical operation. */
static bool FLASH_bIssueEraseLocked(const uint8_t *pu8Cmd, uint8_t u8CmdLen,
                                    uint32_t u32TimeoutMs)
{
    if (!FLASH_bWaitReady())
        return false;

    /* A UDPD wake re-armed the write protection; undo it before the WREN, or
     * the device accepts this erase command and does nothing with it. */
    FLASH_vEnsureUnprotectedLocked();

    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    bool bOk = (FLASH_DRIVER_vWrite((uint8_t *)pu8Cmd, u8CmdLen) == HAL_OK);
    FLASH_DRIVER_vDeselect();

    if (!bOk)
    {
        FLASH_vWriteDisable();   /* never leave the device armed (WEL=1) */
        return false;
    }

    /* Commit before returning - see FLASH_vPageWrite for why a self-contained
     * write matters. */
    if (!FLASH_bWaitReadyTimeout(u32TimeoutMs))
        return false;

    /* Ask the device whether its own erase worked. EPE is the one fault signal
     * that does not depend on reading the array back, so it survives the very
     * transient this board is being investigated for: a corrupted read-back
     * can make a good erase look bad, but it cannot fake EPE. */
    if (FLASH_bEraseProgramErrorLocked())
    {
        bEpeSeen = true;
        if (u16EpeFails < UINT16_MAX) u16EpeFails++;
        DBG("FLASH: device reports EPE after erase - the erase was attempted "
            "and FAILED\r\n");
        FLASH_vPersistHealth();
        return false;
    }

    return true;
}

/* Erase a region and PROVE it happened, self-healing a protected chip.
 *
 * A block-protected device accepts the erase command, never raises WIP and
 * returns to ready, so the command sequence above succeeds while the old
 * contents survive untouched. Reporting that as a successful erase is how a
 * stale FOTA scratch region gets written over and only fails much later at the
 * whole-image XOR - or not at all.
 *
 * On a verify failure the prime suspect is latched block protection, so it is
 * cleared and the erase retried once, here, rather than surfacing a failure the
 * caller would only retry on the next campaign. A field unit cannot be
 * reprogrammed by wire, so it is worth spending one extra erase to avoid
 * needing another whole FOTA window. */
static bool FLASH_bEraseRegionLocked(const uint8_t *pu8Cmd, uint8_t u8CmdLen,
                                     uint32_t u32Base, uint32_t u32Len,
                                     uint32_t u32TimeoutMs)
{
    if (!FLASH_bIssueEraseLocked(pu8Cmd, u8CmdLen, u32TimeoutMs))
        return false;

    if (FLASH_bVerifyErasedLocked(u32Base, u32Len))
        return true;

    uint8_t sr1 = FLASH_u8ReadStatusReg();
    DBG("FLASH: erase @%06lX len %lu did NOT blank - SR1=%02X%s\r\n",
        (unsigned long)u32Base, (unsigned long)u32Len, sr1,
        (sr1 & FLASH_STATUS_PROTECTED) ? " (sector-protected)" : "");

    bool bRetryWorthwhile = false;
    if (sr1 & FLASH_STATUS_PROTECTED)
    {
        bWriteProtectedSeen = true;
        if (FLASH_bGlobalUnprotect())
        {
            bUnprotectFailed = false;
            bRetryWorthwhile = true;
            DBG("FLASH: protection cleared - retrying the erase once\r\n");
        }
        else
        {
            bUnprotectFailed = true;
            DBG("FLASH: unprotect FAILED - SPRL latched with WP# grounded, "
                "writes locked until pin 3 is lifted\r\n");
        }
    }

    if (bRetryWorthwhile &&
        FLASH_bIssueEraseLocked(pu8Cmd, u8CmdLen, u32TimeoutMs) &&
        FLASH_bVerifyErasedLocked(u32Base, u32Len))
    {
        DBG("FLASH: erase @%06lX succeeded after clearing protection\r\n",
            (unsigned long)u32Base);
        /* Still recorded: the chip needed rescuing, and a unit that keeps
         * needing it is telling us something the success return hides. */
        bEraseVerifyFail = true;
        if (u16EraseVerifyFails < UINT16_MAX) u16EraseVerifyFails++;
        FLASH_vPersistHealth();
        return true;
    }

    bEraseVerifyFail = true;
    if (u16EraseVerifyFails < UINT16_MAX) u16EraseVerifyFails++;
    FLASH_vPersistHealth();
    return false;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void FLASH_vInit(void)
{
    FLASH_vLock();

    /* Report what the PREVIOUS boot left behind before overwriting it. This is
     * the only fault record that survives a reset when the flash itself is the
     * casualty — the flash log obviously cannot hold "the flash stopped
     * working", and the UART line that used to carry it is unavailable on a
     * deployed unit. A watchdog reset or brownout otherwise erases all evidence
     * that the tag had been running gated-off. */
    uint32_t u32Prev = TAMP->BKP4R;
    if ((u32Prev & FLASH_HEALTH_BKP_MAGIC_MASK) == FLASH_HEALTH_BKP_MAGIC)
    {
        if ((u32Prev & (FLASH_HEALTH_BIT_EVERABSENT |
                        FLASH_HEALTH_BIT_PROTECTED  |
                        FLASH_HEALTH_BIT_UNPROT_FAIL|
                        FLASH_HEALTH_BIT_ERASEVFAIL)) != 0UL)
        {
            DBG("FLASH: previous boot health - present=%u everAbsent=%u wprot=%u "
                "unprotFail=%u eraseVfyFail=%u lastMfr=%02X\r\n",
                (unsigned)((u32Prev & FLASH_HEALTH_BIT_PRESENT)     ? 1U : 0U),
                (unsigned)((u32Prev & FLASH_HEALTH_BIT_EVERABSENT)  ? 1U : 0U),
                (unsigned)((u32Prev & FLASH_HEALTH_BIT_PROTECTED)   ? 1U : 0U),
                (unsigned)((u32Prev & FLASH_HEALTH_BIT_UNPROT_FAIL) ? 1U : 0U),
                (unsigned)((u32Prev & FLASH_HEALTH_BIT_ERASEVFAIL)  ? 1U : 0U),
                (unsigned)((u32Prev >> 8) & 0xFFU));
        }
    }

    FLASH_vReleaseDeepPowerDown();
    osDelay(1);

    /* RETRIED, never a single shot. This one assignment decides whether every
     * write and erase in the driver is permitted, so it must not be decided by
     * a read that this codebase already documents as untrustworthy in
     * isolation. The unretried version here is what gated a healthy tag's flash
     * off for 18 h on 2026-08-31: one bad read at boot, log frozen, and every
     * FOTA erase failing instantly with FWDONE=ERR / 0 B delivered.
     *
     * The raw bytes are still reported on a mismatch so a genuine fault stays
     * diagnosable: 00s => no clock/data (AF or wiring), FFs => MISO idle / CS
     * not asserting, other => CPOL/CPHA or wrong part. */
    uint8_t id[3]      = {0};
    uint8_t u8Attempts = 0U;
    uint8_t u8Round    = 0U;

    for (u8Round = 1U; u8Round <= FLASH_BOOT_PROBE_ROUNDS; u8Round++)
    {
        bDevicePresent = FLASH_bProbeLocked(id, &u8Attempts);
        if (bDevicePresent)
            break;
        if (u8Round < FLASH_BOOT_PROBE_ROUNDS)
            osDelay(FLASH_BOOT_PROBE_SETTLE_MS);
    }

    bProbedOnce      = true;
    u32LastProbeTick = osKernelGetTickCount();

    if (!bDevicePresent)
    {
        bEverAbsent = true;
        DBG("FLASH: JEDEC ID mismatch after %u rounds x %u attempts - got "
            "%02X %02X %02X (expected %02X %02X %02X) - flash logging and FOTA "
            "staging disabled until a re-probe succeeds\r\n",
            (unsigned)FLASH_BOOT_PROBE_ROUNDS, (unsigned)FLASH_JEDEC_READ_ATTEMPTS,
            id[0], id[1], id[2],
            FLASH_MANUFACTURER_ID, FLASH_DEVICE_ID1, FLASH_DEVICE_ID2);
        FLASH_vPersistHealth();
        FLASH_vUnlock();
        return;
    }

    if ((u8Attempts > 1U) || (u8Round > 1U))
        DBG("FLASH: JEDEC ID %02X %02X %02X OK on round %u attempt %u "
            "(earlier read(s) transient - the chip is fine, the first read was not)\r\n",
            id[0], id[1], id[2], (unsigned)u8Round, (unsigned)u8Attempts);
    else
        DBG("FLASH: JEDEC ID %02X %02X %02X OK\r\n", id[0], id[1], id[2]);

    /* Report protection state and self-heal a chip stuck write-protected. */
    FLASH_vBringUpLocked();
    FLASH_vPersistHealth();

    /* Deliberately not a commented-out FLASH_vChipErase() any more. One
     * uncommented line here wipes every field unit it is flashed to, including
     * the staged OTA image that is the only way to fix it again. If a bring-up
     * wipe is needed, use "tag flash eraseall" - it is one command, it is
     * logged, and it cannot be enabled by accident. */

    FLASH_vUnlock();
}

bool FLASH_bDeviceBusy(void)
{
    FLASH_vLock();
    bool bBusy = (FLASH_u8ReadStatusReg() & FLASH_SR1_RDY_BSY) != 0U;
    FLASH_vUnlock();
    return bBusy;
}

uint8_t FLASH_u8ReadStatusReg(void)
{
    FLASH_vLock();
    FLASH_vEnsureAwake();   /* chokepoint: every read/write/erase polls here */
    uint8_t sr1 = 0U;
    /* FLASH_vReadStatusLocked defaults to busy on a failed or short transfer,
     * so a bad status read can never be mistaken for "ready" - FLASH_bWaitReady
     * would otherwise issue the next command onto a device that might still be
     * mid-program/erase. */
    FLASH_vReadStatusLocked(&sr1, NULL);
    FLASH_vUnlock();
    return sr1;
}

bool FLASH_bVerifyDevice(void)
{
    return FLASH_bVerifyDeviceEx(NULL, NULL);
}

bool FLASH_bVerifyDeviceEx(uint8_t *pu8Id, uint8_t *pu8Attempts)
{
    FLASH_vLock();
    FLASH_vEnsureAwake();
    bool bOk = FLASH_bProbeLocked(pu8Id, pu8Attempts);
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_bDevicePresent(void)
{
    FLASH_vLock();
    bool bPresent = bDevicePresent;
    FLASH_vUnlock();
    return bPresent;
}

bool FLASH_bRecoverDevice(uint8_t *pu8Id, uint8_t *pu8Attempts)
{
    FLASH_vLock();
    FLASH_vEnsureAwake();

    bProbedOnce      = true;
    u32LastProbeTick = osKernelGetTickCount();

    bool bWasPresent = bDevicePresent;
    bool bNowPresent = FLASH_bProbeLocked(pu8Id, pu8Attempts);

    if (bNowPresent)
    {
        bDevicePresent = true;
        if (!bWasPresent)
        {
            if (u16Recoveries < UINT16_MAX) u16Recoveries++;
            DBG("FLASH: device recovered on demand - gate re-opened\r\n");
        }
        /* Re-run bring-up unconditionally: a chip that has just answered may
         * never have been through init (recovered mid-run), and one that has
         * may since have latched protection. Both cases need the SR checked and
         * the device left disarmed. */
        FLASH_vBringUpLocked();
    }
    else
    {
        bEverAbsent = true;

        /* Deliberately does NOT close an already-open gate.
         *
         * The two directions are not symmetric. Wrongly leaving the gate open
         * costs nothing - the individual operations fail on their own, bounded
         * and reported. Wrongly closing it disables logging and FOTA staging
         * wholesale, on a unit that cannot be reprogrammed by wire, which is
         * precisely the failure being fixed here. Since a probe can fail on a
         * perfectly healthy chip (that is the whole premise of the retry
         * logic), letting one failed probe shut a working driver would just
         * reintroduce the bug through a different door.
         *
         * A chip that has genuinely stopped answering still surfaces: reads
         * record FLASH_RDFAIL_* telemetry and erases fail their verify, both of
         * which are visible in the health word. */
        if (bWasPresent)
            DBG("FLASH: probe failed but gate left OPEN - a single failed probe "
                "is not evidence of a dead chip (id=%02X %02X %02X)\r\n",
                au8LastId[0], au8LastId[1], au8LastId[2]);
    }

    FLASH_vPersistHealth();
    FLASH_vUnlock();
    return bDevicePresent;
}

void FLASH_vGetHealth(Flash_Health_t *ptHealth)
{
    if (ptHealth == NULL) return;

    FLASH_vLock();
    ptHealth->bPresent            = bDevicePresent;
    ptHealth->bEverAbsent         = bEverAbsent;
    ptHealth->bWriteProtected     = bWriteProtectedSeen;
    ptHealth->bUnprotectFailed    = bUnprotectFailed;
    ptHealth->bEraseVerifyFail    = bEraseVerifyFail;
    ptHealth->au8LastId[0]        = au8LastId[0];
    ptHealth->au8LastId[1]        = au8LastId[1];
    ptHealth->au8LastId[2]        = au8LastId[2];
    ptHealth->u8LastAttempts      = u8LastAttempts;
    ptHealth->u16ProbeFailures    = u16ProbeFailures;
    ptHealth->u16Recoveries       = u16Recoveries;
    ptHealth->u16EraseVerifyFails = u16EraseVerifyFails;
    ptHealth->bEpeSeen            = bEpeSeen;
    ptHealth->u16EpeFails         = u16EpeFails;
    ptHealth->u16ReadCrcRetries   = u16ReadCrcRetries;
    ptHealth->u16ReadCrcFails     = u16ReadCrcFails;
    FLASH_vUnlock();
}

#ifdef ENABLE_FLASH_READ_CRC

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), bit-at-a-time.
 *
 * Software rather than the STM32 CRC peripheral on purpose: that peripheral is
 * shared state with a configuration and a running remainder, and STOP2 would
 * have to restore it on every wake - a whole new class of parked-by-sleep bug,
 * in the one place in this driver whose entire job is to be more trustworthy
 * than the thing it is checking. Costs ~50 us for 256 bytes at 48 MHz, next to
 * a read that costs ~200 us at 12 MHz. */
static uint16_t FLASH_u16Crc16(const uint8_t *pu8Data, uint16_t u16Len)
{
    uint16_t u16Crc = 0xFFFFU;

    for (uint16_t i = 0U; i < u16Len; i++)
    {
        u16Crc ^= (uint16_t)((uint16_t)pu8Data[i] << 8);
        for (uint8_t b = 0U; b < 8U; b++)
            u16Crc = (u16Crc & 0x8000U) ? (uint16_t)((u16Crc << 1) ^ 0x1021U)
                                        : (uint16_t)(u16Crc << 1);
    }
    return u16Crc;
}

/* Verify buffer. Static rather than on the stack because this runs on whichever
 * task happens to be reading - the DbgLog consumer, the AppTask, a FOTA
 * session - and 256 bytes of stack is not something to spend uninvited on all
 * of them. Safe as a static: every path to it holds the flash mutex. */
static uint8_t au8CrcVerify[FLASH_READ_CRC_MAX_LEN];

/* Confirm a read by reading it again.
 *
 * A CRC computed over a buffer that was read ONCE detects nothing. It is a
 * function of whatever bytes came back, so it matches corrupt data exactly as
 * happily as good data. Detection needs a second opinion, and with no CRC
 * stored alongside the data in flash - that would be a storage-format change
 * reaching into the log FIFO, the OTA scratchpad and the bootloader contract -
 * the only second opinion available is another read.
 *
 * So: re-read, and compare. The comparison is a memcmp, which is exact and
 * free, rather than a CRC equality test, which would be strictly weaker: a
 * 16-bit CRC collides on roughly one differing pair in 65536. The CRC is still
 * computed and reported, because it is what lets a mismatch be described in one
 * 16-bit number in a log line instead of a hex dump.
 *
 * This is the same technique the FOTA chunk path already applies by hand (see
 * the five-consecutive-agreements loop in FOTA_bSendChunk); doing it in the
 * driver puts it under every caller instead of only the one that noticed first.
 *
 * Reads longer than FLASH_READ_CRC_MAX_LEN are verified over their first
 * FLASH_READ_CRC_MAX_LEN bytes. Partial cover is not ideal, but the
 * alternative is either an unbounded buffer or no cover at all, and the callers
 * that read more than a page at a time are streaming sequentially - so a
 * disturbance long enough to matter lands in some chunk prefix. */
static bool FLASH_bVerifyReadLocked(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint16_t u16Cover = (len < FLASH_READ_CRC_MAX_LEN) ? len : FLASH_READ_CRC_MAX_LEN;
    if (u16Cover == 0U)
        return true;

    for (uint8_t u8Try = 0U; u8Try < FLASH_READ_CRC_MAX_RETRIES; u8Try++)
    {
        if (!FLASH_bRawReadLocked(addr, au8CrcVerify, u16Cover, NULL, NULL))
            return false;              /* transport fault, not corruption */

        if (memcmp(buf, au8CrcVerify, u16Cover) == 0)
            return true;

        /* Disagreed. Adopt the newer read and try again: if the FIRST read was
         * the corrupt one, converging back onto it would be the wrong answer
         * twice. */
        if (u16ReadCrcRetries < UINT16_MAX) u16ReadCrcRetries++;
        memcpy(buf, au8CrcVerify, u16Cover);
    }

    if (u16ReadCrcFails < UINT16_MAX) u16ReadCrcFails++;
    DBG("FLASH: read @%06lX len %u never agreed with itself in %u attempts "
        "(crc %04X) - returned bytes are NOT trustworthy\r\n",
        (unsigned long)addr, (unsigned)len,
        (unsigned)FLASH_READ_CRC_MAX_RETRIES,
        (unsigned)FLASH_u16Crc16(buf, u16Cover));
    return false;
}

#endif /* ENABLE_FLASH_READ_CRC */

uint32_t FLASH_u32GetAndClearReadCrcStats(uint16_t *pu16Retries,
                                          uint16_t *pu16Fails)
{
    FLASH_vLock();
    uint16_t u16R = u16ReadCrcRetries;
    uint16_t u16F = u16ReadCrcFails;
    if (pu16Retries != NULL) *pu16Retries = u16R;
    if (pu16Fails   != NULL) *pu16Fails   = u16F;
    u16ReadCrcRetries = 0U;
    u16ReadCrcFails   = 0U;
    FLASH_vUnlock();
    return (uint32_t)u16R + (uint32_t)u16F;
}

bool FLASH_vRead(uint32_t addr, uint8_t *buf, uint16_t len)
{
    FLASH_vLock();
    if (!FLASH_bEnsurePresent())
    {
        FLASH_vRecordReadFail(FLASH_RDFAIL_ABSENT, addr, 0U);
        FLASH_vUnlock();
        return false;
    }

    if (!FLASH_bWaitReady())
    {
        FLASH_vRecordReadFail(FLASH_RDFAIL_WAITREADY, addr, 0U);
        FLASH_vUnlock();
        return false;
    }

    HAL_StatusTypeDef tCmd  = HAL_OK;
    HAL_StatusTypeDef tData = HAL_OK;
    bool bOk = FLASH_bRawReadLocked(addr, buf, len, &tCmd, &tData);

    if (!bOk)
        FLASH_vRecordReadFail((tCmd != HAL_OK) ? FLASH_RDFAIL_CMD : FLASH_RDFAIL_DATA,
                              addr,
                              (uint8_t)((tCmd != HAL_OK) ? tCmd : tData));
#ifdef ENABLE_FLASH_READ_CRC
    else if (!FLASH_bVerifyReadLocked(addr, buf, len))
    {
        FLASH_vRecordReadFail(FLASH_RDFAIL_CRC, addr, 0U);
        bOk = false;
    }
#endif

    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vPageWrite(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    FLASH_vLock();
    if (!FLASH_bEnsurePresent())
    {
        FLASH_vUnlock();
        return false;
    }

    uint8_t cmd[4] = {
        FLASH_CMD_PAGE_PROGRAM,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };
    if (!FLASH_bWaitReady())
    {
        FLASH_vUnlock();
        return false;
    }

    /* A UDPD wake re-armed the write protection; undo it before the WREN, or
     * this page program is accepted and silently discarded. */
    FLASH_vEnsureUnprotectedLocked();

    FLASH_vWriteEnable();
    FLASH_DRIVER_vSelect();
    bool bOk = (FLASH_DRIVER_vWrite(cmd, 4) == HAL_OK) &&
               (FLASH_DRIVER_vWrite(buf, len) == HAL_OK);
    FLASH_DRIVER_vDeselect();
    if (!bOk)
        FLASH_vWriteDisable();   /* never leave the device armed (WEL=1) */
    else
        /* Wait for the program to actually commit before returning. This used
         * to be deferred to the *next* operation's pre-wait, which meant the
         * last write of any burst was left in flight with nothing guarding it
         * — a concurrent deep-power-down or a reset in that window truncated
         * the page. Making the write self-contained is what removes that
         * whole class of silent corruption; the cost is the ~1-3 ms we were
         * already paying, just accounted to the write that incurs it. */
        bOk = FLASH_bWaitReady();

    /* The device's own verdict on the program it just did. Cheap (one status
     * read) and it catches the case a read-back cannot distinguish from a bad
     * read - see FLASH_bEraseProgramErrorLocked. */
    if (bOk && FLASH_bEraseProgramErrorLocked())
    {
        bEpeSeen = true;
        if (u16EpeFails < UINT16_MAX) u16EpeFails++;
        DBG("FLASH: device reports EPE after page program @%06lX - bytes on "
            "the device are NOT what was written\r\n", (unsigned long)addr);
        FLASH_vPersistHealth();
        bOk = false;
    }

    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vSectorErase(uint32_t addr)
{
    FLASH_vLock();
    if (!FLASH_bEnsurePresent())
    {
        FLASH_vUnlock();
        return false;
    }

    uint8_t cmd[4] = {
        FLASH_CMD_SECTOR_ERASE,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };

    /* Erased + verified, with a protection-clearing retry — see
     * FLASH_bEraseRegionLocked. The region checked is the whole sector the
     * address falls in, which is what the device actually erases. */
    bool bOk = FLASH_bEraseRegionLocked(
                   cmd, 4U,
                   addr & ~((uint32_t)FLASH_SECTOR_SIZE_BYTES - 1UL),
                   (uint32_t)FLASH_SECTOR_SIZE_BYTES,
                   FLASH_WAIT_READY_TIMEOUT_MS);
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vBlockErase(uint32_t addr)
{
    FLASH_vLock();
    if (!FLASH_bEnsurePresent())
    {
        FLASH_vUnlock();
        return false;
    }

    uint8_t cmd[4] = {
        FLASH_CMD_BLOCK_ERASE,
        (uint8_t)((addr >> 16) & 0xFFU),
        (uint8_t)((addr >>  8) & 0xFFU),
        (uint8_t)((addr      ) & 0xFFU),
    };

    /* A 64 KB erase is far slower than a sector erase, so it gets its own
     * budget rather than the generic pre-command one. Verified and retried the
     * same way — see FLASH_bEraseRegionLocked. */
    bool bOk = FLASH_bEraseRegionLocked(
                   cmd, 4U,
                   addr & ~((uint32_t)FLASH_BLOCK_SIZE_BYTES - 1UL),
                   (uint32_t)FLASH_BLOCK_SIZE_BYTES,
                   FLASH_BLOCK_ERASE_TIMEOUT_MS);
    FLASH_vUnlock();
    return bOk;
}

bool FLASH_vChipErase(void)
{
    FLASH_vLock();
    if (!FLASH_bEnsurePresent())
    {
        FLASH_vUnlock();
        return false;
    }

    uint8_t cmd = FLASH_CMD_CHIP_ERASE;

    if (!FLASH_bWaitReady())
    {
        FLASH_vUnlock();
        return false;
    }

    /* A chip erase is exactly the operation used to recover a stuck device, so
     * clear any latched block protection first - otherwise the erase is
     * silently ignored and the old contents survive. Only touches the SR when
     * protection is actually set, to avoid needless status-register wear. */
    if (FLASH_u8ReadStatusReg() & FLASH_STATUS_PROTECTED)
        (void)FLASH_bGlobalUnprotect();

    /* Verified like the other two, and doubly worth it here: chip erase is the
     * recovery operation, so silently "succeeding" while doing nothing is the
     * worst failure mode this driver has. Reading back all 512 KB to prove it
     * would be absurd, so FLASH_bVerifyErasedLocked samples the head and tail
     * of the device - where a protected or interrupted erase shows up. */
    bool bReady = FLASH_bEraseRegionLocked(
                      &cmd, 1U, 0UL,
                      (uint32_t)FLASH_NUM_SECTORS * (uint32_t)FLASH_SECTOR_SIZE_BYTES,
                      FLASH_CHIP_ERASE_TIMEOUT_MS);

    FLASH_vUnlock();
    return bReady;
}

void FLASH_vInhibitDeepPowerDown(bool bInhibit)
{
    FLASH_vLock();
    bDpdInhibited = bInhibit;
    if (bInhibit)
        FLASH_vEnsureAwake();   /* wake now; keep it awake for the session */
    FLASH_vUnlock();
}

void FLASH_vDeepPowerDown(void)
{
    FLASH_vLock();
    if (bDpdInhibited)
    {
        FLASH_vUnlock();
        return;     /* OTA in progress — keep the chip awake */
    }
    if (bInDpd)
    {
        FLASH_vUnlock();
        return;     /* already parked — don't churn SPI */
    }

    /* NEVER park the device over a live program/erase. The datasheet says the
     * Ultra-Deep Power-Down command is ignored while an internally self-timed
     * operation is in progress (12.4), so in principle the device defends
     * itself here - but this guard predates that reading and stays anyway: it
     * was written for the previous part, where 0xB9 issued while busy ABORTED
     * the operation and left the page it was committing half-programmed, with
     * nothing anywhere reporting an error. Trusting a datasheet sentence over
     * a check that costs one status read, on the path that silently corrupts
     * a firmware image when it is wrong, is not a trade worth making.
     *
     * This is reachable because the write primitives used to return with the
     * program still in flight (see FLASH_vPageWrite) while this call comes
     * from the DbgLog consumer on a *different* task (Log.c) — so a log drain
     * landing inside an OTA image write silently corrupted that page. The
     * mutex serialises the SPI commands but says nothing about WIP, so it
     * never prevented this on its own.
     *
     * Skip the park rather than block: parking is a pure power optimisation
     * and the caller is a background drain, so the next idle pass gets it. */
    if (FLASH_bDeviceBusy())
    {
        FLASH_vUnlock();
        return;
    }

    /* Ultra-deep, not deep: ~200 nA against ~4.5 uA, and this device spends
     * almost all of its life parked between 1 Hz heartbeats. The cost is paid
     * on the way out - see FLASH_vEnsureAwake for the CS-pulse exit, and
     * bNeedsUnprotect for the write protection the exit re-arms. */
    uint8_t cmd = FLASH_CMD_ULTRA_DEEP_PWR_DN;
    FLASH_DRIVER_vSelect();
    FLASH_DRIVER_vWrite(&cmd, 1);
    FLASH_DRIVER_vDeselect();
    HAL_SYSTEM_vBusyWaitUs(FLASH_TEUDPD_US);   /* tEUDPD - CS high to UDPD */
    bInDpd = true;
    FLASH_vUnlock();
}

/* Force a wake whether or not this driver thinks the device is parked.
 *
 * Called from FLASH_vInit, where the state of the part is genuinely unknown:
 * a watchdog or software reset restarts the MCU without power-cycling the
 * flash, so the device can still be sitting in the UDPD its previous life
 * parked it in. bInDpd, being RAM, comes back false and would have skipped
 * the wake entirely - so the exit sequence is issued unconditionally here.
 *
 * Harmless on an already-awake device: a CS pulse with no clock starts no
 * operation (6, "if the CS pin is deasserted before complete opcode and
 * address information is sent, no operation is done"). */
void FLASH_vReleaseDeepPowerDown(void)
{
    FLASH_vLock();

    FLASH_DRIVER_vSelect();
    HAL_SYSTEM_vBusyWaitUs(FLASH_TCSLU_US);    /* >= tCSLU */
    FLASH_DRIVER_vDeselect();
    HAL_SYSTEM_vBusyWaitUs(FLASH_TXUDPD_US);   /* tXUDPD */

    bInDpd = false;
    /* Assume the worst about protection: if the device really was in UDPD,
     * 12.5.3 has just reset its Sector Protection Registers to the protected
     * default. Bring-up clears the flag again when it finds SWP already 00. */
    bNeedsUnprotect = true;

    FLASH_vUnlock();
}
