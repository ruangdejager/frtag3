/*
 * Flash.h
 *
 * Public API for the AT25EU0041A external NOR flash device driver
 * (U1 on SPI2, both roles; see Flash_Config.h for the part and board notes).
 */

#ifndef DEVICE_FLASH_FLASH_H_
#define DEVICE_FLASH_FLASH_H_

#include <stdint.h>
#include <stdbool.h>

void    FLASH_vInit(void);
bool    FLASH_bDeviceBusy(void);
uint8_t FLASH_u8ReadStatusReg(void);       /* SR1 (05h): SRP0, BP4..0, WEL, BSY */
uint8_t FLASH_u8ReadStatusReg2(void);      /* SR2 (35h): SUS, CMP, LB, QE, SRP1 -
                                            * needed to judge protection, since
                                            * CMP = 1 alone protects everything */
bool    FLASH_bVerifyDevice(void);          /* reads JEDEC ID; DBG-warns on mismatch */

/* As FLASH_bVerifyDevice, but retried (FLASH_JEDEC_READ_ATTEMPTS) and
 * reporting evidence: pu8Id receives the last 3 ID bytes read, pu8Attempts the
 * number of tries used. Both optional (NULL to ignore). Use this where the
 * result is going to be shown to a human or acted on — a bare pass/fail off a
 * single read is not trustworthy enough to call a chip dead. */
bool    FLASH_bVerifyDeviceEx(uint8_t *pu8Id, uint8_t *pu8Attempts);

/* The driver's actual usability gate, as opposed to "can a JEDEC read be
 * coaxed through right now". Every read/write/erase entry point returns false
 * immediately when this is false, so it - not FLASH_bVerifyDevice*() - is what
 * a health check must report. Reporting the probe instead is what let a tag
 * print "SelfTest: flash=OK" for 18 h while the driver was gated off and every
 * FOTA erase was failing on the first call. */
bool    FLASH_bDevicePresent(void);

/* Force a probe now, ignoring the FLASH_REPROBE_INTERVAL_MS cooldown, and on
 * success re-run bring-up (protection check + global unprotect + disarm) and
 * re-open the gate. Returns the resulting gate state. Optional out-params as
 * per FLASH_bVerifyDeviceEx. Use where a human or a health check is asking the
 * question; ordinary traffic recovers on its own via the rate-limited path.
 *
 * Monotonic: this can only ever OPEN the gate, never close one already open. A
 * probe can fail on a healthy chip - that is the whole premise of the retry
 * logic - so letting one failed probe disable logging and FOTA staging would
 * reintroduce, through a different door, the exact fault this exists to undo. */
bool    FLASH_bRecoverDevice(uint8_t *pu8Id, uint8_t *pu8Attempts);

/* Durable health summary. Everything here survives the flash being unusable
 * (it is all in RAM plus a TAMP backup register), which is the point: when the
 * flash is the casualty, the flash log cannot be the place the fault is
 * recorded. Retrievable over FrKernel ("tag <ID> flash") so a field unit that
 * cannot be reached with a UART can still be asked what is wrong. */
typedef struct
{
    bool     bPresent;          /* the gate above - writes/erases permitted   */
    bool     bEverAbsent;       /* gate has been closed at least once         */
    bool     bWriteProtected;   /* BP/SRP0 (SR1) or CMP/SRP1 (SR2) seen set -
                                 * never the power-on default on this part,
                                 * so any 1 here means something wrote the
                                 * non-volatile SR                            */
    bool     bUnprotectFailed;  /* global unprotect ran and did not clear it  */
    bool     bEraseVerifyFail;  /* an erase reported success but did not blank */
    uint8_t  au8LastId[3];      /* JEDEC bytes from the most recent probe     */
    uint8_t  u8LastAttempts;    /* tries the most recent probe needed         */
    uint16_t u16ProbeFailures;  /* probes that never saw the right mfr ID     */
    uint16_t u16Recoveries;     /* times the gate re-opened after being shut  */
    uint16_t u16EraseVerifyFails;
    /* (bEpeSeen / u16EpeFails removed: the AT25EU0041A has no EPE bit - SR1
     *  bit 5 is BP3 - so the erase read-back above is the only erase-failure
     *  signal there is.) */
    uint16_t u16ReadCrcRetries; /* reads that disagreed on the first compare
                                 * and were re-read (see FLASH_vRead)        */
    uint16_t u16ReadCrcFails;   /* reads that never converged within the
                                 * retry budget - returned bytes are suspect */
} Flash_Health_t;

void    FLASH_vGetHealth(Flash_Health_t *ptHealth);

/* Why a FLASH_vRead() returned false. Callers used to get a bare bool, which
 * is not enough to diagnose anything: FOTA_u8CalcImageXorRangeBuf collapsed
 * every one of these into "the checksum is wrong", so a field log could not
 * tell a failed SPI transfer from genuinely bad stored bytes — the exact
 * ambiguity that left the 2026-08-31 "xor verify recovered on attempt 2/8"
 * events undiagnosable. */
typedef enum
{
    FLASH_RDFAIL_NONE = 0,   /* no failure recorded since the last read-out  */
    FLASH_RDFAIL_ABSENT,     /* bDevicePresent false - JEDEC ID never matched */
    FLASH_RDFAIL_WAITREADY,  /* still WIP after FLASH_WAIT_READY_TIMEOUT_MS   */
    FLASH_RDFAIL_CMD,        /* SPI transmit of the 4-byte read command failed */
    FLASH_RDFAIL_DATA,       /* SPI receive of the payload failed             */
    FLASH_RDFAIL_CRC,        /* the transfer succeeded but two reads of the
                              * same bytes disagreed and never converged - so
                              * the bus worked and the DATA is wrong, which is
                              * a different fault from every reason above and
                              * the only one that points at the board rather
                              * than at the driver                            */
} Flash_ReadFailReason_t;

/* Read-failure tally since the last call; reading clears it (same idiom as
 * LORARADIO_u16GetAndClearCadTimeouts — a counter rather than a log line per
 * occurrence, because FLASH_vRead is called ~1900 times per whole-image XOR
 * pass and logging inside that would perturb the timing being measured).
 *
 * The details reported belong to the FIRST failure since the last read-out,
 * not the most recent: what went wrong first is what explains the rest.
 * All three out-params are optional (NULL to ignore). pu8HalStatus carries
 * the HAL_StatusTypeDef of the failing transfer for the CMD/DATA reasons,
 * which is what distinguishes a HAL_TIMEOUT (task starved across the 1000 ms
 * SPI_TIMEOUT) from a HAL_ERROR (peripheral fault). */
uint16_t FLASH_u16GetAndClearReadFails(Flash_ReadFailReason_t *ptReason,
                                       uint32_t *pu32Addr,
                                       uint8_t  *pu8HalStatus);

/* All operations below wait for the device to go ready (bounded timeout)
 * before issuing their command, and return false without touching the
 * flash if the device is still busy or the SPI transaction failed -
 * never issue a command onto a busy/unresponsive device. */
bool    FLASH_vRead(uint32_t addr, uint8_t *buf, uint16_t len);
bool    FLASH_vPageWrite(uint32_t addr, const uint8_t *buf, uint16_t len);
bool    FLASH_vSectorErase(uint32_t addr);  /* erases 4 KB sector containing addr  */
bool    FLASH_vBlockErase(uint32_t addr);   /* erases 64 KB block containing addr  */
bool    FLASH_vChipErase(void);

/* Park the device in Deep Power-Down (B9h) and bring it back (ABh).
 *
 * AT25EU0041A §6.3.7 / §6.3.1: ~0.1 uA typ parked against ~10.5 uA standby
 * (Table 20). This part has only the one power-down mode - no Ultra-Deep
 * Power-Down as the briefly-fitted AT25XE041B had - and leaving it changes no
 * register, so a wake owes nothing to the write path.
 *
 * ONE CONSEQUENCE CALLERS DO NOT SEE, handled inside the driver: while
 * parked the device ignores every command except ABh, status reads included,
 * and after ABh it needs tRES1 (8 us max) with CS# high before the next
 * command is accepted. Every entry point wakes the part on demand first
 * (FLASH_vEnsureAwake), and FLASH_vReleaseDeepPowerDown wakes it
 * unconditionally for the boot path, where a watchdog reset may have left it
 * parked by the previous run. */
void    FLASH_vDeepPowerDown(void);
void    FLASH_vReleaseDeepPowerDown(void);

/* Inhibit (true) or re-allow (false) deep-power-down parking. While
 * inhibited, FLASH_vDeepPowerDown() is a no-op and the device is woken
 * immediately, so the chip stays continuously awake. Used to bracket an
 * OTA session: reads issued shortly after a DPD wake were observed to
 * return corrupted bytes (the bootloader, which never parks the chip
 * mid-verify, reads the same image cleanly), so for the multi-minute OTA
 * transfer we keep the flash awake the whole time. */
void    FLASH_vInhibitDeepPowerDown(bool bInhibit);

/* Read-verify (CRC) tally since the last call; reading clears it. Same
 * read-and-clear idiom as FLASH_u16GetAndClearReadFails, and for the same
 * reason - FLASH_vRead is called ~1900 times per whole-image XOR pass, so a
 * log line per event would perturb the very timing being measured.
 *
 * pu16Retries counts reads that disagreed on the first compare and were
 * recovered by a re-read; pu16Fails counts reads that never converged and
 * whose returned bytes are therefore not trustworthy. A non-zero retry count
 * with a zero fail count is this feature doing its job. Both optional.
 *
 * Always available; returns zeroes when ENABLE_FLASH_READ_CRC is not defined,
 * so callers need no #ifdef. */
uint32_t FLASH_u32GetAndClearReadCrcStats(uint16_t *pu16Retries,
                                          uint16_t *pu16Fails);

#endif /* DEVICE_FLASH_FLASH_H_ */
