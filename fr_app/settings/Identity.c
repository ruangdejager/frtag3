/*
 * Identity.c
 *
 * Production serial number: OTP primary, NOR mirror. Storage, record format and
 * the reasons for both are in Identity_Config.h.
 *
 * Why the serial number is not a setting. The fr9 keeps its serial number in
 * the same record as every user setting and protects it with a "never
 * force-reset this field" rule in its reset code. That covers a factory reset,
 * but not the record being lost: the fr9 erases both settings pages before
 * rewriting, so a power cut at the wrong moment takes the serial number with
 * everything else, and a corrupt record falls back to defaults including the
 * "700000" placeholder serial. A unit's identity has to outlive all of that,
 * so here it is stored where nothing but a deliberate production command can
 * change it.
 *
 * Read rule. The current record is the highest-numbered valid OTP slot. A slot
 * that is not blank but not valid (a cut-off program, a record of a format
 * this firmware does not know) is skipped but counts as used: OTP cannot be
 * programmed over. When OTP holds no valid record the NOR mirror is used if it
 * validates against this die's UID.
 *
 * Write rule. Only IDENTITY_eSetSerial writes, only to the next free slot, and
 * only when the caller names this unit's full radio id. Replacing a different
 * serial needs bCorrection; writing the serial that is already there costs no
 * OTP at all.
 */

#include "Identity.h"
#include "Identity_Config.h"
#include "Identity_Driver.h"
#include "crc32.h"
#include "DbgLog.h"

#include <string.h>

typedef struct {
    uint16_t u16Magic;
    uint8_t  u8Format;
    uint8_t  u8Reserved;
    uint32_t u32Serial;
    uint32_t u32RadioId;
    uint32_t u32Crc;
} Identity_Record_t;

_Static_assert(sizeof(Identity_Record_t) == IDENTITY_RECORD_LEN,
               "identity record must be 16 bytes - two OTP double-words");
_Static_assert((IDENTITY_RECORD_LEN % HAL_OTP_DWORD_BYTES) == 0U,
               "identity record must be whole OTP double-words");
_Static_assert(IDENTITY_OTP_END_ADDR <= (HAL_OTP_BASE_ADDR + HAL_OTP_SIZE_BYTES),
               "identity slots run past the end of OTP");
_Static_assert(IDENTITY_OTP_SLOTS <= 127U, "slot indices are tracked as int8_t");

/* Bytes of the record covered by the CRC: everything before the CRC itself. */
#define IDENTITY_CRC_SPAN   12U

static Identity_Status_t s_tStatus;

/* -------------------------------------------------------------------------- */

static uint32_t IDENTITY_u32RecordCrc(const Identity_Record_t *ptRec)
{
    uint32_t au32Uid[3];
    IDENTITY_DRIVER_vGetUid96(au32Uid);

    /* The UID words go in as they sit in memory, i.e. little-endian on this
     * core - the byte order Identity_Config.h promises off-board tooling. */
    uint32_t u32Crc = CRC32_u32Update(CRC32_INIT, (const uint8_t *)ptRec, IDENTITY_CRC_SPAN);
    u32Crc = CRC32_u32Update(u32Crc, (const uint8_t *)au32Uid, sizeof(au32Uid));
    return CRC32_u32Final(u32Crc);
}

static bool IDENTITY_bIsBlank(const Identity_Record_t *ptRec)
{
    const uint8_t *pu8 = (const uint8_t *)ptRec;
    for (uint8_t i = 0U; i < sizeof(*ptRec); i++)
        if (pu8[i] != 0xFFU)
            return false;
    return true;
}

static bool IDENTITY_bIsValid(const Identity_Record_t *ptRec)
{
    return (ptRec->u16Magic == IDENTITY_MAGIC) &&
           (ptRec->u8Format == IDENTITY_FORMAT) &&
           (ptRec->u32Serial >= IDENTITY_SERIAL_MIN) &&
           (ptRec->u32Serial <= IDENTITY_SERIAL_MAX) &&
           (ptRec->u32Crc == IDENTITY_u32RecordCrc(ptRec));
}

/* Returns the current slot (-1 if none), fills *ptCur with its record and
 * *pu8Free with the number of slots after the last used one. */
static int8_t IDENTITY_i8ScanOtp(Identity_Record_t *ptCur, uint8_t *pu8Free)
{
    int8_t i8LastUsed = -1;
    int8_t i8Current  = -1;
    Identity_Record_t tRec;

    for (uint8_t s = 0U; s < IDENTITY_OTP_SLOTS; s++)
    {
        IDENTITY_DRIVER_vOtpRead(IDENTITY_OTP_BASE_ADDR + (uint32_t)s * IDENTITY_RECORD_LEN,
                                 &tRec, sizeof(tRec));
        if (IDENTITY_bIsBlank(&tRec))
            continue;
        i8LastUsed = (int8_t)s;
        if (IDENTITY_bIsValid(&tRec))
        {
            i8Current = (int8_t)s;
            *ptCur    = tRec;
        }
    }

    *pu8Free = (uint8_t)(IDENTITY_OTP_SLOTS - (uint8_t)(i8LastUsed + 1));
    return i8Current;
}

static bool IDENTITY_bReadMirror(Identity_Record_t *ptRec)
{
    return IDENTITY_DRIVER_bNorRead(IDENTITY_NOR_ADDR, (uint8_t *)ptRec, sizeof(*ptRec));
}

static bool IDENTITY_bMirrorMatches(const Identity_Record_t *ptRec)
{
    Identity_Record_t tMirror;
    return IDENTITY_bReadMirror(&tMirror) && (memcmp(&tMirror, ptRec, sizeof(tMirror)) == 0);
}

/* Rewrite the mirror sector with this record. One erase per actual change:
 * the caller only comes here when the mirror differs. */
static bool IDENTITY_bWriteMirror(const Identity_Record_t *ptRec)
{
    return IDENTITY_DRIVER_bNorEraseSector(IDENTITY_NOR_ADDR) &&
           IDENTITY_DRIVER_bNorWrite(IDENTITY_NOR_ADDR, (const uint8_t *)ptRec, sizeof(*ptRec)) &&
           IDENTITY_bMirrorMatches(ptRec);
}

static void IDENTITY_vSetStatus(Identity_Source_e eSrc, const Identity_Record_t *ptRec,
                                int8_t i8Slot, uint8_t u8Free, bool bMirrorOk)
{
    s_tStatus.eSource       = eSrc;
    s_tStatus.u32Serial     = (eSrc != IDENTITY_SRC_NONE) ? ptRec->u32Serial : 0U;
    s_tStatus.u8OtpSlot     = (i8Slot >= 0) ? (uint8_t)i8Slot : 0U;
    s_tStatus.u8OtpFree     = u8Free;
    s_tStatus.bMirrorOk     = bMirrorOk;
    s_tStatus.bRadioIdMatch = (eSrc != IDENTITY_SRC_NONE) &&
                              (ptRec->u32RadioId == IDENTITY_DRIVER_u32GetRadioId());
}

/* -------------------------------------------------------------------------- */

void IDENTITY_vInit(void)
{
    Identity_Record_t tRec;
    uint8_t u8Free;
    int8_t  i8Slot = IDENTITY_i8ScanOtp(&tRec, &u8Free);

    if (i8Slot >= 0)
    {
        bool bMirrorOk = IDENTITY_bMirrorMatches(&tRec);
        if (!bMirrorOk)
        {
            bMirrorOk = IDENTITY_bWriteMirror(&tRec);
            DBG_LOG("Identity: NOR mirror %s from OTP\r\n", bMirrorOk ? "restored" : "could NOT be restored");
        }
        IDENTITY_vSetStatus(IDENTITY_SRC_OTP, &tRec, i8Slot, u8Free, bMirrorOk);
    }
    else if (IDENTITY_bReadMirror(&tRec) && IDENTITY_bIsValid(&tRec))
    {
        IDENTITY_vSetStatus(IDENTITY_SRC_NOR, &tRec, -1, u8Free, true);
    }
    else
    {
        IDENTITY_vSetStatus(IDENTITY_SRC_NONE, &tRec, -1, u8Free, false);
    }

    if (s_tStatus.eSource == IDENTITY_SRC_NONE)
    {
        DBG_LOG("Identity: NO serial number (radio id %08lX, %u OTP slots free)\r\n",
                (unsigned long)IDENTITY_DRIVER_u32GetRadioId(), (unsigned)u8Free);
        return;
    }

    DBG_LOG("Identity: serial %08lX from %s (radio id %08lX, %u OTP slots free)\r\n",
            (unsigned long)s_tStatus.u32Serial, IDENTITY_pcSourceText(s_tStatus.eSource),
            (unsigned long)IDENTITY_DRIVER_u32GetRadioId(), (unsigned)u8Free);

    /* The CRC already ties the record to this die, so this can only fire if
     * the radio-id derivation itself changed after the record was written. */
    if (!s_tStatus.bRadioIdMatch)
        DBG_LOG("Identity: record was written for radio id %08lX - the radio id derivation has changed\r\n",
                (unsigned long)tRec.u32RadioId);
}

uint32_t IDENTITY_u32GetSerial(void)
{
    return s_tStatus.u32Serial;
}

bool IDENTITY_bSerialFormatOk(uint32_t u32Serial)
{
    const uint32_t u32Ver  = (u32Serial >> IDENTITY_SERIAL_VER_SHIFT) & IDENTITY_SERIAL_NIBBLE_MASK;
    const uint32_t u32Rev  = (u32Serial >> IDENTITY_SERIAL_REV_SHIFT) & IDENTITY_SERIAL_NIBBLE_MASK;
    const uint32_t u32Unit = u32Serial & IDENTITY_SERIAL_UNIT_MASK;

    return (u32Ver == IDENTITY_SERIAL_HW_VERSION) && (u32Rev != 0U) && (u32Unit != 0U);
}

void IDENTITY_vGetStatus(Identity_Status_t *ptStatus)
{
    if (ptStatus != NULL)
        *ptStatus = s_tStatus;
}

Identity_SetResult_e IDENTITY_eSetSerial(uint32_t u32Serial, uint32_t u32RadioId, bool bCorrection)
{
    /* Strict format on write only - see "Serial number format" in
     * Identity_Config.h for why reading stays lenient. */
    if (!IDENTITY_bSerialFormatOk(u32Serial))
        return IDENTITY_SET_BAD_SERIAL;

    /* Full 32-bit id, exact match - deliberately not LORARADIO_bIsOwnId, which
     * accepts the 16-bit form that two units can share. */
    if (u32RadioId != IDENTITY_DRIVER_u32GetRadioId())
        return IDENTITY_SET_WRONG_UNIT;

    Identity_Record_t tCur;
    uint8_t u8Free;
    int8_t  i8Slot = IDENTITY_i8ScanOtp(&tCur, &u8Free);

    if ((i8Slot >= 0) && (tCur.u32Serial == u32Serial))
    {
        bool bMirrorOk = IDENTITY_bMirrorMatches(&tCur) || IDENTITY_bWriteMirror(&tCur);
        IDENTITY_vSetStatus(IDENTITY_SRC_OTP, &tCur, i8Slot, u8Free, bMirrorOk);
        return IDENTITY_SET_SAME;
    }
    if ((i8Slot >= 0) && !bCorrection)
        return IDENTITY_SET_ALREADY_SET;
    if (u8Free == 0U)
        return IDENTITY_SET_OTP_FULL;

    const uint8_t  u8NewSlot = (uint8_t)(IDENTITY_OTP_SLOTS - u8Free);
    const uint32_t u32Addr   = IDENTITY_OTP_BASE_ADDR + (uint32_t)u8NewSlot * IDENTITY_RECORD_LEN;

    Identity_Record_t tNew = {
        .u16Magic   = IDENTITY_MAGIC,
        .u8Format   = IDENTITY_FORMAT,
        .u8Reserved = 0U,
        .u32Serial  = u32Serial,
        .u32RadioId = u32RadioId,
        .u32Crc     = 0U,
    };
    tNew.u32Crc = IDENTITY_u32RecordCrc(&tNew);

    uint64_t au64Dw[2];
    memcpy(au64Dw, &tNew, sizeof(au64Dw));

    DBG_LOG("Identity: writing serial %08lX to OTP slot %u%s\r\n", (unsigned long)u32Serial,
            (unsigned)u8NewSlot, (i8Slot >= 0) ? " (correction)" : "");

    /* Both double-words are attempted even if the first fails: the slot is
     * spent either way, and a half-written slot is simply invalid. */
    bool bProgOk = IDENTITY_DRIVER_bOtpProgram(u32Addr, au64Dw[0]);
    bProgOk      = IDENTITY_DRIVER_bOtpProgram(u32Addr + HAL_OTP_DWORD_BYTES, au64Dw[1]) && bProgOk;

    /* Judge by what OTP now holds, not by the program status alone. */
    i8Slot = IDENTITY_i8ScanOtp(&tCur, &u8Free);
    if (!bProgOk || (i8Slot != (int8_t)u8NewSlot) || (tCur.u32Serial != u32Serial))
    {
        DBG_LOG("Identity: OTP write of slot %u FAILED - %u slots left\r\n",
                (unsigned)u8NewSlot, (unsigned)u8Free);
        /* Status keeps describing whatever is current now (possibly the old
         * serial, possibly none). */
        if (i8Slot >= 0)
            IDENTITY_vSetStatus(IDENTITY_SRC_OTP, &tCur, i8Slot, u8Free, IDENTITY_bMirrorMatches(&tCur));
        else
            s_tStatus.u8OtpFree = u8Free;
        return IDENTITY_SET_OTP_FAIL;
    }

    bool bMirrorOk = IDENTITY_bWriteMirror(&tCur);
    IDENTITY_vSetStatus(IDENTITY_SRC_OTP, &tCur, i8Slot, u8Free, bMirrorOk);
    DBG_LOG("Identity: serial %08lX written to OTP slot %u, mirror %s\r\n",
            (unsigned long)u32Serial, (unsigned)u8NewSlot, bMirrorOk ? "ok" : "FAILED");
    return IDENTITY_SET_OK;
}

const char *IDENTITY_pcSetResultText(Identity_SetResult_e eResult)
{
    switch (eResult)
    {
        case IDENTITY_SET_OK:          return "written";
        case IDENTITY_SET_SAME:        return "already set to this serial";
        case IDENTITY_SET_BAD_SERIAL:  return "serial must be 3Rnnnnnn hex: hardware version 3, revision 1-F, unit 000001-FFFFFF";
        case IDENTITY_SET_WRONG_UNIT:  return "radio id does not match this unit";
        case IDENTITY_SET_ALREADY_SET: return "a different serial is set - add 'fix' to replace it";
        case IDENTITY_SET_OTP_FULL:    return "no free OTP slot left";
        case IDENTITY_SET_OTP_FAIL:    return "OTP write failed";
        default:                       return "?";
    }
}

const char *IDENTITY_pcSourceText(Identity_Source_e eSource)
{
    switch (eSource)
    {
        case IDENTITY_SRC_OTP: return "otp";
        case IDENTITY_SRC_NOR: return "nor-mirror";
        default:               return "none";
    }
}
