/*
 * Identity.h
 *
 * The unit's production serial number: write-once in MCU OTP, mirrored on the
 * external NOR, bound to the radio id. Storage and record format are described
 * in Identity_Config.h; the write rules in Identity.c.
 *
 * The serial number is set during production with the kernel command
 * 'N <serial> <radio id>' (see FrKernel.c) and read with 'n'. It is always
 * 8 hex digits, hardware version / revision / unit number - format in
 * Identity_Config.h.
 */

#ifndef SETTINGS_IDENTITY_H_
#define SETTINGS_IDENTITY_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    IDENTITY_SRC_NONE = 0,   /* no valid record anywhere                     */
    IDENTITY_SRC_OTP,        /* from OTP - the normal case once provisioned  */
    IDENTITY_SRC_NOR,        /* OTP holds none; the NOR mirror had one       */
} Identity_Source_e;

typedef enum {
    IDENTITY_SET_OK = 0,      /* written to OTP and mirrored                  */
    IDENTITY_SET_SAME,        /* already this serial; mirror refreshed only   */
    IDENTITY_SET_BAD_SERIAL,  /* not 0x3Rnnnnnn, see Identity_Config.h        */
    IDENTITY_SET_WRONG_UNIT,  /* radio id given is not this unit's            */
    IDENTITY_SET_ALREADY_SET, /* a different serial is in OTP; needs "fix"    */
    IDENTITY_SET_OTP_FULL,    /* no free OTP slot left                        */
    IDENTITY_SET_OTP_FAIL,    /* OTP program or read-back failed              */
} Identity_SetResult_e;

typedef struct {
    Identity_Source_e eSource;
    uint32_t u32Serial;       /* 0 when eSource is IDENTITY_SRC_NONE          */
    uint8_t  u8OtpSlot;       /* slot of the current OTP record               */
    uint8_t  u8OtpFree;       /* OTP slots still free                         */
    bool     bMirrorOk;       /* NOR mirror matches the current record        */
    bool     bRadioIdMatch;   /* record's radio id == this unit's radio id    */
} Identity_Status_t;

/* Read OTP and the mirror, and repair the mirror from OTP if needed. Needs the
 * external NOR up (FLASH_vInit). Logs what it found. */
void IDENTITY_vInit(void);

/* 0 when the unit has no serial number. */
uint32_t IDENTITY_u32GetSerial(void);

/* True when u32Serial may be written on this code base: hardware version
 * IDENTITY_SERIAL_HW_VERSION, revision 1..F, unit number 1..0xFFFFFF. */
bool IDENTITY_bSerialFormatOk(uint32_t u32Serial);

void IDENTITY_vGetStatus(Identity_Status_t *ptStatus);

/* Program a serial number into OTP and mirror it.
 *
 * u32RadioId must be this unit's full 32-bit radio id: a production station
 * talking to several units on one bench (or one radio channel) cannot then
 * stamp the wrong one. bCorrection must be true to replace a different serial
 * that is already in OTP; without it that is refused.
 *
 * Call from one task at a time (the kernel task). */
Identity_SetResult_e IDENTITY_eSetSerial(uint32_t u32Serial, uint32_t u32RadioId, bool bCorrection);

const char *IDENTITY_pcSetResultText(Identity_SetResult_e eResult);
const char *IDENTITY_pcSourceText(Identity_Source_e eSource);

#endif /* SETTINGS_IDENTITY_H_ */
