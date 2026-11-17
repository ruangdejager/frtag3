/*
 * Identity_Driver.h
 *
 * Maps the identity module's storage access onto Hal (OTP), the external NOR
 * driver (mirror) and the radio id, so Identity.c names no bus API directly.
 */

#ifndef SETTINGS_IDENTITY_DRIVER_H_
#define SETTINGS_IDENTITY_DRIVER_H_

#include "hal_otp.h"
#include "Flash.h"
#include "LoraRadio.h"
#include "LoraRadio_Driver.h"

#define IDENTITY_DRIVER_vOtpRead(addr, dst, len)    HAL_OTP_vRead((addr), (dst), (len))
#define IDENTITY_DRIVER_bOtpProgram(addr, u64)      HAL_OTP_bProgramDoubleWord((addr), (u64))

#define IDENTITY_DRIVER_bNorRead(addr, dst, len)    FLASH_vRead((addr), (dst), (len))
#define IDENTITY_DRIVER_bNorWrite(addr, src, len)   FLASH_vPageWrite((addr), (src), (len))
#define IDENTITY_DRIVER_bNorEraseSector(addr)       FLASH_vSectorErase(addr)

#define IDENTITY_DRIVER_u32GetRadioId()             LORARADIO_u32GetUniqueId()
#define IDENTITY_DRIVER_vGetUid96(pau32Uid)         LORARADIO_DRIVER_vGetUid96(pau32Uid)

#endif /* SETTINGS_IDENTITY_DRIVER_H_ */
