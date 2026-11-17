/*
 * hal_otp.h
 *
 * STM32WL one-time-programmable area: 1 KB at 0x1FFF7000, 128 double-words of
 * 64 bits (RM0461, "OTP area"). A double-word can be programmed once, from the
 * erased state (all ones), and never erased again - not by a mass erase, not by
 * a reflash, not by the bootloader.
 *
 * What lives in it is decided above Hal; see fr_app/settings/Identity_Config.h
 * for the current allocation.
 */

#ifndef INC_HAL_OTP_H_
#define INC_HAL_OTP_H_

#include <stdint.h>
#include <stdbool.h>

#define HAL_OTP_BASE_ADDR       0x1FFF7000UL
#define HAL_OTP_SIZE_BYTES      1024U
#define HAL_OTP_DWORD_BYTES     8U

/* Program one double-word. u32Addr must be 8-byte aligned and inside the OTP
 * area. Returns true only if the flash controller reported success AND the
 * double-word reads back as written; a false return may still have burnt the
 * double-word, so the caller must treat it as used either way. */
bool HAL_OTP_bProgramDoubleWord(uint32_t u32Addr, uint64_t u64Data);

/* Plain memory read; OTP is memory-mapped. */
void HAL_OTP_vRead(uint32_t u32Addr, void *pvDst, uint32_t u32Len);

#endif /* INC_HAL_OTP_H_ */
