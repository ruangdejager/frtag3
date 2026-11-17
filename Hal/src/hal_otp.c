/*
 * hal_otp.c
 *
 * STM32WL OTP area access. See hal_otp.h.
 *
 * Programming follows the same per-operation pattern the bootloader uses for
 * internal flash on this chip (frtag3_bootloader fr_bootloader/src/main.c):
 * Unlock -> clear all latched error flags -> program -> Lock. The flag clear
 * is not optional. The HAL treats any error bit still latched from earlier
 * (OPTVERR from the option-byte load at reset is the usual one) as a failure
 * of THIS operation and returns HAL_ERROR without ever touching the
 * controller.
 *
 * Unlike the bootloader there is no retry. On main flash a retry re-programs
 * the same erased double-word; on OTP a first attempt that failed part way
 * may already have cleared bits, and a second program of a non-erased
 * double-word is refused by the controller (PROGERR) at best. The caller
 * moves to the next free double-word instead.
 *
 * The CPU executes from this same flash, so it stalls on instruction fetch for
 * the ~80 us a double-word program takes. Interrupts stay enabled and are
 * merely late by that much; nothing on this board has a deadline that tight.
 *
 * A double-word whose program was cut by a power loss can hold a pattern its
 * ECC cannot correct, and reading it then raises an ECC fault (NMI). The
 * window is the ~80 us program time during a deliberate production step, and
 * there is no way to read OTP that avoids ECC, so this is accepted rather than
 * handled.
 */

#include "hal_otp.h"
#include "stm32wlxx_hal.h"

#include <string.h>

bool HAL_OTP_bProgramDoubleWord(uint32_t u32Addr, uint64_t u64Data)
{
    if ((u32Addr < HAL_OTP_BASE_ADDR) ||
        (u32Addr > (HAL_OTP_BASE_ADDR + HAL_OTP_SIZE_BYTES - HAL_OTP_DWORD_BYTES)) ||
        ((u32Addr % HAL_OTP_DWORD_BYTES) != 0U))
        return false;

    HAL_StatusTypeDef eStatus;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    eStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, u32Addr, u64Data);
    HAL_FLASH_Lock();

    uint64_t u64Back;
    memcpy(&u64Back, (const void *)u32Addr, sizeof(u64Back));
    return (eStatus == HAL_OK) && (u64Back == u64Data);
}

void HAL_OTP_vRead(uint32_t u32Addr, void *pvDst, uint32_t u32Len)
{
    memcpy(pvDst, (const void *)u32Addr, u32Len);
}
