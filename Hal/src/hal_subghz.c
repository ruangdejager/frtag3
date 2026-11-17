/*
 * hal_subghz.c
 *
 * Sub-GHz radio SPI peripheral initialisation and unique-ID helper.
 *
 * There is no RF switch to drive. The final Tagskaap V3 RevA layout (both the
 * secondary board and the primary daughter board) ties RFO_LP and RFI_P/RFI_N
 * into one matching network on a single antenna port with no switch at all;
 * PB2 and PA9, which drove a GPIO-powered switch on the pre-production board,
 * are unconnected. The RF_SW_VDD power-on/off pair that used to live here is
 * gone with it - see the RF front-end note in hal_bsp.h.
 */

#include "hal_subghz.h"
#include "hal_system.h"   /* Error_Handler */
#include "hal_bsp.h"

SUBGHZ_HandleTypeDef hsubghz;

void HAL_vSUBGHZ_Init(void)
{
    hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_4;
    if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
        Error_Handler();
}

/* --------------------------------------------------------------------------
 * HAL_SUBGHZ_MspInit / HAL_SUBGHZ_MspDeInit
 * Called by HAL_SUBGHZ_Init / HAL_SUBGHZ_DeInit.
 * These override the __weak defaults in stm32wlxx_hal_subghz.c.
 * stm32wlxx_hal_msp.c must NOT define these functions.
 * -------------------------------------------------------------------------- */
void HAL_SUBGHZ_MspInit(SUBGHZ_HandleTypeDef *subghzHandle)
{
    (void)subghzHandle;
    __HAL_RCC_SUBGHZSPI_CLK_ENABLE();
    HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
}

void HAL_SUBGHZ_MspDeInit(SUBGHZ_HandleTypeDef *subghzHandle)
{
    (void)subghzHandle;
    __HAL_RCC_SUBGHZSPI_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);
}

/* --------------------------------------------------------------------------
 * HAL_SUBGHZ_vGetUid96
 * Copies the three words of the 96-bit device unique ID.
 *
 * This used to be HAL_SUBGHZ_vSetUniqueId, which packed UID words 0 and 1 into
 * an 8-byte "DevEUI" buffer that LoraRadio then picked two bytes out of. The
 * radio id now also needs word 2 (the rest of the lot number), and the byte
 * picking is an identity policy that belongs with the radio id, not in Hal.
 * -------------------------------------------------------------------------- */
void HAL_SUBGHZ_vGetUid96(uint32_t pau32Uid[3])
{
    pau32Uid[0] = HAL_GetUIDw0();
    pau32Uid[1] = HAL_GetUIDw1();
    pau32Uid[2] = HAL_GetUIDw2();
}
