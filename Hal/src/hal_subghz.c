/*
 * hal_subghz.c
 *
 * Sub-GHz radio SPI peripheral initialisation and unique-ID helper.
 */

#include "hal_subghz.h"
#include "hal_system.h"
#include "hal_bsp.h"

SUBGHZ_HandleTypeDef hsubghz;

/* RF_SW_VDD state. Read by the path-select call (radio_board_if.c) and by the
 * STOP2 entry path (hal_gpio.c), neither of which can infer it from the pin:
 * during power-up the pin is already HIGH for the whole
 * BSP_RF_SW_VDD_SETTLE_US settle, before the switch is usable, and this flag
 * only becomes true once that settle has finished. */
static volatile bool bRfSwitchPowered = false;

void HAL_vSUBGHZ_Init(void)
{
    hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_4;
    if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
        Error_Handler();
}

/* --------------------------------------------------------------------------
 * HAL_SUBGHZ_vRfSwitchPowerOn — bring the RF switch's supply rail up.
 *
 * Called once where the radio leaves sleep, BEFORE the first SUBGHZ SPI
 * transaction, and never per TX/RX.
 *
 * The settle is not on the TDD critical path and costs nothing in practice:
 * the radio's own wake already spends milliseconds before it is configured —
 * SUBGHZ_CheckDeviceReady spins SUBGHZ_NSS_LOOP_TIME (~1-3 ms at 48 MHz) on
 * the first command out of deep sleep, then SUBGRF_Init waits on the TCXO
 * (RF_WAKEUP_TIME, 1 ms) and a full SUBGRF_Calibrate(0x7F) (~3.5 ms). The
 * 350 us here is spent before all of that, so the rail is long settled by the
 * time the switch is asked to pass anything.
 * -------------------------------------------------------------------------- */
void HAL_SUBGHZ_vRfSwitchPowerOn(void)
{
    if (bRfSwitchPowered)
        return;

    HAL_GPIO_WritePin(BSP_RF_SW_VDD_PORT, BSP_RF_SW_VDD_PIN, GPIO_PIN_SET);
    HAL_SYSTEM_vBusyWaitUs(BSP_RF_SW_VDD_SETTLE_US);

    bRfSwitchPowered = true;
}

/* --------------------------------------------------------------------------
 * HAL_SUBGHZ_vRfSwitchPowerOff — drop the RF switch's supply rail.
 *
 * Called once where the radio enters sleep, after TX/RX has stopped.
 *
 * ORDER IS MANDATORY: CTRL low first, then VDD low. While VDD sits at 0 V a
 * high CTRL forward-biases the CTRL pin's ESD diode into the supply rail,
 * parasitically powering the switch die through the 470R, leaking current and
 * leaving the switch in an undefined state. Never reverse these two writes,
 * and never drop VDD from anywhere that has not already driven CTRL low.
 *
 * SUBGRF_SetSleep already calls RBI_ConfigRFSwitch(RBI_SWITCH_OFF) before it
 * puts the chip to sleep, so CTRL is normally low on entry; the write here is
 * deliberately unconditional rather than trusting that.
 * -------------------------------------------------------------------------- */
void HAL_SUBGHZ_vRfSwitchPowerOff(void)
{
    HAL_GPIO_WritePin(BSP_RF_SW_CTRL_PORT, BSP_RF_SW_CTRL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_RF_SW_VDD_PORT,  BSP_RF_SW_VDD_PIN,  GPIO_PIN_RESET);

    bRfSwitchPowered = false;
}

bool HAL_SUBGHZ_bRfSwitchPowered(void)
{
    return bRfSwitchPowered;
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
 * HAL_SUBGHZ_vSetUniqueId
 * Writes the first 8 bytes of the 96-bit device unique ID into id[].
 * -------------------------------------------------------------------------- */
void HAL_SUBGHZ_vSetUniqueId(uint8_t *id)
{
    uint32_t uid[3];
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();

    id[0] = (uid[0] >> 24) & 0xFF;
    id[1] = (uid[0] >> 16) & 0xFF;
    id[2] = (uid[0] >>  8) & 0xFF;
    id[3] = (uid[0]      ) & 0xFF;
    id[4] = (uid[1] >> 24) & 0xFF;
    id[5] = (uid[1] >> 16) & 0xFF;
    id[6] = (uid[1] >>  8) & 0xFF;
    id[7] = (uid[1]      ) & 0xFF;
}
