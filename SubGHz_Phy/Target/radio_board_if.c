/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    radio_board_if.c
  * @brief   Radio board interface for frtag3.
  *          RF switch: PA9 (RF_SW_CTRL) — HIGH=TX(LP), LOW=RX/OFF
  *                     PB2 (RF_SW_VDD)  — switch supply, owned by hal_subghz
  *          Pins come from hal_bsp.h, not main.h's CubeMX labels, so a .ioc
  *          regeneration cannot silently rename them out from under this file.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "radio_board_if.h"
#include "main.h"
#include "stm32wlxx_hal.h"

#include "hal_bsp.h"
#include "hal_system.h"
#include "hal_subghz.h"
#include "dbg_log.h"

int32_t RBI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(BSP_RF_SW_CTRL_PORT, BSP_RF_SW_CTRL_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = BSP_RF_SW_CTRL_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BSP_RF_SW_CTRL_PORT, &GPIO_InitStruct);

    return 0;
}

int32_t RBI_DeInit(void)
{
    HAL_GPIO_WritePin(BSP_RF_SW_CTRL_PORT, BSP_RF_SW_CTRL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_DeInit(BSP_RF_SW_CTRL_PORT, BSP_RF_SW_CTRL_PIN);
    return 0;
}

/* Path select for an RX<->TX turnaround. RF_SW_VDD must ALREADY be up: the
 * rail follows the radio's power state and is raised once at radio wake
 * (HAL_SUBGHZ_vRfSwitchPowerOn), not here. Powering the switch up inside a
 * path select would hide a sequencing bug behind a 350 us stall on a path that
 * is supposed to cost 50 us, so this reports the fault and refuses instead.
 *
 * The settle below is the switch's own CTRL network (100R x 101 nF); the
 * caller may enable the PA or enter RX once it returns. */
int32_t RBI_ConfigRFSwitch(RBI_Switch_TypeDef Config)
{
    if (!HAL_SUBGHZ_bRfSwitchPowered())
    {
        DBG_LOG("RBI: path select (%d) with RF_SW_VDD down — radio not woken\r\n",
                (int)Config);
        return -1;
    }

    switch (Config)
    {
        case RBI_SWITCH_OFF:
        case RBI_SWITCH_RX:
            HAL_GPIO_WritePin(BSP_RF_SW_CTRL_PORT, BSP_RF_SW_CTRL_PIN, GPIO_PIN_RESET);
            break;
        case RBI_SWITCH_RFO_LP:
        case RBI_SWITCH_RFO_HP:
            HAL_GPIO_WritePin(BSP_RF_SW_CTRL_PORT, BSP_RF_SW_CTRL_PIN, GPIO_PIN_SET);
            break;
        default:
            return -1;
    }

    HAL_SYSTEM_vBusyWaitUs(BSP_RF_SW_CTRL_SETTLE_US);
    return 0;
}

int32_t RBI_GetTxConfig(void)
{
    return RBI_CONF_RFO_LP;
}

int32_t RBI_IsTCXO(void)
{
    return IS_TCXO_SUPPORTED;
}

int32_t RBI_IsDCDC(void)
{
    return IS_DCDC_SUPPORTED;
}

int32_t RBI_GetRFOMaxPowerConfig(RBI_RFOMaxPowerConfig_TypeDef Config)
{
    if (Config == RBI_RFO_LP_MAXPOWER)
        return 15;
    else
        return 22;
}
