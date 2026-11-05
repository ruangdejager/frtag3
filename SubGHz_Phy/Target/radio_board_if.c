/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    radio_board_if.c
  * @brief   Radio board interface for frtag3 (Tagskaap V3 RevA).
  *
  *          No RF switch. Both final boards (the secondary and the primary
  *          daughter board) use the STM32WL "direct tie" front end: RFO_LP and
  *          RFI_P/RFI_N share one matching network to one antenna port, and
  *          RFO_HP is left open. The radio's own LP PA and LNA do the TX/RX
  *          change-over internally, so there is nothing for RBI_ConfigRFSwitch
  *          to drive. PB2 and PA9, which powered and steered a GPIO-supplied
  *          switch on the pre-production board, are unconnected on the final
  *          layout and are parked analog by HAL_GPIO_vInit (see hal_bsp.h).
  *
  *          The TX path is RFO_LP only (RBI_CONF_RFO in radio_board_if.h),
  *          so the LP PA's +15 dBm ceiling is the board's ceiling too.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "radio_board_if.h"
#include "radio_driver.h"   /* TCXO_CTRL_* */

/* TCXO supply, chosen per role by RBI_vSetPrimaryRole(). Defaults to the
 * secondary's 1.7 V: that value is valid on BOTH rails (it is the only
 * in-spec choice on the 1.8 V secondary, and merely conservative on the
 * 3.0 V primary), so a radio init that somehow ran before the role was known
 * would still start the TCXO. See radio_conf.h. */
static uint8_t s_u8TcxoVoltage = (uint8_t)TCXO_CTRL_1_7V;

void RBI_vSetPrimaryRole(bool bPrimary)
{
    s_u8TcxoVoltage = bPrimary ? (uint8_t)TCXO_CTRL_2_2V
                               : (uint8_t)TCXO_CTRL_1_7V;
}

uint8_t RBI_u8GetTcxoVoltage(void)
{
    return s_u8TcxoVoltage;
}

/* Nothing to initialise: no switch, no control pins (see the file header). */
int32_t RBI_Init(void)
{
    return 0;
}

int32_t RBI_DeInit(void)
{
    return 0;
}

/* Called by the vendor driver on every TX/RX/sleep transition. With the
 * direct-tie front end there is no external path to select, so every request
 * succeeds without touching a pin. RFO_HP is not connected on either board; it
 * is accepted here only because RBI_GetTxConfig() never lets the driver ask
 * for it. */
int32_t RBI_ConfigRFSwitch(RBI_Switch_TypeDef Config)
{
    switch (Config)
    {
        case RBI_SWITCH_OFF:
        case RBI_SWITCH_RX:
        case RBI_SWITCH_RFO_LP:
        case RBI_SWITCH_RFO_HP:
            return 0;
        default:
            return -1;
    }
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
