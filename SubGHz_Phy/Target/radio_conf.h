/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    radio_conf.h
  * @author  MCD Application Team
  * @brief   Header of Radio configuration
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RADIO_CONF_H__
#define __RADIO_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "main.h"
#include "stm32_mem.h"       /* RADIO_MEMSET8 def in this file */
#include "mw_log_conf.h"     /* mw trace conf */
#include "radio_board_if.h"  /* low layer api (bsp) */
#include "utilities_def.h"   /* low layer api (bsp) */
#include "hal_subghz.h"      /* HAL_vSUBGHZ_Init declaration */
/* USER CODE BEGIN include */

/* USER CODE END include */

/* Exported types ------------------------------------------------------------*/
extern SUBGHZ_HandleTypeDef hsubghz;
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/**
  * @brief drive value used anytime radio is NOT in TX low power mode
  * @note override the default configuration of radio_driver.c
  */
#define SMPS_DRIVE_SETTING_DEFAULT  SMPS_DRV_40

/**
  * @brief drive value used anytime radio is in TX low power mode
  *        TX low power mode is the worst case because the PA sinks from SMPS
  *        while in high power mode, current is sunk directly from the battery
  * @note override the default configuration of radio_driver.c
  */
#define SMPS_DRIVE_SETTING_MAX      SMPS_DRV_60

/**
  * @brief Provides the frequency of the chip running on the radio and the frequency step
  * @remark These defines are used for computing the frequency divider to set the RF frequency
  * @note override the default configuration of radio_driver.c
  */
#define XTAL_FREQ                   ( 32000000UL )

/**
  * @brief in XO mode, set internal capacitor (from 0x00 to 0x2F starting 11.2pF with 0.47pF steps)
  * @note override the default configuration of radio_driver.c
  */
#define XTAL_DEFAULT_CAP_VALUE      ( 0x20UL )

/**
  * @brief voltage of vdd tcxo.
  * @note override the default configuration of radio_driver.c
  */
#define TCXO_CTRL_VOLTAGE           RBI_u8GetTcxoVoltage()

/**
  * @brief Radio maximum wakeup time (in ms)
  * @note override the default configuration of radio_driver.c
  */
#define RF_WAKEUP_TIME              ( 3UL )

/**
  * @brief DCDC is enabled
  * @remark this define is only used if the DCDC is present on the board
  * @note override the default configuration of radio_driver.c
  */
#define DCDC_ENABLE                 ( 1UL )

/* USER CODE BEGIN EC */
/* ---- Board notes: Tagskaap V3 RevA (frtag3), final boards -----------------
 *
 * These notes live inside this USER CODE block so that a CubeMX regeneration
 * would keep them. The #defines above them are OUTSIDE any guard. frtag3.ioc
 * does not enable the SUBGHZ_PHY middleware, so CubeMX does not touch this
 * file today; if that middleware is ever enabled, first set its TCXO voltage,
 * wake-up time and DCDC parameters to the values above, and protect
 * radio_board_if.c, which is hand-written: only its file header comment sits
 * inside a USER CODE guard, so a regeneration would replace its includes and
 * every RBI_* function body.
 *
 * TCXO part: ECS-TXO-25CSMV-320-AN-TR (both boards), 32 MHz clipped sine,
 * 2.5 x 2.0 mm. From the ECS-TXO-25CSMV datasheet: supply 1.7 V min /
 * 3.465 V max ("MultiVolt"), 2.5 mA max at 26.1-52 MHz, start-up 2 ms max,
 * output 0.8 Vp-p min into 10k // 10 pF, +/-0.2 ppm for a +/-5 % supply
 * change. Suffix "A" = +/-0.5 ppm over temperature, "N" = -40..+85 C. It
 * replaces the NDK END4263D-32M of the pre-production board (1.65 V min,
 * -30 C min). It is powered from the radio's own TCXO regulator output on
 * PB0 (VDD_TCXO) through a ferrite bead (L8) with 10 nF local decoupling.
 * HSE32 clocks the radio only; SYSCLK runs from HSI + PLL (SystemClock_Config
 * in hal_system.c), so a TCXO problem shows up as a radio fault, never as a
 * CPU clock fault.
 *
 * TCXO_CTRL_VOLTAGE is per role (RBI_u8GetTcxoVoltage, radio_board_if.c),
 * because the two boards run from different rails:
 *
 *   SECONDARY, VDD = 1.8 V -> 1.7 V. The only usable setting: 1.6 V is below
 *     the TCXO's 1.7 V minimum and 1.8 V equals the rail, while SX126x
 *     material describes the TCXO supply output as reaching at most about
 *     200 mV below the radio's own supply. 1.7 V therefore sits exactly on
 *     the TCXO's minimum with ~100 mV of regulator headroom - the same
 *     headroom frtag2 ran in the field at 1.8 V, but against a part whose
 *     minimum is 50 mV higher than the NDK's. This is the tightest margin on
 *     the board and is the first thing to measure on a new build.
 *   PRIMARY, VDD = 3.0 V -> 2.2 V. 500 mV above the TCXO's minimum and
 *     800 mV under the rail, so neither end is near a limit. The TCXO's
 *     current hardly changes with its supply and the regulator is linear from
 *     VDD either way, so the higher setting costs nothing measurable.
 *
 *   TODO: measure PB0 while the radio is in RX on a secondary to confirm the
 *   VDD_TCXO it actually delivers is >= 1.7 V, and confirm the regulator's
 *   dropout from the STM32WLE5 datasheet's TCXO regulator table. The 200 mV
 *   figure comes from SX126x material, not from ST.
 *
 * RF_WAKEUP_TIME = 3 ms: SUBGRF_Init passes RF_WAKEUP_TIME << 6 to
 * SUBGRF_SetTcxoMode as the TCXO start-up timeout, in 15.625 us steps, so
 * 3 << 6 = 192 steps = 3.0 ms. The ECS part's datasheet gives 2 ms MAXIMUM
 * start-up; the 1 ms inherited from the NDK board (ST's NUCLEO-WL55JC value)
 * would raise XOSC_START_ERR on a slow-starting unit, which this firmware
 * does not read, so it would fail silently as a radio fault. 3 ms is the 2 ms
 * maximum plus one step of margin for the regulator ramp at the low 1.7 V
 * setting. The "100 ms" remark beside that call in the vendor radio_driver.c
 * is stale vendor text and does not match the code.
 * The radio applies this timeout whenever it starts the TCXO on leaving
 * STDBY_RC. This firmware explicitly drops to STDBY_RC before every CAD and
 * RX and starts every TX from standby (LoraRadio_Driver.c), and SUBGRF_Init
 * itself re-runs on every wake from the cold (WarmStart = 0) radio sleep, so
 * the extra 2 ms is paid on essentially every radio operation. That is a few
 * percent of a 5 ms osDelay the CAD path already takes and is negligible
 * against LoRa air times; nothing in the mesh timing depends on it.
 * RadioGetWakeupTime() also returns this value, but only RadioInit uses it
 * (one delay at radio init).
 *
 * XTAL_DEFAULT_CAP_VALUE is unused on this board: with RBI_IsTCXO() == 1
 * (IS_TCXO_SUPPORTED in radio_board_if.h), SUBGRF_Init writes XTA_TRIM = 0
 * instead, and the trim capacitors only apply in XO (crystal) mode.
 *
 * DCDC_ENABLE = 1: the SMPS inductor (L7, 15 uH) is fitted on both final
 * boards, so the radio runs from the SMPS; see IS_DCDC_SUPPORTED in
 * radio_board_if.h. On the 1.8 V secondary this is the same operating point
 * frtag2 ran in the field.
 *
 * RF front end: no RF switch, RFO_LP only (direct tie) - see radio_board_if.c.
 */
/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
#ifndef CRITICAL_SECTION_BEGIN
/**
  * @brief macro used to enter the critical section
  */
#define CRITICAL_SECTION_BEGIN( )      UTILS_ENTER_CRITICAL_SECTION( )
#endif /* !CRITICAL_SECTION_BEGIN */
#ifndef CRITICAL_SECTION_END
/**
  * @brief macro used to exit the critical section
  */
#define CRITICAL_SECTION_END( )        UTILS_EXIT_CRITICAL_SECTION( )
#endif /* !CRITICAL_SECTION_END */

/* Function mapping */
/**
  * @brief SUBGHZ interface init to radio Middleware
  */
#define RADIO_INIT                              HAL_vSUBGHZ_Init

/**
  * @brief Delay interface to radio Middleware
  */
#define RADIO_DELAY_MS                          HAL_Delay

/**
  * @brief Memset utilities interface to radio Middleware
  */
#define RADIO_MEMSET8( dest, value, size )      UTIL_MEM_set_8( dest, value, size )

/**
  * @brief Memcpy utilities interface to radio Middleware
  */
#define RADIO_MEMCPY8( dest, src, size )        UTIL_MEM_cpy_8( dest, src, size )

/* USER CODE BEGIN EM */
/* Radio IRQ deferral (frtag2). By default radio.c runs RadioIrqProcess()
 * straight from the SUBGHZ ISR, and in there LoRaMac's TX-done handler arms
 * its RX-window timers - which on this project are FreeRTOS software timers
 * (Core/Src/timer_if.c) and must not be started from an ISR. These hooks are
 * ST's own seam for moving that work to a task: the ISR only latches the IRQ
 * and wakes the LoRaWAN session loop (fr_app/Worker/LoRaWan/LoRaWan.c), which
 * then calls RadioIrqProcess() from task context.
 *
 * The mesh never takes this path: LoraRadio_Driver.c registers its own DIO
 * callback with SUBGRF_Init() and never goes through radio.c. */
void LORAWAN_vRegisterRadioIrqProcess(void (*pfnProcess)(void));
void LORAWAN_vRadioIrqFromIsr(void);
#define RADIO_IRQ_PROCESS_INIT()   LORAWAN_vRegisterRadioIrqProcess(RadioIrqProcess)
#define RADIO_IRQ_PROCESS()        LORAWAN_vRadioIrqFromIsr()
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __RADIO_CONF_H__*/
