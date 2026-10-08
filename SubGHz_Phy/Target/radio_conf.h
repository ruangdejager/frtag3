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
#define TCXO_CTRL_VOLTAGE           TCXO_CTRL_1_7V

/**
  * @brief Radio maximum wakeup time (in ms)
  * @note override the default configuration of radio_driver.c
  */
#define RF_WAKEUP_TIME              ( 1UL )

/**
  * @brief DCDC is enabled
  * @remark this define is only used if the DCDC is present on the board
  * @note override the default configuration of radio_driver.c
  */
#define DCDC_ENABLE                 ( 1UL )

/* USER CODE BEGIN EC */
/* ---- Board notes: Tagskaap V3 RevA (frtag3), VDD = VDDA = 2.0 V -----------
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
 * TCXO part: NDK NT2016SF series, NDK spec END4263D, ordered as END4263D-32M,
 * 32 MHz. From NDK spec sheet ETS11B-07494: supply 1.65 V min / 3.0 V typ /
 * 3.65 V max, 2.0 mA max; clipped sine output, DC-coupled, 0.8 Vp-p minimum
 * (specified at Vcc = 3.0 V); +/-2.5 ppm over the -30..+85 C operating range
 * (note that the TCXO is rated only to -30 C while the MCU is rated to
 * -40 C); +/-0.1 ppm for a +/-5 % change in Vcc. It is the same NT2016SF
 * series as the END5875A part ST fits on the NUCLEO-WL55JC. The TCXO is
 * powered from the radio's own TCXO regulator output on PB0 (VDD_TCXO),
 * which is fed from the 2.0 V rail. HSE32 clocks the radio only; SYSCLK runs
 * from HSI + PLL (SystemClock_Config in hal_system.c), so a TCXO problem
 * shows up as a radio fault, never as a CPU clock fault.
 *
 * TCXO_CTRL_VOLTAGE = 1.7 V, deliberately not 1.8 V:
 *   - It is 50 mV above the TCXO's 1.65 V minimum supply, before any
 *     regulator tolerance is counted.
 *   - It is 300 mV below the 2.0 V rail. SX126x documentation describes the
 *     TCXO supply output as reaching at most about 200 mV below the radio's
 *     own supply, so 1.8 V would sit exactly on that limit with nothing left
 *     for VDD tolerance. On frtag2's 1.8 V rail the same 1.7 V setting left
 *     only 100 mV of headroom. The 200 mV figure comes from SX126x material;
 *     the STM32WLE5 datasheet's own TCXO regulator table has not been checked.
 *   - It is ST's own NUCLEO-WL55JC default for this TCXO family, and the
 *     value used in ST's CubeWL SubGHz_Phy examples.
 *   TODO: confirm the VDD_TCXO accuracy and dropout from the STM32WLE5
 *   datasheet's TCXO regulator table, or measure PB0 while the radio is in RX,
 *   to prove that at least 1.65 V reaches the TCXO on this 2.0 V rail. Do not
 *   move to 1.8 V without that measurement.
 *
 * RF_WAKEUP_TIME = 1 ms: SUBGRF_Init passes RF_WAKEUP_TIME << 6 to
 * SUBGRF_SetTcxoMode as the TCXO start-up timeout, in 15.625 us steps, so
 * 1 << 6 = 64 steps = 1.0 ms. The "100 ms" remark beside that call in the
 * vendor radio_driver.c is stale vendor text and does not match the code.
 * The radio applies this timeout whenever it starts the TCXO on leaving
 * STDBY_RC. This firmware explicitly drops to STDBY_RC before every CAD and
 * RX and starts every TX from standby (LoraRadio_Driver.c), so the timeout is
 * in play on essentially every radio operation; SUBGRF_Init itself also
 * re-runs on every wake from the cold (WarmStart = 0) radio sleep. ETS11B-07494 gives NO start-up time
 * for the END4263D, so 1 ms is not taken from the part's datasheet: it is the
 * value ST uses on the NUCLEO-WL55JC for the same NT2016SF series. A TCXO
 * that is not ready in time raises XOSC_START_ERR
 * (SUBGRF_GetDeviceErrors().Fields.XoscStart), which this firmware does not
 * currently read, so a late TCXO would fail silently as a radio fault.
 *   TODO: confirm the END4263D start-up time with NDK, or measure it, or
 *   read XoscStart once after the first radio init and log it.
 *
 * XTAL_DEFAULT_CAP_VALUE is unused on this board: with RBI_IsTCXO() == 1
 * (IS_TCXO_SUPPORTED in radio_board_if.h), SUBGRF_Init writes XTA_TRIM = 0
 * instead, and the trim capacitors only apply in XO (crystal) mode.
 *
 * DCDC_ENABLE = 1: the SMPS inductor is fitted on V3 RevA, as it was on
 * frtag2, so the radio runs from the SMPS; see IS_DCDC_SUPPORTED in
 * radio_board_if.h.
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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __RADIO_CONF_H__*/
