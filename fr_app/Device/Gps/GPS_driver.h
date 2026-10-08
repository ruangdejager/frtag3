/*
 * GPS_driver.h
 *
 * GNSS module board-level driver macros for frtag3 (Tagskaap V3 RevA). The
 * GNSS receiver is fitted on secondaries only.
 *
 * Power is a single GNSS_ON pin (PA0, BSP_GNSS_ON): it drives the enable of
 * the LDO that supplies GNSS_VCC, active high. On a primary the same pin is
 * the fr9 wake line instead (see Farmranger.h), and these macros are never
 * used. The module's 1.8 V I/O sits on PB6/PB7 (USART1). The power-up order
 * (rail, settle, then pins) and the power-down order (pins, then rail) live
 * in one place in GPS.c, GPS_vHwPowerOn() / GPS_vHwPowerOff(); call those
 * rather than sequencing these macros by hand.
 */

#ifndef DEVICE_GPS_GPS_DRIVER_H_
#define DEVICE_GPS_GPS_DRIVER_H_

#include "hal_bsp.h"
#include "hal_gpio.h"
#include "hal_uart.h"

#define GPS_DRIVER_vInitGPS(drv)        HAL_UART_vSetup(drv, GPS_UART, FLOWCONTROL_NONE)

#define GPS_DRIVER_vEnableUart(drv)     HAL_UART_vEnable(drv)
#define GPS_DRIVER_vDisableUart(drv)    HAL_UART_vDisable(drv)
#define GPS_DRIVER_vClearBuffer(drv)    HAL_UART_vClearBuffer(drv)

#define GPS_DRIVER_vPowerEnHigh() \
    HAL_GPIO_WritePin(BSP_GNSS_ON_PORT, BSP_GNSS_ON_PIN, GPIO_PIN_SET)

#define GPS_DRIVER_bPowerEnGet() \
    HAL_GPIO_ReadPin(BSP_GNSS_ON_PORT, BSP_GNSS_ON_PIN)

#define GPS_DRIVER_vPowerEnLow() \
    HAL_GPIO_WritePin(BSP_GNSS_ON_PORT, BSP_GNSS_ON_PIN, GPIO_PIN_RESET)

#endif /* DEVICE_GPS_GPS_DRIVER_H_ */
