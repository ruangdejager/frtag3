/*
 * LoraRadio_Driver.h
 *
 * Low-level LoRa radio driver — thin wrapper around the STM32WL SUBGHZ HAL
 * and the SX126x radio driver library.
 */

#ifndef DEVICE_LORARADIO_DRIVERS_LORARADIO_DRIVER_H_
#define DEVICE_LORARADIO_DRIVERS_LORARADIO_DRIVER_H_

#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>

#include "hal_subghz.h"
#include "LoraRadio.h"
#include "radio_driver.h"

void     LORARADIO_DRIVER_vInit(void);

/* The MCU's 96-bit unique ID, which the radio id is derived from. */
#define  LORARADIO_DRIVER_vGetUid96(pau32Uid)   HAL_SUBGHZ_vGetUid96(pau32Uid)
bool     LORARADIO_DRIVER_bTransmitPayload(uint8_t *payload, uint8_t payload_length);
bool     LORARADIO_DRIVER_bTransmitPayloadEx(uint8_t *payload, uint8_t payload_length,
                                             uint16_t u16Preamble, bool bLog);
bool     LORARADIO_DRIVER_bReceivePayload(LoraRadio_Packet_t *rxParams);
void     LORARADIO_DRIVER_vEnterRxMode(uint32_t u32RxTimeout);
void     LORARADIO_DRIVER_vEnterRxWindow(uint32_t u32WindowMs);
uint32_t LORARADIO_DRIVER_u32GetRandomNumber(uint32_t max_value);
void     LORARADIO_DRIVER_vEnterCAD(void);
void     LORARADIO_DRIVER_vEnterDeepSleep(void);
void     LORARADIO_DRIVER_vWakeUp(void);

#endif /* DEVICE_LORARADIO_DRIVERS_LORARADIO_DRIVER_H_ */
