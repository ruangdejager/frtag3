/*
 * hal_spi.h
 *
 * SPI drivers for:
 *   SPI1 — accelerometer (LIS2DH or compatible), PA1/PA6/PA7/PA11
 *   SPI2 — external NOR flash (AT25XE041B),     PA5/PA8/PA10/PA15
 *
 * Both peripherals are initialised by a single HAL_SPI_vInit() call.
 */

#ifndef INC_HAL_SPI_H_
#define INC_HAL_SPI_H_

#include <stdint.h>
#include "stm32wle5xx.h"
#include "stm32wlxx_hal.h"
#include "hal_bsp.h"
#include "Acc_Config.h"

#define SPI_TIMEOUT 1000

/* -----------------------------------------------------------------------
 * SPI1 — Accelerometer
 * ----------------------------------------------------------------------- */
extern SPI_HandleTypeDef hAccSpi;

/* Restore the ACC bus if a STOP2 sleep parked it (no-op otherwise). Folded into
 * the chip-select so every transaction is guaranteed an awake bus, while idle
 * 1 Hz wakes that never select the ACC skip the re-init entirely. */
void HAL_SPI_ACC_vEnsureAwake(void);

#define HAL_SPI_ACC_vSpiWritePacket(txData, len)       HAL_SPI_Transmit(&hAccSpi, txData, len, SPI_TIMEOUT)
#define HAL_SPI_ACC_vSpiReadPacket(rxData, len)        HAL_SPI_Receive(&hAccSpi, rxData, len, SPI_TIMEOUT)
#define HAL_SPI_ACC_vSpiReadWrite(txData, rxData, len) HAL_SPI_TransmitReceive(&hAccSpi, txData, rxData, len, SPI_TIMEOUT)
#define HAL_SPI_ACC_vSelect()                          (HAL_SPI_ACC_vEnsureAwake(), \
                                                        HAL_GPIO_WritePin(BSP_ACC_CS_PORT, BSP_ACC_CS_PIN, GPIO_PIN_RESET))
#define HAL_SPI_ACC_vDeselect()                        HAL_GPIO_WritePin(BSP_ACC_CS_PORT, BSP_ACC_CS_PIN, GPIO_PIN_SET)

/* -----------------------------------------------------------------------
 * SPI2 — External NOR Flash (AT25XE041B, 512 KB)
 * PA5 MISO = AF3, PA8 SCK = AF5, PA10 MOSI = AF5  (mixed AFs, set in MspInit)
 * ----------------------------------------------------------------------- */
extern SPI_HandleTypeDef hFlashSpi;

/* SCK rate for the flash bus, in one place.
 *
 * The binding limit is NOT the 85 MHz fCLK headline in the AT25XE041B
 * datasheet: that figure is for all operations "including 0Bh opcode". This
 * driver reads with 03h (Read Array, low frequency), whose ceiling is
 * fRDLF = 25 MHz at 1.65-3.6 V and 33 MHz only at 2.3-3.6 V
 * (DS-AT25XE041B-062 Rev. I, Table 18). This board runs at 1.8 V and a
 * sibling board at 3.0 V, so the 25 MHz figure is the one that has to hold.
 *
 * PCLK1 is 48 MHz (SystemClock_Config in hal_system.c), so /4 gives 12 MHz -
 * comfortably inside fRDLF on both boards, and the highest standard prescaler
 * at or below the 16 MHz ceiling this bring-up is working to.
 *
 * It was 750 kHz before (prescaler /64), behind a comment claiming "~0.5 MHz
 * at 32 MHz PCLK" that had been wrong since the clock tree moved to 48 MHz.
 * Raising it shortens a whole-image OTA XOR pass from ~1.5 s to ~95 ms, which
 * is a real change to the OTA verify timing - see the field evidence recorded
 * against OTA_VERIFY_TX_SETTLE_MS in Fota_Config.h, which was measured at the
 * old rate and is not directly comparable to logs taken at this one. */
#define FLASH_SPI_PRESCALER     SPI_BAUDRATEPRESCALER_4
#define FLASH_SPI_CLOCK_HZ      12000000UL

#define FLASH_SPI               SPI2
#define FLASH_SPI_CLK_ENABLE()  __HAL_RCC_SPI2_CLK_ENABLE()
#define FLASH_SPI_CLK_DISABLE() __HAL_RCC_SPI2_CLK_DISABLE()
#define FLASH_PORT_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

/* Restore the flash bus if a STOP2 sleep parked it (no-op otherwise). Folded
 * into the chip-select so a logging flash write after a wake always finds an
 * awake bus, while idle wakes that don't touch the flash skip the re-init. */
void HAL_SPI_FLASH_vEnsureAwake(void);

/* Full-duplex read: clocks out 0xFF dummies to clock in 'len' bytes. Required
 * because HAL_SPI_Receive() does NOT generate clock in 2-line master mode
 * (it waits for RXNE that never arrives). Returns HAL_OK only if every
 * chunk transferred without error/timeout. */
HAL_StatusTypeDef HAL_SPI_FLASH_vReadPacket(uint8_t *rx, uint16_t len);

#define HAL_SPI_FLASH_vSpiWritePacket(tx, len)      HAL_SPI_Transmit(&hFlashSpi, (uint8_t *)(tx), (len), SPI_TIMEOUT)
#define HAL_SPI_FLASH_vSpiReadPacket(rx, len)       HAL_SPI_FLASH_vReadPacket((rx), (len))
#define HAL_SPI_FLASH_vSpiReadWrite(tx, rx, len)    HAL_SPI_TransmitReceive(&hFlashSpi, (uint8_t *)(tx), (rx), (len), SPI_TIMEOUT)
#define HAL_SPI_FLASH_vSelect()                     (HAL_SPI_FLASH_vEnsureAwake(), \
                                                    HAL_GPIO_WritePin(BSP_FLASH_CS_PORT, BSP_FLASH_CS_PIN, GPIO_PIN_RESET))
#define HAL_SPI_FLASH_vDeselect()                   HAL_GPIO_WritePin(BSP_FLASH_CS_PORT, BSP_FLASH_CS_PIN, GPIO_PIN_SET)

/* -----------------------------------------------------------------------
 * Shared init — call once at boot and once after every STOP2 wake.
 * Initialises both SPI1 (ACC) and SPI2 (flash) in a single call.
 * ----------------------------------------------------------------------- */
void HAL_SPI_vInit(void);

/* Flag both SPI buses as torn down by STOP2; each is re-initialised lazily on
 * its next chip-select. Call from the wake path instead of HAL_SPI_vInit() so
 * idle 1 Hz wakes don't re-init buses they never use. */
void HAL_SPI_vMarkParked(void);

#endif /* INC_HAL_SPI_H_ */
