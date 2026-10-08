/*
 * hal_spi.h
 *
 * SPI drivers for:
 *   SPI1 — accelerometer (LIS2DH or compatible), PA1/PA6/PA7/PA11
 *          (SECONDARY only; no accelerometer is fitted on a V3 RevA primary)
 *   SPI2 — external NOR flash (AT25XE041B),     PA5/PA8/PA10/PA15
 *          (both roles)
 *
 * Each bus has its own init, called once at boot by init.c:
 * HAL_SPI_FLASH_vInit() on both roles, HAL_SPI_ACC_vInit() on a secondary
 * only. A bus whose init is never called stays off for the whole boot.
 *
 * STOP2 is handled per bus as well: HAL_SPI_vOnSleep() parks the pins of every
 * started bus on the way into STOP2, and each bus is restored lazily at its
 * next chip-select (HAL_SPI_*_vEnsureAwake, folded into HAL_SPI_*_vSelect), so
 * a 1 Hz wake that touches neither bus pays nothing. The chip selects
 * themselves (PA11, PA15) are plain GPIO outputs set high by HAL_GPIO_vInit and
 * held high through STOP2.
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
 * SPI1 — Accelerometer (secondary only)
 * PA1 SCK, PA6 MISO, PA7 MOSI, all AF5; PA11 CS is a plain GPIO output.
 * Mode 0, prescaler /128 from the 48 MHz APB2 clock = 375 kHz.
 *
 * HAL_SPI_ACC_vSpiReadPacket uses HAL_SPI_Receive(), which in 2-line master
 * mode does clock: it shifts the receive buffer's previous contents out on
 * MOSI while reading. The accelerometer ignores SDI once a read address is
 * in, so that is harmless here.
 * ----------------------------------------------------------------------- */
extern SPI_HandleTypeDef hAccSpi;

/* Start SPI1 and enrol it in the STOP2 park and lazy restore. Call once at
 * boot, on a SECONDARY only, before ACC_vInit(): every ACC register access goes
 * through HAL_SPI_ACC_vSelect, and on a bus that was never started its
 * transfers return HAL_BUSY and do nothing. */
void HAL_SPI_ACC_vInit(void);

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
 * (DS-AT25XE041B-062 Rev. I, Table 18). This board (Tagskaap V3 RevA) runs
 * VDD = 2.0 V, below the 2.3 V needed for the 33 MHz figure, so the 25 MHz
 * figure is the one that has to hold.
 *
 * PCLK1 is 48 MHz (SystemClock_Config in hal_system.c), so /4 gives 12 MHz -
 * comfortably inside fRDLF, and the highest standard prescaler at or below
 * the 16 MHz ceiling this bring-up is working to.
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

/* Full-duplex read: clocks out deterministic 0xFF dummies to clock in 'len'
 * bytes. HAL_SPI_Receive() would clock too (in 2-line master mode it falls
 * through to TransmitReceive with the receive buffer as the transmit buffer),
 * but it would put that buffer's stale contents on MOSI; this keeps MOSI at a
 * defined all-ones and stops at the first failed chunk. Returns HAL_OK only if
 * every chunk transferred without error/timeout. */
HAL_StatusTypeDef HAL_SPI_FLASH_vReadPacket(uint8_t *rx, uint16_t len);

#define HAL_SPI_FLASH_vSpiWritePacket(tx, len)      HAL_SPI_Transmit(&hFlashSpi, (uint8_t *)(tx), (len), SPI_TIMEOUT)
#define HAL_SPI_FLASH_vSpiReadPacket(rx, len)       HAL_SPI_FLASH_vReadPacket((rx), (len))
#define HAL_SPI_FLASH_vSpiReadWrite(tx, rx, len)    HAL_SPI_TransmitReceive(&hFlashSpi, (uint8_t *)(tx), (rx), (len), SPI_TIMEOUT)
#define HAL_SPI_FLASH_vSelect()                     (HAL_SPI_FLASH_vEnsureAwake(), \
                                                    HAL_GPIO_WritePin(BSP_FLASH_CS_PORT, BSP_FLASH_CS_PIN, GPIO_PIN_RESET))
#define HAL_SPI_FLASH_vDeselect()                   HAL_GPIO_WritePin(BSP_FLASH_CS_PORT, BSP_FLASH_CS_PIN, GPIO_PIN_SET)

/* Start SPI2 and enrol it in the STOP2 park and lazy restore. Call once at
 * boot, on both roles, before FLASH_vInit() and before anything can DBG_LOG to
 * the external flash. */
void HAL_SPI_FLASH_vInit(void);

/* -----------------------------------------------------------------------
 * STOP2
 * ----------------------------------------------------------------------- */

/* Park every started SPI bus for STOP2: SCK and MOSI driven low as GPIO
 * outputs (SCK first), MISO analog with no pull, and the bus flagged so its
 * next select re-initialises it. A bus that was never started is left alone,
 * so on a primary PA1/PA6/PA7 are never touched. Called by
 * HAL_SYSTEM_vEnterStop2 with interrupts masked, immediately before
 * HAL_GPIO_vOnSleep() gates the GPIO port clocks; register-only. See hal_spi.c
 * for why the pins are driven rather than left floating. */
void HAL_SPI_vOnSleep(void);

#endif /* INC_HAL_SPI_H_ */
