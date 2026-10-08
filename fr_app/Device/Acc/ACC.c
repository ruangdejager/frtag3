/*
 * ACC.c
 *
 * LIS2DH accelerometer driver.
 *
 * Initialises the device via a register configuration table and exposes
 * the public API for ID verification, FIFO sample count and data read.
 *
 * Secondary only on Tagskaap V3 RevA: a primary has no accelerometer fitted,
 * and init.c neither starts the SPI1 bus (HAL_SPI_ACC_vInit) nor calls into
 * this module there. Everything below assumes the bus has been started.
 * ACC_INT (PC13, the part's INT1) is routed on the board but unused and parked
 * analog; Movement polls the FIFO level instead (ACC_u8NumSamplesInFifo).
 */

#include <string.h>

#include "ACC.h"
#include "platform.h"
#include "FreeRTOS.h"
#include "task.h"

/* SPI frame bit positions */
#define SPI_READ_NWRITE_BP      7
#define SPI_MULTI_NSINGLE_BP    6

/* Nominal register configuration (startup sequence per AN3308) */
static const acc_reg_config_t AccRegConfig_P[] =
{
    { ACC_CTRL_REG0,        0b00010000 },   /* default */
    { ACC_TEMP_CFG_REG,     0b00000000 },   /* default */
    { ACC_CTRL_REG1,        0b00110111 },   /* normal mode (LPen=0; HR=0 in CTRL_REG4), 25 Hz ODR, XYZ enabled */
    { ACC_CTRL_REG2,        0b00000000 },   /* default */
    { ACC_CTRL_REG3,        0b00000010 },   /* FIFO overrun on INT1 */
    { ACC_CTRL_REG4,        0b00000000 },   /* default */
    { ACC_CTRL_REG5,        0b01000000 },   /* FIFO enabled */
    { ACC_CTRL_REG6,        0b00000000 },   /* default */
    { ACC_REFERENCE,        0b00000000 },   /* default */
    { ACC_FIFO_CTRL_REG,    0b10000000 },   /* stream mode, event trigger INT1 */
    { ACC_INT1_CFG,         0b00000000 },   /* default */
    { ACC_INT1_THS,         0b00000000 },   /* default */
    { ACC_INT1_DUR,         0b00000000 },   /* default */
    { ACC_INT2_CFG,         0b00000000 },   /* default */
    { ACC_INT2_THS,         0b00000000 },   /* default */
    { ACC_INT2_DUR,         0b00000000 },   /* default */
    { ACC_CLICK_CFG,        0b00000000 },   /* default */
    { ACC_CLICK_THS,        0b00000000 },   /* default */
    { ACC_TIME_LIMIT,       0b00000000 },   /* default */
    { ACC_TIME_LATENCY,     0b00000000 },   /* default */
    { ACC_TIME_WINDOW,      0b00000000 },   /* default */
    { ACC_ACT_THS,          0b00000000 },   /* default */
    { ACC_ACT_DUR,          0b00000000 },   /* default */
    { ACC_CTRL_REG5,        0b01000000 },   /* repeat per AN3308 startup */
};

/*
 * Known-bad register configuration captured from a field-returned unit
 * (May 2022) where CTRL_REG5 had FIFO disabled — kept for test/recovery use.
 */
static const acc_reg_config_t AccRegConfigErr_P[] =
{
    { ACC_CTRL_REG0,        0x10 },
    { ACC_TEMP_CFG_REG,     0x00 },
    { ACC_CTRL_REG1,        0x37 },
    { ACC_CTRL_REG2,        0x00 },
    { ACC_CTRL_REG3,        0x02 },
    { ACC_CTRL_REG4,        0x00 },
    { ACC_CTRL_REG5,        0x00 },   /* ERROR — should be 0x40 (FIFO enabled) */
    { ACC_CTRL_REG6,        0x00 },
    { ACC_REFERENCE,        0x00 },
    { ACC_FIFO_CTRL_REG,    0x80 },
    { ACC_INT1_CFG,         0x00 },
    { ACC_INT1_THS,         0x00 },
    { ACC_INT1_DUR,         0x00 },
    { ACC_INT2_CFG,         0x00 },
    { ACC_INT2_THS,         0x00 },
    { ACC_INT2_DUR,         0x00 },
    { ACC_CLICK_CFG,        0x00 },
    { ACC_CLICK_THS,        0x00 },
    { ACC_TIME_LIMIT,       0x00 },
    { ACC_TIME_LATENCY,     0x00 },
    { ACC_TIME_WINDOW,      0x00 },
    { ACC_ACT_THS,          0x00 },
    { ACC_ACT_DUR,          0x00 },
    { ACC_CTRL_REG5,        0x00 },   /* ERROR (repeat) */
};

/* Bus serialisation.
 *
 * Every SPI1 transaction below (select, transfer, deselect) runs with the
 * scheduler suspended. The accelerometer bus has more than one task-context
 * user on a secondary: the Movement task (osPriorityLow) reads the FIFO every
 * second from the moment MOVE_vInit() starts it, and the boot self-test
 * (SELFTEST_vRunAndReport, init task, osPriorityNormal) reads WHO_AM_I after
 * that. With nothing serialising them, the init task could preempt Movement
 * inside an open chip-select window, or inside the lazy SPI1 re-init that
 * HAL_SPI_ACC_vSelect() runs after a STOP2 wake, and corrupt both transfers or
 * the hAccSpi handle itself. (The flash bus has FLASH_vLock for the same
 * reason; this bus had nothing.)
 *
 * Scheduler suspension rather than a mutex: a transaction is at most a few
 * bytes at 375 kHz (~150 us), interrupts stay live (HAL_GetTick runs off
 * TIM16, so the HAL SPI timeouts still work), and it also makes the lazy
 * re-init atomic against the other task. Nothing inside the suspended region
 * may block: no osDelay, no logging - the retry delay in ACC_bDeviceIdOkEx
 * sits outside these helpers. */
#define ACC_BUS_vLock()     vTaskSuspendAll()
#define ACC_BUS_vUnlock()   ((void)xTaskResumeAll())

/* Private prototypes */
void    _vDeviceReadRegArray(uint8_t u8RegAddrStart, uint8_t *pau8Buf, uint8_t u8NumBytes);
void    _vDeviceWriteReg(uint8_t u8RegAddr, uint8_t u8RegData);
void    _vDeviceRegsConfig(const acc_reg_config_t *pRegConfig_P, uint8_t u8RegConfigSize);

/* --------------------------------------------------------------------------
 * ACC_vInit
 * -------------------------------------------------------------------------- */
void ACC_vInit(void)
{
    _vDeviceRegsConfig(AccRegConfig_P, sizeof(AccRegConfig_P) / sizeof(acc_reg_config_t));
}

/* --------------------------------------------------------------------------
 * _vDeviceReadRegArray — read N sequential registers via SPI
 * -------------------------------------------------------------------------- */
void _vDeviceReadRegArray(uint8_t u8RegAddrStart, uint8_t *pau8Buf, uint8_t u8NumBytes)
{
    uint8_t u8Data;

    u8RegAddrStart &= ACC_SPI_ADDR_MASK;
    u8Data = (1 << SPI_READ_NWRITE_BP) | (1 << SPI_MULTI_NSINGLE_BP) | u8RegAddrStart;

    ACC_BUS_vLock();
    HAL_SPI_ACC_vSelect();
    bool bOk = (HAL_SPI_ACC_vSpiWritePacket(&u8Data, 1) == HAL_OK);
    if (bOk)
        bOk = (HAL_SPI_ACC_vSpiReadPacket(pau8Buf, u8NumBytes) == HAL_OK);
    HAL_SPI_ACC_vDeselect();
    ACC_BUS_vUnlock();

    /* A failed transfer (HAL_BUSY on a bus that was never started, or a
     * timeout) leaves the caller's buffer untouched or half-written, and the
     * single-register read below used to hand back an uninitialised stack
     * byte as the "last byte" in the WHO_AM_I diagnostics. Return a defined
     * 0x00 instead: it can never equal ACC_WHO_AM_I_VALUE, and it reads as
     * "no response" rather than 0xFF, which ACC_bDeviceIdOkEx reports as an
     * idle MISO. */
    if (!bOk)
        memset(pau8Buf, 0x00, u8NumBytes);
}

/* --------------------------------------------------------------------------
 * _u8DeviceReadReg — read one register via SPI
 * -------------------------------------------------------------------------- */
uint8_t _u8DeviceReadReg(uint8_t u8RegAddr)
{
    uint8_t u8Data = 0x00U;
    _vDeviceReadRegArray(u8RegAddr, &u8Data, 1);
    return u8Data;
}

/* --------------------------------------------------------------------------
 * _vDeviceWriteReg — write one register via SPI
 * -------------------------------------------------------------------------- */
void _vDeviceWriteReg(uint8_t u8RegAddr, uint8_t u8RegData)
{
    uint8_t u8Data;

    u8RegAddr &= ACC_SPI_ADDR_MASK;
    u8Data = (0 << SPI_READ_NWRITE_BP) | (0 << SPI_MULTI_NSINGLE_BP) | u8RegAddr;
    ACC_BUS_vLock();
    HAL_SPI_ACC_vSelect();
    HAL_SPI_ACC_vSpiWritePacket(&u8Data, 1);
    HAL_SPI_ACC_vSpiWritePacket(&u8RegData, 1);
    HAL_SPI_ACC_vDeselect();
    ACC_BUS_vUnlock();
}

/* --------------------------------------------------------------------------
 * _vDeviceRegsConfig — write an arbitrary register configuration table
 * -------------------------------------------------------------------------- */
void _vDeviceRegsConfig(const acc_reg_config_t *pRegConfig_P, uint8_t u8RegConfigSize)
{
    for (uint8_t i = 0; i < u8RegConfigSize; i++)
    {
        _vDeviceWriteReg(pRegConfig_P[i].u8Name, pRegConfig_P[i].u8Value);
    }
}

/* --------------------------------------------------------------------------
 * ACC_u8GetDeviceId
 * -------------------------------------------------------------------------- */
uint8_t ACC_u8GetDeviceId(void)
{
    return _u8DeviceReadReg(ACC_WHO_AM_I);
}

/* --------------------------------------------------------------------------
 * ACC_bDeviceIdOk — returns true when WHO_AM_I matches expected value
 * -------------------------------------------------------------------------- */
bool ACC_bDeviceIdOk(void)
{
    return ACC_bDeviceIdOkEx(NULL, NULL);
}

bool ACC_bDeviceIdOkEx(uint8_t *pu8Id, uint8_t *pu8Attempts)
{
    uint8_t u8Id  = 0U;
    bool    bOk   = false;
    uint8_t u8Try = 0U;

    /* Retried, not single-shot — same reasoning as FLASH_bVerifyDeviceEx.
     * A lone WHO_AM_I read has been seen to come back wrong from a perfectly
     * healthy part. On frtag2 (V2) hardware that was blamed on supply-rail sag
     * from a concurrent radio/GPS current spike, but that mechanism was never
     * tested and TX was not shown to sag the rail; the likelier cause is noise
     * on the very long MISO/MOSI runs over a non-continuous ground plane (see
     * OTA_LORA_CHUNK_GAP_MS in Fota_Config.h). V3 RevA is a new layout and has
     * not been characterised for it either way. The first read after
     * HAL_SPI_ACC_vSelect() re-inits a parked bus was considered the most
     * exposed. Whatever the cause, calling one bad read "ACC FAIL" is the same
     * false negative the flash test used to produce, so the read is retried.
     *
     * The raw byte goes back to the caller so a real failure is diagnosable:
     *   0x00 -> no clock/data (AF, wiring, bus not restored)
     *   0xFF -> MISO idle / CS never asserted
     *   else -> CPOL/CPHA, timing, or genuinely the wrong part */
    for (u8Try = 1U; u8Try <= ACC_WHO_AM_I_READ_ATTEMPTS; u8Try++)
    {
        u8Id = ACC_u8GetDeviceId();
        if (u8Id == ACC_WHO_AM_I_VALUE) { bOk = true; break; }

        osDelay(2);   /* short gap before re-reading */
    }

    if (pu8Id != NULL)
        *pu8Id = u8Id;
    if (pu8Attempts != NULL)
        *pu8Attempts = (u8Try > ACC_WHO_AM_I_READ_ATTEMPTS)
                     ? ACC_WHO_AM_I_READ_ATTEMPTS : u8Try;

    return bOk;
}

/* --------------------------------------------------------------------------
 * ACC_u8NumSamplesInFifo
 * -------------------------------------------------------------------------- */
uint8_t ACC_u8NumSamplesInFifo(void)
{
    return (_u8DeviceReadReg(ACC_FIFO_SRC_REG) & 0x1F);
}

/* --------------------------------------------------------------------------
 * ACC_vGetAccSample — read one XYZ sample (6 bytes) from the FIFO
 * -------------------------------------------------------------------------- */
void ACC_vGetAccSample(acc_t *pAcc)
{
    _vDeviceReadRegArray(0x28, (uint8_t *)pAcc, 6);
}

/* --------------------------------------------------------------------------
 * ACC_vTestRegsConfigError — load the known-bad config for recovery testing
 * -------------------------------------------------------------------------- */
void ACC_vTestRegsConfigError(void)
{
    _vDeviceRegsConfig(AccRegConfigErr_P, sizeof(AccRegConfigErr_P) / sizeof(acc_reg_config_t));
}
