/*
 * hal_spi.c
 *
 * SPI drivers for:
 *   SPI1 — accelerometer (LIS2DH or compatible), PA1/PA6/PA7/PA11.
 *          Fitted on a SECONDARY only: a Tagskaap V3 RevA primary has no
 *          accelerometer, so SPI1 is never started there.
 *   SPI2 — external NOR flash (AT25XE041B),     PA5/PA8/PA10/PA15.
 *          Fitted on both roles.
 *
 * Each bus has its own init. HAL_SPI_FLASH_vInit() runs on both roles, once at
 * boot and before FLASH_vInit(); HAL_SPI_ACC_vInit() runs on a secondary only,
 * before ACC_vInit(). This file stays role-agnostic on purpose: init.c already
 * branches on the role and decides which buses exist, and all this file knows
 * is whether a bus was started. A bus that was never started is never touched
 * again - it is not parked at STOP2 entry and not restored at a later select -
 * so on a primary this firmware never configures or clocks SPI1, and
 * PA1/PA6/PA7 keep their reset analog state for the life of the boot.
 *
 * STOP2: HAL_SPI_vOnSleep() (called by HAL_SYSTEM_vEnterStop2 just before
 * HAL_GPIO_vOnSleep) drives SCK and MOSI of every started bus low as plain GPIO
 * outputs, parks its MISO analog with no pull, and flags the bus parked. The
 * first select after a wake re-initialises that bus (HAL_SPI_*_vEnsureAwake),
 * which puts its pins back on their alternate function. The two chip selects
 * (PA11, PA15) are plain GPIO outputs owned by HAL_GPIO_vInit and stay driven
 * high through STOP2; nothing in this file initialises them.
 */

#include "hal_spi.h"
#include "hal_bsp.h"
#include "hal_gpio.h"              /* HAL_GPIO_vInitOutput, HAL_GPIO_vInitAnalogNoPull */
#include "stm32wle5xx.h"

#include <stdbool.h>
#include <string.h>

SPI_HandleTypeDef hAccSpi;
SPI_HandleTypeDef hFlashSpi;

/* Per-bus "this bus exists" latches. Set the first time init.c starts the bus
 * (HAL_SPI_FLASH_vInit / HAL_SPI_ACC_vInit) and never cleared. They are the only
 * thing that decides whether STOP2 entry touches a bus's pins at all, which is
 * what keeps a primary's SPI1 pins (no accelerometer fitted) out of the sleep
 * path entirely. */
static volatile bool bAccSpiInUse;
static volatile bool bFlashSpiInUse;

/* Set by HAL_SPI_vOnSleep at the moment a bus's pins leave their alternate
 * function, cleared when that bus is re-initialised. Each bus must be restored
 * before its next transaction - but only the bus actually used. Deferring this
 * to the select chokepoint (HAL_SPI_*_vEnsureAwake) means an idle 1 Hz wake
 * re-inits neither: the flash bus only wakes when something logs, the ACC bus
 * only on a movement read.
 *
 * Because only a bus that is in use is ever parked, a bus that was never
 * started can never be "restored" either. The old HAL_SPI_vMarkParked() armed
 * both flags on every wake on both roles, so any stray HAL_SPI_ACC_vSelect()
 * on a primary would have run MspInit, switched PA1/PA6/PA7 to AF5 and enabled
 * the SPI1 clock behind init.c's back. Now such a select finds the flag false,
 * toggles PA11 (which has nothing on it) and its transfers return HAL_BUSY
 * against a handle that was never initialised.
 *
 * volatile: written on the STOP2 entry path with interrupts masked and read
 * from task context. */
static volatile bool bAccSpiParked;
static volatile bool bFlashSpiParked;

static void HAL_SPI_vInitAccHw(void);
static void HAL_SPI_vInitFlashHw(void);
static void HAL_SPI_vInitAfPin(GPIO_TypeDef *pPort, uint32_t u32Pin, uint32_t u32Pull,
                               uint32_t u32Speed, uint32_t u32Alternate);

/* --------------------------------------------------------------------------
 * HAL_SPI_FLASH_vReadPacket
 * Reads 'len' bytes from the flash by clocking out 0xFF dummy bytes via
 * full-duplex TransmitReceive. Chunked so the dummy buffer stays small.
 *
 * Why not HAL_SPI_Receive(): in 2-line master mode the vendor HAL_SPI_Receive
 * does generate the clock - it falls through to
 * HAL_SPI_TransmitReceive(hspi, pData, pData, ...) - but what it shifts out on
 * MOSI is whatever the receive buffer happened to hold beforehand. The flash
 * ignores SI once a read command and its address are in, so that would work,
 * but a defined all-ones MOSI costs nothing and removes "what did we just send
 * it" from every bus capture. (An earlier version of this comment claimed
 * HAL_SPI_Receive does not clock at all; that is not true of this vendor HAL.)
 *
 * Returns HAL_OK only if every chunk transferred cleanly; on the first
 * non-OK chunk the transfer is aborted immediately so the caller can
 * deselect without clocking further (possibly stale) bytes into rx.
 * -------------------------------------------------------------------------- */
HAL_StatusTypeDef HAL_SPI_FLASH_vReadPacket(uint8_t *rx, uint16_t len)
{
    uint8_t  au8Dummy[64];
    memset(au8Dummy, 0xFF, sizeof(au8Dummy));

    uint16_t u16Off = 0U;
    while (u16Off < len)
    {
        uint16_t u16N = (uint16_t)((len - u16Off) > sizeof(au8Dummy)
                                   ? sizeof(au8Dummy) : (len - u16Off));
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hFlashSpi, au8Dummy, rx + u16Off, u16N, SPI_TIMEOUT);
        if (status != HAL_OK)
            return status;
        u16Off = (uint16_t)(u16Off + u16N);
    }
    return HAL_OK;
}

/* --------------------------------------------------------------------------
 * HAL_SPI_vInitAccHw / HAL_SPI_vInitFlashHw
 * Bring up one SPI peripheral:
 *   SPI1 — accelerometer (PA1 SCK / PA6 MISO / PA7 MOSI, AF5), mode 0, /128
 *          from the 48 MHz APB2 clock = 375 kHz
 *   SPI2 — external NOR flash (PA5 MISO AF3 / PA8 SCK AF5 / PA10 MOSI AF5),
 *          mode 0, FLASH_SPI_PRESCALER (12 MHz, see hal_spi.h)
 *
 * Used for the boot-time start (HAL_SPI_ACC_vInit / HAL_SPI_FLASH_vInit) and
 * for the lazy restore after STOP2 (HAL_SPI_*_vEnsureAwake). The handle state is
 * forced to RESET first so HAL_SPI_Init() re-runs HAL_SPI_MspInit() on every
 * call: MspInit is what puts the SPI pins back on their alternate function.
 * That matters after STOP2 because HAL_SPI_vOnSleep() has switched them to
 * plain GPIO (SCK/MOSI driven low, MISO analog); without re-running MspInit the
 * pins would stay GPIO and the bus would be dead - and not obviously so: the
 * SPI still completes every transfer HAL_OK, but MISO is analog, so every byte
 * shifts in as 00h. A JEDEC probe reads 00 00 00, and a status read returns
 * SR1 = 00h, which looks READY rather than busy - the dangerous direction,
 * because FLASH_bWaitReady would let the next command onto a device that is
 * still mid-program. Plain HAL_SPI_Init() skips MspInit once the handle has left
 * RESET.
 *
 * Both buses are mode 0 (CPOL low, CPHA first edge). HAL_SPI_vOnSleep drives
 * SCK low on that basis, and the SCK pull-down in HAL_SPI_MspInit exists for
 * it; changing either bus's CPOL means changing both of those as well.
 * -------------------------------------------------------------------------- */
static void HAL_SPI_vInitAccHw(void)
{
    /* Force a full re-init (incl. MspInit -> pin restore) on every call. */
    hAccSpi.State                  = HAL_SPI_STATE_RESET;
    hAccSpi.Instance               = ACC_SPI;
    hAccSpi.Init.Mode              = SPI_MODE_MASTER;
    hAccSpi.Init.Direction         = SPI_DIRECTION_2LINES;
    hAccSpi.Init.DataSize          = SPI_DATASIZE_8BIT;
    hAccSpi.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hAccSpi.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hAccSpi.Init.NSS               = SPI_NSS_SOFT;
    hAccSpi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;   /* 375 kHz */
    hAccSpi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hAccSpi.Init.TIMode            = SPI_TIMODE_DISABLE;
    hAccSpi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hAccSpi.Init.CRCPolynomial     = 7;
    hAccSpi.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hAccSpi.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
    HAL_SPI_Init(&hAccSpi);
    bAccSpiParked = false;
}

static void HAL_SPI_vInitFlashHw(void)
{
    hFlashSpi.State                  = HAL_SPI_STATE_RESET;
    hFlashSpi.Instance               = FLASH_SPI;
    hFlashSpi.Init.Mode              = SPI_MODE_MASTER;
    hFlashSpi.Init.Direction         = SPI_DIRECTION_2LINES;
    hFlashSpi.Init.DataSize          = SPI_DATASIZE_8BIT;
    hFlashSpi.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hFlashSpi.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hFlashSpi.Init.NSS               = SPI_NSS_SOFT;
    hFlashSpi.Init.BaudRatePrescaler = FLASH_SPI_PRESCALER;       /* 12 MHz, see hal_spi.h */
    hFlashSpi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hFlashSpi.Init.TIMode            = SPI_TIMODE_DISABLE;
    hFlashSpi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hFlashSpi.Init.CRCPolynomial     = 7;
    hFlashSpi.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hFlashSpi.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
    HAL_SPI_Init(&hFlashSpi);
    bFlashSpiParked = false;
}

/* --------------------------------------------------------------------------
 * HAL_SPI_FLASH_vInit / HAL_SPI_ACC_vInit — start one bus, once, at boot.
 *
 * Marking the bus in use is what enrols it in the STOP2 park and the lazy
 * restore; a bus whose init is never called stays off for the whole boot.
 * Called from the init task before systemReadyForSleep is set, so no STOP2
 * entry can fall between the latch and the hardware init.
 * -------------------------------------------------------------------------- */
void HAL_SPI_FLASH_vInit(void)
{
    bFlashSpiInUse = true;
    HAL_SPI_vInitFlashHw();
}

void HAL_SPI_ACC_vInit(void)
{
    bAccSpiInUse = true;
    HAL_SPI_vInitAccHw();
}

/* --------------------------------------------------------------------------
 * HAL_SPI_vOnSleep — park every started SPI bus for STOP2.
 *
 * Called from HAL_SYSTEM_vEnterStop2 with interrupts masked (PRIMASK set in
 * vPortSuppressTicksAndSleep), immediately before HAL_GPIO_vOnSleep() gates the
 * GPIO port clocks - so the port clocks are still on here, which the GPIO
 * register writes below need. Register-only: no RTOS calls, no logging, no
 * delays.
 *
 * For each bus that init.c started:
 *   SCK   driven LOW as a GPIO output, the mode-0 (CPOL low) idle level
 *   MOSI  driven LOW as a GPIO output
 *   MISO  analog with no pull (the line belongs to the device, which drives it
 *         or leaves it Hi-Z, so there is nothing for the MCU to drive; analog
 *         mode also disconnects the MCU's own input buffer)
 * then the bus is flagged parked so its next select restores it.
 *
 * Why drive SCK/MOSI rather than leave them analog, as this path used to: the
 * accelerometer's chip select (PA11) sits HIGH through STOP2, and with CS high
 * the LIS2DH's I2C interface is live - SPC is SCL, SDI is SDA, SDO is the SA0
 * input with its internal pull-up. Analog-parked MCU pins leave SCL and SDA
 * floating into live CMOS inputs. frtag2's measurements on its primary's rail
 * (the accelerometer was fitted there but unused) show what that costs: the
 * part in its unconfigured power-on state with undefined I/O drew ~115 uA;
 * configured and running at 25 Hz with an undrained FIFO ~45 uA; and full
 * power-down (CTRL_REG1 = 0) was WORSE than the configured ~45 uA state (no
 * figure was recorded), because the floating SPI
 * inputs into the now-idle I/O crowbarred. Floating inputs into the part cost
 * more than an active part does. The flash's SI and SCK are CMOS inputs too,
 * and get the same treatment for the same reason. (frtag2's answer on its
 * primary was a dedicated low-power register profile for the unused part. A
 * V3 RevA primary has no accelerometer, so that profile is gone; on a
 * secondary, where the part is fitted and running, this park is what keeps
 * its inputs defined.)
 *
 * SCK before MOSI: with CS high an SDA (MOSI) falling edge while SCL (SCK) is
 * high would be an I2C START condition to the accelerometer. Taking SCK low
 * first means MOSI can never form a START or STOP whatever level it moves
 * from. LOW rather than HIGH for MOSI because an I2C acknowledge can only pull
 * SDA low, so a low drive can never contend with the part.
 *
 * HAL_GPIO_vInitOutput writes the output level before switching the pin to
 * output, so neither pin glitches when it leaves AF. SCK is in practice already
 * low at this point (after the first transfer SPE stays set, so the SPI is
 * driving SCK at its idle level); MOSI can sit at either level, holding the
 * last bit shifted out, which is why the SCK-first order above matters.
 * MISO parking relies on
 * HAL_GPIO_vInitAnalogNoPull clearing the pull bits, which the vendor
 * HAL_GPIO_Init leaves untouched in analog mode - without that, PA5's pull-up
 * bits would stay set in PUPDR (a reserved analog encoding; the pull itself is
 * disabled in hardware while analog, but comes back live on the next switch
 * out of analog).
 *
 * This must never run with a CS window open. That holds today: STOP2 is only
 * entered from the idle task, which runs only when every task is blocked, and
 * neither the ACC nor the flash path blocks between select and deselect (the
 * flash driver busy-waits there, see FLASH_vEnsureAwake).
 * -------------------------------------------------------------------------- */
void HAL_SPI_vOnSleep(void)
{
    if (bAccSpiInUse)
    {
        HAL_GPIO_vInitOutput(BSP_ACC_SCK_PORT,  BSP_ACC_SCK_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_vInitOutput(BSP_ACC_MOSI_PORT, BSP_ACC_MOSI_PIN, GPIO_PIN_RESET);
        HAL_GPIO_vInitAnalogNoPull(BSP_ACC_MISO_PORT, BSP_ACC_MISO_PIN);
        bAccSpiParked = true;
    }

    if (bFlashSpiInUse)
    {
        HAL_GPIO_vInitOutput(BSP_FLASH_SCK_PORT,  BSP_FLASH_SCK_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_vInitOutput(BSP_FLASH_MOSI_PORT, BSP_FLASH_MOSI_PIN, GPIO_PIN_RESET);
        HAL_GPIO_vInitAnalogNoPull(BSP_FLASH_MISO_PORT, BSP_FLASH_MISO_PIN);
        bFlashSpiParked = true;
    }
}

/* Restore one bus on demand. No-op unless a STOP2 sleep parked it, so only the
 * first select after a wake pays the (cheap, register-only) re-init. The parked
 * flag can only be true for a bus that is in use, so this never brings up a
 * bus that init.c did not start. */
void HAL_SPI_ACC_vEnsureAwake(void)
{
    if (bAccSpiParked)
        HAL_SPI_vInitAccHw();
}

void HAL_SPI_FLASH_vEnsureAwake(void)
{
    if (bFlashSpiParked)
        HAL_SPI_vInitFlashHw();
}

/* --------------------------------------------------------------------------
 * HAL_SPI_vInitAfPin — configure one SPI signal pin for its alternate function.
 *
 * A fresh, fully populated GPIO_InitTypeDef per pin, using that pin's own port.
 * The previous MspInit reused one struct across pins, so the flash SCK and MOSI
 * silently inherited the MISO pin's pull-up; setting every field per pin is
 * what stops that class of leak.
 * -------------------------------------------------------------------------- */
static void HAL_SPI_vInitAfPin(GPIO_TypeDef *pPort, uint32_t u32Pin, uint32_t u32Pull,
                               uint32_t u32Speed, uint32_t u32Alternate)
{
    GPIO_InitTypeDef tGpio = {0};

    tGpio.Pin       = u32Pin;
    tGpio.Mode      = GPIO_MODE_AF_PP;
    tGpio.Pull      = u32Pull;
    tGpio.Speed     = u32Speed;
    tGpio.Alternate = u32Alternate;
    HAL_GPIO_Init(pPort, &tGpio);
}

/* --------------------------------------------------------------------------
 * HAL_SPI_MspInit — called from HAL_SPI_Init()
 *
 * Signal pins only. The chip selects are NOT configured here: PA11 (ACC) and
 * PA15 (flash) are driven high once by HAL_GPIO_vInit in main(), before
 * SystemClock_Config, and STOP2 retains them. The flash CS used to be set up
 * here with HAL_GPIO_Init before its level was written, which pulsed it low
 * for a few cycles on the first init (the output data register resets to 0)
 * and re-ran HAL_GPIO_Init on PA15 at every lazy re-init after a wake.
 *
 * SCK pull-down, both buses: both are mode 0 (CPOL low). HAL_SPI_Init leaves
 * SPE clear after every (re)init until the first transfer sets it, and while
 * SPE is clear the SPI does not drive its pins, so SCK would float. The
 * reference manual's advice is to pull SCK to its CPOL idle level, and that is
 * what this does. On the flash bus it also fixes a real wrinkle: SCK used to
 * inherit the MISO pin's pull-up, so the first command after every STOP2 wake
 * began with SCK HIGH at the CS falling edge (mode-3 framing) followed by an
 * extra SCK falling edge when SPE came on. The AT25XE041B accepts mode 3, so it
 * worked, but an extra SCK edge is exactly the transport fault Flash_SelfTest's
 * "shifted" classification exists to catch. The vendor HAL_GPIO_Init writes
 * the pull before it switches the mode, so the pull is in place before the pin
 * becomes AF.
 * -------------------------------------------------------------------------- */
void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{
    if (spiHandle->Instance == ACC_SPI)
    {
        ACC_SPI_CLK_ENABLE();
        ACC_PORT_CLK_ENABLE();

        /* All three AF5 (SPI1 on port A), LOW speed: 375 kHz needs nothing
         * faster.
         *
         * PA1 (SCK) gets the pull-down described above. On this bus it also
         * holds SCL low while SPE is clear, so the accelerometer's I2C
         * interface - live whenever CS is high - cannot see a START condition
         * from MOSI floating in that window.
         *
         * PA6 (MISO) has NO pull, and must never get a pull-down: the LIS2DH's
         * SDO/SA0 pin has an internal pull-up that stays connected
         * (CTRL_REG0 = 0x10 leaves SDO_PU_DISC = 0), and an MCU pull-down
         * would fight it continuously, as a static divider current, for as
         * long as the bus is up. That internal pull-up is also why an idle
         * MISO reads back 0xFF here (see ACC_bDeviceIdOkEx). PA7 (MOSI) needs
         * no pull: the SPI drives it from the first transfer on, and STOP2
         * drives it low. */
        HAL_SPI_vInitAfPin(BSP_ACC_SCK_PORT,  BSP_ACC_SCK_PIN,  GPIO_PULLDOWN,
                           GPIO_SPEED_FREQ_LOW, BSP_ACC_SPI_AF);
        HAL_SPI_vInitAfPin(BSP_ACC_MOSI_PORT, BSP_ACC_MOSI_PIN, GPIO_NOPULL,
                           GPIO_SPEED_FREQ_LOW, BSP_ACC_SPI_AF);
        HAL_SPI_vInitAfPin(BSP_ACC_MISO_PORT, BSP_ACC_MISO_PIN, GPIO_NOPULL,
                           GPIO_SPEED_FREQ_LOW, BSP_ACC_SPI_AF);
    }
    else if (spiHandle->Instance == FLASH_SPI)
    {
        FLASH_SPI_CLK_ENABLE();
        FLASH_PORT_CLK_ENABLE();

        /* PA8 (SCK) and PA10 (MOSI) are AF5, PA5 (MISO) is AF3 - SPI2's
         * functions sit on different AF numbers on port A.
         *
         * PA5 (MISO) keeps its pull-up. It keeps the line defined while the
         * flash is deselected or absent, so a read of an unpopulated device
         * returns all-ones rather than a floating pattern that could look like
         * data - and Flash_SelfTest's "all FFh = device not driving MISO"
         * classification depends on it. PA10 (MOSI) needs no pull: the SPI
         * drives it from the first transfer on, and STOP2 drives it low.
         *
         * GPIO_SPEED_FREQ_HIGH on all three signal pins, raised from LOW when
         * the bus went from 750 kHz to 12 MHz. A LOW-speed pad's slew is
         * specified for far slower edges than a 12 MHz clock needs, and on a
         * 2.0 V rail the margin between a slow edge and a missed setup time is
         * not somewhere to economise - this driver is being asked to prove
         * that reads are trustworthy, so the pads must not be the reason they
         * are not.
         *
         * The other side of that trade is not settled, though. The likelier
         * explanation for the TX-correlated corrupt reads seen on frtag2 is
         * noise on the very long MISO/MOSI runs over a non-continuous ground
         * plane (see OTA_LORA_CHUNK_GAP_MS in Fota_Config.h), and fast edges on
         * long traces are themselves a signal-integrity question: ringing and
         * crosstalk grow with edge rate. Whether a slower pad or a series
         * resistor serves V3 RevA better is a hardware measurement to make, not
         * a setting to change blind here, so the validated speeds stay. */
        HAL_SPI_vInitAfPin(BSP_FLASH_SCK_PORT,  BSP_FLASH_SCK_PIN,  GPIO_PULLDOWN,
                           GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI2);
        HAL_SPI_vInitAfPin(BSP_FLASH_MOSI_PORT, BSP_FLASH_MOSI_PIN, GPIO_NOPULL,
                           GPIO_SPEED_FREQ_HIGH, GPIO_AF5_SPI2);
        HAL_SPI_vInitAfPin(BSP_FLASH_MISO_PORT, BSP_FLASH_MISO_PIN, GPIO_PULLUP,
                           GPIO_SPEED_FREQ_HIGH, GPIO_AF3_SPI2);
    }
}
