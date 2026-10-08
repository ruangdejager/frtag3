/*
 * hal_uart.c
 *
 * Interrupt-driven UART driver with ring-buffer TX/RX for the frtag3 board
 * (Tagskaap V3 RevA).
 *
 * USART2 = debug UART (PA2/PA3).
 * USART1 = GPS / Farmranger UART (PB6/PB7), mutually exclusive by device role:
 * the GNSS receiver on a SECONDARY, the fr9 logger link on a PRIMARY. The pin
 * mapping is standard on both roles (PB6 = USART1_TX, PB7 = USART1_RX); what
 * differs per role is the baud (HAL_UART_vInit) and the pin electricals
 * (HAL_UART_vEnable).
 *
 * Per-UART receive notifications are dispatched through weak callback stubs
 * (UART1_vNotifyOnRX / UART2_vNotifyOnRX). Override these in the application
 * layer to forward bytes to the relevant module without creating a compile-time
 * dependency here.
 */

#include "hal_uart.h"
#include "hal_bsp.h"
#include "hal_gpio.h"

#include "stm32wlxx.h"
#include "stm32wlxx_ll_usart.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_hal_rcc.h"

#include "cmsis_os2.h"

static hal_uart_t *uart1_buffer = NULL;
static hal_uart_t *uart2_buffer = NULL;

/* Upper bound on how long HAL_UART_vTxPutBuffer() waits for TX-ring space
 * before giving up. Normal drain is sub-millisecond (ISR-paced at the baud
 * rate); only a wedged/clock-gated USART sits full this long, and spinning
 * forever there hangs the calling task — fatal when it holds a lock (the GPS
 * dispatcher spins under xGpsLock). 2 s is orders of magnitude past any real
 * transfer, so it never trips in normal operation. */
#define HAL_UART_TX_STALL_TIMEOUT_MS   2000U

/* Device role, latched once at boot from the strap (HAL_UART_vSetRole) so the
 * USART1 baud (HAL_UART_vInit) and PB6/PB7 drive mode (HAL_UART_vEnable) can
 * be chosen without re-reading PB12 — the strap pin is parked analog right
 * after that single read. The default (false, i.e. SECONDARY) deliberately
 * selects the GNSS-safe pin configuration (PB6 open-drain, PB7 no pull), so
 * if USART1 were ever enabled before the role is latched it could still not
 * drive or pull the 1.8 V GNSS I/O above its rail. */
static bool s_bPrimaryRole = false;

/* Set once HAL_UART_vInit() has configured USART1, at its first (boot) call.
 * Only ever written by HAL_UART_vInit(); see the comment there for why USART1
 * is configured exactly once. */
static bool s_bUsart1Configured = false;

static usart_buf_ptr_t UART_u8RxBufUsedSpace(hal_uart_t *pHandle);
static usart_buf_ptr_t UART_u8RxBufFreeSpace(hal_uart_t *pHandle);

/* Weak RX-notification callbacks — override in the application. */
__attribute__((weak)) void UART1_vNotifyOnRX(void) {}
__attribute__((weak)) void UART2_vNotifyOnRX(void) {}

/* --------------------------------------------------------------------------
 * HAL_UART_vSetRole
 * Latch the device role once, after it has been read from the strap at boot.
 * Lets HAL_UART_vInit() pick the USART1 baud and HAL_UART_vEnable() pick the
 * PB6/PB7 drive mode without touching PB12 again, so the strap pin can be
 * permanently parked after that single read.
 * -------------------------------------------------------------------------- */
void HAL_UART_vSetRole(bool bPrimary)
{
    s_bPrimaryRole = bPrimary;
}

/* --------------------------------------------------------------------------
 * HAL_UART_vInit
 * One-time peripheral initialisation (clock + LL config). Call once at boot.
 * -------------------------------------------------------------------------- */
void HAL_UART_vInit(void)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};

    /* Snapshot whether USART1 (GPS on a secondary, Farmranger on a primary) is
     * already clocked — i.e. its owning driver has it powered and may be
     * actively transmitting right now. This function is called from the
     * wake-path tasks (schedule / AppTask) mainly to re-arm the debug UART
     * after STOP2, but it also re-inits USART1 and clock-gates it at the end.
     * Doing that to a USART1 another task is mid-transmit on kills the TX
     * clock, so its ISR never drains the ring and HAL_UART_vTxPutBuffer()
     * spins forever — hanging the GPS dispatcher while it holds xGpsLock:
     * the deep-sleep lock never releases (no STOP2, red LED stays on) and the
     * next GPS_vRequestFix() blocks on that mutex, so scheduled discovery
     * wakes stop. So only (re)configure USART1 when it is IDLE; when it's in
     * use, leave it entirely to its owner. USART1's baud/mode registers are
     * retained across STOP2 and clock-gating, so the boot-time init (the one
     * call where USART1 is guaranteed idle) is all it ever needs.
     *
     * The idle snapshot alone was not enough. It is a check-then-act: the
     * wake-path callers (DeviceDiscovery AppTask / CheckWakeupSched,
     * FARMRANGER_vUartOnWake) run at osPriorityNormal, the same priority as
     * the GPS dispatcher, with tick time-slicing on. A task switch between
     * this read and the __HAL_RCC_USART1_CLK_DISABLE() at the bottom let the
     * dispatcher enable USART1 and start a GNSS session, which this code then
     * clock-gated underneath it: a dead GNSS UART for the whole session (the
     * VALSET dropped after the 2 s TX stall, then a TTFF timeout). So USART1 is
     * now configured once, at the first call - init.c's boot call, made from
     * the init task before any USART1 owner exists - and never touched again.
     * Later calls cannot race because they no longer go near USART1 at all.
     * Nothing needs the re-init: HAL_UART_vDisable() only clears UE and gates
     * the clock, the configuration registers keep their boot values, and
     * HAL_UART_vEnable() sets UE again. The idle snapshot stays as a second
     * guard on that first call. */
    bool bUsart1Idle = (__HAL_RCC_USART1_IS_CLK_ENABLED() != 1);

    __HAL_RCC_USART2_CLK_ENABLE();

    /* Debug UART (USART2, PA2/PA3) — debug log and FrKernel console only. The
     * Farmranger link is USART1, configured below. */
    USART_InitStruct.BaudRate             = BSP_DEBUG_UART_BAUD;
    USART_InitStruct.DataWidth            = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits             = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity               = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection    = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl  = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling         = LL_USART_OVERSAMPLING_16;

    LL_USART_Init(BSP_DEBUG_USART_INSTANCE, &USART_InitStruct);
    LL_USART_SetHWFlowCtrl(BSP_DEBUG_USART_INSTANCE, LL_USART_HWCONTROL_NONE);
    LL_USART_ConfigAsyncMode(BSP_DEBUG_USART_INSTANCE);
    LL_USART_Enable(BSP_DEBUG_USART_INSTANCE);
    NVIC_SetPriority(BSP_DEBUG_USART_IRQn,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));

    /* GPS / Farmranger UART (USART1), mutually exclusive by device role:
     * a SECONDARY runs GNSS at 9600, the MAX-M10S factory default (the
     * firmware never changes the receiver's baud: the cold-start VALSET set
     * in GPS.c carries no CFG-UART1-BAUDRATE key); a PRIMARY has no GNSS
     * fitted and uses this UART as the Farmranger link to the fr9 board, whose
     * UART is fixed at 115200. The role was latched once at boot from the
     * strap (HAL_UART_vSetRole, fed by DEVICE_DISCOVERY_vConfigDeviceRole),
     * and the strap pin (PB12) has since been parked analog — so use the
     * cached role here instead of re-reading it. This and every wake-path
     * re-init therefore leave PB12 untouched.
     *
     * Pin mapping is STANDARD on both roles: PB6 = USART1_TX, PB7 = USART1_RX.
     * frtag2 primaries set CR2.SWAP because the harness to the fr9 crossed
     * TX/RX the opposite way to the GNSS wiring. On frtag3 that crossing is
     * being fixed on the fr9 side, so the tag leaves SWAP at its reset value
     * and the only per-role difference left is electrical (PB6 open-drain or
     * push-pull, PB7 pull), applied in HAL_UART_vEnable(). This is a
     * deployment-order dependency and a hardware-compatibility requirement: a
     * frtag3 primary paired with an fr9 (or a harness) that still crosses
     * TX/RX never receives RDY, so every logger session — upload, timestamp
     * and settings sync, OTA pull — fails, with the "RDY not received" log
     * line and the per-step failures behind it to show for it. Worse, the old
     * crossing puts the fr9's TX on PB6, which is push-pull on a primary
     * (BSP_GPS_UART_TX_MODE_FR9), so the two outputs fight on every 0 bit
     * either side sends. Only pair a frtag3 primary with a corrected fr9;
     * there is no firmware fallback.
     *
     * Done once, at the boot call (see s_bUsart1Configured above), and
     * skipped if USART1 is somehow already in use even then. */
    if (!s_bUsart1Configured && bUsart1Idle)
    {
        s_bUsart1Configured = true;

        __HAL_RCC_USART1_CLK_ENABLE();

        bool bPrimary = s_bPrimaryRole;

        USART_InitStruct.BaudRate = bPrimary ? BSP_FARMRANGER_UART_BAUD : BSP_GPS_UART_BAUD;
        LL_USART_Init(BSP_GPS_USART_INSTANCE, &USART_InitStruct);
        LL_USART_SetHWFlowCtrl(BSP_GPS_USART_INSTANCE, LL_USART_HWCONTROL_NONE);
        LL_USART_ConfigAsyncMode(BSP_GPS_USART_INSTANCE);
        /* Standard TX/RX mapping on both roles (see above). SWAP resets to 0,
         * the frtag2 bootloader runs HAL_DeInit() (a forced peripheral reset)
         * before jumping here, and nothing in this firmware sets it, so this
         * line records the intent rather than changing anything. The bit is
         * only writable while UE = 0, which holds at the boot call and on the
         * first re-init after HAL_UART_vDisable() has cleared UE. */
        LL_USART_SetTXRXSwap(BSP_GPS_USART_INSTANCE, LL_USART_TXRX_STANDARD);
        LL_USART_Enable(BSP_GPS_USART_INSTANCE);
        NVIC_SetPriority(BSP_GPS_USART_IRQn,
                         NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));

        /* Leave the clock disabled until needed — re-enabled by HAL_UART_vSetup */
        __HAL_RCC_USART1_CLK_DISABLE();
    }

    __HAL_RCC_USART2_CLK_DISABLE();
}

/* --------------------------------------------------------------------------
 * HAL_UART_vSetup — bind a hal_uart_t instance to a physical UART
 * -------------------------------------------------------------------------- */
void HAL_UART_vSetup(hal_uart_t *drv, hal_uart_id_t uart_id, hal_uart_fc_t flowcontrol)
{
    bool clkWasDisabled = false;

    drv->flowcontrol      = flowcontrol;
    drv->buffer.RX_Head   = 0;
    drv->buffer.RX_Tail   = 0;
    drv->buffer.TX_Head   = 0;
    drv->buffer.TX_Tail   = 0;

    switch (uart_id)
    {
        case DEBUG_UART:
            clkWasDisabled = (__HAL_RCC_USART2_IS_CLK_ENABLED() != 1);
            __HAL_RCC_USART2_CLK_ENABLE();
            drv->usart   = BSP_DEBUG_USART_INSTANCE;
            uart2_buffer = drv;
            HAL_NVIC_EnableIRQ(BSP_DEBUG_USART_IRQn);
            break;
        case GPS_UART:
            clkWasDisabled = (__HAL_RCC_USART1_IS_CLK_ENABLED() != 1);
            __HAL_RCC_USART1_CLK_ENABLE();
            drv->usart   = BSP_GPS_USART_INSTANCE;
            uart1_buffer = drv;
            HAL_NVIC_EnableIRQ(BSP_GPS_USART_IRQn);
            break;
        default:
            break;
    }

    drv->uart_id = uart_id;
    LL_USART_EnableIT_RXNE(drv->usart);

    if (clkWasDisabled)
    {
        if (drv->usart == USART1)
            __HAL_RCC_USART1_CLK_DISABLE();
        else if (drv->usart == USART2)
            __HAL_RCC_USART2_CLK_DISABLE();
    }
}

void HAL_UART_vSetFlowControlFunctions(hal_uart_t *drv,
                                       setRtsAssert setRtsFunction,
                                       getCTSPin    getCtsFunction)
{
    drv->flowControlGetCTS        = getCtsFunction;
    drv->flowControlSetAssertRts  = setRtsFunction;
}

/* --------------------------------------------------------------------------
 * HAL_UART_vEnable
 * Attach the UART's pins to the peripheral (AF7), enable the USART and clear
 * the ring.
 *
 * Every GPIO_InitTypeDef field is set inside each case rather than once at
 * the top of the function. The two UARTs need different electricals. frtag2
 * applied one shared default (AF push-pull with pull-up) to PB6/PB7 and the
 * debug pins alike, and on a frtag3 secondary that is exactly the setting
 * that drives 2.0 V into the 1.8 V GNSS I/O. Setting each case in full keeps
 * one UART's choice from leaking into the other's.
 *
 * USART1 (GPS_UART) electricals follow the role latched by
 * HAL_UART_vSetRole(); the values live in hal_bsp.h (BSP_GPS_UART_TX_MODE_*,
 * BSP_GPS_UART_RX_PULL_*):
 *   SECONDARY (GNSS)  PB6 open-drain, no pull; PB7 no pull. The GNSS I/O is a
 *                     1.8 V domain (abs max 1.98 V) against the MCU's 2.0 V
 *                     VDD, and R18 (4.7k) pulls PB6 up to GNSS_VCC, so the
 *                     MCU may only ever pull PB6 low and must not pull PB7 up
 *                     to its own rail. The caller (GPS_vHwPowerOn in GPS.c)
 *                     raises GNSS_ON and waits for the rail before this runs.
 *   PRIMARY (fr9)     PB6 push-pull, no pull; PB7 pull-up. No GNSS is fitted,
 *                     so a driven TX keeps the link independent of whether R18
 *                     and the GNSS LDO are populated, and the pull-up holds RX
 *                     idle-high while the fr9 is not driving it.
 * The s_bPrimaryRole default (false) selects the SECONDARY set, the one that
 * cannot harm the GNSS I/O.
 *
 * PB7 (RX) is attached before PB6 (TX), in two separate HAL_GPIO_Init calls,
 * so the receive side is already in its final state when the transmit pin
 * starts presenting an idle level to the far end. The vendor HAL_GPIO_Init
 * writes OTYPER before MODER, so switching PB6 from analog to AF open-drain
 * never passes through push-pull on the way. HAL_UART_vDisable() puts both
 * pins back to analog through HAL_GPIO_DeInit(), which also clears OTYPER
 * and PUPDR, so nothing from one session carries into the next.
 * -------------------------------------------------------------------------- */
void HAL_UART_vEnable(hal_uart_t *drv)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    switch (drv->uart_id)
    {
        case DEBUG_UART:
            __HAL_RCC_USART2_CLK_ENABLE();
            /* PA2 = USART2_TX, PA3 = USART2_RX: push-pull with pull-up, as
             * on frtag2. Only the debug cable is on these pins. */
            GPIO_InitStruct.Pin       = BSP_DEBUG_UART_TX_PIN | BSP_DEBUG_UART_RX_PIN;
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull      = GPIO_PULLUP;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
            GPIO_InitStruct.Alternate = BSP_DEBUG_UART_AF;
            HAL_GPIO_Init(BSP_DEBUG_UART_TX_PORT, &GPIO_InitStruct);
            NVIC_EnableIRQ(BSP_DEBUG_USART_IRQn);
            break;
        case GPS_UART:
            __HAL_RCC_USART1_CLK_ENABLE();
            /* PB7 = USART1_RX: GNSS TXD on a secondary, fr9 TX on a primary.
             * The output type is irrelevant for a receive pin; AF_PP is the
             * neutral choice. */
            GPIO_InitStruct.Pin       = BSP_GPS_UART_RX_PIN;
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull      = s_bPrimaryRole ? BSP_GPS_UART_RX_PULL_FR9
                                                       : BSP_GPS_UART_RX_PULL_GNSS;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
            GPIO_InitStruct.Alternate = BSP_GPS_UART_AF;
            HAL_GPIO_Init(BSP_GPS_UART_RX_PORT, &GPIO_InitStruct);

            /* PB6 = USART1_TX: GNSS RXD on a secondary, fr9 RX on a primary.
             * Never pulled internally on either role. */
            GPIO_InitStruct.Pin       = BSP_GPS_UART_TX_PIN;
            GPIO_InitStruct.Mode      = s_bPrimaryRole ? BSP_GPS_UART_TX_MODE_FR9
                                                       : BSP_GPS_UART_TX_MODE_GNSS;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
            GPIO_InitStruct.Alternate = BSP_GPS_UART_AF;
            HAL_GPIO_Init(BSP_GPS_UART_TX_PORT, &GPIO_InitStruct);
            NVIC_EnableIRQ(BSP_GPS_USART_IRQn);
            break;
        default:
            break;
    }

    LL_USART_Enable(drv->usart);
    HAL_UART_vClearBuffer(drv);
}

void HAL_UART_vDisable(hal_uart_t *drv)
{
    LL_USART_Disable(drv->usart);

    switch (drv->uart_id)
    {
        case DEBUG_UART:
            HAL_GPIO_DeInit(BSP_DEBUG_UART_TX_PORT,
                            BSP_DEBUG_UART_TX_PIN | BSP_DEBUG_UART_RX_PIN);
            NVIC_DisableIRQ(BSP_DEBUG_USART_IRQn);
            __HAL_RCC_USART2_CLK_DISABLE();
            break;
        case GPS_UART:
            HAL_GPIO_DeInit(BSP_GPS_UART_TX_PORT,
                            BSP_GPS_UART_TX_PIN | BSP_GPS_UART_RX_PIN);
            NVIC_DisableIRQ(BSP_GPS_USART_IRQn);
            __HAL_RCC_USART1_CLK_DISABLE();
            break;
        default:
            break;
    }
}

void HAL_UART_vClearBuffer(hal_uart_t *drv)
{
    drv->buffer.RX_Head = 0;
    drv->buffer.RX_Tail = 0;
    drv->buffer.TX_Head = 0;
    drv->buffer.TX_Tail = 0;
}

bool HAL_UART_u8TxFreeSpace(hal_uart_t *drv)
{
    usart_buf_ptr_t tempHead = (drv->buffer.TX_Head + 1) & USART_TX_BUFFER_MASK;
    usart_buf_ptr_t tempTail = drv->buffer.TX_Tail;
    return (tempHead != tempTail);
}

bool HAL_UART_u8TxBufferEmpty(hal_uart_t *drv)
{
    return (drv->buffer.TX_Head == drv->buffer.TX_Tail);
}

/* True once the TX ring is drained AND the final byte has left the shift
 * register (TC set) — i.e. the line is idle and safe to reconfigure or tear
 * down without truncating an in-flight byte. Level-triggered, so a caller can
 * poll this to wait out a transmission regardless of how many ring-empty
 * events the ISR saw along the way. */
bool HAL_UART_bTxIdle(hal_uart_t *drv)
{
    return HAL_UART_u8TxBufferEmpty(drv) && LL_USART_IsActiveFlag_TC(drv->usart);
}

bool HAL_UART_vTxPutByte(hal_uart_t *drv, uint8_t data)
{
    if (!HAL_UART_u8TxFreeSpace(drv))
        return false;

    usart_buf_ptr_t tempTX_Head = drv->buffer.TX_Head;
    drv->buffer.TX[tempTX_Head] = data;
    drv->buffer.TX_Head = (tempTX_Head + 1) & USART_TX_BUFFER_MASK;
    HAL_UART_vEnableTXInterrupt(drv);
    return true;
}

void HAL_UART_vTxPutBuffer(hal_uart_t *drv, const uint8_t *data, uint16_t length)
{
    for (uint16_t idx = 0; idx < length; idx++)
    {
        /* Push into the interrupt-driven TX ring; only yield when the ring is
         * full (waiting for the ISR to drain), not after every byte. Otherwise
         * output is throttled to ~1 KB/s, which makes a flash-log dump take
         * minutes. With this, throughput is limited by the UART baud rate.
         *
         * Bounded: a ring that stays full for HAL_UART_TX_STALL_TIMEOUT_MS
         * means the TX ISR isn't draining it (clock gated / peripheral wedged).
         * Spinning forever there would hang this task; if it holds a lock the
         * whole subsystem wedges (this is exactly how a USART1 re-init racing a
         * live GPS transfer used to hang the GPS dispatcher under xGpsLock).
         * Drop the rest of the buffer instead — recoverable, unlike a deadlock. */
        uint32_t u32WaitedMs = 0U;
        while (!HAL_UART_vTxPutByte(drv, data[idx]))
        {
            if (u32WaitedMs >= HAL_UART_TX_STALL_TIMEOUT_MS)
                return;
            osDelay(1);
            u32WaitedMs++;
        }
    }
}

/*
 * HAL_UART_vTxPutBufferBlocking
 *
 * Polling transmit that bypasses the TX ring buffer and its IRQ entirely —
 * safe to call with global interrupts disabled or from a fault/overflow
 * handler where the scheduler and the TX-complete ISR will never run again.
 */
void HAL_UART_vTxPutBufferBlocking(hal_uart_t *drv, const uint8_t *data, uint16_t length)
{
    for (uint16_t idx = 0; idx < length; idx++)
    {
        while (!LL_USART_IsActiveFlag_TXE(drv->usart));
        LL_USART_TransmitData8(drv->usart, data[idx]);
    }
    while (!LL_USART_IsActiveFlag_TC(drv->usart));
}

bool HAL_UART_bRxDataAvailable(hal_uart_t *usart_data)
{
    usart_buf_ptr_t tempHead = usart_data->buffer.RX_Head;
    usart_buf_ptr_t tempTail = usart_data->buffer.RX_Tail;

    if (tempHead == tempTail)
    {
        if (usart_data->flowcontrol == FLOWCONTROL_FUNCTION)
            usart_data->flowControlSetAssertRts(true);
        return false;
    }
    return true;
}

bool UART_bReadByte(hal_uart_t *pHandle, uint8_t *pu8Byte)
{
    if (pHandle == NULL)
        return false;

    if (!HAL_UART_bRxDataAvailable(pHandle))
        return false;

    *pu8Byte = pHandle->buffer.RX[pHandle->buffer.RX_Tail];
    pHandle->buffer.RX_Tail = (pHandle->buffer.RX_Tail + 1) & USART_RX_BUFFER_MASK;
    return true;
}

void HAL_UART_vEnableTXInterrupt(hal_uart_t *drv)
{
    LL_USART_EnableIT_TXE(drv->usart);
}

static usart_buf_ptr_t UART_u8RxBufUsedSpace(hal_uart_t *pHandle)
{
    usart_buf_ptr_t tempHead = pHandle->buffer.RX_Head;
    usart_buf_ptr_t tempTail = pHandle->buffer.RX_Tail;
    return ((tempHead - tempTail) & USART_RX_BUFFER_MASK);
}

static usart_buf_ptr_t UART_u8RxBufFreeSpace(hal_uart_t *pHandle)
{
    return ((USART_RX_BUFFER_SIZE - 1) - UART_u8RxBufUsedSpace(pHandle));
}

/* --------------------------------------------------------------------------
 * HAL_UART_vInterrupt — universal ISR handler, call from USARTx_IRQHandler
 * -------------------------------------------------------------------------- */
void HAL_UART_vInterrupt(USART_TypeDef *USARTx)
{
    hal_uart_t *drv = NULL;

    if      (USARTx == USART1) drv = uart1_buffer;
    else if (USARTx == USART2) drv = uart2_buffer;

    if (drv == NULL) return;

    /* --- RX --- */
    if (LL_USART_IsEnabledIT_RXNE(drv->usart) &&
        LL_USART_IsActiveFlag_RXNE(drv->usart))
    {
        uint8_t data = LL_USART_ReceiveData8(drv->usart);

        usart_buf_ptr_t tempRX_Head = (drv->buffer.RX_Head + 1) & USART_RX_BUFFER_MASK;
        if (tempRX_Head != drv->buffer.RX_Tail)
        {
            drv->buffer.RX[drv->buffer.RX_Head] = data;
            drv->buffer.RX_Head = tempRX_Head;
        }
        /* else: buffer full — drop byte */

        if (UART_u8RxBufFreeSpace(drv) <= USART_RX_BUFFER_FREE_SPACE_LWM)
        {
            if ((drv->flowcontrol == FLOWCONTROL_FUNCTION) &&
                (drv->flowControlSetAssertRts != NULL))
                drv->flowControlSetAssertRts(false);
        }

        /* Dispatch to registered application callback */
        if (USARTx == USART1)      UART1_vNotifyOnRX();
        else if (USARTx == USART2) UART2_vNotifyOnRX();
    }

    /* --- TX --- */
    if (LL_USART_IsEnabledIT_TXE(drv->usart) &&
        LL_USART_IsActiveFlag_TXE(drv->usart))
    {
        usart_buf_ptr_t tempTX_Tail = drv->buffer.TX_Tail;
        if (drv->buffer.TX_Head == tempTX_Tail)
        {
            LL_USART_DisableIT_TXE(drv->usart);
            HAL_UART_vTxCompleteISR(drv);
        }
        else
        {
            if ((drv->flowcontrol == FLOWCONTROL_FUNCTION) &&
                (drv->flowControlGetCTS != NULL) &&
                (drv->flowControlGetCTS()))
            {
                LL_USART_DisableIT_TXE(drv->usart);
            }
            else
            {
                uint8_t data = drv->buffer.TX[tempTX_Tail];
                drv->buffer.TX_Tail = (tempTX_Tail + 1) & USART_TX_BUFFER_MASK;
                LL_USART_TransmitData8(drv->usart, data);
            }
        }
    }

    /* Clear miscellaneous status flags */
    LL_USART_ClearFlag_CM(drv->usart);
    LL_USART_ClearFlag_EOB(drv->usart);
    LL_USART_ClearFlag_FE(drv->usart);
    LL_USART_ClearFlag_IDLE(drv->usart);
    LL_USART_ClearFlag_LBD(drv->usart);
    LL_USART_ClearFlag_NE(drv->usart);
    LL_USART_ClearFlag_ORE(drv->usart);
    LL_USART_ClearFlag_PE(drv->usart);
    LL_USART_ClearFlag_RTO(drv->usart);
    LL_USART_ClearFlag_TCBGT(drv->usart);
    LL_USART_ClearFlag_WKUP(drv->usart);
    LL_USART_ClearFlag_nCTS(drv->usart);
}

void HAL_UART_vSetSpeed(hal_uart_t *drv, uint32_t new_speed)
{
    if (drv->usart == USART1)
    {
        USART1->CR1 &= ~USART_CR1_UE;
        USART1->BRR  = (uint16_t)(__LL_USART_DIV_SAMPLING16(
                            LL_RCC_GetUSARTClockFreq(LL_RCC_USART1_CLKSOURCE),
                            LL_USART_PRESCALER_DIV1, new_speed));
        USART1->CR1 |= USART_CR1_UE;
    }
    else if (drv->usart == USART2)
    {
        USART2->CR1 &= ~USART_CR1_UE;
        USART2->BRR  = (uint16_t)(__LL_USART_DIV_SAMPLING16(
                            LL_RCC_GetUSARTClockFreq(LL_RCC_USART2_CLKSOURCE),
                            LL_USART_PRESCALER_DIV1, new_speed));
        USART2->CR1 |= USART_CR1_UE;
    }
}

__attribute__((weak))
void HAL_UART_vTxCompleteISR(hal_uart_t *drv)
{
    (void)drv;
    /* Default: do nothing */
}
