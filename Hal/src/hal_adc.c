/*
 * hal_adc.c
 *
 * ADC driver — battery (PA12/IN8), VSOLAR (PB3/IN2), plus the internal
 * VREFINT channel. VREFINT is converted alongside every external sample to
 * measure the true VDDA (the ADC reference; 2.0 V nominal on Tagskaap V3
 * RevA), so no result depends on the rail being exactly nominal.
 *
 * The peripheral is shared. Use HAL_ADC_vLock / HAL_ADC_vUnlock around
 * each full enable → convert → disable sequence.
 */

#include "hal_adc.h"
#include "hal_bsp.h"
#include "stm32wlxx.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile bool   adc_flag;
static uint16_t        adc_value;
static osMutexId_t     adc_mutex;

/* Set true by HAL_ADC_vMarkParked() on each STOP2 wake; cleared by a full
 * HAL_ADC_vInit(). The ADC is only used every ~10 s (battery + solar), so
 * instead of paying the init + calibration cost on every 1 Hz wake we defer
 * it: HAL_ADC_vLock() re-inits lazily on the first access after a sleep. */
static volatile bool   bAdcParked;

ADC_HandleTypeDef hadc;

void HAL_ADC_vInit(void)
{
    hadc.Instance                   = ADC;
    /* ADC clock = PCLK2 / 4 = 12 MHz (PCLK2 is 48 MHz). This comment used to
     * claim a 16 MHz ADC limit that /2 = 24 MHz would exceed. That figure was
     * wrong: the vendor HAL (stm32wlxx_hal_adc.c) gives the STM32WL ADC clock
     * maximum as 35 MHz, and SystemClock_Config runs the core in voltage
     * scale 1, so /2 would also be in spec.
     * History (frtag2 commit 96003e0): VREFINT converted ~2 % high, so the
     * computed VDDA came out ~2 % low. The prescaler went from /2 to /4 and
     * the sampling time was raised in the same change that added the offset
     * calibration below. The calibration is what actually fixed the error,
     * and with all three in place the computed VDDA matched a multimeter to
     * ~1 mV. /4 is kept as the conservative, hardware-validated setting; /2
     * has not been re-validated with calibration in place. */
    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait      = DISABLE;
    hadc.Init.LowPowerAutoPowerOff  = DISABLE;
    hadc.Init.ContinuousConvMode    = DISABLE;
    hadc.Init.NbrOfConversion       = 1;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    /* 160.5 cycles (~13 us at the 12 MHz ADC clock): the high-impedance
     * VREFINT internal channel needs a long settling window or it converts
     * high and corrupts the VDDA estimate. Both channels use common time 1. */
    hadc.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_160CYCLES_5;
    hadc.Init.SamplingTimeCommon2   = ADC_SAMPLETIME_160CYCLES_5;
    hadc.Init.OversamplingMode      = DISABLE;
    hadc.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;

    /* Recover from a stale error state before a lazy re-init.
     *
     * The vendor HAL latches HAL_ADC_STATE_ERROR_INTERNAL in hadc.State on any
     * 2 ms ADC timeout - ADC_Enable not seeing ADRDY (HAL_ADC_vEnable ignores
     * that return), ADC_ConversionStop not seeing ADSTART clear, ADC_Disable
     * refusing - and only HAL_ADC_DeInit (State back to RESET) ever clears it.
     * HAL_ADC_Init refuses while the bit is set, so one transient during a
     * battery or solar sample turned the next lazy HAL_ADC_vInit() after a
     * STOP2 wake into Error_Handler(): interrupts off, spin, watchdog reset.
     * The failed sample itself has already been dealt with by its caller
     * (Battery tallies it as a failed sample, Solar skips it), so the stale
     * flag is the only thing left to stop it rebooting the board.
     *
     * So on a re-init (State no longer RESET): first try to bring the
     * peripheral to a clean stop - stop any conversion and disable it, the
     * same as HAL_ADC_vDisable() - and then, if the hardware really is
     * disabled with no conversion in flight, clear the stale error bits and
     * error code so HAL_ADC_Init can run. If the hardware is still enabled or
     * converting after that, the bits are left alone and HAL_ADC_Init fails
     * into Error_Handler as before: an ADC that cannot be stopped is a real
     * fault, not a stale flag, and a reset is the right answer to it. This is
     * the house fail-safe idiom - only evidence that the hardware is actually
     * idle reopens the gate. The boot call (State == RESET) skips all of this;
     * HAL_ADC_Init starts from a clean handle there. */
    if (hadc.State != HAL_ADC_STATE_RESET)
    {
        if ((LL_ADC_IsEnabled(hadc.Instance) != 0UL) ||
            (LL_ADC_REG_IsConversionOngoing(hadc.Instance) != 0UL))
        {
            (void)HAL_ADC_Stop_IT(&hadc);
        }

        if ((LL_ADC_IsEnabled(hadc.Instance) == 0UL) &&
            (LL_ADC_REG_IsConversionOngoing(hadc.Instance) == 0UL))
        {
            CLEAR_BIT(hadc.State, HAL_ADC_STATE_ERROR_INTERNAL |
                                  HAL_ADC_STATE_ERROR_CONFIG);
            hadc.ErrorCode = HAL_ADC_ERROR_NONE;
        }
    }

    if (HAL_ADC_Init(&hadc) != HAL_OK)
        Error_Handler();

    /* Calibrate the ADC offset. The STM32WL SAR ADC powers up uncalibrated and
     * carries a stable few-percent error until this runs — which is what made
     * VREFINT read ~2 % high and corrupted the VDDA estimate. Must run with the
     * ADC disabled. That holds at boot (right after init) and on a lazy
     * re-init, because every lock holder disables the ADC before
     * HAL_ADC_vUnlock() and HAL_ADC_vDisable() stops a pending conversion
     * first. HAL_ADC_vInit() runs once at boot (init.c), then again lazily
     * from HAL_ADC_vLock() on the first ADC access after each STOP2 wake
     * (flagged by HAL_ADC_vMarkParked()). It never runs on the wake path
     * itself. Every sample taken after a sleep therefore converts on a freshly
     * calibrated ADC, which matters because the ADC loses power in STOP2. */
    if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
        Error_Handler();

    /* Enable the VREFINT internal channel path here, while the ADC is disabled
     * (CCR may only be written then; calibration leaves it disabled), so the
     * VREFINT buffer has started before its first conversion. The vendor
     * start-up figure is LL_ADC_DELAY_VREFINT_STAB_US = 12 us. The old
     * argument here ("sampling runs every 10 s") no longer holds, because
     * this write now also happens on the lazy re-init inside HAL_ADC_vLock(),
     * immediately before a sample. The real margins are these. On the battery
     * path the VREFINT conversion follows the 40 ms BAT_MEAS_BIAS settle
     * delay plus the battery conversion, so it is thousands of times longer
     * than needed. On the solar path it follows ADC_Enable and the VSOLAR
     * conversion (160.5 + 12.5 ADC clocks, ~14.4 us at 12 MHz), and VREFINT's
     * own 160.5-cycle sampling window (~13.4 us) ends no earlier than ~28 us
     * after this write: more than twice the 12 us figure. There is no
     * explicit wait for that reason; any change that shortens the sampling
     * time or reorders the solar conversions must revisit it.
     * HAL_ADC_ConfigChannel for the regular battery/solar channels leaves this
     * internal path untouched. */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc.Instance),
                                   LL_ADC_PATH_INTERNAL_VREFINT);

    /* The ADC is shared between the battery and solar workers — serialize.
     * Create the mutex ONCE. HAL_ADC_vInit() is re-run lazily from
     * HAL_ADC_vLock() on the first ADC access after every STOP2 wake, that is
     * with adc_mutex already HELD by the caller. Re-creating the mutex there
     * would leak the old one on every post-sleep sample until the heap is
     * exhausted and configASSERT traps. It would also break the
     * serialisation: the caller would keep holding the old mutex forever
     * while its HAL_ADC_vUnlock() tried to release a new one it never took,
     * and the next worker would lock that new mutex while the first was still
     * inside its sequence. The guard keeps creation on the boot path (init.c)
     * only. (Before the lazy re-init, frtag2 ran HAL_ADC_vInit() from the
     * masked-interrupt STOP2 wake path every second, where the per-wake
     * pvPortMalloc was both a leak and illegal, and froze deep sleep; frtag2
     * commit eb2a7fd.) */
    if (adc_mutex == NULL)
    {
        adc_mutex = osMutexNew(NULL);
        configASSERT(adc_mutex != NULL);
    }

    /* The peripheral is now fully initialised and calibrated. */
    bAdcParked = false;
}

/* --------------------------------------------------------------------------
 * HAL_ADC_vMarkParked — note that STOP2 has torn down the ADC.
 *
 * Called from the tickless-idle wake path (interrupts masked), so it must do
 * no peripheral work — just flag that a re-init is owed. The next
 * HAL_ADC_vLock() pays the init + calibration cost, and only when a sample is
 * actually taken. Idle 1 Hz wakes that never sample the ADC skip it entirely.
 * -------------------------------------------------------------------------- */
void HAL_ADC_vMarkParked(void)
{
    bAdcParked = true;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (adcHandle->Instance == ADC)
    {
        __HAL_RCC_ADC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin  = BSP_BAT_MEAS_VOLT_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(BSP_BAT_MEAS_VOLT_PORT, &GPIO_InitStruct);

        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin  = BSP_VSOLAR_MEAS_PIN;
        HAL_GPIO_Init(BSP_VSOLAR_MEAS_PORT, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
    if (adcHandle->Instance == ADC)
    {
        __HAL_RCC_ADC_CLK_DISABLE();
        HAL_GPIO_DeInit(BSP_BAT_MEAS_VOLT_PORT, BSP_BAT_MEAS_VOLT_PIN);
        HAL_NVIC_DisableIRQ(ADC_IRQn);
    }
}

void HAL_ADC_vEnable(void)      { ADC_Enable(&hadc); }
bool HAL_ADC_bIsEnabled(void)   { return LL_ADC_IsEnabled(hadc.Instance) == 1; }
/* --------------------------------------------------------------------------
 * HAL_ADC_vDisable — stop any pending conversion, then disable the ADC.
 *
 * This used to call ADC_Disable() alone. ADC_Disable refuses while ADSTART=1
 * (a conversion was started and has not completed, which is the case when a
 * caller's EOC wait times out). It then returns HAL_ERROR, leaves the ADC
 * enabled and sets the sticky HAL_ADC_STATE_ERROR_INTERNAL bit in hadc.State.
 * The next lazy HAL_ADC_vInit() after a STOP2 wake then failed: HAL_ADC_Init
 * refuses while that error bit is set or a conversion is ongoing, and the
 * calibration cannot run on an enabled ADC that it cannot disable. Either
 * way it ended in Error_Handler, an interrupts-off spin until the watchdog
 * reset the board. (HAL_ADC_vInit() now also clears a stale error bit on a
 * lazy re-init once the peripheral is confirmed idle, which covers the other
 * timeouts that latch it.)
 *
 * HAL_ADC_Stop_IT (vendor stm32wlxx_hal_adc.c) is ADC_ConversionStop (set
 * ADSTP and wait for ADSTART to clear), then clear the EOC/EOS/OVR interrupt
 * enables, then ADC_Disable. With no conversion pending it is just the
 * disable. Clearing the interrupt enables changes nothing, because
 * HAL_ADC_Start_IT re-enables them for every conversion
 * (HAL_ADC_vStartConversion). It takes the HAL handle lock and uses
 * HAL_GetTick timeouts, so it must only run in task context with adc_mutex
 * held, which is true of both callers (BAT_DRIVER_vDisable and
 * SOLAR_DRIVER_vDisable). The return value is deliberately ignored: a caller
 * that needs certainty re-checks HAL_ADC_bIsEnabled(), as BatSampleTask does.
 * -------------------------------------------------------------------------- */
void HAL_ADC_vDisable(void)
{
    (void)HAL_ADC_Stop_IT(&hadc);
}

bool HAL_ADC_bGetInterruptFlag(void)    { return adc_flag; }
void HAL_ADC_vClearInterruptFlag(void)  { adc_flag = false; }

void HAL_ADC_vSelectChannel(hal_adc_channel_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    switch (channel)
    {
        case BAT_VOLTAGE_CHANNEL:
            sConfig.Channel = BSP_BAT_ADC_VOLTAGE_CHANNEL;
            break;
        case VSOLAR_VOLTAGE_CHANNEL:
            sConfig.Channel = BSP_VSOLAR_ADC_CHANNEL;
            break;
        case VREFINT_CHANNEL:
            sConfig.Channel = ADC_CHANNEL_VREFINT;
            break;
        default:
            break;
    }
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);
}

void HAL_ADC_vStartConversion(hal_adc_channel_t channel)
{
    HAL_ADC_vSelectChannel(channel);
    HAL_ADC_vClearInterruptFlag();
    HAL_ADC_Start_IT(&hadc);
}

uint16_t HAL_ADC_u16GetResult(void)
{
    return adc_value;
}

uint16_t HAL_ADC_u16VddaFromVrefint(uint16_t u16VrefintRaw)
{
    if (u16VrefintRaw == 0U)
        return 0U;

    /* __LL_ADC_CALC_VREFANALOG_VOLTAGE uses the factory VREFINT_CAL, acquired
     * at Vref+ = VDDA = 3.3 V (VREFINT_CAL_VREF in stm32wlxx_ll_adc.h), to
     * recover the true VDDA from a VREFINT conversion. The result does not
     * depend on the nominal rail, which is 2.0 V on Tagskaap V3 RevA (raw
     * around 2480 with a typical VREFINT_CAL).
     *
     * Range-check the full 32-bit quotient BEFORE narrowing it. A garbage raw
     * such as 73 gives ~68 V (67989 mV), which a (uint16_t) cast would wrap to
     * a plausible-looking ~2.45 V and feed straight into the battery and solar
     * results. A value outside the ADC supply range is reported as 0
     * ("invalid") instead. Capping VDDA at HAL_ADC_VDDA_MAX_MV (3.6 V) also
     * keeps every downstream millivolt product inside uint16_t: the largest
     * possible divider result is 4095 x 3600 x 4.3 / 4095 = 15480 mV. */
    uint32_t u32VddaMv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(u16VrefintRaw,
                                                           LL_ADC_RESOLUTION_12B);

    if ((u32VddaMv < HAL_ADC_VDDA_MIN_MV) || (u32VddaMv > HAL_ADC_VDDA_MAX_MV))
        return 0U;

    return (uint16_t)u32VddaMv;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *my_hadc)
{
    UNUSED(my_hadc);
    adc_flag  = true;
    adc_value = (uint16_t)HAL_ADC_GetValue(&hadc);
}

void HAL_ADC_vLock(void)
{
    osMutexAcquire(adc_mutex, osWaitForever);

    /* Lazy restore after a STOP2 sleep: re-init + recalibrate only now, on the
     * first actual access, instead of on every 1 Hz heartbeat wake. Done under
     * the lock so the re-init cannot race a concurrent battery/solar sample. */
    if (bAdcParked)
        HAL_ADC_vInit();
}
void HAL_ADC_vUnlock(void) { osMutexRelease(adc_mutex); }
