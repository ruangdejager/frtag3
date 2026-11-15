/*
 * hal_gpio.c
 *
 * GPIO safe-state initialisation and STOP2 pin parking for the Tagskaap V3
 * RevA board (frtag3). The pin map and the per-role population live in
 * hal_bsp.h.
 *
 * HAL_GPIO_vInit() is called from main(), right after HAL_Init() and before
 * SystemClock_Config(). Several board pins have no external pull (PA0, PA4,
 * PA11) and float from reset until firmware drives them. When the
 * image starts without the bootloader (a debugger launch), the LSE start
 * inside SystemClock_Config() can take up to LSE_STARTUP_TIMEOUT (5 s) - or
 * end in Error_Handler() if the crystal never starts - so the safe state goes
 * in before it. Behind the frtag3 bootloader the LSE is already running and
 * the wait is effectively zero, and the bootloader has applied these same
 * levels itself before its own LSE start (frtag3_bootloader Hal/src/hal_gpio.c
 * - keep the two in step); its HAL_DeInit() at the jump resets the ports, so
 * this re-applies them a few microseconds later. Running that early
 * means running at the 4 MHz reset MSI clock with no kernel: HAL_GPIO_vInit()
 * must stay free of RTOS calls, logging and delays.
 *
 * Peripheral alternate-function pins (UART TX/RX, SPI SCK/MISO/MOSI, ADC
 * inputs) belong to their HAL drivers. STOP2 parking is split the same way:
 * HAL_GPIO_vOnSleep() below parks the UART pins, while the SPI pins are parked
 * by HAL_SPI_vOnSleep() in hal_spi.c, which HAL_SYSTEM_vEnterStop2() calls
 * immediately before HAL_GPIO_vOnSleep().
 */

#include "hal_gpio.h"
#include "hal_bsp.h"

#include "stm32wlxx.h"

static bool bLedsAllowed = true;

void HAL_GPIO_vInit(void)
{
    /* Port clocks first: every pin write below goes through BSRR (the output
     * helpers write the level before switching the mode), and a BSRR write is
     * silently lost while its port clock is off. GPIOH is not enabled - its
     * only bonded pin is PH3 (BOOT0), which is never touched. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* LEDs: red and green come up lit, as the boot indicator. Done first so
     * that an image started without the bootloader that then hangs in the LSE
     * start (SystemClock_Config, which main() calls right after this function)
     * shows two solid LEDs rather than looking dead. Behind the bootloader an
     * LSE failure hangs before the application runs at all. The third LED
     * (PB4: blue on a secondary, yellow on a primary) is driven OFF here and
     * is not part of the boot indicator - see LED_AUX in hal_bsp.h. */
    BSP_LED_Init(LED_RED);
    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_AUX);

    /* PA0, low. Role-dependent and initialised here only: the GNSS LDO enable
     * on a SECONDARY, the fr9 wake line on a PRIMARY (see hal_bsp.h). Low means
     * GNSS off / no fr9 session request. It has no external pull, so it floats
     * until this line; its owner (GPS.c or Farmranger.c) raises it later. */
    HAL_GPIO_vInitOutput(BSP_GNSS_ON_PORT, BSP_GNSS_ON_PIN, GPIO_PIN_RESET);

    /* Chip selects, idle high (deselected).
     *
     * ACC_CS (PA11) is driven on both roles: the role is not known yet, and on
     * a primary - where no accelerometer is fitted and the line has no
     * external pull - driving an empty footprint high costs nothing.
     *
     * FLASH_CS (PA15) is configured here and only here; afterwards only the
     * select/deselect macros in hal_spi.h write its level. It resets as AF0 JTDI with
     * a weak internal pull-up, which used to be all that held it until the SPI
     * driver came up. The SPI MSP init no longer touches it, so the lazy SPI
     * re-init after a STOP2 wake never re-initialises it, and STOP2 retains the
     * driven high level. */
    HAL_GPIO_vInitOutput(BSP_ACC_CS_PORT,   BSP_ACC_CS_PIN,   GPIO_PIN_SET);
    HAL_GPIO_vInitOutput(BSP_FLASH_CS_PORT, BSP_FLASH_CS_PIN, GPIO_PIN_SET);

    /* Flash SCK (PA8) and MOSI (PA10), driven low. This is the same state the
     * STOP2 park leaves them in (both SPI buses are CPOL = 0, so low is the
     * clock's idle level), and it gives the flash - fitted on both roles - quiet,
     * defined inputs from here until HAL_SPI_FLASH_vInit() switches the two
     * pins to their SPI alternate function. Flash MISO (PA5) stays at its reset
     * analog state until then.
     *
     * The accelerometer's SPI pins (PA1/PA6/PA7) are not touched: the role is
     * not known yet, and on a primary they must stay at reset analog because
     * nothing is fitted behind them. */
    HAL_GPIO_vInitOutput(BSP_FLASH_SCK_PORT,  BSP_FLASH_SCK_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_vInitOutput(BSP_FLASH_MOSI_PORT, BSP_FLASH_MOSI_PIN, GPIO_PIN_RESET);

    /* Battery divider bias (PA4), off. Load-switch enable, active high, no
     * external pull. BAT_MEAS_VOLT (PA12) resets analog and is left to
     * hal_adc. */
    HAL_GPIO_vInitOutput(BSP_BAT_BIAS_ENABLE_PORT, BSP_BAT_BIAS_ENABLE_PIN, GPIO_PIN_RESET);

    /* PB2 and PA9, unconnected on both final boards. They drove a GPIO-powered
     * RF switch on the pre-production board; the final layout has no switch
     * (see the RF front-end note in hal_bsp.h). Both reset analog already, so
     * this only makes the state explicit and clears any pull. */
    HAL_GPIO_vInitAnalogNoPull(BSP_UNUSED_PB2_PORT, BSP_UNUSED_PB2_PIN);
    HAL_GPIO_vInitAnalogNoPull(BSP_UNUSED_PA9_PORT, BSP_UNUSED_PA9_PIN);

    /* VSOLAR_MEAS (PB3) resets as AF0 JTDO/TRACESWO, which leaves a digital
     * input stage sitting on the solar divider voltage until hal_adc takes the
     * pin over. Park it analog with no pull now. */
    HAL_GPIO_vInitAnalogNoPull(BSP_VSOLAR_MEAS_PORT, BSP_VSOLAR_MEAS_PIN);

    /* Role strap (PB12): input with NO internal pull. The board's 10k external
     * pull-down defines LOW and a primary straps the pin HIGH, so an internal
     * pull could only fight the strap. DEVICE_DISCOVERY_vConfigDeviceRole()
     * reads it once and then parks it analog with the pull cleared. */
    HAL_GPIO_vInitInput(BSP_ROLE_BIT0_PORT, BSP_ROLE_BIT0_PIN, GPIO_NOPULL);

    /* PC13 = ACC_INT, routed from the LIS2DH INT1 but unused (and undriven on a
     * primary, where no accelerometer is fitted). Parked analog with no pull on
     * both roles, once, here. It is deliberately not in the STOP2 park mask, so
     * a future EXTI use is not silently undone at every sleep; see hal_bsp.h for
     * what using it requires. */
    HAL_GPIO_vInitAnalogNoPull(BSP_ACC_INT_PORT, BSP_ACC_INT_PIN);

    /* Never touched, here or anywhere else: PA13/PA14 (SWD), PC14/PC15 (LSE
     * crystal), PH3 (BOOT0) and PB0 (radio TCXO supply). See hal_bsp.h. */
}

/* STOP2 park masks: the UART alternate-function pins. Left as AF through
 * STOP2 they would sit as undriven inputs or idle outputs against an external
 * device that may itself be unpowered, so they are parked analog with their
 * pulls cleared (HAL_GPIO_vInitAnalogNoPull) before sleep. Nothing re-arms
 * them on the wake path: each owner re-applies its pin configuration through
 * HAL_UART_vEnable() on its next use (DEBUG_vInit for the debug UART, GPS.c or
 * Farmranger.c for USART1). USART2 = debug (PA2/PA3), USART1 = GPS/Farmranger
 * (PB6/PB7).
 *
 * The SPI pins are NOT in these masks. HAL_SPI_vOnSleep() (hal_spi.c) parks
 * them, called from HAL_SYSTEM_vEnterStop2() immediately before this file's
 * HAL_GPIO_vOnSleep() while the port clocks are still on, and only for a bus
 * that has actually been started: SCK and then MOSI are driven low (the
 * CPOL = 0 idle level) and MISO goes analog. A plain analog park would leave
 * SCK/MOSI floating into the LIS2DH's I2C inputs, which are live while its
 * chip select is high.
 *
 * NOT parked, and intentionally held across STOP2 to keep externals in a known
 * state (GPIO state is retained in STOP2, so driven pins keep their level):
 * PA0 (GNSS_ON on a secondary / fr9 wake on a primary) at whatever level its
 * owner drives, PA4 BAT_BIAS low, PA11 ACC_CS high, PA15 FLASH_CS high, and
 * the PB4/PB5/PB8 LEDs.
 *
 * Parked once and never re-parked: PB12 (ROLE_BIT0, parked analog by the role
 * read at boot), PB2, PA9 and PC13 (parked by HAL_GPIO_vInit), and the ADC inputs
 * PA12/PB3 (analog). Nothing re-drives any of them, so they need no entry
 * here. PC13 in particular must stay out of these masks if ACC_INT is ever used
 * as an EXTI wake source: re-parking it at every STOP2 entry would silently
 * kill that wake.
 *
 * PA13/PA14 (SWD) are deliberately absent: they stay in their reset debug
 * function at all times, STOP2 included.
 */
#define HAL_GPIO_SLEEP_ANALOG_GPIOA  (BSP_DEBUG_UART_TX_PIN | BSP_DEBUG_UART_RX_PIN)
#define HAL_GPIO_SLEEP_ANALOG_GPIOB  (BSP_GPS_UART_TX_PIN | BSP_GPS_UART_RX_PIN)

void HAL_GPIO_vOnSleep(void)
{
    /* Park the UART AF pins as analog BEFORE gating the port clocks (register
     * writes need the clock on). The SPI pins were already parked by
     * HAL_SPI_vOnSleep(), which the caller runs just before this function. */
    HAL_GPIO_vInitAnalogNoPull(GPIOA, HAL_GPIO_SLEEP_ANALOG_GPIOA);
    HAL_GPIO_vInitAnalogNoPull(GPIOB, HAL_GPIO_SLEEP_ANALOG_GPIOB);

    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
}

void HAL_GPIO_OnWake(void)
{
    /* Only the GPIO port clocks are restored here — the minimum the idle 1 Hz
     * wake needs (the LED indicator, and the lazy SPI restore that may
     * follow). No peripheral pins are re-armed on the wake path: a module that
     * needs the debug UART, SPI or ADC brings it up itself when it acquires the
     * sleep lock (DEBUG_vInit / the SPI select chokepoints / HAL_ADC_vLock), so
     * an idle wake that uses none of them pays nothing.
     *
     * The debug UART pins in particular are NOT re-armed here: while a sleep
     * lock is held the device stays on the LIGHT WFI path (no STOP2, no pin
     * parking), so DEBUG_vInit's pin setup survives the whole active window;
     * and during idle the 1 Hz UART marker is suppressed, so the pins are not
     * needed. USART1 (GPS/Farmranger) and ADC pins are likewise left to their
     * owners. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
}

bool HAL_GPIO_bLedsAllowed(void)
{
    return bLedsAllowed;
}

void HAL_GPIO_vAllowLeds(bool allow)
{
    bLedsAllowed = allow;
}

void HAL_GPIO_vInitIntPullup(GPIO_TypeDef *GPIOx, uint32_t gpio_pin)
{
    GPIO_InitTypeDef gpio;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);
}

void HAL_GPIO_vInitOutput(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, GPIO_PinState gpio_pinstate)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Pin   = gpio_pin;

    /* Level first, mode second. HAL_GPIO_WritePin() latches the output level
     * in ODR through BSRR/BRR, so when HAL_GPIO_Init() then switches MODER to
     * output the pin starts out driving the requested level, instead of
     * driving whatever ODR already held (0 after reset) for the few cycles
     * until a later write. With the old order that gap put a short low pulse
     * on both chip selects (PA11, PA15) at every boot.
     *
     * The port clock must already be enabled: with it off the BSRR write is
     * silently lost and the pin comes up at the stale ODR level. Every caller
     * today runs with the clock on (HAL_GPIO_vInit enables it first, and the
     * STOP2 SPI park runs before HAL_GPIO_vOnSleep gates it). */
    HAL_GPIO_WritePin(GPIOx, gpio_pin, gpio_pinstate);
    HAL_GPIO_Init(GPIOx, &gpio);
}

void HAL_GPIO_vInitInput(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, uint32_t gpio_pull)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = gpio_pull;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);
}

void HAL_GPIO_vInitAnalogNoPull(GPIO_TypeDef *GPIOx, uint32_t gpio_pin)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Pin   = gpio_pin;
    HAL_GPIO_Init(GPIOx, &gpio);

    /* The vendor HAL_GPIO_Init() does not write PUPDR at all when the mode is
     * analog (Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_gpio.c skips its
     * pull-up/pull-down step for MODE_ANALOG), so the GPIO_NOPULL above is
     * ignored and the pin's previous PUPDR value (a UART pull-up, PB4's NJTRST
     * reset pull-up, a strap input's pull) stays in the register. The
     * reference manual says the weak pulls are disabled by hardware while
     * MODER is analog, and lists analog with a non-zero PUPDR as a reserved
     * configuration. Clearing the field keeps the register in the documented
     * analog state, matches what this function's name promises, and stops a
     * stale pull from coming back live the moment the pin is next switched to
     * input, output or AF. This is register hygiene, not a measured STOP2
     * current fix.
     *
     * So clear the 2-bit PUPDR field of every pin in the mask explicitly, in a
     * single read-modify-write, after MODER is already analog (the input buffer
     * is off by then, so there is no window with a floating digital input).
     * HAL_GPIO_DeInit() would also clear PUPDR, but it is not used here because
     * it additionally clears the EXTI configuration of any line whose EXTICR
     * selects this port - and EXTICR = 0, the reset value, selects port A.
     *
     * The read-modify-write is not atomic, exactly like HAL_GPIO_Init()'s own.
     * That is fine for the callers: the boot path, the init task's one-off role
     * read, and STOP2 entry with interrupts masked; no ISR reconfigures GPIO. */
    uint32_t u32PupdMask = 0U;
    for (uint32_t u32Pos = 0U; u32Pos < 16U; u32Pos++)
    {
        if ((gpio_pin & (1UL << u32Pos)) != 0U)
        {
            u32PupdMask |= (GPIO_PUPDR_PUPD0 << (u32Pos * 2U));
        }
    }
    GPIOx->PUPDR &= ~u32PupdMask;
}
