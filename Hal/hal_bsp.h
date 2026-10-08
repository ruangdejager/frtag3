/*
 * hal_bsp.h
 *
 * Board support definitions for the Tagskaap V3 RevA board (frtag3):
 * STM32WLE5CCU6 in UFQFPN48, VDD = VDDA = 2.0 V (frtag2 ran at 1.8 V).
 *
 * This file is the pin map of record. frtag3.ioc and Core/Inc/main.h carry the
 * same net names for reference only: Core/Src/main.c is not built, so nothing
 * generated from the .ioc ever configures a pin, and the .ioc can only describe
 * one population (it records the fully populated SECONDARY).
 *
 * What is fitted depends on the device role (ROLE_BIT0, PB12):
 *
 *   PRIMARY    No GNSS module and no accelerometer. USART1 (PB6/PB7) is the
 *              Farmranger link to the fr9 board, PA0 is the fr9 wake line, and
 *              SPI1 (PA1/PA6/PA7) is never started.
 *   SECONDARY  GNSS module and LIS2DH accelerometer fitted. USART1 talks to the
 *              GNSS receiver, PA0 enables the GNSS LDO, SPI1 runs the
 *              accelerometer.
 *
 * The external NOR flash (SPI2) is fitted on both roles.
 *
 * The role is not known until DEVICE_DISCOVERY_vConfigDeviceRole() reads the
 * strap, so HAL_GPIO_vInit() - which runs from main() before the clock tree is
 * up - only drives pins whose safe state is the same on both roles. PA0, PA4,
 * PB2, PA9 and PA11 have no external pull on this board: each one floats from
 * reset until firmware drives it, which is why that safe state is set before
 * SystemClock_Config()'s LSE start (up to LSE_STARTUP_TIMEOUT, 5 s, when the
 * image runs without the bootloader; behind the bootloader the LSE is already
 * running) rather than from the init task.
 */

#ifndef HAL_BSP_H_
#define HAL_BSP_H_

#include "stm32wlxx_hal.h"
#include "build_config.h"   /* ENABLE_LEDS */

/* -----------------------------------------------------------------------
 * LED
 *
 * PB5 = red, PB8 = green. PB8 carried the yellow LED on frtag2; on frtag3 the
 * part is green but the pin and its job are unchanged, so LED_GREEN keeps the
 * enum value (1) that LED_YELLOW had. Both LEDs are active high (pin high =
 * LED on).
 * ----------------------------------------------------------------------- */
#define BSP_LED_RED_PORT        GPIOB
#define BSP_LED_RED_PIN         GPIO_PIN_5
#define BSP_LED_GREEN_PORT      GPIOB
#define BSP_LED_GREEN_PIN       GPIO_PIN_8

typedef enum {
    LED_RED    = 0,
    LED_GREEN  = 1,
} Led_TypeDef;

static inline void BSP_LED_Init(Led_TypeDef led)
{
#ifdef ENABLE_LEDS
    /* Clock, then level, then mode. The BSRR write that latches the output
     * level is silently lost while the port clock is off, so the clock comes
     * first. Writing the level before HAL_GPIO_Init switches MODER to output
     * means the pin starts driving the level asked for, instead of the stale
     * ODR value (0 after reset) for the few cycles until the write. Both LEDs
     * come up lit here on purpose: that is the boot indicator. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    if (led == LED_RED)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        HAL_GPIO_WritePin(BSP_LED_RED_PORT, BSP_LED_RED_PIN, GPIO_PIN_SET);
        gpio.Pin = BSP_LED_RED_PIN;
        HAL_GPIO_Init(BSP_LED_RED_PORT, &gpio);
    }
    else
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        HAL_GPIO_WritePin(BSP_LED_GREEN_PORT, BSP_LED_GREEN_PIN, GPIO_PIN_SET);
        gpio.Pin = BSP_LED_GREEN_PIN;
        HAL_GPIO_Init(BSP_LED_GREEN_PORT, &gpio);
    }
#else
    /* LEDs terminated: park both pins analog (reset state) regardless of which
     * LED was requested, so they draw nothing and are never driven. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = BSP_LED_RED_PIN | BSP_LED_GREEN_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);
    (void)led;
#endif
}

static inline void BSP_LED_On(Led_TypeDef led)
{
#ifdef ENABLE_LEDS
    if (led == LED_RED)
        HAL_GPIO_WritePin(BSP_LED_RED_PORT, BSP_LED_RED_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(BSP_LED_GREEN_PORT, BSP_LED_GREEN_PIN, GPIO_PIN_SET);
#else
    (void)led;
#endif
}

static inline void BSP_LED_Off(Led_TypeDef led)
{
#ifdef ENABLE_LEDS
    if (led == LED_RED)
        HAL_GPIO_WritePin(BSP_LED_RED_PORT, BSP_LED_RED_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(BSP_LED_GREEN_PORT, BSP_LED_GREEN_PIN, GPIO_PIN_RESET);
#else
    (void)led;
#endif
}

static inline void BSP_LED_Toggle(Led_TypeDef led)
{
#ifdef ENABLE_LEDS
    if (led == LED_RED)
        HAL_GPIO_TogglePin(BSP_LED_RED_PORT, BSP_LED_RED_PIN);
    else
        HAL_GPIO_TogglePin(BSP_LED_GREEN_PORT, BSP_LED_GREEN_PIN);
#else
    (void)led;
#endif
}

/* -----------------------------------------------------------------------
 * Debug UART (USART2) — PA2/PA3
 * ----------------------------------------------------------------------- */
#define BSP_DEBUG_USART_INSTANCE    USART2
#define BSP_DEBUG_USART_IRQn        USART2_IRQn
#define BSP_DEBUG_UART_TX_PORT      GPIOA
#define BSP_DEBUG_UART_TX_PIN       GPIO_PIN_2
#define BSP_DEBUG_UART_RX_PORT      GPIOA
#define BSP_DEBUG_UART_RX_PIN       GPIO_PIN_3
#define BSP_DEBUG_UART_AF           GPIO_AF7_USART2
#define BSP_DEBUG_UART_BAUD         115200

/* -----------------------------------------------------------------------
 * GPS / Farmranger UART (USART1, AF7) — PB6 = USART1_TX, PB7 = USART1_RX
 *
 * One peripheral, two mutually exclusive users, chosen by ROLE_BIT0:
 *   SECONDARY  the GNSS module. The net names are the module's view, so
 *              GNSS_RX (PB6) is the MCU's TX and GNSS_TX (PB7) the MCU's RX.
 *   PRIMARY    the Farmranger link to the fr9 board (no GNSS fitted).
 *
 * The pin mapping is the standard one on BOTH roles: the firmware no longer
 * sets the USART1 TX/RX SWAP bit. frtag2 swapped TX and RX inside the MCU on
 * a primary to undo crossed wiring towards the fr9; that crossing is being
 * fixed on the fr9 side instead. A frtag3 primary therefore needs an fr9 that
 * carries that fix - against an unfixed fr9 the two TX lines face each other,
 * the fr9 never answers RDY, and every logger session fails. See
 * HAL_UART_vInit() in hal_uart.c.
 *
 * What does differ per role is the baud (HAL_UART_vInit) and the pin
 * electricals (HAL_UART_vEnable), both defined below.
 * ----------------------------------------------------------------------- */
#define BSP_GPS_USART_INSTANCE      USART1
#define BSP_GPS_USART_IRQn          USART1_IRQn
#define BSP_GPS_UART_TX_PORT        GPIOB
#define BSP_GPS_UART_TX_PIN         GPIO_PIN_6
#define BSP_GPS_UART_RX_PORT        GPIOB
#define BSP_GPS_UART_RX_PIN         GPIO_PIN_7
#define BSP_GPS_UART_AF             GPIO_AF7_USART1

/* GNSS receiver factory default baud. The firmware never changes it: the
 * cold-start UBX-CFG-VALSET set in GPS.c carries no CFG-UART1-BAUDRATE key,
 * so the receiver and this MCU stay at 9600 for the life of every session. */
#define BSP_GPS_UART_BAUD           9600

/* On a PRIMARY, USART1 is repurposed as the Farmranger link to the fr9 board.
 * That board's frtag UART runs at a fixed 115200, so the link must match -
 * the GPS default (9600) above only applies to a SECONDARY running GNSS. */
#define BSP_FARMRANGER_UART_BAUD    115200

/* USART1 pin electricals, selected per role by HAL_UART_vEnable(). The role is
 * latched by HAL_UART_vSetRole() at boot, before USART1 is first enabled, and
 * the latch defaults to the SECONDARY (GNSS-safe) choice. Neither role puts an
 * MCU pull on PB6.
 *
 * SECONDARY, PB6 (MCU TX -> GNSS_RX): open-drain. The GNSS module's I/O runs at
 * 1.8 V with a 1.98 V absolute maximum, while this MCU's VDD is 2.0 V, so the
 * MCU must never drive the line to its own rail. It only ever pulls PB6 low;
 * R18 (4.7k to GNSS_VCC) supplies the high level at the module's own voltage,
 * and with the module unpowered nothing is pushed into it. */
#define BSP_GPS_UART_TX_MODE_GNSS   GPIO_MODE_AF_OD

/* PRIMARY, PB6 (MCU TX -> fr9 RX): push-pull. No GNSS module is fitted, so
 * nothing on the net is limited to 1.8 V, and driving both levels keeps the
 * link independent of whether R18 and the GNSS LDO happen to be populated on
 * a primary. */
#define BSP_GPS_UART_TX_MODE_FR9    GPIO_MODE_AF_PP

/* SECONDARY, PB7 (GNSS_TX -> MCU RX): no MCU pull. The module drives the line
 * whenever it is powered; an internal pull-up would lift a tri-stated or
 * unpowered module TXD to 2.0 V, above its 1.98 V absolute maximum. The price
 * is that RX floats while the module is off, which is why the GNSS self-test
 * in GPS.c looks for NMEA '$' sentence starts rather than trusting a raw byte
 * count that line noise could satisfy. */
#define BSP_GPS_UART_RX_PULL_GNSS   GPIO_NOPULL

/* PRIMARY, PB7 (fr9 TX -> MCU RX): internal pull-up, so the receiver idles
 * high (no spurious start bits or breaks) whenever the fr9 is not driving. */
#define BSP_GPS_UART_RX_PULL_FR9    GPIO_PULLUP

/* -----------------------------------------------------------------------
 * PA0 — role-dependent OUTPUT, active high, no external pull
 *
 * SECONDARY  GNSS_ON: the enable of the LDO that supplies GNSS_VCC (a single
 *            pin; the original frtag needed three for GPS power).
 * PRIMARY    No GNSS is fitted; the same pin is the fr9 wake line. It is an
 *            output from this MCU, not an interrupt input: the fr9 opens a
 *            Farmranger session on its rising edge, after a ~20 ms debounce.
 *
 * With no external pull the pin floats from reset until firmware drives it,
 * which a primary's fr9 could take for a session request and which leaves a
 * secondary's LDO enable undefined. HAL_GPIO_vInit() therefore drives it low
 * once, early in main(), and only its owner raises it afterwards (GPS.c on a
 * secondary, Farmranger.c on a primary).
 *
 * BSP_GNSS_ON_* is the single physical definition. BSP_FR_WAKE_* are aliases
 * of it, so the two names can never drift onto different pins.
 * ----------------------------------------------------------------------- */
#define BSP_GNSS_ON_PORT            GPIOA
#define BSP_GNSS_ON_PIN             GPIO_PIN_0
#define BSP_FR_WAKE_PORT            BSP_GNSS_ON_PORT
#define BSP_FR_WAKE_PIN             BSP_GNSS_ON_PIN

/* -----------------------------------------------------------------------
 * Accelerometer SPI (SPI1, AF5) — PA1 SCK, PA6 MISO (ACC_SO),
 * PA7 MOSI (ACC_SI), PA11 CS
 *
 * The LIS2DH is fitted on a SECONDARY only. A PRIMARY has no accelerometer,
 * so SPI1 is never started there (init.c calls HAL_SPI_ACC_vInit() on the
 * secondary branch only) and PA1/PA6/PA7 stay in their reset analog state.
 *
 * SPI mode 0 (CPOL = 0, CPHA = 0) at 375 kHz. CS (PA11) has no external pull;
 * HAL_GPIO_vInit() drives it high on both roles because the role is not known
 * yet at that point. On a primary that drives an empty footprint, which costs
 * nothing.
 *
 * While CS is high the LIS2DH's I2C interface is live on the same pins (SCK is
 * its SCL, MOSI its SDA), so leaving SCK/MOSI floating through STOP2 would
 * hand the part undefined I2C inputs. HAL_SPI_vOnSleep() (hal_spi.c) therefore
 * drives SCK low and then MOSI low before STOP2 - SCK first, because SDA
 * falling while SCL is high is an I2C START - and parks MISO analog.
 * ----------------------------------------------------------------------- */
#define BSP_ACC_CS_PIN              GPIO_PIN_11
#define BSP_ACC_CS_PORT             GPIOA
#define BSP_ACC_SCK_PIN             GPIO_PIN_1
#define BSP_ACC_SCK_PORT            GPIOA
#define BSP_ACC_MISO_PIN            GPIO_PIN_6
#define BSP_ACC_MISO_PORT           GPIOA
#define BSP_ACC_MOSI_PIN            GPIO_PIN_7
#define BSP_ACC_MOSI_PORT           GPIOA

/* PC13 = ACC_INT, routed from the LIS2DH INT1 output but unused; it is kept on
 * the board in case it is needed later. HAL_GPIO_vInit() parks it analog with
 * its pull cleared on both roles (on a primary nothing drives it at all), and
 * it is deliberately not in any STOP2 park mask. To use it: configure it as an
 * EXTI input with no pull, keep it out of every STOP2 park (re-parking it at
 * each sleep would silently kill the wake), and add an EXTI15_10_IRQHandler
 * first - the startup file maps that vector to Default_Handler, an infinite
 * loop, so arming the line without a handler hangs the MCU. */
#define BSP_ACC_INT_PIN             GPIO_PIN_13
#define BSP_ACC_INT_PORT            GPIOC

/* -----------------------------------------------------------------------
 * Battery measurement (ADC)
 *
 * BAT_MEAS_VOLT = PA12 = ADC_IN8, through a 33k (top) / 10k (bottom) divider,
 * ratio 4.3. The divider is switched by BAT_MEAS_BIAS = PA4, a load-switch
 * enable that is active high with no external pull: HAL_GPIO_vInit() drives
 * it low and it is raised only for the duration of a sample.
 *
 * The ADC reference is VDDA, 2.0 V nominal on this board. The firmware does
 * not rely on the nominal value: it measures VDDA through VREFINT on every
 * sample. Full scale at the battery is therefore about 4.3 x 2.0 V = 8.6 V.
 * ----------------------------------------------------------------------- */
#define BSP_BAT_BIAS_ENABLE_PORT        GPIOA
#define BSP_BAT_BIAS_ENABLE_PIN         GPIO_PIN_4
#define BSP_BAT_MEAS_VOLT_PORT          GPIOA
#define BSP_BAT_MEAS_VOLT_PIN           GPIO_PIN_12
#define BSP_BAT_ADC_VOLTAGE_CHANNEL     ADC_CHANNEL_8

/* -----------------------------------------------------------------------
 * External Flash SPI (SPI2) — PA5/PA8/PA10/PA15
 * AT25XE041B-MAHN-T, 512 KB NOR flash, fitted on both roles.
 *
 * CS (PA15) is configured as an output, initially high, by HAL_GPIO_vInit()
 * and nowhere else; after that only HAL_SPI_FLASH_vSelect()/vDeselect() in
 * hal_spi.h write its level. The SPI MSP init does not touch it, so a lazy re-init after STOP2 never glitches it,
 * and STOP2 retains the driven level. SCK (PA8) and MOSI (PA10) are driven low
 * at boot until HAL_SPI_FLASH_vInit() switches them to their alternate
 * function.
 * ----------------------------------------------------------------------- */
#define BSP_FLASH_MISO_PORT         GPIOA
#define BSP_FLASH_MISO_PIN          GPIO_PIN_5
#define BSP_FLASH_SCK_PORT          GPIOA
#define BSP_FLASH_SCK_PIN           GPIO_PIN_8
#define BSP_FLASH_MOSI_PORT         GPIOA
#define BSP_FLASH_MOSI_PIN          GPIO_PIN_10
#define BSP_FLASH_CS_PORT           GPIOA
#define BSP_FLASH_CS_PIN            GPIO_PIN_15

/* -----------------------------------------------------------------------
 * Solar panel voltage measurement (ADC)
 *
 * VSOLAR_MEAS = PB3 = ADC_IN2, through a 33k (top) / 10k (bottom) divider,
 * ratio 4.3. Same reference as the battery: VDDA, 2.0 V nominal, measured
 * through VREFINT on every sample, so full scale is about 8.6 V.
 *
 * PB3 resets as AF0 (JTDO/TRACESWO), which leaves a digital input stage
 * sitting on the divider voltage. HAL_GPIO_vInit() parks it analog with no
 * pull straight away, before the ADC driver takes it over.
 * ----------------------------------------------------------------------- */
#define BSP_VSOLAR_MEAS_PORT            GPIOB
#define BSP_VSOLAR_MEAS_PIN             GPIO_PIN_3
#define BSP_VSOLAR_ADC_CHANNEL          ADC_CHANNEL_2

/* -----------------------------------------------------------------------
 * RF switch — supply rail (PB2) and path select (PA9)
 *
 * RF_SW_VDD (PB2, HIGH = switch powered) gates the switch's own supply and
 * follows the RADIO's power state, not individual TX/RX operations: asserted
 * once when the SubGHz radio leaves sleep, held across every RX<->TX
 * turnaround, de-asserted once when the radio goes back to sleep. The switch
 * draws ~35 uA, nothing beside the radio's mA while awake, and cycling it per
 * turnaround would add the settle below to every transition for no gain.
 *
 * RF_SW_CTRL (PA9, HIGH = TX/LP, LOW = RX/OFF) selects the path and is only
 * ever driven while RF_SW_VDD is already high.
 *
 * Bring-up and teardown order is mandatory, see HAL_SUBGHZ_vRfSwitchPowerOn /
 * _vRfSwitchPowerOff: CTRL must never be high while VDD is at 0 V.
 *
 * Settle times come from the board's RC networks:
 *   VDD:  470R x ~100 nF, tau = 47 us, allow 7 tau
 *   CTRL: 100R x 101 nF,  ~50 us
 * ----------------------------------------------------------------------- */
#define BSP_RF_SW_VDD_PORT          GPIOB
#define BSP_RF_SW_VDD_PIN           GPIO_PIN_2
#define BSP_RF_SW_VDD_SETTLE_US     350U

#define BSP_RF_SW_CTRL_PORT         GPIOA
#define BSP_RF_SW_CTRL_PIN          GPIO_PIN_9
#define BSP_RF_SW_CTRL_SETTLE_US    50U

/* -----------------------------------------------------------------------
 * Device role sense — PB12 ROLE_BIT0 (primary = HIGH / secondary = LOW)
 *
 * A 10k external pull-down on the board sets LOW; a primary straps the pin
 * HIGH. HAL_GPIO_vInit() configures it as an input with NO internal pull (the
 * board defines the level, and an internal pull would only fight the strap).
 * DEVICE_DISCOVERY_vConfigDeviceRole() reads it ONCE, then parks it analog
 * with the pull cleared so its input stage draws nothing for the rest of the
 * device's life. Only ROLE_BIT0 exists; VER_BIT1/VER_BIT2 were removed in
 * frtag2.
 * ----------------------------------------------------------------------- */
#define BSP_ROLE_BIT0_PORT          GPIOB
#define BSP_ROLE_BIT0_PIN           GPIO_PIN_12

/* -----------------------------------------------------------------------
 * Pins that are not application GPIOs
 *
 * None of these is configured by the GPIO code, and none may ever be added to
 * a STOP2 park mask or to any "set every pin analog" loop:
 *
 * PC14/PC15  OSC32_IN/OSC32_OUT: the 32.768 kHz LSE crystal, fitted. The RTC
 *            runs from it and all timekeeping depends on it - the calendar,
 *            the 1 Hz heartbeat wake and the STOP2 tick reconciliation in
  *            vPortSuppressTicksAndSleep(). The frtag2 bootloader starts it
 *            first, in its own SystemClock_Config() (an LSE failure hangs the
 *            bootloader, before this image runs); the LSE keeps running across
 *            the jump and is re-asserted by
 *            SystemClock_Config() (hal_system.c) with RCC_LSEDRIVE_LOW; if it
 *            never starts, HAL_RCC_OscConfig() gives up after
 *            LSE_STARTUP_TIMEOUT (5 s) and boot hangs in Error_Handler().
 * PB0        VDD_TCXO: the supply output of the sub-GHz radio's TCXO
 *            regulator, feeding the 32 MHz TCXO (NDK END4263D-32M). Its level
 *            is TCXO_CTRL_VOLTAGE in radio_conf.h. Not a GPIO.
 * PA13/PA14  SWDIO/SWCLK: left in their reset debug function at all times,
 *            STOP2 included, so firmware never cuts SWD off. (Debugging across
 *            STOP2 still needs the debugger to set DBGMCU DBG_STOP.)
 * PH3        BOOT0: external pull-down (boot from main flash). Never touched.
 *
 * PB4 is a real GPIO but is unused on this board. Its reset state is AF0
 * NJTRST with the internal pull-up on, which would sit there forever;
 * HAL_GPIO_vInit() parks it analog and clears that pull.
 * ----------------------------------------------------------------------- */
#define BSP_UNUSED_PB4_PORT         GPIOB
#define BSP_UNUSED_PB4_PIN          GPIO_PIN_4

#endif /* HAL_BSP_H_ */
