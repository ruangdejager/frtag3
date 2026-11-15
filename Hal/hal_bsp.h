/*
 * hal_bsp.h
 *
 * Board support definitions for the Tagskaap V3 RevA boards (frtag3):
 * STM32WLE5CCU6 in UFQFPN48. Two physical boards share this firmware and the
 * same MCU pinout core, and the role strap (ROLE_BIT0, PB12) tells them apart:
 *
 *   SECONDARY  "Tagskaap3RevA" - the collar tag. VDD = VDDA = 1.8 V from a
 *              TPS62740 buck with VSEL4..1 all strapped low (frtag2 also ran
 *              at 1.8 V; an earlier frtag3 prototype ran at 2.0 V). Battery
 *              charger from the solar input, battery and solar measurement,
 *              GNSS, accelerometer, RGB status LED.
 *   PRIMARY    "Tagskaap3RevA_DB" - the daughter board that plugs into the
 *              fr9. VDD = VDDA = 3.0 V from its own TPS62740 with VSEL4/VSEL3
 *              high and VSEL2/VSEL1 low. Its VBAT and the buck's enable
 *              (RADIO_DB_ON) both come from the fr9 over J5, so the fr9 can
 *              power the whole board down. No charger, no battery or solar
 *              measurement, no GNSS, no accelerometer; three discrete LEDs.
 *
 * Firmware must therefore never assume one rail voltage for both roles. The
 * ADC measures VDDA through VREFINT on every sample, so the measurement path
 * does not care; the radio's TCXO supply does (see radio_conf.h).
 *
 * This file is the pin map of record. frtag3.ioc and Core/Inc/main.h carry the
 * same net names for reference only: Core/Src/main.c is not built, so nothing
 * generated from the .ioc ever configures a pin, and the .ioc can only describe
 * one population (it records the fully populated SECONDARY).
 *
 * What is fitted depends on the device role (ROLE_BIT0, PB12):
 *
 *   PRIMARY    No GNSS module and no accelerometer. USART1 (PB6/PB7) is the
 *              Farmranger link to the fr9 board (nets FR_RX/FR_TX, named from
 *              the fr9's side), PA0 is the fr9 wake line (FR_AT), and SPI1
 *              (PA1/PA6/PA7) is never started. PA1, PA4, PA6, PA7, PA9, PA11,
 *              PA12, PB2, PB3 and PC13 are unconnected on this board.
 *   SECONDARY  GNSS module and LIS2DH accelerometer fitted. USART1 talks to the
 *              GNSS receiver, PA0 enables the GNSS LDO, SPI1 runs the
 *              accelerometer. PA9 and PB2 are unconnected.
 *
 * Both roles: external NOR flash on SPI2, 32.768 kHz LSE crystal, the same
 * TCXO, and the same switchless RF front end (RFO_LP only).
 *
 * The role is not known until DEVICE_DISCOVERY_vConfigDeviceRole() reads the
 * strap, so HAL_GPIO_vInit() - which runs from main() before the clock tree is
 * up - only drives pins whose safe state is the same on both roles. PA0, PA4
 * and PA11 have no external pull on either board: each one floats from
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
 * Three LED pins on both roles, all active high (pin high = LED on):
 *
 *   PB5  red      both roles
 *   PB8  green    both roles (PB8 carried the yellow LED on frtag2; LED_GREEN
 *                 keeps the enum value (1) that LED_YELLOW had)
 *   PB4  LED_AUX  BLUE on a secondary, YELLOW on a primary
 *
 * The drive differs per board but not the polarity. On a SECONDARY the three
 * are the dies of one RGB package (IN-P55TATRGB) whose anodes sit on VBAT
 * through series resistors, each cathode switched by an N-MOSFET (DMG1012T)
 * whose gate is the MCU pin, with a 100k gate pull-down. On a PRIMARY they are
 * three discrete LEDs driven straight from the pin through a series resistor
 * to ground. Either way, low or analog = off.
 *
 * LED_AUX was an unused pin (PB4, parked analog) until the final layout put an
 * LED on it. Nothing in the application lights it yet; it is initialised off,
 * is not part of the boot indicator, and is available through the same
 * BSP_LED_* calls as the other two.
 * ----------------------------------------------------------------------- */
#define BSP_LED_RED_PORT        GPIOB
#define BSP_LED_RED_PIN         GPIO_PIN_5
#define BSP_LED_GREEN_PORT      GPIOB
#define BSP_LED_GREEN_PIN       GPIO_PIN_8
#define BSP_LED_AUX_PORT        GPIOB
#define BSP_LED_AUX_PIN         GPIO_PIN_4

typedef enum {
    LED_RED    = 0,
    LED_GREEN  = 1,
    LED_AUX    = 2,   /* blue (secondary) / yellow (primary) */
} Led_TypeDef;

/* All three LEDs are on GPIOB; this maps an LED to its pin. */
static inline uint16_t BSP_LED_u16Pin(Led_TypeDef led)
{
    switch (led)
    {
        case LED_RED:   return BSP_LED_RED_PIN;
        case LED_GREEN: return BSP_LED_GREEN_PIN;
        default:        return BSP_LED_AUX_PIN;
    }
}

static inline void BSP_LED_Init(Led_TypeDef led)
{
#ifdef ENABLE_LEDS
    /* Clock, then level, then mode. The BSRR write that latches the output
     * level is silently lost while the port clock is off, so the clock comes
     * first. Writing the level before HAL_GPIO_Init switches MODER to output
     * means the pin starts driving the level asked for, instead of the stale
     * ODR value (0 after reset) for the few cycles until the write. Red and
     * green come up lit here on purpose: that is the boot indicator. LED_AUX
     * comes up off. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = BSP_LED_u16Pin(led);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, (uint16_t)gpio.Pin,
                      (led == LED_AUX) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOB, &gpio);
#else
    /* LEDs terminated: park every LED pin analog (reset state) regardless of
     * which LED was requested, so they draw nothing and are never driven. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = BSP_LED_RED_PIN | BSP_LED_GREEN_PIN | BSP_LED_AUX_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);
    /* PB4 resets as AF0 NJTRST with its internal pull-up on, and the vendor
     * HAL_GPIO_Init() leaves PUPDR alone in analog mode; clear it so the pull
     * neither lights a primary's yellow LED faintly nor comes back live if the
     * pin is ever re-configured. */
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD8);
    (void)led;
#endif
}

static inline void BSP_LED_On(Led_TypeDef led)
{
#ifdef ENABLE_LEDS
    HAL_GPIO_WritePin(GPIOB, BSP_LED_u16Pin(led), GPIO_PIN_SET);
#else
    (void)led;
#endif
}

static inline void BSP_LED_Off(Led_TypeDef led)
{
#ifdef ENABLE_LEDS
    HAL_GPIO_WritePin(GPIOB, BSP_LED_u16Pin(led), GPIO_PIN_RESET);
#else
    (void)led;
#endif
}

static inline void BSP_LED_Toggle(Led_TypeDef led)
{
#ifdef ENABLE_LEDS
    HAL_GPIO_TogglePin(GPIOB, BSP_LED_u16Pin(led));
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
 * SECONDARY, PB6 (MCU TX -> GNSS RX, MIA-M10Q pin H1): push-pull. The final
 * layout leaves R21, the 4.7k pull-up from this net to GNSS_VCC, unfitted
 * (DNP), so an open-drain TX would never reach a high level and the receiver
 * would see a permanently low line: no UBX configuration, and no
 * UBX-RXM-PMREQ before power-down. Push-pull is safe on the final board
 * because the secondary's rail is 1.8 V, the same as the module's I/O domain
 * (V_IO = GNSS_VCC = 1.8 V from the TPS7A0218, VIO_SEL tied low; abs max
 * 1.98 V). The earlier 2.0 V prototype needed open-drain plus that pull-up for
 * exactly this reason. What still matters is never driving the pin while the
 * module is unpowered: GPS_vHwPowerOn/GPS_vHwPowerOff (GPS.c) order the rail
 * before the pins on the way up and the pins before the rail on the way down. */
#define BSP_GPS_UART_TX_MODE_GNSS   GPIO_MODE_AF_PP

/* PRIMARY, PB6 (MCU TX -> fr9 RX): push-pull. No GNSS module is fitted, so
 * nothing on the net is limited to 1.8 V, and driving both levels keeps the
 * link independent of whether R21 and the GNSS LDO happen to be populated on
 * a primary. */
#define BSP_GPS_UART_TX_MODE_FR9    GPIO_MODE_AF_PP

/* SECONDARY, PB7 (GNSS TX, module pin G1 -> MCU RX): no MCU pull. The module
 * drives the line whenever it is powered; an internal pull-up would back-feed
 * an unpowered module's TXD from the 1.8 V rail. The price
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
 * SECONDARY  GNSS_ON: the EN pin of the TPS7A0218 1.8 V LDO that supplies
 *            GNSS_VCC (the module's VCC and V_IO). The module's V_BCKP is on
 *            the always-on 1.8 V VCC rail, so its backup domain (BBR + RTC,
 *            with its own 32.768 kHz crystal) survives GNSS_ON low - which
 *            is why GPS.c sends UBX-RXM-PMREQ before dropping the rail.
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
 * SECONDARY only: on a primary PA4 and PA12 are unconnected (the fr9 owns the
 * battery there).
 *
 * The ADC reference is VDDA, 1.8 V nominal on the secondary. The firmware does
 * not rely on the nominal value: it measures VDDA through VREFINT on every
 * sample. Full scale at the battery is therefore about 4.3 x 1.8 V = 7.7 V.
 * ----------------------------------------------------------------------- */
#define BSP_BAT_BIAS_ENABLE_PORT        GPIOA
#define BSP_BAT_BIAS_ENABLE_PIN         GPIO_PIN_4
#define BSP_BAT_MEAS_VOLT_PORT          GPIOA
#define BSP_BAT_MEAS_VOLT_PIN           GPIO_PIN_12
#define BSP_BAT_ADC_VOLTAGE_CHANNEL     ADC_CHANNEL_8

/* -----------------------------------------------------------------------
 * External Flash SPI (SPI2) — PA5/PA8/PA10/PA15
 * AT25EU0041A, 512 KB NOR flash, fitted on both roles. CS has a 10k external
 * pull-up to VCC (R1); WP and HOLD are left unconnected on the PCB.
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
 * ratio 4.3. Same reference as the battery: VDDA, 1.8 V nominal, measured
 * through VREFINT on every sample, so full scale is about 7.7 V. SECONDARY
 * only; PB3 is unconnected on a primary.
 *
 * PB3 resets as AF0 (JTDO/TRACESWO), which leaves a digital input stage
 * sitting on the divider voltage. HAL_GPIO_vInit() parks it analog with no
 * pull straight away, before the ADC driver takes it over.
 * ----------------------------------------------------------------------- */
#define BSP_VSOLAR_MEAS_PORT            GPIOB
#define BSP_VSOLAR_MEAS_PIN             GPIO_PIN_3
#define BSP_VSOLAR_ADC_CHANNEL          ADC_CHANNEL_2

/* -----------------------------------------------------------------------
 * RF front end - no RF switch
 *
 * Both final boards use the STM32WL switchless ("direct tie") front end:
 * RFO_LP and RFI_P/RFI_N meet in one matching network on one antenna port (a
 * chip antenna on the secondary, a U.FL connector on the primary), and RFO_HP
 * is left open. The radio changes over between its LP PA and its LNA
 * internally, so firmware has no path to select; radio_board_if.c's
 * RBI_ConfigRFSwitch() is a no-op and RBI_GetTxConfig() reports RFO_LP only.
 *
 * PB2 (RF_SW_VDD) and PA9 (RF_SW_CTRL) powered and steered a GPIO-supplied
 * switch on the pre-production layout. On the final boards both are
 * unconnected, and HAL_GPIO_vInit() parks them analog with no pull.
 * ----------------------------------------------------------------------- */
#define BSP_UNUSED_PB2_PORT         GPIOB
#define BSP_UNUSED_PB2_PIN          GPIO_PIN_2
#define BSP_UNUSED_PA9_PORT         GPIOA
#define BSP_UNUSED_PA9_PIN          GPIO_PIN_9

/* -----------------------------------------------------------------------
 * Device role sense — PB12 ROLE_BIT0 (primary = HIGH / secondary = LOW)
 *
 * R9, 10k: to GND on the secondary board (LOW), to VCC on the primary
 * daughter board (HIGH). HAL_GPIO_vInit() configures it as an input with NO internal pull (the
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
  *            vPortSuppressTicksAndSleep(). The frtag3 bootloader starts it
 *            first, in its own SystemClock_Config() (an LSE failure hangs the
 *            bootloader, before this image runs); the LSE keeps running across
 *            the jump and is re-asserted by
 *            SystemClock_Config() (hal_system.c) with RCC_LSEDRIVE_LOW; if it
 *            never starts, HAL_RCC_OscConfig() gives up after
 *            LSE_STARTUP_TIMEOUT (5 s) and boot hangs in Error_Handler().
 * PB0        VDD_TCXO: the supply output of the sub-GHz radio's TCXO
 *            regulator, feeding the 32 MHz TCXO (ECS-TXO-25CSMV-320-AN-TR)
 *            through a ferrite bead. Its level is TCXO_CTRL_VOLTAGE in
 *            radio_conf.h, and it differs per role. Not a GPIO.
 * PA13/PA14  SWDIO/SWCLK: left in their reset debug function at all times,
 *            STOP2 included, so firmware never cuts SWD off. (Debugging across
 *            STOP2 still needs the debugger to set DBGMCU DBG_STOP.)
 * PH3        BOOT0: external pull-down (boot from main flash). Never touched.
 * ----------------------------------------------------------------------- */

#endif /* HAL_BSP_H_ */
