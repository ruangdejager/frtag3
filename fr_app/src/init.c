/*
 * init.c
 *
 * Boot-sequence task.
 * Runs once immediately after the scheduler starts, initialises all
 * hardware and software subsystems in the correct order, then exits.
 *
 * The board GPIO safe state (LEDs, enables, chip-selects, RF switch, unused
 * pins) is NOT set here. HAL_GPIO_vInit() runs in main(), right after
 * HAL_Init() and before SystemClock_Config(): when the image runs without the
 * bootloader, the LSE start there can take seconds, and PA0/PA4/PB2/PA9/PA11 have no external pulls, so the pins
 * have to reach a safe state before it rather than after it.
 *
 * Order matters:
 *   1. Device-role detection (primary vs secondary)
 *   2. UART (required by DBG macros)
 *   3. Flash SPI bus, DbgLog, ext-flash, text log
 *   4. ADC, TIM2 tick counter
 *   5. Platform (creates heartbeat task — must come before HAL_RTC_vInit)
 *   6. Settings
 *   7. RTC  (starts 1 Hz wakeup interrupt — heartbeat task must exist)
 *   8. LoRa radio stack
 *   9. Role-specific subsystems (the accelerometer SPI bus is started here,
 *      on a secondary only)
 *  10. Time, battery, power management
 *  11. Boot self-test
 */

#include "init.h"
#include "build_config.h"
#include "version_config.h"
#include "hal_rtc.h"
#include "hal_timer.h"
#include "hal_uart.h"
#include "hal_spi.h"
#include "hal_system.h"
#include "platform.h"
#include "hal_gpio.h"
#include "hal_adc.h"
#include "hal_wdt.h"
#include "settings.h"
#include "main.h"

#include "Gps.h"
#include "Acc.h"

#include "Movement.h"
#include "LoraRadio.h"
#include "MeshNetwork.h"
#include "DeviceDiscovery.h"
#include "Farmranger.h"
#include "FrKernel_Config.h"
#include "Battery.h"
#include "SolarPower.h"
#include "Power.h"
#include "FrKernel.h"

#include "flashLog.h"
#include "Flash.h"
#include "Log.h"
#include "Fota.h"
#include "DbgLog.h"
#include "SelfTest.h"


#include "cmsis_os2.h"

volatile bool systemReadyForSleep = false;

void INIT_vInitialization(void *parameters)
{
    (void)parameters;

    FLASHLOG_vInit();

    /* No HAL_GPIO_vInit() here any more: the board GPIO safe state is set in
     * main() before SystemClock_Config() (see the file header), so by the time
     * this task runs every board pin is already in its defined state. */

    /* Determine primary / secondary role before UART init. This is the only
     * read of the role strap (PB12); it parks the pin analog immediately after,
     * so latch the role into the UART layer here for the USART1 baud and
     * PB6/PB7 drive mode choice rather than having HAL_UART_vInit() re-read the
     * strap on every (re-)init. */
    DEVICE_DISCOVERY_vConfigDeviceRole();
    HAL_UART_vSetRole(DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY);

    HAL_UART_vInit();

    /* The flash bus (SPI2) must be up before FLASH_vInit and any DBG_LOG call
     * that writes to ext-flash. The flash is fitted on both roles. The
     * accelerometer bus (SPI1) is started further down, on a secondary only:
     * there is no accelerometer on a V3 RevA primary. */
    HAL_SPI_FLASH_vInit();

    DBGLOG_vInit();

    /* Storage backend + log-FIFO recovery BEFORE the first DBG_LOG.
     *
     * Ordering is load-bearing, not cosmetic. DBG_LOG publishes to the DbgLog
     * ring and wakes its consumer task, which runs at osPriorityAboveNormal
     * and so preempts this osPriorityNormal init task immediately. The
     * consumer's flash sink is LOG_vWrite(), whose head pointer is a
     * zero-initialised static - and 0 is OTA_SCRATCH_START_ADDR, the base of
     * the staged firmware image. Logging before LOG_vInit() therefore aimed
     * the log's own sector-erase at the image. Because it is boot-only code
     * the damage only ever appeared after a power cycle, with the image
     * verifying clean right up until the reset.
     *
     * FOTA_vInit() is deliberately after LOG_vInit() too: it DBG_LOGs on its
     * metadata-migration path, which sat squarely in the same window.
     * (LOG_vWrite() now range-checks as well - see the guard there - so this
     * ordering is the belt and that check is the braces.) */
    FLASH_vInit();
    LOG_vInit();
    /* One-time OTA partition migration + latch installed-version into
     * TAMP->BKP3R for the bootloader to compare against (see Fota_Config.h). */
    FOTA_vInit();

    /* Record the active build_config.h feature set at the top of every boot
     * log, so "what did I actually flash" is recoverable from the terminal
     * (or, once persisted, the flash log) instead of only from whatever
     * .cproject or build_config.h happened to say at build time. Only lists
     * options this TU actually saw defined -- the string is assembled by the
     * preprocessor, not read back at runtime.
     *
     * Now that this runs after LOG_vInit(), the banner actually reaches the
     * persisted log - previously it was queued while bDevicePresent was still
     * false and silently dropped, which is why "Build config:" never appeared
     * in any field-pulled flash log. */
    DBG_LOG("\r\n\r\nBuild config:\r\n"
        " FW v%u.%u.%u\r\n",
        (unsigned)VERSION_SW_MAJOR,
        (unsigned)VERSION_SW_MINOR,
        (unsigned)VERSION_SW_PATCH);
    DBG_LOG(""
#ifdef FRKERNEL_INTERFACE_UART
        " FRKERNEL_INTERFACE_UART\r\n"
#endif
#ifdef FRKERNEL_INTERFACE_LORA
        " FRKERNEL_INTERFACE_LORA\r\n"
#endif
#ifdef FRKERNEL_INTERFACE_LORA_BRIDGE
        " FRKERNEL_INTERFACE_LORA_BRIDGE\r\n"
#endif
#ifdef DEBUG_OUTPUT_UART
        " DEBUG_OUTPUT_UART\r\n"
#endif
#ifdef DEBUG_OUTPUT_USB
        " DEBUG_OUTPUT_USB\r\n"
#endif
#ifdef ENABLE_LEDS
        " ENABLE_LEDS\r\n"
#endif
#ifdef ENABLE_FLASH_LOG
        " ENABLE_FLASH_LOG\r\n"
#endif
#ifdef ENABLE_MOVEMENT_SENSOR_DATA_API
        " ENABLE_MOVEMENT_SENSOR_DATA_API\r\n"
#endif
#ifdef ENABLE_FLASH_SELFTEST_DESTRUCTIVE
        " ENABLE_FLASH_SELFTEST_DESTRUCTIVE\r\n"
#endif
#ifdef ENABLE_FLASH_READ_CRC
        " ENABLE_FLASH_READ_CRC\r\n"
#endif
#ifdef ENABLE_FLASH_STRESS_TEST
        " ENABLE_FLASH_STRESS_TEST\r\n"
#endif
#ifdef ENABLE_MESH_DIAG_COUNTERS
        " ENABLE_MESH_DIAG_COUNTERS\r\n"
#endif
#ifdef ENABLE_MESH_VERBOSE_LOG
        " ENABLE_MESH_VERBOSE_LOG\r\n"
#endif
#ifdef ENABLE_SOLAR_PERIODIC_LOG
        " ENABLE_SOLAR_PERIODIC_LOG\r\n"
#endif
#ifdef ENABLE_GPS_PERIODIC_LOG
        " ENABLE_GPS_PERIODIC_LOG\r\n"
#endif
#ifdef ENABLE_GNSS_LOGGER_BUILD
        " ENABLE_GNSS_LOGGER_BUILD\r\n"
#endif
#ifdef ENABLE_GNSS_NMEA_GGA
        " ENABLE_GNSS_NMEA_GGA\r\n"
#endif
#ifdef TIME_TZ_DEFAULT_BRISBANE
        " TIME_TZ_DEFAULT_BRISBANE\r\n"
#endif
        "\r\n");

    FLASHLOG_vDump();     /* no-op unless ENABLE_FLASH_LOG + DEBUG_OUTPUT_UART both defined */

    // Enable WDT
    HAL_WDT_vInit();
    HAL_WDT_vReset();

    HAL_ADC_vInit();
    HAL_TIMER_vInit();

    /* Platform must be created before RTC so the heartbeat task handle
     * exists when the first RTC wakeup ISR fires. */
    PLATFORM_vInit();
    SETTINGS_vInit();

    HAL_RTC_vInit();

    osDelay(100);

    LORARADIO_vInit();
    FRKERNEL_vInit();

#ifdef FRKERNEL_INTERFACE_LORA_BRIDGE
    /*
     * UART<->LoRa FrKernel bridge (bench test rig): this primary's only job
     * is relaying whatever the user types over the debug UART out as a LoRa
     * FrKernel command, and printing whatever answer comes back. MeshNetwork
     * supplies the radio TX/RX plumbing (its parser task is what delivers
     * inbound FrKernel packets to FRKERNEL.c's callback); DeviceDiscovery,
     * Farmranger, GPS and Movement are all skipped -- a scheduled campaign or
     * logger session would fight this device for the radio/UART instead of
     * just relaying.
     */
    MESHNETWORK_vInit();

    /* This rig has to stay reachable at any moment: a sleep lock acquired
     * and never released holds the whole system on the tickless-idle LIGHT
     * WFI path forever, so it never drops into STOP2. That matters for two
     * things at once -- STOP2 parks the debug UART pins to analog (the
     * terminal would go dead), and it suspends the CPU core entirely, so
     * the LoRa radio task could only service RX/TX once per 1 Hz RTC
     * heartbeat instead of promptly. Without this the bridge looks "asleep"
     * within moments of boot completing. */
    SYSTEM_vSleepLockAcquire();

    /* Sample the channel noise floor for the whole life of this build. The
     * bridge is the rig a range test is watched from, and a beacon's RSSI
     * means little without the noise it is competing against — without this
     * every beacon line it prints would read "nf=n/a".
     *
     * Affordable precisely here and nowhere else: it costs the radio task one
     * extra SUBGRF read per idle pass of its 50 ms loop, which is nothing on a
     * mains-or-bench rig that is already pinned out of STOP2 by the lock
     * above, and would be a needless wakeup cost on a battery-powered node. */
    LORARADIO_vSetNoiseFloorSampling(true);
#else
    MESHNETWORK_vInit();
    DEVICE_DISCOVERY_vInit();

    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
    {
        SOLAR_vInit();
        /* The accelerometer bus has to be up before ACC_vInit(): every
         * register write goes through HAL_SPI_ACC_vSelect, and on a bus that
         * was never started the transfers return HAL_BUSY and do nothing -
         * the AN3308 bring-up would be silently skipped. */
        HAL_SPI_ACC_vInit();
        ACC_vInit();
        MOVE_vInit();
        GPS_vInit();
    }
    else
    {
        /* No accelerometer on a Tagskaap V3 RevA primary, so nothing of it is
         * touched here. SPI1 is never started, so PA1/PA6/PA7 keep their reset
         * analog state and are left out of the STOP2 park. PA11 (ACC_CS) was
         * driven high by HAL_GPIO_vInit before the role was known; nothing is
         * fitted on it and it has no external pull, so that costs nothing.
         * Movement is not started either: MOVE_eGetState() reports its MOVING
         * default, as it always has on a primary, and the boot self-test
         * reports acc=n/a.
         *
         * frtag2 primaries did carry an accelerometer they did not use, and
         * applied a low-power idle register profile to it here; its current
         * measurements now live with HAL_SPI_vOnSleep in hal_spi.c, where they
         * are the reason a secondary's ACC pins are driven in STOP2. */

        /* Primary: bring up the Farmranger UART link to the fr9 logger board.
         * (Debug UART is a separate peripheral, so the two no longer conflict.) */
        FARMRANGER_vInit();
    }
#endif /* FRKERNEL_INTERFACE_LORA_BRIDGE */

    TIME_vInit();
    BAT_vInit();
    POWER_vInit();

    /* Peripheral smoke test — checks GPS UART and ACC WHO_AM_I on a secondary,
     * and the ext-flash on both roles. Records each result for later FrKernel
     * queries and flashes a green-LED error code for anything that failed.
     * Blocking; returns once the LED sequence is done (no delay on a clean
     * boot). Never fatal — normal boot always proceeds after this. */
    SELFTEST_vRunAndReport();

    BSP_LED_Off(LED_GREEN);

    /* Let the whole boot-time DBG_LOG output -- including the build-config
     * line above -- actually drain out of DbgLog's ring and off the wire
     * before this task exits. systemReadyForSleep is what gates the first
     * STOP2 entry (INIT_bIsSleepReady(), checked by
     * vPortSuppressTicksAndSleep()), so delaying it here is what makes the
     * boot log's completion a precondition of the first sleep, not a race
     * against it (same "let the log line drain" pattern used before
     * FrKernel's prodsleep and FOTA_vArmBootloaderAndReset's reset). */
    osDelay(100);

    systemReadyForSleep = true;

    osThreadExit();
}

bool INIT_bIsSleepReady(void)
{
    return systemReadyForSleep;
}
