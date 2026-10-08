/*
 * main.c  (fr_app/src/main.c)
 *
 * Application entry point for the frtag firmware.
 *
 * Responsibilities:
 *   - Enable configurable fault handlers (BusFault, MemManage)
 *   - Capture and clear the RCC reset-cause register (CSR) before any
 *     HAL call that might clear it
 *   - HAL_Init()
 *   - HAL_GPIO_vInit() — board GPIO safe state, BEFORE the clock tree
 *     (Hal/src/hal_gpio.c; see the comment at the call for why)
 *   - SystemClock_Config() — 48 MHz PLL and the LSE crystal
 *     (Hal/src/hal_system.c)
 *   - Initialise the CMSIS-RTOS v2 kernel
 *   - Create the single boot-sequence task (INIT_vInitialization)
 *   - Start the scheduler
 *
 * All remaining subsystem initialisation happens inside
 * INIT_vInitialization() once the scheduler is running. HAL_GPIO_vInit() is
 * the one exception, because it has to run before SystemClock_Config().
 */

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "tag_hal.h"
#include "platform.h"
#include "init.h"
#include "cmsis_os2.h"

#include "hal_system.h"

static uint32_t csr;

/* --------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------- */
int main(void)
{
    /* Enable configurable fault handlers for easier debug */
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

    /* Snapshot RCC reset-cause flags before HAL_Init() clears them */
    csr = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    HAL_Init();

    /* Board pins to their safe state BEFORE the clock tree comes up.
     *
     * Several pins on this board have no external pull (PA0, PA4, PB2, PA9,
     * PA11), so from reset they float: PA0 is the fr9 wake line on a primary
     * and the GNSS LDO enable on a secondary, PA11 is the accelerometer chip
     * select, PA9/PB2 drive the RF switch. SystemClock_Config() below starts
     * the LSE crystal. Behind the frtag2 bootloader (every deployed board)
     * the LSE is already running: the bootloader starts it in its own
     * SystemClock_Config() and HAL_RCC_DeInit() leaves the backup domain
     * alone, so the wait here is effectively zero, and any multi-second cold
     * start or LSE-failure hang happens inside the bootloader, before this
     * code runs. The bootloader has also already started the IWDG (prescaler
     * 64, reload 4095 on the ~32 kHz LSI, about 8 s), which cannot be stopped,
     * so a hang later in the app is reset by it. Only an image started
     * without the bootloader (a debugger launch straight into this image)
     * does the full cold start here - up to LSE_STARTUP_TIMEOUT (5 s) - and
     * hangs for good in Error_Handler() if the crystal never starts, because
     * this application only starts its own watchdog later, from the init
     * task; in that case both LEDs stay lit instead of the board looking dead.
     * Either way, driving the pins first is the earliest point the
     * application can reach, so its own float window is as short as it can
     * be.
     *
     * HAL_GPIO_vInit() therefore runs at the 4 MHz reset MSI clock before the
     * kernel exists, and must stay free of RTOS calls, logging and delays.
     *
     * The window before the application is outside this repo: the frtag2
     * bootloader runs first and calls HAL_DeInit() before jumping here, so
     * every pin is back at its reset state on entry, and the unpulled ones
     * float for the whole bootloader run. Only that bootloader can close that
     * window. */
    HAL_GPIO_vInit();

    /* 48 MHz PLL from HSI, plus the LSE (hal_system.c) */
    SystemClock_Config();

    /* ---- CMSIS-RTOS v2 / FreeRTOS ------------------------------------- */
    osKernelInitialize();

    static const osThreadAttr_t initTask_attr = {
        .name       = "InitTask",
        .stack_size = configMINIMAL_STACK_SIZE * 8,  /* 1024 bytes */
        .priority   = osPriorityNormal,
    };
    osThreadNew(INIT_vInitialization, NULL, &initTask_attr);

    osKernelStart();

    /* Should never be reached — scheduler never returns */
    while (1) { }
}

/* --------------------------------------------------------------------------
 * u32GetCSR — returns the RCC reset-cause register captured at startup
 * Callers (e.g. FLASHLOG in init.c) use this to log the reset reason.
 * -------------------------------------------------------------------------- */
uint32_t u32GetCSR(void)
{
    return csr;
}
